/*
 * Copyright (c) 2026 G1_Hand 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    finger_task.c
 * @author  maximilian
 * @version V1.0.0
 * @date    2026-06-08
 * @brief   手指电机通讯任务实现
 * @attention
 *
 * Copyright (c) 2026 G1_Hand 项目组.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "finger_task.h"

#include "board.h"
#include "bsp_systick.h"
#include "drv_rs485.h"
#include "fsm.h"
#include "protocol_parser.h"

#include "log.h"

#include <stdio.h>
#include <string.h>

/* Private constants ---------------------------------------------------------*/

/** @brief 应答帧头 */
static const uint8_t s_response_header[] = { 0xAA, 0x55 };
#define RESPONSE_HEADER_LEN 2U

/** @brief 每端口 parser 输入缓冲区大小 */
#define PARSER_INPUT_BUF_SIZE 256U

/** @brief 每端口 parser 输出缓冲区大小 */
#define PARSER_OUTPUT_BUF_SIZE 64U

/** @brief 主循环轮询中单次最大读取字节数 */
#define MAX_RX_READ_PER_POLL 64U

/* Private types -------------------------------------------------------------*/

/**
 * @brief 端口发送 FSM 状态枚举
 */
typedef enum {
    PORT_TX_IDLE = 0, /**< 空闲，可发送下一帧 */
    PORT_TX_WAITING_RESPONSE, /**< 等待应答 */
    PORT_TX_STATE_COUNT /**< 状态总数 */
} port_tx_state_t;

/**
 * @brief 每端口运行时状态
 */
typedef struct {
    bool active; /**< 端口是否已激活 */
    rs485_port_t port; /**< 端口号 */

    /* 协议解析器 */
    protocol_parser_context_t parser; /**< 应答帧解析器上下文 */
    uint8_t parser_input_buf[PARSER_INPUT_BUF_SIZE]; /**< 解析器输入 kfifo 缓冲 */
    uint8_t parser_output_buf[PARSER_OUTPUT_BUF_SIZE]; /**< 解析器输出缓冲 */

    /* 发送 FSM */
    fsm_t tx_fsm; /**< 发送状态机上下文 */
    fsm_handler_t tx_handlers[PORT_TX_STATE_COUNT]; /**< 状态处理器表 */
    fsm_guard_t tx_transitions[PORT_TX_STATE_COUNT
        * PORT_TX_STATE_COUNT]; /**< 转换矩阵 */

    /* 发送运行时数据 */
    finger_handle_t* current_motor; /**< 当前等待应答的电机 */
    uint32_t cmd_send_ms; /**< 命令发送时刻（毫秒） */
    uint32_t last_cmd_ms; /**< 上一条命令完成时刻（毫秒） */

    /* 统计 */
    uint32_t tx_count; /**< 累计发送帧数 */
    uint32_t rx_count; /**< 累计接收帧数 */
    uint32_t timeout_count; /**< 累计超时次数 */
} port_ctx_t;

/* Private variables ---------------------------------------------------------*/

static port_ctx_t s_motor_port1; /**< 电机端口1状态 (RS485_PORT_MOTOR1) */
static port_ctx_t s_motor_port2; /**< 电机端口2状态 (RS485_PORT_MOTOR2) — TX 镜像 + 独立 RX */
static bool s_finger_task_initialized; /**< 任务初始化标志 */

/** @brief 发送状态名称表（FSM 调试用） */
static const char* s_tx_state_names[] = {
    "IDLE",
    "WAIT_RESP",
};

/* Private function prototypes -----------------------------------------------*/

static bool finger_task_is_timeout(uint32_t start_ms, uint32_t timeout_ms);
static uint16_t finger_parser_get_len_cb(uint8_t* buffer, uint16_t len);
static protocol_parser_error_t finger_parser_check_cb(uint8_t* buffer,
    uint16_t len);

/* FSM handlers */
static fsm_state_t port_tx_idle_handler(fsm_t* ctx);
static fsm_state_t port_tx_waiting_handler(fsm_t* ctx);

static void finger_task_init_port(port_ctx_t* ctx, rs485_port_t port,
    const char* name);
static void finger_task_process_rx(port_ctx_t* port_ctx);
static void finger_task_dispatch_response(port_ctx_t* port_ctx,
    const uint8_t* frame, uint16_t len);
static finger_handle_t* finger_task_find_pending_motor(rs485_port_t port);

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 检查是否超时
 * @param start_ms 起始毫秒时间戳
 * @param timeout_ms 超时毫秒数
 * @return true 已超时，false 未超时
 */
static bool finger_task_is_timeout(uint32_t start_ms, uint32_t timeout_ms)
{
    return (millis() - start_ms) >= timeout_ms;
}

/**
 * @brief 帧长度计算回调：读取帧长度字段（字节 2）
 * @param buffer 帧缓冲区
 * @param len 当前数据长度
 * @return 完整帧长度，0 表示数据不完整
 */
static uint16_t finger_parser_get_len_cb(uint8_t* buffer, uint16_t len)
{
    (void)len;

    if (!buffer) {
        return 0;
    }

    /* 应答帧字节 2 为长度字段（帧头 0xAA 0x55 之后） */
    uint16_t frame_len = (uint16_t)buffer[RESPONSE_HEADER_LEN];

    /* 最小帧长校验：帧头(2)+长度(1)+ID(1)+指令(1)+控制表(1)+校验(1)=7 */
    if (frame_len < 7) {
        return 0;
    }

    return frame_len;
}

/**
 * @brief 帧校验回调：累加校验和验证
 * @param buffer 完整帧数据
 * @param len 帧长度
 * @return 校验结果
 */
static protocol_parser_error_t finger_parser_check_cb(uint8_t* buffer,
    uint16_t len)
{
    if (!buffer || len < 2) {
        return PROTOCOL_PARSER_ERROR_CHECKSUM;
    }

    /* 累加除最后一个字节（校验和）外的所有字节 */
    uint32_t sum = 0;
    for (uint16_t i = 0; i < len - 1; i++) {
        sum += buffer[i];
    }

    uint8_t expected = (uint8_t)(sum & 0xFFU);
    uint8_t received = buffer[len - 1];

    if (expected != received) {
        return PROTOCOL_PARSER_ERROR_CHECKSUM;
    }

    return PROTOCOL_PARSER_OK;
}

/* --- FSM 状态处理器 --- */

/**
 * @brief IDLE 状态处理器：扫描待发送命令并启动发送
 * @param ctx FSM 上下文
 * @return 下一状态
 */
static fsm_state_t port_tx_idle_handler(fsm_t* ctx)
{
    port_ctx_t* port_ctx = (port_ctx_t*)fsm_user_data(ctx);

    /* 检查命令间隔 */
    if (port_ctx->last_cmd_ms != 0) {
        if (!finger_task_is_timeout(port_ctx->last_cmd_ms,
                FINGER_TASK_INTER_CMD_MS)) {
            return PORT_TX_IDLE;
        }
    }

    /* 处理用户请求的命令 */
    finger_handle_t* motor = finger_task_find_pending_motor(port_ctx->port);

    if (!motor) {
        return PORT_TX_IDLE;
    }

    /* 构建命令帧 */
    uint8_t* frame = NULL;
    uint16_t frame_len = 0;
    finger_error_t err = finger_build_command(motor, &frame, &frame_len);
    if (FINGER_IS_ERR(err) || !frame || frame_len == 0) {
        return PORT_TX_IDLE;
    }

    /* 诊断：打印每帧发送数据 */
    LOG_I("finger", "TX:%d帧,端口%d,电机ID=0x%02X(len=%u)",
        (unsigned int)port_ctx->tx_count, port_ctx->port,
        (unsigned int)motor->config.motor_id, frame_len);
    // LOG_HEXDUMP("finger", frame, frame_len);

    /* 发送 */
    uint32_t now_ticks = millis();
    hpm_stat_t stat = rs485_send(port_ctx->port, frame, frame_len);
    if (stat != status_success) {
        LOG_E("finger", "端口%d 发送失败, 错误码=%d", port_ctx->port, stat);
        return PORT_TX_IDLE;
    }

    port_ctx->current_motor = motor;
    port_ctx->cmd_send_ms = now_ticks;
    port_ctx->tx_count++;

    return PORT_TX_WAITING_RESPONSE;
}

/**
 * @brief WAITING_RESPONSE 状态处理器：等待应答或超时
 * @param ctx FSM 上下文
 * @return 下一状态
 */
static fsm_state_t port_tx_waiting_handler(fsm_t* ctx)
{
    port_ctx_t* port_ctx = (port_ctx_t*)fsm_user_data(ctx);

    /* 检查超时 */
    if (finger_task_is_timeout(port_ctx->cmd_send_ms,
            FINGER_TASK_RESPONSE_TIMEOUT_MS)) {
        port_ctx->timeout_count++;

        /* 清除电机待发送状态，避免同一帧反复重发 */
        if (port_ctx->current_motor) {
            finger_cmd_type_t timed_out_cmd = port_ctx->current_motor->pending_cmd;
            port_ctx->current_motor->pending_cmd = FINGER_CMD_NONE;
            port_ctx->current_motor->response_pending = false;

            /* 通知上层超时 */
            if (port_ctx->current_motor->response_cb) {
                port_ctx->current_motor->response_cb(
                    port_ctx->current_motor, timed_out_cmd, false,
                    port_ctx->current_motor->callback_user_data);
            }
        }

        port_ctx->current_motor = NULL;
        port_ctx->last_cmd_ms = millis();

        return PORT_TX_IDLE;
    }

    return PORT_TX_WAITING_RESPONSE;
}

/* --- 端口初始化 --- */

/**
 * @brief 初始化单个端口（协议解析器 + 发送 FSM）
 * @param ctx  端口上下文指针
 * @param port RS-485 端口号
 * @param name 解析器名称
 */
static void finger_task_init_port(port_ctx_t* ctx, rs485_port_t port,
    const char* name)
{
    memset(ctx, 0, sizeof(port_ctx_t));
    ctx->port = port;
    ctx->current_motor = NULL;

    /* 1. 初始化协议解析器 */
    protocol_parser_config_t parser_cfg;
    parser_cfg.name = name;
    parser_cfg.header = s_response_header;
    parser_cfg.header_len = RESPONSE_HEADER_LEN;
    parser_cfg.footer = NULL;
    parser_cfg.footer_len = 0;
    parser_cfg.get_len_cb = finger_parser_get_len_cb;
    parser_cfg.check_cb = finger_parser_check_cb;
    parser_cfg.input_buffer = ctx->parser_input_buf;
    parser_cfg.input_buffer_len = PARSER_INPUT_BUF_SIZE;
    parser_cfg.output_buffer = ctx->parser_output_buf;
    parser_cfg.output_buffer_len = PARSER_OUTPUT_BUF_SIZE;

    protocol_parser_error_t err = protocol_parser_init(&ctx->parser, &parser_cfg);
    if (err != PROTOCOL_PARSER_OK) {
        LOG_E("finger", "端口%d 协议解析器初始化失败, 错误码=%d", port, err);
        return;
    }

    /* 2. 初始化发送 FSM */
    memset(ctx->tx_handlers, 0, sizeof(ctx->tx_handlers));
    memset(ctx->tx_transitions, 0, sizeof(ctx->tx_transitions));

    ctx->tx_handlers[PORT_TX_IDLE] = port_tx_idle_handler;
    ctx->tx_handlers[PORT_TX_WAITING_RESPONSE] = port_tx_waiting_handler;

    fsm_config_t fsm_cfg = {
        .handlers = ctx->tx_handlers,
        .transitions = ctx->tx_transitions,
        .state_count = PORT_TX_STATE_COUNT,
        .entry_cb = NULL,
        .exit_cb = NULL,
        .state_names = s_tx_state_names,
        .user_data = ctx,
    };

    /* 全连通转换矩阵（IDLE ↔ WAITING_RESPONSE 可任意切换） */
    fsm_fill(&fsm_cfg, fsm_always_true);

    fsm_err_t fsm_err = fsm_init(&ctx->tx_fsm, PORT_TX_IDLE, &fsm_cfg);
    if (fsm_err != FSM_OK) {
        LOG_E("finger", "端口%d FSM 初始化失败, 错误码=%d", port, fsm_err);
        return;
    }

    ctx->active = true;

    LOG_I("finger", "端口%d (%s) 已初始化 (解析器+FSM)", port, name);
}

/* --- 接收处理 --- */

/**
 * @brief 处理端口的接收数据
 * @param port_ctx 端口上下文
 */
static void finger_task_process_rx(port_ctx_t* port_ctx)
{
    /* 读取 RS-485 接收缓冲区 */
    uint32_t avail = rs485_rx_available(port_ctx->port);
    if (avail == 0) {
        return;
    }

    uint8_t buf[MAX_RX_READ_PER_POLL];
    uint32_t read_len = (avail > sizeof(buf)) ? sizeof(buf) : avail;
    read_len = rs485_rx_read(port_ctx->port, buf, read_len);

    if (read_len > 0) {
        /* 打印原始 RX 数据 */
        LOG_I("finger", "RX 端口%d 收到%u字节", port_ctx->port,
            (unsigned int)read_len);
        LOG_HEXDUMP("finger", buf, read_len);

        protocol_parser_error_t err = protocol_parser_feed(
            &port_ctx->parser, buf, read_len);
        if (err != PROTOCOL_PARSER_OK) {
            /* 喂入失败（如 FIFO 溢出），清空解析器重来 */
            (void)protocol_parser_clear(&port_ctx->parser);
        }
    }
}

/**
 * @brief 将解析出的应答帧分发到对应的电机实例
 * @param port_ctx 端口上下文
 * @param frame 完整应答帧
 * @param len 帧长度
 */
static void finger_task_dispatch_response(port_ctx_t* port_ctx,
    const uint8_t* frame, uint16_t len)
{
    /*
     * 应答帧结构: [0xAA][0x55][长度][ID][指令][控制表][数据...][校验]
     * ID 在字节 3（偏移 3）
     */
    if (len < 7) {
        return;
    }

    uint8_t motor_id = frame[3];
    finger_handle_t* motor = finger_get_by_id(motor_id, port_ctx->port);

    LOG_I("finger", "RX 端口%d 解析完成: 电机=0x%02X 帧长=%u",
        port_ctx->port, (unsigned int)motor_id, (unsigned int)len);
    LOG_HEXDUMP("finger", frame, len);

    if (!motor) {
        LOG_W("finger", "RX 端口%d 未找到电机实例 0x%02X",
            port_ctx->port, (unsigned int)motor_id);
        return;
    }

    port_ctx->rx_count++;

    /* 分发给电机服务层处理 */
    (void)finger_process_response(motor, frame, len);

    /* 如果当前正在等待此电机的应答，通过 FSM 切换回 IDLE */
    if (port_ctx->current_motor == motor) {
        port_ctx->current_motor = NULL;
        port_ctx->last_cmd_ms = millis();
        (void)fsm_goto(&port_ctx->tx_fsm, PORT_TX_IDLE);
    }
}

/**
 * @brief 在指定端口寻找待发送命令的电机
 * @param port RS-485 端口号
 * @return 电机句柄，无待发送命令返回 NULL
 */
static finger_handle_t* finger_task_find_pending_motor(rs485_port_t port)
{
    clist_head_t* head = finger_get_head();
    if (!head) {
        return NULL;
    }

    finger_handle_t* h;
    clist_for_each_entry(h, head, node)
    {
        if (h->config.rs485_port == port
            && h->response_pending
            && h->pending_cmd != FINGER_CMD_NONE) {
            return h;
        }
    }

    return NULL;
}

/* Exported functions --------------------------------------------------------*/

void finger_task_init_transport(void)
{
    if (s_finger_task_initialized) {
        return;
    }

    /* 初始化电机 RS-485 端口硬件（MOTOR1 + MOTOR2） */
    rs485_init(RS485_PORT_MOTOR1);
    rs485_init(RS485_PORT_MOTOR2);

    /* 初始化两个端口的解析器 + 发送 FSM */
    finger_task_init_port(&s_motor_port1, RS485_PORT_MOTOR1, "motor1_parser");
    finger_task_init_port(&s_motor_port2, RS485_PORT_MOTOR2, "motor2_parser");

    s_finger_task_initialized = true;
    LOG_I("finger", "传输层已初始化, MCHTMR=24MHz, 端口: MOTOR1+MOTOR2");
}

void finger_task_init(void)
{
    /* @deprecated 向后兼容空壳，实际初始化由 motor_link_task_init() 完成 */
    LOG_W("finger", "finger_task_init() 已弃用, 请使用 motor_link_task_init()");
}

void finger_task_poll(void)
{
    if (!s_finger_task_initialized) {
        return;
    }

    /* 双端口独立轮询 */
    port_ctx_t* ports[] = { &s_motor_port1, &s_motor_port2 };
    for (int i = 0; i < 2; i++) {
        port_ctx_t* ctx = ports[i];
        if (!ctx->active) {
            continue;
        }

        /* 1. 读取接收数据并喂入解析器 */
        finger_task_process_rx(ctx);

        /* 2. 更新解析器空闲计时器 */
        (void)protocol_parser_tick(&ctx->parser);

        /* 3. 尝试解析完整应答帧 */
        uint16_t frame_len = 0;
        uint8_t* frame_data = NULL;
        protocol_parser_error_t parse_err = protocol_parser_parse(
            &ctx->parser, &frame_len, &frame_data);

        if (parse_err == PROTOCOL_PARSER_OK) {
            finger_task_dispatch_response(ctx, frame_data, frame_len);
        }

        /* 4. 驱动发送 FSM */
        (void)fsm_step(&ctx->tx_fsm);
    }
}

bool finger_task_get_stats(uint8_t port, uint32_t* p_tx_count,
    uint32_t* p_rx_count, uint32_t* p_timeout_count)
{
    (void)port;

    if (!s_motor_port1.active) {
        return false;
    }

    if (p_tx_count) {
        *p_tx_count = s_motor_port1.tx_count;
    }
    if (p_rx_count) {
        *p_rx_count = s_motor_port1.rx_count;
    }
    if (p_timeout_count) {
        *p_timeout_count = s_motor_port1.timeout_count;
    }

    return true;
}
