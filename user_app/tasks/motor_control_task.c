/*
 * Copyright (c) 2026 G1_Hand 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    motor_control_task.c
 * @author  maximilian
 * @version V3.0.0
 * @date    2026-06-11
 * @brief   CAN-FD 电机控制桥接任务实现
 *
 * 使用 protocol_parser 解析 CAN-FD 接收帧，protocol_packer 构建发送帧。
 * 替代原 canfd_task 演示任务。
 */

/* Includes ------------------------------------------------------------------*/
#include "motor_control_task.h"

#include "bsp_systick.h"
#include "drv_can.h"
#include "motor_control.h"
#include "protocol_packer.h"
#include "protocol_parser.h"

#include "log.h"

#include <string.h>

/* Private constants ---------------------------------------------------------*/

/** @brief 心跳间隔 (ms) */
#define HEARTBEAT_INTERVAL_MS 100U

/** @brief 解析器输入 kfifo 大小 */
#define PARSER_INPUT_BUF_SIZE 256U

/** @brief 解析器/打包器输出缓冲区大小 */
#define PROTO_OUTPUT_BUF_SIZE 64U

/** @brief 协议帧头 */
static const uint8_t s_proto_header[] = { MOTOR_CONTROL_HEADER_VAL };

/** @brief 协议帧尾 */
static const uint8_t s_proto_footer[] = { MOTOR_CONTROL_ENDER_VAL };

/* Private variables ---------------------------------------------------------*/

static drv_can_context_t* s_can_ctx;

/* --- 协议解析器（RX）--- */
static protocol_parser_context_t s_rx_parser;
static uint8_t s_parser_input_buf[PARSER_INPUT_BUF_SIZE];
static uint8_t s_parser_output_buf[PROTO_OUTPUT_BUF_SIZE];

/* --- 协议打包器（TX）--- */
static protocol_packer_context_t s_tx_packer;
static uint8_t s_packer_output_buf[PROTO_OUTPUT_BUF_SIZE];

static volatile uint32_t s_rx_count;
static uint32_t s_tx_count;
static uint32_t s_rx_err_count; /**< 协议帧处理错误计数 */
static uint32_t s_last_heartbeat_ms;
static bool s_safe_mode;
static bool s_ep_warned;

/* Private function prototypes -----------------------------------------------*/

static uint32_t motor_control_task_get_ticks(void);

static void motor_control_task_rx_callback(drv_can_context_t* ctx,
    const mcan_rx_message_t* rx_msg);

/* --- 协议解析器回调 --- */
static uint16_t motor_control_parser_get_len_cb(uint8_t* buffer, uint16_t len);
static protocol_parser_error_t motor_control_parser_check_cb(uint8_t* buffer,
    uint16_t len);

/* --- 协议打包器回调 --- */
static protocol_packer_error_t motor_control_packer_checksum_cb(
    const uint8_t* data, uint16_t len, uint8_t* checksum_out,
    uint16_t* checksum_len);

/* --- CAN-FD DLC 转换 --- */
static uint8_t can_dlc_to_bytes(uint8_t dlc);
static uint8_t can_bytes_to_dlc(uint8_t byte_len);

static bool motor_control_task_send_frame(uint32_t can_id,
    const uint8_t* data, uint8_t byte_len);

/** @brief 将 CAN 总线状态值转换为中文描述 */
static const char* motor_control_task_bus_state_name(drv_can_state_t state);

/*
 * ============================================================================
 * 公共 API — 任务初始化
 * ============================================================================
 */

void motor_control_task_init_transport(void)
{
    /* 1. CAN4 硬件配置 */
    const drv_can_config_t can4_config = {
        .name = "can4",
        .base = HPM_MCAN4,
        .irq_num = IRQn_MCAN4,
        .clock_name = clock_can4,
        .mode = mcan_mode_normal,
        .nominal_baudrate = DRV_CAN_DEFAULT_NOMINAL_BAUDRATE,
        .data_baudrate = DRV_CAN_DEFAULT_DATA_BAUDRATE,
        .txd_ioc_pad = IOC_PAD_PA16,
        .txd_ioc_func = IOC_PA16_FUNC_CTL_MCAN4_TXD,
        .rxd_ioc_pad = IOC_PAD_PA17,
        .rxd_ioc_func = IOC_PA17_FUNC_CTL_MCAN4_RXD,
        .has_stb = true,
        .stb_ioc_pad = IOC_PAD_PA18,
        .stb_ioc_func = IOC_PA18_FUNC_CTL_GPIO_A_18,
        .stb_gpio = HPM_GPIO0,
        .stb_gpio_port = GPIO_DO_GPIOA,
        .stb_gpio_pin = 18,
        .stb_active_level = 0,
        .tdc_manual = true,
        .tdc_ssp_offset = 10,
        .tdc_filter_win = 10,
        .irq_priority = 1,
    };

    LOG_I("motor_ctrl", "正在初始化 CAN4...");

    s_can_ctx = drv_can_init(&can4_config);
    if (!s_can_ctx) {
        LOG_E("motor_ctrl", "drv_can_init() 失败");
        return;
    }

    drv_can_set_rx_callback(s_can_ctx, motor_control_task_rx_callback);

    /* 2. 初始化协议解析器（RX） */
    protocol_parser_config_t parser_cfg;
    memset(&parser_cfg, 0, sizeof(parser_cfg));
    parser_cfg.name = "canfd_rx";
    parser_cfg.header = s_proto_header;
    parser_cfg.header_len = 1;
    parser_cfg.footer = s_proto_footer;
    parser_cfg.footer_len = 1;
    parser_cfg.get_len_cb = motor_control_parser_get_len_cb;
    parser_cfg.check_cb = motor_control_parser_check_cb;
    parser_cfg.input_buffer = s_parser_input_buf;
    parser_cfg.input_buffer_len = PARSER_INPUT_BUF_SIZE;
    parser_cfg.output_buffer = s_parser_output_buf;
    parser_cfg.output_buffer_len = PROTO_OUTPUT_BUF_SIZE;

    protocol_parser_error_t parser_err = protocol_parser_init(&s_rx_parser,
        &parser_cfg);
    if (parser_err != PROTOCOL_PARSER_OK) {
        LOG_E("motor_ctrl", "协议解析器初始化失败, 错误码=%d",
            (int)parser_err);
        return;
    }

    /* 3. 初始化协议打包器（TX） */
    protocol_packer_config_t packer_cfg;
    memset(&packer_cfg, 0, sizeof(packer_cfg));
    packer_cfg.name = "canfd_tx";
    packer_cfg.header = s_proto_header;
    packer_cfg.header_len = 1;
    packer_cfg.footer = s_proto_footer;
    packer_cfg.footer_len = 1;
    packer_cfg.checksum_cb = motor_control_packer_checksum_cb;
    packer_cfg.fill_len_cb = NULL;
    packer_cfg.checksum_len = 1;
    packer_cfg.output_buffer = s_packer_output_buf;
    packer_cfg.output_buffer_len = PROTO_OUTPUT_BUF_SIZE;

    protocol_packer_error_t packer_err = protocol_packer_init(&s_tx_packer,
        &packer_cfg);
    if (packer_err != PROTOCOL_PACKER_OK) {
        LOG_E("motor_ctrl", "协议打包器初始化失败, 错误码=%d",
            (int)packer_err);
        return;
    }

    /* 4. 初始化心跳计时 */
    s_last_heartbeat_ms = motor_control_task_get_ticks();
    s_rx_count = 0;
    s_tx_count = 0;
    s_rx_err_count = 0;
    s_safe_mode = false;
    s_ep_warned = false;

    LOG_I("motor_ctrl", "传输层已初始化: CAN4 1M/5M (解析器+打包器)");
}

void motor_control_task_init(void)
{
    /* @deprecated 向后兼容空壳，实际初始化由 motor_link_task_init() 完成 */
    LOG_W("motor_ctrl", "motor_control_task_init() 已弃用, 请使用 motor_link_task_init()");
}

/*
 * ============================================================================
 * 公共 API — 任务轮询
 * ============================================================================
 */

void motor_control_task_poll(void)
{
    if (!s_can_ctx) {
        return;
    }

    uint32_t now = motor_control_task_get_ticks();

    /* 1. 检查 CAN 总线状态 */
    drv_can_state_t bus_state = drv_can_get_state(s_can_ctx);
    if (bus_state == DRV_CAN_STATE_BUS_OFF) {
        LOG_W("motor_ctrl", "CAN 总线关闭, 正在恢复...");
        drv_can_recover(s_can_ctx);
        s_last_heartbeat_ms = now;
        s_ep_warned = false;
        return;
    }
    if (bus_state == DRV_CAN_STATE_ERROR_PASSIVE) {
        if (!s_ep_warned) {
            LOG_W("motor_ctrl", "CAN 总线错误被动");
            s_ep_warned = true;
        }
    } else {
        s_ep_warned = false;
    }

    /* 2. 排空 CAN-FD 接收 → protocol_parser feed → parse → 分发 */
    while (drv_can_rx_available(s_can_ctx) >= sizeof(mcan_rx_message_t)) {
        mcan_rx_message_t rx_msg;
        drv_can_error_t err = drv_can_rx_read(s_can_ctx, &rx_msg);
        if (err != DRV_CAN_OK) {
            LOG_W("motor_ctrl", "CAN RX 读取失败, 错误码=%d", (int)err);
            continue;
        }

        if (rx_msg.std_id != MOTOR_CONTROL_CAN_ID_CMD || rx_msg.use_ext_id) {
            continue;
        }

        uint8_t data_bytes = can_dlc_to_bytes(rx_msg.dlc & 0x0FU);

        LOG_D("motor_ctrl", "CAN RX ← ID=0x%03X DLC=%u 数据=%u字节",
            (unsigned int)rx_msg.std_id, (unsigned int)(rx_msg.dlc & 0x0FU),
            (unsigned int)data_bytes);

        /* 喂入协议解析器 */
        protocol_parser_error_t feed_err = protocol_parser_feed(
            &s_rx_parser, rx_msg.data_8, data_bytes);
        if (feed_err != PROTOCOL_PARSER_OK) {
            LOG_W("motor_ctrl", "协议解析器 feed 失败, 错误码=%d, 清空缓冲区",
                (int)feed_err);
            (void)protocol_parser_clear(&s_rx_parser);
            continue;
        }
    }

    /* 3. 从协议解析器提取完整帧（已校验 header + footer + checksum）
     *    使用 while 循环一次 poll 排空所有完整帧，避免垃圾残留导致重复报错 */
    while (kfifo_len(&s_rx_parser.fifo)) {
        uint16_t frame_len = 0;
        uint8_t* frame_data = NULL;

        protocol_parser_error_t parse_err = protocol_parser_parse(&s_rx_parser,
            &frame_len, &frame_data);
        if (parse_err == PROTOCOL_PARSER_OK) {
            /* 解析器返回的完整帧可直接转换为 canfd_protocol_t */
            const canfd_protocol_t* proto = (const canfd_protocol_t*)frame_data;

            LOG_I("motor_ctrl", "CAN 帧已解析: len=%u cmd=0x%02X flags=0x%02X datalen=%u",
                (unsigned int)frame_len, (unsigned int)proto->cmd,
                (unsigned int)proto->flags, (unsigned int)proto->datalen);

            if (!s_safe_mode) {
                motor_control_error_t mc_err = motor_control_process_rx_frame(proto);
                if (MOTOR_CONTROL_IS_ERR(mc_err)) {
                    s_rx_err_count++;
                    LOG_W("motor_ctrl", "帧处理失败, 错误码=%d", (int)mc_err);
                }
            } else {
                LOG_W("motor_ctrl", "安全模式已激活, 帧被丢弃");
            }
            /* 继续解析下一个帧，直到 kfifo 为空或不完整 */
            continue;
        }

        if (parse_err == PROTOCOL_PARSER_ERROR_INCOMPLETE) {
            /* 数据尚未收齐，等待下一次 feed */
            break;
        }

        if (parse_err == PROTOCOL_PARSER_ERROR_IDLE_TIMEOUT) {
            /* 空闲超时, 解析器内部已丢弃1字节 → 继续尝试解析剩余数据 */
            continue;
        }

        /* 不可恢复错误: 帧头/帧尾/校验失败 → 清空解析器，丢弃垃圾 */
        LOG_W("motor_ctrl", "协议解析失败(状态=%d), 清空解析器缓冲区",
            (int)parse_err);
        (void)protocol_parser_clear(&s_rx_parser);
        break;
    }

    /* 4. 更新解析器空闲计时器 */
    (void)protocol_parser_tick(&s_rx_parser);
}

/*
 * ============================================================================
 * 私有函数 — 定时器
 * ============================================================================
 */

static uint32_t motor_control_task_get_ticks(void)
{
    return millis();
}

static void motor_control_task_rx_callback(drv_can_context_t* ctx,
    const mcan_rx_message_t* rx_msg)
{
    (void)ctx;
    (void)rx_msg;
    s_rx_count++;
}

/*
 * ============================================================================
 * 协议解析器回调
 * ============================================================================
 */

static uint16_t motor_control_parser_get_len_cb(uint8_t* buffer, uint16_t len)
{
    (void)len;

    /* buffer[0]=header, buffer[1]=cmd, buffer[2]=flags, buffer[3]=datalen */
    if (len < 4) {
        return 0;
    }

    uint8_t datalen = buffer[3];

    /* 仅允许固定长度: 0 (查询), 18 (2B×9电机), 36 (4B×9电机 SET_POSITION_SPEED) */
    if (datalen != 0 && datalen != MOTOR_CONTROL_DATA_BYTES
        && datalen != MOTOR_CONTROL_DATA_MAX) {
        LOG_W("motor_ctrl", "解析器 get_len: datalen=%u 无效 (仅允许 0/18/36)",
            (unsigned int)datalen);
        return 0;
    }

    /* header(1) + cmd(1) + flags(1) + datalen(1) + data(datalen) + crc(1) + footer(1) */
    return (uint16_t)(MOTOR_CONTROL_FRAME_OVERHEAD + datalen);
}

static protocol_parser_error_t motor_control_parser_check_cb(uint8_t* buffer,
    uint16_t len)
{
    /*
     * 帧结构: [header][cmd][flags][datalen][data...][crc][footer]
     * 校验和 = sum(data[0..datalen-1]) & 0xFF，与 buffer[4+datalen] 处的 crc_check 比较
     * buffer 布局: [0]=header, [1]=cmd, [2]=flags, [3]=datalen,
     *              [4..4+datalen-1]=data, [4+datalen]=crc_check, [5+datalen]=footer
     */
    if (!buffer || len < 6) {
        return PROTOCOL_PARSER_ERROR_CHECKSUM;
    }

    uint8_t datalen = buffer[3];
    uint32_t sum = 0;
    for (uint16_t i = 0; i < datalen; i++) {
        sum += buffer[4 + i];
    }

    uint8_t expected = (uint8_t)(sum & 0xFFU);
    uint8_t received = buffer[4 + datalen]; /* crc_check 紧接 data 之后 */

    if (expected != received) {
        LOG_W("motor_ctrl", "解析器校验失败: 期望=0x%02X 实际=0x%02X, cmd=0x%02X datalen=%u",
            (unsigned int)expected, (unsigned int)received,
            (unsigned int)buffer[1], (unsigned int)datalen);
        return PROTOCOL_PARSER_ERROR_CHECKSUM;
    }

    return PROTOCOL_PARSER_OK;
}

/*
 * ============================================================================
 * 协议打包器回调
 * ============================================================================
 */

static protocol_packer_error_t motor_control_packer_checksum_cb(
    const uint8_t* data, uint16_t len, uint8_t* checksum_out,
    uint16_t* checksum_len)
{
    if (!data || !checksum_out || !checksum_len) {
        return PROTOCOL_PACKER_ERROR_NULL_PTR;
    }

    /*
     * 帧布局: data[0]=header, data[1]=cmd, data[2]=flags, data[3]=datalen,
     *         data[4..4+datalen-1] = 电机数据
     * 校验和 = sum(data[4..4+datalen-1]) & 0xFF
     */
    if (len < 4) {
        *checksum_out = 0;
        *checksum_len = 1;
        return PROTOCOL_PACKER_OK;
    }

    uint8_t datalen = data[3];
    uint32_t sum = 0;
    for (uint16_t i = 0; i < datalen; i++) {
        sum += data[4 + i];
    }

    *checksum_out = (uint8_t)(sum & 0xFFU);
    *checksum_len = 1;

    return PROTOCOL_PACKER_OK;
}

/*
 * ============================================================================
 * CAN-FD DLC 转换
 * ============================================================================
 */

/**
 * @brief 将 CAN-FD DLC 值转换为实际数据字节数
 *
 * CAN-FD 标准 DLC 编码:
 *   DLC 0-8  →  0-8 字节 (直通)
 *   DLC 9    → 12 字节
 *   DLC 10   → 16 字节
 *   DLC 11   → 20 字节
 *   DLC 12   → 24 字节
 *   DLC 13   → 32 字节
 *   DLC 14   → 48 字节
 *   DLC 15   → 64 字节
 *
 * @param dlc CAN-FD DLC 值 (0-15)
 * @return 实际数据字节数
 */
static uint8_t can_dlc_to_bytes(uint8_t dlc)
{
    if (dlc <= 8U) {
        return dlc;
    }

    switch (dlc) {
    case 9:
        return 12U;
    case 10:
        return 16U;
    case 11:
        return 20U;
    case 12:
        return 24U;
    case 13:
        return 32U;
    case 14:
        return 48U;
    case 15:
        return 64U;
    default:
        return 0U; /* 无效 DLC，返回 0 */
    }
}

/**
 * @brief 将实际数据字节数转换为 CAN-FD DLC 值（向上取整）
 *
 * CAN-FD 帧编码规则:
 *   0-8 字节   → DLC 0-8 (直通)
 *   9-12 字节  → DLC 9
 *   13-16 字节 → DLC 10
 *   17-20 字节 → DLC 11
 *   21-24 字节 → DLC 12
 *   25-32 字节 → DLC 13
 *   33-48 字节 → DLC 14
 *   49-64 字节 → DLC 15
 *
 * @param byte_len 实际数据字节数 (0-64)
 * @return 对应的 CAN-FD DLC 值
 */
static uint8_t can_bytes_to_dlc(uint8_t byte_len)
{
    if (byte_len <= 8U) {
        return byte_len;
    }
    if (byte_len <= 12U) {
        return 9U;
    }
    if (byte_len <= 16U) {
        return 10U;
    }
    if (byte_len <= 20U) {
        return 11U;
    }
    if (byte_len <= 24U) {
        return 12U;
    }
    if (byte_len <= 32U) {
        return 13U;
    }
    if (byte_len <= 48U) {
        return 14U;
    }

    return 15U; /* 49-64 字节 */
}

/*
 * ============================================================================
 * CAN-FD 总线状态转换
 * ============================================================================
 */

static const char* motor_control_task_bus_state_name(drv_can_state_t state)
{
    switch (state) {
    case DRV_CAN_STATE_ERROR_ACTIVE:
        return "正常";
    case DRV_CAN_STATE_ERROR_WARNING:
        return "警告";
    case DRV_CAN_STATE_ERROR_PASSIVE:
        return "错误被动";
    case DRV_CAN_STATE_BUS_OFF:
        return "总线关闭";
    default:
        return "未知";
    }
}

/*
 * ============================================================================
 * CAN-FD 发送辅助函数
 * ============================================================================
 */

static bool motor_control_task_send_frame(uint32_t can_id,
    const uint8_t* data, uint8_t byte_len)
{
    if (!s_can_ctx || !data || byte_len == 0
        || byte_len > MOTOR_CONTROL_FRAME_OVERHEAD + MOTOR_CONTROL_DATA_MAX) {
        return false;
    }

    mcan_tx_frame_t tx_frame;
    memset(&tx_frame, 0, sizeof(tx_frame));

    tx_frame.std_id = can_id;
    tx_frame.use_ext_id = 0;
    tx_frame.canfd_frame = 1;
    tx_frame.bitrate_switch = 1;
    tx_frame.dlc = can_bytes_to_dlc(byte_len);

    (void)memcpy(tx_frame.data_8, data, byte_len);

    /* 提取协议帧关键字段用于日志 */
    uint8_t tx_cmd = (byte_len >= 2U) ? data[1] : 0U;
    uint8_t tx_datalen = (byte_len >= 4U) ? data[3] : 0U;

    // LOG_I("motor_ctrl", "CAN TX → ID=0x%03X DLC=%u 字节=%u cmd=0x%02X datalen=%u",
    //     (unsigned int)can_id, (unsigned int)(tx_frame.dlc & 0x0FU),
    //     (unsigned int)byte_len, (unsigned int)tx_cmd, (unsigned int)tx_datalen);

    drv_can_error_t err = drv_can_tx_send_nonblocking(s_can_ctx, &tx_frame, NULL);

    if (err != DRV_CAN_OK) {
        LOG_E("motor_ctrl", "CAN TX 发送失败, 错误码=%d", (int)err);
        return false;
    }

    return true;
}

void motor_control_task_flush_response(motor_control_handle_t* h)
{
    if (!h || !h->initialized) {
        return;
    }

    canfd_protocol_t resp;
    if (!motor_control_pop_response(h, &resp)) {
        return;
    }

    /* 提取 payload: cmd + flags + datalen + data[0..datalen-1] */
    uint16_t payload_len = (uint16_t)(3U + resp.datalen);
    uint8_t* frame_out = NULL;
    uint16_t frame_out_len = 0;

    protocol_packer_error_t pack_err = protocol_packer_pack(
        &s_tx_packer, &resp.cmd, payload_len,
        &frame_out, &frame_out_len);

    if (pack_err == PROTOCOL_PACKER_OK && frame_out && frame_out_len > 0) {
        LOG_I("motor_ctrl", "应答帧发送 → 电机%u cmd=0x%02X(%s) datalen=%u 帧长=%u",
            (unsigned int)h->config.motor_id,
            (unsigned int)resp.cmd,
            motor_control_cmd_name(resp.cmd),
            (unsigned int)resp.datalen,
            (unsigned int)frame_out_len);

        if (motor_control_task_send_frame(MOTOR_CONTROL_CAN_ID_RESP,
                frame_out, (uint8_t)frame_out_len)) {
            s_tx_count++;
        }
    } else {
        LOG_E("motor_ctrl", "应答帧打包失败, 错误码=%d", (int)pack_err);
    }
}

void motor_control_task_send_heartbeat(uint16_t fault_bitmap)
{
    /*
     * 心跳 payload: cmd=0xF0, flags=0, datalen=4,
     * data[0..3] = bus_state + motor_count + fault_bitmap(LE)
     */
    uint8_t hb_payload[3 + 4]; /* cmd + flags + datalen + 4 bytes data */
    hb_payload[0] = MOTOR_CONTROL_CMD_HEARTBEAT;
    hb_payload[1] = 0;
    hb_payload[2] = 4; /* datalen */

    drv_can_state_t bus_state = drv_can_get_state(s_can_ctx);
    hb_payload[3] = (uint8_t)bus_state;
    hb_payload[4] = MOTOR_CONTROL_TASK_MAX_MOTORS;
    hb_payload[5] = (uint8_t)(fault_bitmap & 0xFFU);
    hb_payload[6] = (uint8_t)((fault_bitmap >> 8) & 0xFFU);

    uint8_t* frame_out = NULL;
    uint16_t frame_out_len = 0;

    protocol_packer_error_t err = protocol_packer_pack(&s_tx_packer,
        hb_payload, sizeof(hb_payload), &frame_out, &frame_out_len);

    if (err == PROTOCOL_PACKER_OK && frame_out && frame_out_len > 0) {
        // LOG_I("motor_ctrl", "心跳帧发送 → 总线=%s 电机数=%u 故障位图=0x%04X",
        //     motor_control_task_bus_state_name(bus_state),
        //     (unsigned int)MOTOR_CONTROL_TASK_MAX_MOTORS,
        //     (unsigned int)fault_bitmap);

        (void)motor_control_task_send_frame(MOTOR_CONTROL_CAN_ID_RESP,
            frame_out, (uint8_t)frame_out_len);
    } else {
        LOG_E("motor_ctrl", "心跳帧打包失败, 错误码=%d", (int)err);
    }
}
