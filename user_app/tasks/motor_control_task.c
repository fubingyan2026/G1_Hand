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

#include "drv_can.h"
#include "hpm_clock_drv.h"
#include "hpm_mchtmr_drv.h"
#include "hpm_soc.h"
#include "motor_control.h"
#include "protocol_packer.h"
#include "protocol_parser.h"

#include <stdio.h>
#include <string.h>

/* Private constants ---------------------------------------------------------*/

/** @brief 心跳间隔 (ms) */
#define HEARTBEAT_INTERVAL_MS  100U

/** @brief 解析器输入 kfifo 大小 */
#define PARSER_INPUT_BUF_SIZE  256U

/** @brief 解析器/打包器输出缓冲区大小 */
#define PROTO_OUTPUT_BUF_SIZE  64U

/** @brief 电机名称字符串长度 */
#define MOTOR_NAME_MAX_LEN     12U

/** @brief 协议帧头 */
static const uint8_t s_proto_header[] = { MOTOR_CONTROL_HEADER_VAL };

/** @brief 协议帧尾 */
static const uint8_t s_proto_footer[] = { MOTOR_CONTROL_ENDER_VAL };

/* --- DLC ↔ 字节数 转换表 --- */

static const uint8_t s_dlc_to_bytes[16] = {
    0, 1, 2, 3, 4, 5, 6, 7,
    8, 12, 16, 20, 24, 32, 48, 64,
};

static const uint8_t s_bytes_to_dlc[65] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 9, 9, 9,
    10, 10, 10, 10,
    11, 11, 11, 11,
    12, 12, 12, 12,
    13, 13, 13, 13, 13, 13, 13, 13,
    14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14,
    15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15,
};

/* Private variables ---------------------------------------------------------*/

static drv_can_context_t* s_can_ctx;
static motor_control_handle_t s_motor_instances[MOTOR_CONTROL_TASK_MAX_MOTORS];
static char s_motor_names[MOTOR_CONTROL_TASK_MAX_MOTORS][MOTOR_NAME_MAX_LEN];

/* --- 协议解析器（RX）--- */
static protocol_parser_context_t s_rx_parser;
static uint8_t s_parser_input_buf[PARSER_INPUT_BUF_SIZE];
static uint8_t s_parser_output_buf[PROTO_OUTPUT_BUF_SIZE];

/* --- 协议打包器（TX）--- */
static protocol_packer_context_t s_tx_packer;
static uint8_t s_packer_output_buf[PROTO_OUTPUT_BUF_SIZE];

static volatile uint32_t s_rx_count;
static uint32_t s_tx_count;
static uint32_t s_ticks_per_ms;
static uint32_t s_last_heartbeat_ticks;
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

static bool motor_control_task_send_frame(uint32_t can_id,
    const uint8_t* data, uint8_t byte_len);
static void motor_control_task_send_heartbeat(void);
static uint16_t motor_control_task_get_fault_bitmap(void);

/*
 * ============================================================================
 * 公共 API — 任务初始化
 * ============================================================================
 */

void motor_control_task_init(void)
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

    printf("\n[MOTOR_CTRL_TASK] Initializing CAN4...\n");

    s_can_ctx = drv_can_init(&can4_config);
    if (!s_can_ctx) {
        printf("[MOTOR_CTRL_TASK] ERROR: drv_can_init failed!\n");
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
        printf("[MOTOR_CTRL_TASK] ERROR: parser init failed: %d\n",
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
        printf("[MOTOR_CTRL_TASK] ERROR: packer init failed: %d\n",
            (int)packer_err);
        return;
    }

    /* 4. 初始化 motor_control 服务 */
    motor_control_error_t mc_err = motor_control_init();
    if (MOTOR_CONTROL_IS_ERR(mc_err)) {
        printf("[MOTOR_CTRL_TASK] ERROR: motor_control init failed: %d\n",
            (int)mc_err);
        return;
    }

    /* 5. 注册 9 个电机 */
    uint8_t idx = 0;

    for (uint8_t id = 1; id <= 5; id++) {
        snprintf(s_motor_names[idx], sizeof(s_motor_names[0]),
            "motor_%u", (unsigned int)id);
        motor_control_config_t cfg = {
            .name = s_motor_names[idx],
            .motor_id = id,
            .finger_motor_id = id,
            .finger_port = RS485_PORT_MOTOR1,
        };
        memset(&s_motor_instances[idx], 0, sizeof(motor_control_handle_t));
        mc_err = motor_control_register_static(&cfg, &s_motor_instances[idx]);
        if (MOTOR_CONTROL_IS_OK(mc_err)) {
            finger_set_response_callback(s_motor_instances[idx].finger,
                motor_control_on_finger_response, &s_motor_instances[idx]);
        } else {
            printf("[MOTOR_CTRL_TASK] Register motor_%u (PORT1) failed: %d\n",
                (unsigned int)id, (int)mc_err);
        }
        idx++;
    }

    for (uint8_t id = 6; id <= 9; id++) {
        snprintf(s_motor_names[idx], sizeof(s_motor_names[0]),
            "motor_%u", (unsigned int)id);
        motor_control_config_t cfg = {
            .name = s_motor_names[idx],
            .motor_id = id,
            .finger_motor_id = id,
            .finger_port = RS485_PORT_MOTOR2,
        };
        memset(&s_motor_instances[idx], 0, sizeof(motor_control_handle_t));
        mc_err = motor_control_register_static(&cfg, &s_motor_instances[idx]);
        if (MOTOR_CONTROL_IS_OK(mc_err)) {
            finger_set_response_callback(s_motor_instances[idx].finger,
                motor_control_on_finger_response, &s_motor_instances[idx]);
        } else {
            printf("[MOTOR_CTRL_TASK] Register motor_%u (PORT2) failed: %d\n",
                (unsigned int)id, (int)mc_err);
        }
        idx++;
    }

    /* 6. 计算 MCHTMR ticks/ms */
    uint32_t mchtmr_freq = clock_get_frequency(clock_mchtmr0);
    s_ticks_per_ms = mchtmr_freq / 1000U;
    if (s_ticks_per_ms == 0) {
        s_ticks_per_ms = 1;
    }

    s_last_heartbeat_ticks = motor_control_task_get_ticks();
    s_rx_count = 0;
    s_tx_count = 0;
    s_safe_mode = false;
    s_ep_warned = false;

    printf("[MOTOR_CTRL_TASK] Initialized: CAN4 1M/5M, %u motors, "
        "heartbeat %ums (parser + packer)\n",
        MOTOR_CONTROL_TASK_MAX_MOTORS, HEARTBEAT_INTERVAL_MS);
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
        printf("[MOTOR_CTRL_TASK] BUS-OFF, recovering...\n");
        drv_can_recover(s_can_ctx);
        s_last_heartbeat_ticks = now;
        s_ep_warned = false;
        return;
    }
    if (bus_state == DRV_CAN_STATE_ERROR_PASSIVE) {
        if (!s_ep_warned) {
            printf("[MOTOR_CTRL_TASK] WARNING: error-passive\n");
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
            continue;
        }

        if (rx_msg.std_id != MOTOR_CONTROL_CAN_ID_CMD || rx_msg.use_ext_id) {
            continue;
        }

        uint8_t data_bytes = s_dlc_to_bytes[rx_msg.dlc & 0x0FU];

        /* 喂入协议解析器 */
        protocol_parser_error_t feed_err = protocol_parser_feed(
            &s_rx_parser, rx_msg.data_8, data_bytes);
        if (feed_err != PROTOCOL_PARSER_OK) {
            (void)protocol_parser_clear(&s_rx_parser);
            continue;
        }
    }

    /* 3. 从协议解析器提取完整帧（已校验 header + footer + checksum） */
    {
        uint16_t frame_len = 0;
        uint8_t* frame_data = NULL;

        while (protocol_parser_parse(&s_rx_parser, &frame_len, &frame_data)
            == PROTOCOL_PARSER_OK) {
            /* 解析器返回的完整帧可直接转换为 canfd_protocol_t */
            const canfd_protocol_t* proto = (const canfd_protocol_t*)frame_data;

            if (!s_safe_mode) {
                (void)motor_control_process_rx_frame(proto);
            }
        }
    }

    /* 4. 更新解析器空闲计时器 */
    (void)protocol_parser_tick(&s_rx_parser);

    /* 5. 扫描所有电机，用打包器构建并发送应答帧 */
    for (uint8_t i = 0; i < MOTOR_CONTROL_TASK_MAX_MOTORS; i++) {
        motor_control_handle_t* h = &s_motor_instances[i];

        if (!h->initialized) {
            continue;
        }

        canfd_protocol_t resp;
        if (motor_control_pop_response(h, &resp)) {
            /* 提取 payload: cmd + flags + datalen + data[0..datalen-1] */
            uint16_t payload_len = (uint16_t)(3U + resp.datalen);
            uint8_t* frame_out = NULL;
            uint16_t frame_out_len = 0;

            protocol_packer_error_t pack_err = protocol_packer_pack(
                &s_tx_packer, &resp.cmd, payload_len,
                &frame_out, &frame_out_len);

            if (pack_err == PROTOCOL_PACKER_OK && frame_out && frame_out_len > 0) {
                if (motor_control_task_send_frame(MOTOR_CONTROL_CAN_ID_RESP,
                        frame_out, (uint8_t)frame_out_len)) {
                    s_tx_count++;
                }
            }
        }
    }

    /* 6. 周期性心跳 */
    {
        uint32_t elapsed = now - s_last_heartbeat_ticks;
        if (elapsed >= HEARTBEAT_INTERVAL_MS * s_ticks_per_ms) {
            s_last_heartbeat_ticks = now;
            motor_control_task_send_heartbeat();
        }
    }
}

/*
 * ============================================================================
 * 私有函数 — 定时器
 * ============================================================================
 */

static uint32_t motor_control_task_get_ticks(void)
{
    return (uint32_t)mchtmr_get_count(HPM_MCHTMR);
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

    if (datalen > MOTOR_CONTROL_DATA_MAX || (datalen & 1U)) {
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
     * XOR 校验: buffer[0] ^ ... ^ buffer[len-3] == buffer[len-2]
     * (crc 位于倒数第二字节, footer 位于最后一字节)
     */
    if (!buffer || len < 3) {
        return PROTOCOL_PARSER_ERROR_CHECKSUM;
    }

    uint8_t xor_val = 0;
    for (uint16_t i = 0; i < len - 2; i++) {
        xor_val ^= buffer[i];
    }

    if (xor_val != buffer[len - 2]) {
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

    uint8_t xor_val = 0;
    for (uint16_t i = 0; i < len; i++) {
        xor_val ^= data[i];
    }

    *checksum_out = xor_val;
    *checksum_len = 1;

    return PROTOCOL_PACKER_OK;
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
    tx_frame.dlc = s_bytes_to_dlc[byte_len];

    (void)memcpy(tx_frame.data_8, data, byte_len);

    drv_can_error_t err = drv_can_tx_send_nonblocking(s_can_ctx, &tx_frame, NULL);

    return (err == DRV_CAN_OK);
}

static void motor_control_task_send_heartbeat(void)
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
    uint16_t fault_bitmap = motor_control_task_get_fault_bitmap();
    hb_payload[3] = (uint8_t)bus_state;
    hb_payload[4] = MOTOR_CONTROL_TASK_MAX_MOTORS;
    hb_payload[5] = (uint8_t)(fault_bitmap & 0xFFU);
    hb_payload[6] = (uint8_t)((fault_bitmap >> 8) & 0xFFU);

    uint8_t* frame_out = NULL;
    uint16_t frame_out_len = 0;

    protocol_packer_error_t err = protocol_packer_pack(&s_tx_packer,
        hb_payload, sizeof(hb_payload), &frame_out, &frame_out_len);

    if (err == PROTOCOL_PACKER_OK && frame_out && frame_out_len > 0) {
        (void)motor_control_task_send_frame(MOTOR_CONTROL_CAN_ID_RESP,
            frame_out, (uint8_t)frame_out_len);
    }
}

static uint16_t motor_control_task_get_fault_bitmap(void)
{
    uint16_t bitmap = 0;
    for (uint8_t i = 0; i < MOTOR_CONTROL_TASK_MAX_MOTORS; i++) {
        if (s_motor_instances[i].initialized
            && s_motor_instances[i].fault_flags != 0) {
            bitmap |= (uint16_t)(1U << i);
        }
    }
    return bitmap;
}
