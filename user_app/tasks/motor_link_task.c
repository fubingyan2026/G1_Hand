/*
 * Copyright (c) 2026 G1_Hand 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    motor_link_task.c
 * @author  maximilian
 * @version V1.0.0
 * @date    2026-06-11
 * @brief   电机数据链路任务实现
 *
 * 集中管理 CAN-FD motor_id ↔ RS-485 finger_motor_id ↔ RS-485 port 的映射关系，
 * 统一持有 finger handle 和 motor_control handle 的实例数组，
 * 按正确顺序初始化完整数据链路并提供单一 poll 入口。
 */

/* Includes ------------------------------------------------------------------*/
#include "motor_link_task.h"

#include "bsp_systick.h"
#include "finger.h"
#include "finger_task.h"
#include "motor_control.h"
#include "motor_control_task.h"

#include "log.h"

#include <string.h>

/* Private constants ---------------------------------------------------------*/

/** @brief 手指电机名称最大长度 */
#define FINGER_NAME_MAX_LEN 12U

/** @brief motor_control 实例名称最大长度 */
#define MOTOR_CTRL_NAME_MAX_LEN 12U

/** @brief 心跳间隔 (ms) */
#define MOTOR_LINK_HEARTBEAT_INTERVAL_MS 100U

/* Private variables ---------------------------------------------------------*/

/**
 * @brief 静态数据链路映射表
 *
 * 定义每个 CAN-FD 协议电机 ID 对应的 RS-485 地址和物理端口。
 * 这是整个系统唯一的 ID/端口 映射来源，修改映射只需改动此表。
 *
 *   CAN-FD ID | RS-485 ID | RS-485 端口
 *   ----------|-----------|-------------
 *      1      |     1     | MOTOR1
 *      2      |     2     | MOTOR1
 *      3      |     3     | MOTOR1
 *      4      |     4     | MOTOR1
 *      5      |     5     | MOTOR1
 *      6      |     6     | MOTOR2
 *      7      |     7     | MOTOR2
 *      8      |     8     | MOTOR2
 *      9      |     9     | MOTOR2
 */
static const motor_link_entry_t s_motor_link_map[MOTOR_LINK_MAX_MOTORS] = {
    { 1, 1, RS485_PORT_MOTOR1 },
    { 2, 2, RS485_PORT_MOTOR1 },
    { 3, 3, RS485_PORT_MOTOR1 },
    { 4, 4, RS485_PORT_MOTOR1 },
    { 5, 5, RS485_PORT_MOTOR1 },
    { 6, 6, RS485_PORT_MOTOR2 },
    { 7, 7, RS485_PORT_MOTOR2 },
    { 8, 8, RS485_PORT_MOTOR2 },
    { 9, 9, RS485_PORT_MOTOR2 },
};

/** @brief finger 电机实例数组（静态分配） */
static finger_handle_t s_finger_instances[MOTOR_LINK_MAX_MOTORS];

/** @brief finger 电机 TX 帧缓冲区（每个 32 字节） */
static uint8_t s_finger_tx_buffers[MOTOR_LINK_MAX_MOTORS][FINGER_TX_BUFFER_SIZE];

/** @brief finger 电机名称字符串（静态存储，避免悬空指针） */
static char s_finger_names[MOTOR_LINK_MAX_MOTORS][FINGER_NAME_MAX_LEN];

/** @brief motor_control 实例数组（静态分配） */
static motor_control_handle_t s_motor_ctrl_instances[MOTOR_LINK_MAX_MOTORS];

/** @brief motor_control 实例名称字符串 */
static char s_motor_ctrl_names[MOTOR_LINK_MAX_MOTORS][MOTOR_CTRL_NAME_MAX_LEN];

/** @brief 上次心跳发送时刻（毫秒） */
static uint32_t s_last_heartbeat_ms;

/** @brief 任务初始化标志 */
static bool s_motor_link_initialized;

/* Private function prototypes -----------------------------------------------*/

static uint16_t motor_link_get_fault_bitmap(void);
static bool motor_link_is_heartbeat_due(uint32_t now_ms);

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 遍历所有 motor_control 实例，计算故障位图
 * @return 16 位故障位图（bit i 对应 motor_id=i+1）
 */
static uint16_t motor_link_get_fault_bitmap(void)
{
    uint16_t bitmap = 0;
    for (uint8_t i = 0; i < MOTOR_LINK_MAX_MOTORS; i++) {
        if (s_motor_ctrl_instances[i].initialized
            && s_motor_ctrl_instances[i].fault_flags != 0) {
            bitmap |= (uint16_t)(1U << i);
        }
    }
    return bitmap;
}

/**
 * @brief 检查是否到达心跳发送间隔
 * @param now_ms 当前毫秒时间戳
 * @return true 应发送心跳，false 未到间隔
 */
static bool motor_link_is_heartbeat_due(uint32_t now_ms)
{
    return ((now_ms - s_last_heartbeat_ms) >= MOTOR_LINK_HEARTBEAT_INTERVAL_MS);
}

/* Exported functions --------------------------------------------------------*/

void motor_link_task_init(void)
{
    if (s_motor_link_initialized) {
        return;
    }

    LOG_I("motor_link", "初始化电机数据链路, 共 %u 个电机 (端口1: %u, 端口2: %u)",
        MOTOR_LINK_MAX_MOTORS, MOTOR_LINK_PORT1_COUNT, MOTOR_LINK_PORT2_COUNT);

    /* --- 阶段 1: 初始化 finger 服务层 --- */
    finger_error_t f_err = finger_init();
    if (FINGER_IS_ERR(f_err)) {
        LOG_E("motor_link", "finger_init() 失败, 错误码=%d", (int)f_err);
        return;
    }

    /* --- 阶段 2: 初始化 finger_task 传输层（RS-485 硬件 + parser + FSM） --- */
    finger_task_init_transport();

    /* --- 阶段 3: 遍历映射表，注册全部 finger 电机实例 --- */
    for (uint8_t i = 0; i < MOTOR_LINK_MAX_MOTORS; i++) {
        const motor_link_entry_t* entry = &s_motor_link_map[i];

        snprintf(s_finger_names[i], sizeof(s_finger_names[0]),
            "finger_%u", (unsigned int)entry->finger_motor_id);

        finger_config_t cfg = {
            .name = s_finger_names[i],
            .motor_id = entry->finger_motor_id,
            .rs485_port = entry->finger_port,
            .tx_buffer = s_finger_tx_buffers[i],
            .tx_buffer_len = FINGER_TX_BUFFER_SIZE,
            .reduction_ratio = 1080,
            .pole_pairs = 2,
        };

        f_err = finger_register_static(&cfg, &s_finger_instances[i]);
        if (FINGER_IS_ERR(f_err)) {
            LOG_E("motor_link", "注册手指 %s (CAN-ID:%u, 端口%u) 失败, 错误码=%d",
                cfg.name, entry->motor_id,
                (entry->finger_port == RS485_PORT_MOTOR1) ? 1U : 2U,
                (int)f_err);
        }
    }

    /* --- 阶段 4: 初始化 motor_control_task 传输层（CAN-FD 硬件 + parser/packer） --- */
    motor_control_task_init_transport();

    /* --- 阶段 5: 初始化 motor_control 服务层 --- */
    motor_control_error_t mc_err = motor_control_init();
    if (MOTOR_CONTROL_IS_ERR(mc_err)) {
        LOG_E("motor_link", "motor_control_init() 失败, 错误码=%d", (int)mc_err);
        return;
    }

    /* --- 阶段 6: 遍历映射表，注册全部 motor_control 实例并连线 finger 回调 --- */
    for (uint8_t i = 0; i < MOTOR_LINK_MAX_MOTORS; i++) {
        const motor_link_entry_t* entry = &s_motor_link_map[i];

        snprintf(s_motor_ctrl_names[i], sizeof(s_motor_ctrl_names[0]),
            "motor_%u", (unsigned int)entry->motor_id);

        motor_control_config_t cfg = {
            .name = s_motor_ctrl_names[i],
            .motor_id = entry->motor_id,
            .finger_motor_id = entry->finger_motor_id,
            .finger_port = entry->finger_port,
        };

        memset(&s_motor_ctrl_instances[i], 0, sizeof(motor_control_handle_t));
        mc_err = motor_control_register_static(&cfg, &s_motor_ctrl_instances[i]);

        if (MOTOR_CONTROL_IS_OK(mc_err)) {
            /* 关键连线：finger 应答 → motor_control_on_finger_response → 更新缓存/构建 CAN-FD 应答 */
            finger_set_response_callback(s_motor_ctrl_instances[i].finger,
                motor_control_on_finger_response, &s_motor_ctrl_instances[i]);

            LOG_I("motor_link", "已连接: CAN-ID:%u → RS485-ID:%u (端口%u, %s)",
                entry->motor_id, entry->finger_motor_id,
                (entry->finger_port == RS485_PORT_MOTOR1) ? 1U : 2U,
                s_motor_ctrl_names[i]);
        } else {
            LOG_E("motor_link", "注册电机 %s (CAN-ID:%u) 失败, 错误码=%d",
                s_motor_ctrl_names[i], entry->motor_id, (int)mc_err);
        }
    }

    /* 心跳计时器初始化 */
    s_last_heartbeat_ms = millis();

    s_motor_link_initialized = true;

    LOG_I("motor_link", "初始化完成: %u 个电机已连接",
        MOTOR_LINK_MAX_MOTORS);
}

void motor_link_task_poll(void)
{
    if (!s_motor_link_initialized) {
        return;
    }

    /* 1. finger_task 传输层轮询（RS-485 RX/TX） */
    finger_task_poll();

    /* 2. motor_control_task 传输层轮询（CAN-FD RX + 解析 + 分发） */
    motor_control_task_poll();

    /* 3. 扫描所有 motor_control 实例，弹出待发送 CAN-FD 应答帧并发送 */
    for (uint8_t i = 0; i < MOTOR_LINK_MAX_MOTORS; i++) {
        motor_control_task_flush_response(&s_motor_ctrl_instances[i]);
    }

    /* 4. 周期性心跳（每 100ms） */
    uint32_t now_ms = millis();
    if (motor_link_is_heartbeat_due(now_ms)) {
        s_last_heartbeat_ms = now_ms;
        motor_control_task_send_heartbeat(motor_link_get_fault_bitmap());
    }
}
