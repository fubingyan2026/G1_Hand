/*
 * Copyright (c) 2026 G1_Hand 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    finger_task.h
 * @brief   手指电机通讯任务
 *
 * 管理多路 RS-485 总线上的手指电机通讯调度：
 * - 每端口内嵌 protocol_parser 解析应答帧
 * - 半双工总线仲裁（IDLE → SEND → WAIT_RESPONSE）
 * - 轮询调度所有已注册手指电机的待发送命令
 */

#ifndef FINGER_TASK_H
#define FINGER_TASK_H

#include <stdbool.h>
#include <stdint.h>

#include "finger.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported constants --------------------------------------------------------*/

/** @brief MOTOR1 端口支持的最大手指电机数量 */
#define FINGER_TASK_MAX_MOTORS_PORT1  5U

/** @brief MOTOR2 端口支持的最大手指电机数量 */
#define FINGER_TASK_MAX_MOTORS_PORT2  4U

/** @brief 两个端口合计最大手指电机数量 */
#define FINGER_TASK_MAX_MOTORS \
    (FINGER_TASK_MAX_MOTORS_PORT1 + FINGER_TASK_MAX_MOTORS_PORT2)

/** @brief 应答超时时间（毫秒） */
#define FINGER_TASK_RESPONSE_TIMEOUT_MS  50U

/** @brief 命令间隔时间（毫秒），协议要求 ≥1ms */
#define FINGER_TASK_INTER_CMD_MS  5U

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 初始化手指电机通讯任务
 *
 * 初始化全部 RS-485 端口（UART1/14/8），
 * 为每个端口创建 protocol_parser 实例用于解析应答帧。
 * 应在 finger_init() 和 rs485_init_all() 之后调用一次。
 */
void finger_task_init(void);

/**
 * @brief 手指电机通讯任务轮询
 *
 * 在主循环中周期性调用，处理：
 * - 各端口 RS-485 接收数据读取并喂入 protocol_parser
 * - 完整应答帧解析并分发到对应手指电机实例
 * - 待发送命令调度与半双工总线仲裁
 * - 应答超时检测与重试
 */
void finger_task_poll(void);

/**
 * @brief 获取指定端口的通讯统计信息
 * @param port RS-485 端口号
 * @param p_tx_count 输出：累计发送帧数
 * @param p_rx_count 输出：累计接收帧数
 * @param p_timeout_count 输出：累计超时次数
 * @return true 端口活跃，false 端口未启用
 */
bool finger_task_get_stats(uint8_t port, uint32_t* p_tx_count,
    uint32_t* p_rx_count, uint32_t* p_timeout_count);

#ifdef __cplusplus
}
#endif

#endif /* FINGER_TASK_H */
