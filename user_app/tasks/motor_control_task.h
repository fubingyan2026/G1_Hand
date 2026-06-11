/*
 * Copyright (c) 2026 G1_Hand 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    motor_control_task.h
 * @author  maximilian
 * @version V1.0.0
 * @date    2026-06-11
 * @brief   CAN-FD 电机控制桥接任务
 *
 * 初始化 CAN4 CAN-FD 驱动 + motor_control 服务层，
 * 在主循环中轮询 CAN-FD 接收、协议解析、电机指令分发和应答发送。
 */

#ifndef __MOTOR_CONTROL_TASK_H
#define __MOTOR_CONTROL_TASK_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Exported constants --------------------------------------------------------*/

/** @brief 支持的最大电机数量 */
#define MOTOR_CONTROL_TASK_MAX_MOTORS  9U

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 初始化 CAN-FD 电机控制桥接任务
 *
 * 初始化顺序：
 *   1. CAN4 CAN-FD 驱动 (PA16/PA17/PA18, TCAN1044, 1M/5M baud)
 *   2. protocol_parser 实例（header=0x5A 0xA5, checksum=XOR）
 *   3. motor_control 服务层 + 9 个电机实例注册
 *   4. finger 应答回调注册
 *
 * @note 必须在 finger_task_init() 之后调用（依赖 finger 实例已注册）
 */
void motor_control_task_init(void);

/**
 * @brief CAN-FD 电机控制任务轮询
 *
 * 每次主循环调用，处理：
 *   - CAN-FD 总线状态检查和自动恢复
 *   - CAN-FD 接收帧排空 → protocol_parser feed → 协议帧解析 → 指令分发
 *   - 所有电机待发送 CAN-FD 应答帧扫描和发送
 *   - 周期性心跳帧发送 (每 100ms)
 */
void motor_control_task_poll(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CONTROL_TASK_H */
