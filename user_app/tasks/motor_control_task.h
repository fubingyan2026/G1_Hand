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

/* 前向声明 */
typedef struct motor_control_handle motor_control_handle_t;

#ifdef __cplusplus
extern "C" {
#endif

/* Exported constants --------------------------------------------------------*/

/** @brief 支持的最大电机数量 */
#define MOTOR_CONTROL_TASK_MAX_MOTORS  9U

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 初始化 CAN-FD 电机控制传输层
 *
 * 仅初始化 CAN4 CAN-FD 硬件 + protocol_parser + protocol_packer。
 * 不注册 motor_control 电机实例 — 实例注册由 motor_link_task 统一管理。
 *
 * 初始化顺序：
 *   1. CAN4 CAN-FD 驱动 (PA16/PA17/PA18, TCAN1044, 1M/5M baud)
 *   2. protocol_parser 实例（header='s', footer='e', XOR checksum）
 *   3. protocol_packer 实例
 *
 * @note 调用前需先完成 motor_control_init() 初始化服务层
 */
void motor_control_task_init_transport(void);

/**
 * @brief 初始化 CAN-FD 电机控制桥接任务（已废弃）
 *
 * @deprecated 请使用 motor_control_task_init_transport() + motor_link_task_init()
 *             替代。此函数仅保留用于向后兼容。
 */
void motor_control_task_init(void);

/**
 * @brief CAN-FD 电机控制传输层轮询
 *
 * 每次主循环调用，处理：
 *   - CAN-FD 总线状态检查和自动恢复
 *   - CAN-FD 接收帧排空 → protocol_parser feed → 协议帧解析 → 指令分发
 *   - 解析器空闲计时器更新
 *
 * @note 应答帧弹出/打包/发送和心跳发送由 motor_link_task 统一调度
 */
void motor_control_task_poll(void);

/**
 * @brief 弹出待发送应答帧、打包并发送到 CAN-FD 总线
 *
 * 对指定 motor_control 实例调用 motor_control_pop_response()，
 * 若有待发送应答帧，通过 protocol_packer 打包后发送到 CAN-FD 总线。
 *
 * @param h motor_control 实例句柄
 */
void motor_control_task_flush_response(motor_control_handle_t* h);

/**
 * @brief 发送 CAN-FD 心跳帧
 *
 * @param fault_bitmap 故障位图（bit i 对应 motor_id=i+1 的故障状态）
 */
void motor_control_task_send_heartbeat(uint16_t fault_bitmap);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CONTROL_TASK_H */
