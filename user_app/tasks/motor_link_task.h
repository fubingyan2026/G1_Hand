/*
 * Copyright (c) 2026 G1_Hand 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    motor_link_task.h
 * @author  maximilian
 * @version V1.0.0
 * @date    2026-06-11
 * @brief   电机数据链路任务 — 显式管理 CAN-FD ↔ RS-485 电机 ID/端口映射
 *
 * 集中定义 CAN-FD motor_id ↔ RS-485 finger_motor_id ↔ RS-485 port 的对应关系，
 * 统一管理 finger handle 和 motor_control handle 的实例数组与生命周期，
 * 向 main() 提供单一 init/poll 入口。
 */

#ifndef __MOTOR_LINK_TASK_H
#define __MOTOR_LINK_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdint.h>

#include "drv_rs485.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 单条数据链路映射条目
 *
 * 描述一个 CAN-FD 协议电机到 RS-485 物理电机的完整对应关系。
 */
typedef struct {
    uint8_t motor_id;           /**< CAN-FD 协议中的电机 ID（1-9） */
    uint8_t finger_motor_id;    /**< 对应 RS-485 总线上的电机地址 */
    rs485_port_t finger_port;   /**< 所属 RS-485 端口 */
} motor_link_entry_t;

/* Exported constants --------------------------------------------------------*/

/** @brief 系统中电机总数 */
#define MOTOR_LINK_MAX_MOTORS  9U

/** @brief PORT1（RS485_PORT_MOTOR1）上挂载的电机数量 */
#define MOTOR_LINK_PORT1_COUNT  5U

/** @brief PORT2（RS485_PORT_MOTOR2）上挂载的电机数量 */
#define MOTOR_LINK_PORT2_COUNT  4U

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 初始化电机数据链路任务
 *
 * 按正确顺序初始化整个电机通讯链路：
 *   1. finger 服务层 (finger_init)
 *   2. finger_task 传输层（RS-485 硬件 + 端口 parser/FSM）
 *   3. 遍历映射表注册全部 finger 电机实例
 *   4. motor_control_task 传输层（CAN-FD 硬件 + parser/packer）
 *   5. motor_control 服务层 (motor_control_init)
 *   6. 遍历映射表注册全部 motor_control 实例并连线 finger 回调
 *
 * @note 调用一次即可，替代原来分散的 finger_init + finger_task_init +
 *       motor_control_task_init 等多个调用
 */
void motor_link_task_init(void);

/**
 * @brief 电机数据链路任务轮询
 *
 * 在主循环中周期性调用，处理：
 *   - finger_task 传输层：RS-485 接收/解析/分发 + TX FSM 驱动
 *   - motor_control_task 传输层：CAN-FD 接收/解析/分发 + 总线监控
 *   - motor_control 实例应答帧扫描、打包和 CAN-FD 发送
 *   - CAN-FD 心跳帧发送
 */
void motor_link_task_poll(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_LINK_TASK_H */
