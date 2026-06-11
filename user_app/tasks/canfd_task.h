/*
 * Copyright (c) 2026 G1_Hand 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    canfd_task.h
 * @brief   CAN-FD 收发测试任务 — 周期性发送 CAN-FD 帧并轮询接收
 */

#ifndef __CANFD_TASK_H
#define __CANFD_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 初始化 CAN-FD 测试任务
 *
 * 配置 CAN4 (PA16/PA17/PA18)，初始化 drv_can 实例，
 * 注册 ISR 接收回调，输出初始化状态。
 */
void canfd_task_init(void);

/**
 * @brief CAN-FD 测试任务轮询
 *
 * - 每 ~2 秒发送一条 CAN-FD 测试帧
 * - 检查接收 kfifo 并打印收到的消息
 */
void canfd_task_poll(void);

#ifdef __cplusplus
}
#endif

#endif /* __CANFD_TASK_H */
