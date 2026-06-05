/*
 * Copyright (c) 2024 G1_Hand 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    uart_task.h
 * @brief   RS-485 DMA 收发任务接口
 */

#ifndef UART_TASK_H
#define UART_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 RS-485 收发任务
 *
 * 初始化全部三路 RS-485 端口（UART8/14/15），
 * 注册接收回调，发送测试消息。
 * 应在 board_init() 之后调用一次。
 */
void uart_task_init(void);

/**
 * @brief RS-485 收发主循环轮询
 *
 * 在主循环中周期性调用，处理：
 * - 各端口接收数据读取
 * - 发送队列积压监控
 */
void uart_task_poll(void);

#ifdef __cplusplus
}
#endif

#endif /* UART_TASK_H */
