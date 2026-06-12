/*
 * Copyright (c) 2026 G1_Hand 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    log_task.h
 * @brief   日志输出任务
 *
 * 将 middlewares/log 模块的格式化日志数据通过 drv_log_uart
 * 经 UART0 DMA 发送到串口终端。
 */

#ifndef LOG_TASK_H
#define LOG_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化日志任务
 *
 * 依次初始化 middlewares/log 模块和 drv_log_uart 驱动，
 * 分配全部所需静态缓冲区。
 */
void log_task_init(void);

/**
 * @brief 日志任务轮询
 *
 * 检查 TX DMA 状态，若空闲则从 log 模块的 kfifo 中取出
 * 格式化后的日志数据，通过 UART0 DMA 发送到串口终端。
 * 需在主循环中周期性调用。
 */
void log_task_poll(void);

#ifdef __cplusplus
}
#endif

#endif /* LOG_TASK_H */
