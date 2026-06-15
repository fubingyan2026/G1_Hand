/*
 * Copyright (c) 2024 G1_Hand 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    rs485_sample_task.h
 * @brief   RS-485 DMA 收发示例任务接口
 */

#ifndef RS485_SAMPLE_TASK_H
#define RS485_SAMPLE_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 RS-485 示例任务
 *
 * 初始化指定 RS-485 端口，注册接收回调。
 * 应在 board_init() 之后调用一次。
 */
void rs485_sample_task_init(void);

/**
 * @brief RS-485 示例任务主循环轮询
 *
 * 在主循环中周期性调用，处理：
 * - 端口接收数据读取
 * - 周期性测试消息发送
 */
void rs485_sample_task_poll(void);

#ifdef __cplusplus
}
#endif

#endif /* RS485_SAMPLE_TASK_H */
