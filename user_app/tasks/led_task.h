/*
 * Copyright (c) 2026 G1_Hand 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    led_task.h
 * @brief   LED 状态指示任务
 *
 * 基于 middlewares/service/led.c 库管理 PF02 状态指示灯。
 * 支持循环编码闪烁，闪烁间隔 250ms。
 */

#ifndef LED_TASK_H
#define LED_TASK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 LED 任务
 *
 * 注册 PF02 状态指示灯实例，配置闪烁参数（250ms / 3 次），
 * 启动循环闪烁模式。
 */
void led_task_init(void);

/**
 * @brief LED 任务轮询
 *
 * 驱动 LED 库 FSM 状态机，需在主循环中以固定周期调用。
 */
void led_task_poll(void);

/**
 * @brief 启动 LED 编码闪烁
 *
 * @param count    闪烁次数（0 = 无限循环）
 * @param cycle_ms 闪烁半周期 (ms)，即每次电平翻转的间隔
 */
void led_task_start_blink(uint16_t count, uint16_t cycle_ms);

#ifdef __cplusplus
}
#endif

#endif /* LED_TASK_H */
