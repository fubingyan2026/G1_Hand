/*
 * Copyright (c) 2026 G1_Hand 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    drv_led.h
 * @brief   LED 设备驱动
 *
 * 管理 PF02 状态指示 LED 的 GPIO 控制。
 * 引脚配置为推挽输出，低电平点亮。
 */

#ifndef DRV_LED_H
#define DRV_LED_H

#include "hpm_gpio_drv.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------------------------
 * 硬件配置宏
 * -------------------------------------------------------------------------- */

#define DRV_LED_GPIO_PORT HPM_GPIO0 /**< GPIO 控制器基地址 */
#define DRV_LED_GPIO_PORT_IDX GPIO_DO_GPIOF /**< 端口索引 (GPIOF = 5) */
#define DRV_LED_GPIO_PIN_IDX 2 /**< 引脚索引 (PF02) */
#define DRV_LED_IOC_PAD IOC_PAD_PF02 /**< IOC PAD 索引 */
#define DRV_LED_IOC_FUNC IOC_PF02_FUNC_CTL_GPIO_F_02 /**< IOC 功能选择 (GPIO) */

#define DRV_LED_ON_LEVEL 1 /**< 点亮电平（低电平有效） */
#define DRV_LED_OFF_LEVEL 0 /**< 熄灭电平 */

#define DRV_LED_INITIAL_LEVEL DRV_LED_OFF_LEVEL /**< 上电初始状态：熄灭 */

/**
 * @brief 初始化 LED 硬件
 *
 * 配置 PF02 引脚为 GPIO 推挽输出：
 * - 设置 IOC 引脚复用为 GPIO 功能
 * - 使能 GPIO 时钟
 * - 设置初始输出电平为 OFF
 *
 * 应在 board_init() 之后调用一次。
 */
void drv_led_init(void);

/**
 * @brief 点亮 LED
 */
void drv_led_on(void);

/**
 * @brief 熄灭 LED
 */
void drv_led_off(void);

/**
 * @brief 翻转 LED 输出电平
 */
void drv_led_toggle(void);

/**
 * @brief 设置 LED 状态
 *
 * @param on  true = 点亮，false = 熄灭
 */
void drv_led_set(bool on);

bool drv_led_read(void);

#ifdef __cplusplus
}
#endif

#endif /* DRV_LED_H */
