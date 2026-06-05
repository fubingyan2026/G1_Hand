/*
 * Copyright (c) 2024 G1_Hand 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    bsp_gpio.h
 * @brief   BSP GPIO 抽象层
 *
 * 对 HPM SDK hpm_gpio_drv.h 的薄层封装。
 * 使用标准的 HPM GPIO_Type + 端口索引 + 引脚索引 寻址方式。
 */

#ifndef BSP_GPIO_H
#define BSP_GPIO_H

#include "hpm_gpio_drv.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 配置 GPIO 引脚为输入模式
 *
 * @param port     GPIO 基地址（例如 HPM_GPIO0）
 * @param port_idx 端口索引（使用 GPIO_GET_PORT_INDEX(pin_num)）
 * @param pin_idx  引脚索引（使用 GPIO_GET_PIN_INDEX(pin_num)）
 */
void bsp_gpio_set_input(GPIO_Type *port, uint32_t port_idx, uint8_t pin_idx);

/**
 * @brief 配置 GPIO 引脚为输出模式（不设置初始电平）
 *
 * @param port     GPIO 基地址
 * @param port_idx 端口索引
 * @param pin_idx  引脚索引
 */
void bsp_gpio_set_output(GPIO_Type *port, uint32_t port_idx, uint8_t pin_idx);

/**
 * @brief 配置 GPIO 引脚为输出模式，并设置初始电平
 *
 * @param port          GPIO 基地址
 * @param port_idx     端口索引
 * @param pin_idx      引脚索引
 * @param initial_level 初始输出电平（0 = 低电平，1 = 高电平）
 */
void bsp_gpio_init(GPIO_Type *port, uint32_t port_idx, uint8_t pin_idx, uint8_t initial_level);

/**
 * @brief 写 GPIO 引脚输出电平
 *
 * @param port     GPIO 基地址
 * @param port_idx 端口索引
 * @param pin_idx  引脚索引
 * @param level    输出电平（0 = 低电平，1 = 高电平）
 */
void bsp_gpio_write(GPIO_Type *port, uint32_t port_idx, uint8_t pin_idx, uint8_t level);

/**
 * @brief 读 GPIO 引脚输入电平
 *
 * @param port     GPIO 基地址
 * @param port_idx 端口索引
 * @param pin_idx  引脚索引
 * @return        引脚电平（0 = 低电平，1 = 高电平）
 */
uint8_t bsp_gpio_read(GPIO_Type *port, uint32_t port_idx, uint8_t pin_idx);

/**
 * @brief 翻转 GPIO 引脚输出电平
 *
 * @param port     GPIO 基地址
 * @param port_idx 端口索引
 * @param pin_idx  引脚索引
 */
void bsp_gpio_toggle(GPIO_Type *port, uint32_t port_idx, uint8_t pin_idx);

#ifdef __cplusplus
}
#endif

#endif /* BSP_GPIO_H */
