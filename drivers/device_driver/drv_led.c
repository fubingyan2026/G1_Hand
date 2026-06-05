/*
 * Copyright (c) 2026 G1_Hand 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    drv_led.c
 * @brief   LED 设备驱动实现 — PC28 GPIO 推挽输出控制
 */

#include "drv_led.h"
#include "bsp_gpio.h"
#include "hpm_clock_drv.h"
#include "hpm_iomux.h"
#include "hpm_soc.h"

void drv_led_init(void)
{
    /* 配置 IOC 引脚复用为 GPIO 功能 */
    HPM_IOC->PAD[DRV_LED_IOC_PAD].FUNC_CTL = DRV_LED_IOC_FUNC;

    /* 使能 GPIO 时钟（board_init 已启用，此调用为防御性编程，无副作用） */
    clock_add_to_group(clock_gpio, 0);

    /* 配置 PC28 为输出，设置初始电平 */
    bsp_gpio_init(DRV_LED_GPIO_PORT, DRV_LED_GPIO_PORT_IDX,
        DRV_LED_GPIO_PIN_IDX, DRV_LED_INITIAL_LEVEL);
}

void drv_led_on(void)
{
    bsp_gpio_write(DRV_LED_GPIO_PORT, DRV_LED_GPIO_PORT_IDX,
        DRV_LED_GPIO_PIN_IDX, DRV_LED_ON_LEVEL);
}

void drv_led_off(void)
{
    bsp_gpio_write(DRV_LED_GPIO_PORT, DRV_LED_GPIO_PORT_IDX,
        DRV_LED_GPIO_PIN_IDX, DRV_LED_OFF_LEVEL);
}

void drv_led_toggle(void)
{
    bsp_gpio_toggle(DRV_LED_GPIO_PORT, DRV_LED_GPIO_PORT_IDX,
        DRV_LED_GPIO_PIN_IDX);
}

void drv_led_set(bool on)
{
    bsp_gpio_write(DRV_LED_GPIO_PORT, DRV_LED_GPIO_PORT_IDX,
        DRV_LED_GPIO_PIN_IDX,
        on ? DRV_LED_ON_LEVEL : DRV_LED_OFF_LEVEL);
}

bool drv_led_read(void)
{
    return bsp_gpio_read(DRV_LED_GPIO_PORT, DRV_LED_GPIO_PORT_IDX,
               DRV_LED_GPIO_PIN_IDX)
        == DRV_LED_ON_LEVEL;
}
