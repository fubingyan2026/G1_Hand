/*
 * Copyright (c) 2026 G1_Hand 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    bsp_systick.c
 * @brief   BSP 系统节拍定时工具实现
 *
 * 基于 HPM MCHTMR 机器定时器 (24MHz) 实现延时、微秒/毫秒时间戳和性能计时。
 * - delay_us / delay_ms → 委托 clock_cpu_delay_us / clock_cpu_delay_ms（CPU 周期精确延时）
 * - micros / millis     → 读取 MCHTMR 计数器并转换为微秒 / 毫秒
 * - BSP_SYSTICK_TIME_DEFINE / BSP_SYSTICK_TIME_HOOK → 编译期多通道性能计时（宏）
 */

/* Includes ------------------------------------------------------------------*/
#include "bsp_systick.h"

#include "hpm_clock_drv.h"
#include "hpm_mchtmr_drv.h"
#include "hpm_soc.h"

/* Private constants ---------------------------------------------------------*/

/** @brief MCHTMR 每微秒计数值 — 24MHz 安全默认值 */
#define BSP_SYSTICK_DEFAULT_US_FACTOR (24U)

/** @brief MCHTMR 每毫秒计数值 — 24MHz 安全默认值 */
#define BSP_SYSTICK_DEFAULT_MS_FACTOR (24000U)

/* Private variables ---------------------------------------------------------*/

/** @brief MCHTMR 每微秒计数值（delay_init 时由 MCHTMR 频率计算） */
static uint32_t s_ticks_per_us = BSP_SYSTICK_DEFAULT_US_FACTOR;

/** @brief MCHTMR 每毫秒计数值（delay_init 时由 MCHTMR 频率计算） */
static uint32_t s_ticks_per_ms = BSP_SYSTICK_DEFAULT_MS_FACTOR;

/* Exported functions --------------------------------------------------------*/

void delay_init(void)
{
    uint32_t mchtmr_freq = clock_get_frequency(clock_mchtmr0);

    if (mchtmr_freq == 0) {
        /* MCHTMR 时钟未初始化（board_init_clock 尚未调用），保留默认因子 */
        return;
    }

    s_ticks_per_us = mchtmr_freq / 1000000U;
    s_ticks_per_ms = mchtmr_freq / 1000U;

    /* 防御性检查：防止除零（MCHTMR 频率正常运行时不会触发） */
    if (s_ticks_per_us == 0) {
        s_ticks_per_us = BSP_SYSTICK_DEFAULT_US_FACTOR;
    }
    if (s_ticks_per_ms == 0) {
        s_ticks_per_ms = BSP_SYSTICK_DEFAULT_MS_FACTOR;
    }
}

void delay_us(const uint16_t nus)
{
    clock_cpu_delay_us(nus);
}

void delay_ms(const uint16_t nms)
{
    clock_cpu_delay_ms(nms);
}

uint32_t micros(void)
{
    return (uint32_t)(mchtmr_get_count(HPM_MCHTMR) / s_ticks_per_us);
}

uint32_t millis(void)
{
    return (uint32_t)(mchtmr_get_count(HPM_MCHTMR) / s_ticks_per_ms);
}
