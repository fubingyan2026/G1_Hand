/*
 * Copyright (c) 2026 G1_Hand 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    bsp_systick.h
 * @brief   BSP 系统节拍定时工具 — 延时、微秒/毫秒时间戳、性能计时
 *
 * 提供统一的时间基准 API，底层基于 HPM MCHTMR 机器定时器 (24MHz)。
 */

#ifndef BSP_SYSTICK_H
#define BSP_SYSTICK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化延时参数
 *
 * 根据 MCHTMR 时钟频率计算微秒和毫秒的计数值因子。
 * 必须在板级时钟初始化（board_init_clock）之后调用。
 * 若未调用，模块使用 24MHz 安全默认值（24 ticks/μs, 24000 ticks/ms）。
 */
void delay_init(void);

/**
 * @brief 微秒级延时
 *
 * 通过 CPU 周期计数实现微秒级别的精确忙等待延时。
 *
 * @param nus 需要延时的微秒数
 */
void delay_us(uint16_t nus);

/**
 * @brief 毫秒级延时
 *
 * 通过 CPU 周期计数实现毫秒级别的精确忙等待延时。
 *
 * @param nms 需要延时的毫秒数
 */
void delay_ms(uint16_t nms);

/**
 * @brief 获取微秒级时间戳
 *
 * 读取 MCHTMR 机器定时器计数器并转换为微秒。
 *
 * @return 自系统启动以来的微秒数（约 71 分钟回绕）
 */
uint32_t micros(void);

/**
 * @brief 获取系统运行时间（毫秒）
 *
 * 读取 MCHTMR 机器定时器计数器并转换为毫秒。
 *
 * @return 自系统启动以来的毫秒数（约 49.7 天回绕）
 */
uint32_t millis(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_SYSTICK_H */
