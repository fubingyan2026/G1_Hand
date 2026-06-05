/*
 * Copyright (c) 2026 G1_Hand 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    led_task.c
 * @brief   LED 状态指示任务实现
 *
 * 使用 middlewares/service/led.c 库管理 PC28 状态指示灯：
 * - 通过 FSM 状态机控制 ON/OFF/BLINK_CODE 三种状态
 * - 闪烁完成后自动重新触发，实现循环闪烁
 * - 时间基准使用 MCHTMR 机器定时器（ms 精度）
 */

#include "led_task.h"
#include "drv_led.h"
#include "hpm_clock_drv.h"
#include "hpm_mchtmr_drv.h"
#include "led.h"

#define LED_TASK_DEFAULT_BLINK_COUNT 3 /**< 每次循环闪烁次数 */
#define LED_TASK_DEFAULT_BLINK_CYCLE_MS 200 /**< 闪烁半周期 (ms) */
#define LED_TASK_DEFAULT_BLINK_WAIT_MS 1000 /**< 两次闪烁循环之间的等待间隔 (ms) */

static led_handle_t s_led_handle; /**< 状态 LED 静态实例 */
static uint32_t s_ticks_per_ms; /**< MCHTMR 每毫秒计数值 */

/**
 * @brief 获取系统毫秒时间戳
 * @return 自启动以来的毫秒数
 */
static uint32_t led_task_get_time_ms(void)
{
    return (uint32_t)(mchtmr_get_count(HPM_MCHTMR) / s_ticks_per_ms);
}

static void led_task_write_pin(bool on)
{
    drv_led_set(on);
}

/**
 * @brief LED 状态变化回调
 * @note  当闪烁计数完成后 FSM 自动切换到 OFF 状态，
 *        此时重新触发 BLINK_CODE 实现循环闪烁。
 */
static void led_task_on_state_change(led_handle_t* instance,
    led_state_t new_state, void* user_data)
{
    (void)user_data;

    if (new_state == LED_STATE_OFF) {
        led_task_start_blink(LED_TASK_DEFAULT_BLINK_COUNT,
            LED_TASK_DEFAULT_BLINK_CYCLE_MS);
    }
}

void led_task_start_blink(uint16_t count, uint16_t cycle_ms)
{
    led_cmd_t cmd = {
        .led_set_state = LED_STATE_BLINK_CODE,
        .led_blink_cycle_ms = cycle_ms,
        .led_blink_wait_ms = LED_TASK_DEFAULT_BLINK_WAIT_MS,
        .led_blink_code_counts = count,
    };

    /* 先更新闪烁参数，再触发状态切换 */
    led_set_blink_interval(&s_led_handle, &cmd);
    led_set_state(&s_led_handle, LED_STATE_BLINK_CODE);
}

void led_task_init(void)
{

    /* 初始化状态指示 LED */
    drv_led_init();

    /* 计算 MCHTMR ticks/ms */
    uint32_t mchtmr_freq = clock_get_frequency(clock_mchtmr0);
    s_ticks_per_ms = mchtmr_freq / 1000;

    if (s_ticks_per_ms == 0) {
        /* 防止除零错误，设置一个合理的默认值 */
        s_ticks_per_ms = 1;
    }

    /* 初始化 LED 子系统 */
    led_init(led_task_get_time_ms);

    /* 配置 LED 实例 */
    led_config_t config = {
        .name = "status",
        .init_state = LED_STATE_OFF,
        .write_pin = led_task_write_pin,
    };

    led_register_static(&config, &s_led_handle);

    /* 注册状态变化回调（用于循环闪烁） */
    led_set_callbacks(&s_led_handle, led_task_on_state_change, NULL, NULL, NULL);

    /* 设置初始闪烁参数并启动 */
    led_task_start_blink(LED_TASK_DEFAULT_BLINK_COUNT,
        LED_TASK_DEFAULT_BLINK_CYCLE_MS);
}

void led_task_poll(void)
{
    led_task_refresh();
}
