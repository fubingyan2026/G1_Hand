/*
 * Copyright (c) 2022 HPMicro
 * Copyright (c) 2026 G1_Hand 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    app_main.c
 * @brief   G1_Hand 灵巧手主入口
 *
 * 初始化硬件后进入主循环，运行 RS-485 DMA 收发任务。
 */

#include "board.h"
#include "bsp_systick.h"
#include "drv_led.h"
#include "led_task.h"
#include "log_task.h"
#include "motor_link_task.h"

int main(void)
{
    /* 板级初始化（时钟、调试串口、引脚等） */
    board_init();

    /* 初始化系统节拍（MCHTMR → 延时/时间戳） */
    delay_init();
    /* 初始化日志输出任务（log 模块 + UART0 DMA 驱动） */
    log_task_init();

    LOG_I("clock","%s clock summary", BOARD_NAME);
    LOG_I("clock","cpu0:\t\t %dHz", clock_get_frequency(clock_cpu0));
    LOG_I("clock","cpu1:\t\t %dHz", clock_get_frequency(clock_cpu1));
    LOG_I("clock","ahb:\t\t %luHz", clock_get_frequency(clock_ahb0));
    LOG_I("clock","axif:\t\t %dHz", clock_get_frequency(clock_axif));
    LOG_I("clock","axis:\t\t %dHz", clock_get_frequency(clock_axis));
    LOG_I("clock","axic:\t\t %dHz", clock_get_frequency(clock_axic));
    LOG_I("clock","axin:\t\t %dHz", clock_get_frequency(clock_axin));
    LOG_I("clock","xpi0:\t\t %dHz", clock_get_frequency(clock_xpi0));
    LOG_I("clock","femc:\t\t %luHz", clock_get_frequency(clock_femc));
    LOG_I("clock","mchtmr0:\t %dHz", clock_get_frequency(clock_mchtmr0));
    LOG_I("clock","mchtmr1:\t %dHz", clock_get_frequency(clock_mchtmr1));
    LOG_I("clock","==============================\n");
    /* 初始化 LED 闪烁任务（250ms 循环闪烁） */
    led_task_init();

    /* 初始化完整电机数据链路（finger 服务层 → RS-485 传输层 → finger 实例
     * → CAN-FD 传输层 → motor_control 服务层 → motor_control 实例 → 回调连线） */
    motor_link_task_init();

    /* 主循环 */
    while (1) {
        led_task_poll();
        log_task_poll();
        motor_link_task_poll();
    }

    return 0;
}
