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
#include "drv_led.h"
#include "led_task.h"
#include "uart_task.h"

int main(void)
{
    /* 板级初始化（时钟、调试串口、引脚等） */
    board_init();

    /* 初始化 LED 闪烁任务（250ms 循环闪烁） */
    led_task_init();
    /* 初始化 RS-485 DMA 收发任务 */
    //uart_task_init();

    /* 主循环 */
    while (1) {
        led_task_poll();
        //uart_task_poll();
    }

    return 0;
}
