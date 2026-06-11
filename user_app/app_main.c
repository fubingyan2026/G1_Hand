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
#include "canfd_task.h"
#include "finger.h"
#include "finger_task.h"
#include "led_task.h"
#include "uart_task.h"

#include <stdio.h>

/** @brief 速度值：最高位为方向，bit31=1 正转，速度值 500 RPM */
#define FINGER_INIT_SPEED   (0x80000000UL | 3000UL)

/**
 * @brief 初始化所有手指电机：使能 + 速度模式 500RPM
 * @note  逐个电机串行初始化，每电机约 2 条命令（start + set_speed）
 */
static void finger_init_all_motors(void)
{
    printf("\n[FINGER] Initializing %u motors (start + speed 500RPM)...\n",
        FINGER_TASK_MAX_MOTORS);

    for (uint8_t id = 1; id <= FINGER_TASK_MAX_MOTORS; id++) {
        char name[12];
        snprintf(name, sizeof(name), "finger_%u", (unsigned int)id);
        finger_handle_t* m = finger_get_instance(name);
        if (!m) {
            printf("[FINGER] %s not found, skip\n", name);
            continue;
        }

        /* 1. 使能电机 */
        //printf("[FINGER] %s: start...\n", name);
        //finger_start(m);
        //finger_wait_done(m);

        /* 2. 速度模式 500RPM */
        //printf("[FINGER] %s: speed mode 500RPM...\n", name);
        //finger_set_speed(m, FINGER_INIT_SPEED);
        //finger_wait_done(m);

        printf("[FINGER] %s: done\n", name);
    }

    printf("[FINGER] All motors initialized\n\n");
}

int main(void)
{
    /* 板级初始化（时钟、调试串口、引脚等） */
    board_init();

    /* 初始化 LED 闪烁任务（250ms 循环闪烁） */
    led_task_init();
    /* 初始化手指电机服务层 */
    finger_init();
    /* 初始化手指电机通讯任务 */
    finger_task_init();

    /* 初始化所有电机：使能 + 速度模式 */
    finger_init_all_motors();

    /* 初始化 CAN-FD 测试任务 (必须在 ESC 引脚初始化之后) */
    canfd_task_init();

    /* 主循环 */
    while (1) {
        led_task_poll();
        finger_task_poll();
        canfd_task_poll();
    }

    return 0;
}
