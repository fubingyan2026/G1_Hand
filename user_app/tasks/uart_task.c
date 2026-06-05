/*
 * Copyright (c) 2026 G1_Hand 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    uart_task.c
 * @brief   RS-485 DMA 收发任务
 *
 * 演示 RS-485 驱动的完整用法：
 * - 初始化三路 RS-485 端口
 * - 外部端口 (UART8) 做回环测试 — 收到数据后原样返回
 * - 电机端口 (UART14/15) 接收数据存入队列，应用层轮询读出
 */
#include "uart_task.h"
#include "board.h"
#include "drv_rs485.h"
#include "hpm_clock_drv.h"
#include <stdio.h>

/* --------------------------------------------------------------------------
 * 配置
 * -------------------------------------------------------------------------- */

#define TEST_PORT           RS485_PORT_EXT    /**< 回环测试端口 */
#define TEST_ECHO_ENABLED   1                 /**< 是否启用回环测试 */

/* 发送测试数据 */
static const uint8_t s_test_msg[] = "G1_Hand RS485 DMA Test\r\n";

/* --------------------------------------------------------------------------
 * 接收回调：收到数据时在 ISR 中调用
 * -------------------------------------------------------------------------- */

static void on_rx_frame(rs485_port_t port, uint8_t *data, uint32_t len)
{
    (void)port;
    (void)data;
    (void)len;

#if TEST_ECHO_ENABLED
    /*
     * 回环测试：将收到的数据原样发回。
     * 注意：这里在 ISR 中调用 rs485_send_dma()，因为 kfifo 有锁保护，
     * 且 DMA 发送是链式出队的，所以是安全的。
     */
    if (port == TEST_PORT) {
        rs485_send_dma(port, data, len);
    }
#endif
}

/* --------------------------------------------------------------------------
 * 任务入口
 * -------------------------------------------------------------------------- */

void uart_task_init(void)
{
    printf("\n========================================\n");
    printf("  G1_Hand RS-485 DMA 收发任务启动\n");
    printf("========================================\n\n");

    /* 1. 初始化全部三路 RS-485 端口 */
    printf("[UART] 初始化 RS-485 端口...\n");
    rs485_init_all();

    /* 2. 注册接收回调 */
    rs485_set_rx_callback(RS485_PORT_EXT, on_rx_frame);
    rs485_set_rx_callback(RS485_PORT_MOTOR1, on_rx_frame);
    rs485_set_rx_callback(RS485_PORT_MOTOR2, on_rx_frame);
    printf("[UART] 三路端口初始化完成，DMA 环形接收已启动\n");

    /* 3. 发送一条测试消息（阻塞模式） */
    printf("[UART] 发送测试消息 (阻塞模式)...\n");
    hpm_stat_t stat = rs485_send(TEST_PORT, s_test_msg, sizeof(s_test_msg) - 1);
    if (stat == status_success) {
        printf("[UART] 测试消息发送成功\n");
    } else {
        printf("[UART] 测试消息发送失败! stat=%d\n", stat);
    }
}

/**
 * @brief RS-485 收发主循环
 *
 * 在主循环中周期性调用。轮询各端口是否有新数据到达，
 * 以及检查发送管线状态。
 */
void uart_task_poll(void)
{
    /* 轮询各端口接收数据 */
    for (int i = 0; i < RS485_PORT_MAX; i++) {
        uint32_t avail = rs485_rx_available((rs485_port_t)i);
        if (avail > 0) {
            /*
             * 有数据到达。对于电机端口 (MOTOR1/MOTOR2)，
             * 应用层在此处理协议解析。
             *
             * 对于外部端口 (EXT)，回环已在 ISR 回调中处理，
             * 此处仅统计接收量。
             */
            if (i != RS485_PORT_EXT) {
                /* 读取并处理电机通讯数据 */
                uint8_t buf[64];
                uint32_t read_len = (avail > sizeof(buf)) ? sizeof(buf) : avail;
                rs485_rx_read((rs485_port_t)i, buf, read_len);
                /* TODO: 在此添加协议解析逻辑 */
            }
        }
    }

    /*
     * 周期性检查发送管线状态（可选：上报/日志）。
     * rs485_is_tx_idle() 返回 true 表示无 DMA 运行且队列为空。
     */
    static uint32_t s_tick = 0;
    s_tick++;
    if (s_tick >= 10000) {
        s_tick = 0;
        for (int i = 0; i < RS485_PORT_MAX; i++) {
            if (!rs485_is_tx_idle((rs485_port_t)i)) {
                uint32_t pending = rs485_tx_pending((rs485_port_t)i);
                if (pending > 128) {
                    printf("[UART] 端口%d 发送队列积压: %u 字节\n", i, pending);
                }
            }
        }
    }
}
