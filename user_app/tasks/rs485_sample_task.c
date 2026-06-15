/*
 * Copyright (c) 2026 G1_Hand 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    rs485_sample_task.c
 * @brief   RS-485 DMA 收发示例任务
 *
 * 演示 RS-485 驱动的用法：
 * - 初始化指定 RS-485 端口
 * - 回环测试 — 收到数据后原样返回
 * - 周期性发送测试消息
 */
#include "rs485_sample_task.h"
#include "board.h"
#include "drv_rs485.h"
#include "hpm_clock_drv.h"
#include "log.h"

/* --------------------------------------------------------------------------
 * 配置
 * -------------------------------------------------------------------------- */

#define TEST_PORT RS485_PORT_MOTOR1 /**< 回环测试端口 */
#define TEST_ECHO_ENABLED 1 /**< 是否启用回环测试 */

/* 发送测试数据 */
static const uint8_t s_test_msg[] = "12345678\r\n";

/* --------------------------------------------------------------------------
 * 接收回调：收到数据时在 ISR 中调用
 * -------------------------------------------------------------------------- */

static void on_rx_frame(rs485_port_t port, uint8_t* data, uint32_t len)
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

void rs485_sample_task_init(void)
{
    LOG_I("rs485_sample", "RS-485 DMA 示例任务启动");

    /* 1. 初始化测试端口 */
    LOG_I("rs485_sample", "初始化 RS-485 端口...");
    rs485_init(TEST_PORT);

    /* 2. 注册接收回调 */
    rs485_set_rx_callback(TEST_PORT, on_rx_frame);
    LOG_I("rs485_sample", "端口初始化完成, DMA 环形接收已启动");
}

/**
 * @brief RS-485 示例任务主循环
 *
 * 在主循环中周期性调用。轮询端口是否有新数据到达，
 * 以及周期性发送测试消息。
 */
void rs485_sample_task_poll(void)
{
    /* 轮询端口接收数据 */
    uint32_t avail = rs485_rx_available(TEST_PORT);
    if (avail > 0) {
        uint8_t buf[128];
        uint32_t read_len = (avail > sizeof(buf)) ? sizeof(buf) : avail;
        rs485_rx_read(TEST_PORT, buf, read_len);
        /* TODO: 在此添加协议解析逻辑 */
    }

    /*
     * 周期性发送测试消息。
     * rs485_is_tx_idle() 返回 true 表示 DMA 空闲。
     */
    static uint32_t s_tick = 0;
    s_tick++;
    if (s_tick >= 200000) {
        s_tick = 0;
        hpm_stat_t stat = rs485_send(TEST_PORT, s_test_msg, sizeof(s_test_msg) - 1);
        if (stat == status_success) {
            LOG_D("rs485_sample", "测试消息发送成功");
        } else {
            LOG_E("rs485_sample", "测试消息发送失败, 状态码=%d", stat);
        }
    }
}
