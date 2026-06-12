/*
 * Copyright (c) 2026 G1_Hand 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    log_task.c
 * @brief   日志输出任务实现
 *
 * 胶合 middlewares/log 模块与 drv_log_uart 驱动：
 * - 初始化阶段：配置并启动 log 模块和 UART0 DMA 驱动
 * - 轮询阶段：从 log 模块 kfifo 中取出格式化日志，通过 UART0 DMA 发出
 */

/* Includes ------------------------------------------------------------------*/
#include "log_task.h"

#include "bsp_systick.h"
#include "drv_log_uart.h"
#include "log.h"

/* Private constants ---------------------------------------------------------*/

/** @brief 本地 TX DMA 缓冲区大小 */
#define LOG_TASK_TX_BUF_SIZE (256)

/* Private variables ---------------------------------------------------------*/

/** @brief 本地 TX DMA 缓冲区 */
static uint8_t s_tx_buf[LOG_TASK_TX_BUF_SIZE];

/** @brief 日志串口驱动上下文 */
static drv_log_uart_context_t s_log_uart_ctx;

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 初始化日志任务
 */
void log_task_init(void)
{
    /* 初始化日志模块（使用 millis() 作为时间戳） */
    log_config_t log_cfg = {
        .name = "G1_Hand",
        .get_timestamp_cb = millis,
    };
    log_init(&log_cfg);

    /* 初始化 UART0 DMA 驱动 */
    drv_log_uart_config_t uart_cfg = {
        .baudrate = DRV_LOG_UART_DEFAULT_BAUDRATE,
        .tx_dma_ch = DRV_LOG_UART_DEFAULT_TX_DMA_CH,
    };
    drv_log_uart_init(&s_log_uart_ctx, &uart_cfg);
}

/**
 * @brief 日志任务轮询
 *
 * 桥接 middlewares/log 与 drivers/drv_log_uart：
 * 1. 更新 TX DMA 完成状态
 * 2. 若 DMA 空闲，从 log 模块 kfifo 取出数据通过 UART0 DMA 发出
 */
void log_task_poll(void)
{
    /* 更新 TX DMA 完成状态 */
    drv_log_uart_poll(&s_log_uart_ctx);

    /* 若 DMA 空闲，检查日志模块是否有数据待发送 */
    if (!drv_log_uart_is_tx_busy(&s_log_uart_ctx)) {
        uint32_t log_len = log_tx_len();
        if (log_len > 0) {
            if (log_len > sizeof(s_tx_buf)) {
                log_len = sizeof(s_tx_buf);
            }
            uint32_t actual = log_tx_get(s_tx_buf, log_len);
            if (actual > 0) {
                drv_log_uart_send(&s_log_uart_ctx, s_tx_buf, actual);
            }
        }
    }
}
