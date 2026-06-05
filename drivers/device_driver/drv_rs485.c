/*
 * Copyright (c) 2024 G1_Hand 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    drv_rs485.c
 * @brief   RS-485 半双工驱动实现 — 中断驱动的 TX 队列 + DMA 环形 RX
 */

#include "drv_rs485.h"
#include "bsp_gpio.h"
#include "bsp_uart.h"
#include "hpm_dmamux_src.h"
#include "hpm_gpio_drv.h"
#include "hpm_iomux.h"
#include "hpm_soc.h"
#include "kfifo.h"

/* --------------------------------------------------------------------------
 * 引脚定义
 * -------------------------------------------------------------------------- */

#define IOC_PB00 32
#define IOC_PB01 33
#define IOC_PB02 34
#define IOC_PB24 56
#define IOC_PB25 57
#define IOC_PB26 58
#define IOC_PB29 61
#define IOC_PB30 62
#define IOC_PB31 63

#define GPIO_PORT_PB 1

#define PB00_PIN 0
#define PB01_PIN 1
#define PB02_PIN 2
#define PB24_PIN 24
#define PB25_PIN 25
#define PB26_PIN 26
#define PB29_PIN 29
#define PB30_PIN 30
#define PB31_PIN 31

/* --------------------------------------------------------------------------
 * DMA 通道分配
 * -------------------------------------------------------------------------- */

#define DMA_CH_UART15_TX 2
#define DMA_CH_UART15_RX 3
#define DMA_CH_UART14_TX 4
#define DMA_CH_UART14_RX 5
#define DMA_CH_UART8_TX 6
#define DMA_CH_UART8_RX 7

/* 每次从 TX 队列中取出的最大字节数 */
#define RS485_TX_CHUNK_SIZE 64U

/* --------------------------------------------------------------------------
 * 端口上下文
 * -------------------------------------------------------------------------- */

typedef struct {
    bsp_uart_config_t uart_cfg;
    GPIO_Type* de_port;
    uint32_t de_port_idx;
    uint8_t de_pin_idx;

    /* 接收 */
    uint8_t rx_dma_buf[RS485_RX_CIRC_BUF_SIZE];
    uint8_t rx_ring_storage[RS485_RX_RING_BUF_SIZE];
    kfifo_t rx_fifo;
    rs485_rx_callback_t rx_cb;

    /* 发送 — 中断驱动队列 */
    uint8_t tx_ring_storage[RS485_TX_RING_BUF_SIZE];
    kfifo_t tx_fifo;
    bool tx_running; /* DMA 正在运行或队列非空 */
    uint8_t tx_chunk[RS485_TX_CHUNK_SIZE]; /* 出队暂存 */
} rs485_port_ctx_t;

static rs485_port_ctx_t s_ports[RS485_PORT_MAX];

/* --------------------------------------------------------------------------
 * 前置声明
 * -------------------------------------------------------------------------- */

static void rs485_rx_callback(uint8_t* data, uint32_t len);
static void rs485_tx_dma_done(void* user_data);

/* ======================================================================
 * 初始化
 * ====================================================================== */

void rs485_init(rs485_port_t port)
{
    if (port >= RS485_PORT_MAX)
        return;

    rs485_port_ctx_t* ctx = &s_ports[port];

    /* 初始化接收 / 发送环形缓冲区 */
    kfifo_init(&ctx->rx_fifo, ctx->rx_ring_storage, RS485_RX_RING_BUF_SIZE, NULL);
    kfifo_init(&ctx->tx_fifo, ctx->tx_ring_storage, RS485_TX_RING_BUF_SIZE, NULL);
    ctx->tx_running = false;

    /* 按端口配置 UART */
    switch (port) {
    case RS485_PORT_MOTOR1: /* UART15: TX=PB31, RX=PB30, DE=PB29 */
        ctx->uart_cfg.base = HPM_UART15;
        ctx->uart_cfg.baudrate = RS485_DEFAULT_BAUDRATE;
        ctx->uart_cfg.clk_name = clock_uart15;
        ctx->uart_cfg.irq_num = IRQn_UART15;
        ctx->uart_cfg.tx_ioc_pad = IOC_PB31;
        ctx->uart_cfg.tx_ioc_func = IOC_PB31_FUNC_CTL_UART15_TXD;
        ctx->uart_cfg.rx_ioc_pad = IOC_PB30;
        ctx->uart_cfg.rx_ioc_func = IOC_PB30_FUNC_CTL_UART15_RXD;
        ctx->uart_cfg.tx_dma_ch = DMA_CH_UART15_TX;
        ctx->uart_cfg.rx_dma_ch = DMA_CH_UART15_RX;
        ctx->uart_cfg.tx_dma_req = HPM_DMA_SRC_UART15_TX;
        ctx->uart_cfg.rx_dma_req = HPM_DMA_SRC_UART15_RX;
        ctx->de_port = HPM_GPIO0;
        ctx->de_port_idx = GPIO_PORT_PB;
        ctx->de_pin_idx = PB29_PIN;
        break;

    case RS485_PORT_MOTOR2: /* UART14: TX=PB24, RX=PB25, DE=PB26 */
        ctx->uart_cfg.base = HPM_UART14;
        ctx->uart_cfg.baudrate = RS485_DEFAULT_BAUDRATE;
        ctx->uart_cfg.clk_name = clock_uart14;
        ctx->uart_cfg.irq_num = IRQn_UART14;
        ctx->uart_cfg.tx_ioc_pad = IOC_PB24;
        ctx->uart_cfg.tx_ioc_func = IOC_PB24_FUNC_CTL_UART14_TXD;
        ctx->uart_cfg.rx_ioc_pad = IOC_PB25;
        ctx->uart_cfg.rx_ioc_func = IOC_PB25_FUNC_CTL_UART14_RXD;
        ctx->uart_cfg.tx_dma_ch = DMA_CH_UART14_TX;
        ctx->uart_cfg.rx_dma_ch = DMA_CH_UART14_RX;
        ctx->uart_cfg.tx_dma_req = HPM_DMA_SRC_UART14_TX;
        ctx->uart_cfg.rx_dma_req = HPM_DMA_SRC_UART14_RX;
        ctx->de_port = HPM_GPIO0;
        ctx->de_port_idx = GPIO_PORT_PB;
        ctx->de_pin_idx = PB26_PIN;
        break;

    case RS485_PORT_EXT: /* UART8: TX=PB00, RX=PB01, DE=PB02 */
        ctx->uart_cfg.base = HPM_UART8;
        ctx->uart_cfg.baudrate = RS485_DEFAULT_BAUDRATE;
        ctx->uart_cfg.clk_name = clock_uart8;
        ctx->uart_cfg.irq_num = IRQn_UART8;
        ctx->uart_cfg.tx_ioc_pad = IOC_PB00;
        ctx->uart_cfg.tx_ioc_func = IOC_PB00_FUNC_CTL_UART8_TXD;
        ctx->uart_cfg.rx_ioc_pad = IOC_PB01;
        ctx->uart_cfg.rx_ioc_func = IOC_PB01_FUNC_CTL_UART8_RXD;
        ctx->uart_cfg.tx_dma_ch = DMA_CH_UART8_TX;
        ctx->uart_cfg.rx_dma_ch = DMA_CH_UART8_RX;
        ctx->uart_cfg.tx_dma_req = HPM_DMA_SRC_UART8_TX;
        ctx->uart_cfg.rx_dma_req = HPM_DMA_SRC_UART8_RX;
        ctx->de_port = HPM_GPIO0;
        ctx->de_port_idx = GPIO_PORT_PB;
        ctx->de_pin_idx = PB02_PIN;
        break;

    default:
        return;
    }

    /* 通用配置 */
    ctx->uart_cfg.dma_controller = HPM_HDMA;
    ctx->uart_cfg.dmamux_controller = HPM_DMAMUX;
    ctx->uart_cfg.rx_circ_buf = ctx->rx_dma_buf;
    ctx->uart_cfg.rx_circ_buf_size = RS485_RX_CIRC_BUF_SIZE;
    ctx->uart_cfg.rx_idle_threshold = RS485_RX_IDLE_THRESHOLD;
    ctx->uart_cfg.rx_callback = rs485_rx_callback;
    ctx->rx_cb = NULL;

    /* 注册 TX 完成回调（HDMA ISR → 链式出队） */
    bsp_uart_set_tx_callback(&ctx->uart_cfg, rs485_tx_dma_done,
        (void*)(uintptr_t)port);

    /* DE 引脚初始化为低电平（接收模式） */
    bsp_gpio_init(ctx->de_port, ctx->de_port_idx, ctx->de_pin_idx, 0);

    /* 初始化 UART */
    bsp_uart_init(&ctx->uart_cfg);
}

void rs485_init_all(void)
{
    for (int i = 0; i < RS485_PORT_MAX; i++) {
        rs485_init((rs485_port_t)i);
    }
}

/* ======================================================================
 * 接收
 * ====================================================================== */

void rs485_set_rx_callback(rs485_port_t port, rs485_rx_callback_t cb)
{
    if (port < RS485_PORT_MAX) {
        s_ports[port].rx_cb = cb;
    }
}

static void rs485_rx_callback(uint8_t* data, uint32_t len)
{
    rs485_port_ctx_t* ctx = NULL;
    rs485_port_t port;

    for (int i = 0; i < RS485_PORT_MAX; i++) {
        if (data >= s_ports[i].rx_dma_buf && data < (s_ports[i].rx_dma_buf + RS485_RX_CIRC_BUF_SIZE)) {
            ctx = &s_ports[i];
            port = (rs485_port_t)i;
            break;
        }
    }

    if (!ctx || len == 0)
        return;

    kfifo_put(&ctx->rx_fifo, data, len);

    if (ctx->rx_cb) {
        ctx->rx_cb(port, data, len);
    }
}

uint32_t rs485_rx_available(rs485_port_t port)
{
    if (port >= RS485_PORT_MAX)
        return 0;
    return kfifo_len(&s_ports[port].rx_fifo);
}

uint32_t rs485_rx_read(rs485_port_t port, uint8_t* out, uint32_t max_len)
{
    if (port >= RS485_PORT_MAX)
        return 0;
    return kfifo_get(&s_ports[port].rx_fifo, out, max_len);
}

/* ======================================================================
 * 发送 — 中断驱动队列
 * ====================================================================== */

/**
 * @brief 从 TX 队列取数据并启动 DMA（内部函数）
 *
 * 调用时机：(1) 用户第一次调用 rs485_send_dma 时
 *          (2) 上一段 DMA 完成后 HDMA ISR 回调
 */
static void rs485_tx_kick(rs485_port_t port)
{
    rs485_port_ctx_t* ctx = &s_ports[port];
    uint32_t pending = kfifo_len(&ctx->tx_fifo);

    if (pending == 0) {
        /* 队列已空 — 等待移位寄存器清空后拉低 DE */
        bsp_uart_flush(&ctx->uart_cfg);
        bsp_gpio_write(ctx->de_port, ctx->de_port_idx, ctx->de_pin_idx, 0);
        ctx->tx_running = false;
        return;
    }

    /* 从队列取出一段数据（最多 RS485_TX_CHUNK_SIZE 字节） */
    uint32_t chunk = (pending > RS485_TX_CHUNK_SIZE) ? RS485_TX_CHUNK_SIZE : pending;
    kfifo_get(&ctx->tx_fifo, ctx->tx_chunk, chunk);

    /* 启动 DMA 发送 */
    bsp_uart_send_dma(&ctx->uart_cfg, ctx->tx_chunk, chunk);
}

/**
 * @brief HDMA ISR 回调：上一段 DMA 发送完成
 *
 * 检查队列是否有更多数据 → 有则继续发送，无则收尾（flush + 拉低 DE）。
 */
static void rs485_tx_dma_done(void* user_data)
{
    rs485_port_t port = (rs485_port_t)(uintptr_t)user_data;
    rs485_port_ctx_t* ctx = &s_ports[port];

    if (kfifo_is_empty(&ctx->tx_fifo)) {
        /* 队列已空，发送管线收尾 */
        bsp_uart_flush(&ctx->uart_cfg);
        bsp_gpio_write(ctx->de_port, ctx->de_port_idx, ctx->de_pin_idx, 0);
        ctx->tx_running = false;
    } else {
        /* 队列还有数据，启动下一段 DMA */
        uint32_t pending = kfifo_len(&ctx->tx_fifo);
        uint32_t chunk = (pending > RS485_TX_CHUNK_SIZE) ? RS485_TX_CHUNK_SIZE : pending;
        kfifo_get(&ctx->tx_fifo, ctx->tx_chunk, chunk);
        bsp_uart_send_dma(&ctx->uart_cfg, ctx->tx_chunk, chunk);
    }
}

hpm_stat_t rs485_send_dma(rs485_port_t port, const uint8_t* data, uint32_t len)
{
    if (port >= RS485_PORT_MAX || !data || len == 0) {
        return status_fail;
    }

    rs485_port_ctx_t* ctx = &s_ports[port];

    /* 入队 */
    uint32_t written = kfifo_put(&ctx->tx_fifo, data, len);
    if (written != len) {
        return status_fail; /* 队列满 */
    }

    /* 若无 DMA 运行中，启动首次发送 */
    if (!ctx->tx_running) {
        ctx->tx_running = true;
        bsp_gpio_write(ctx->de_port, ctx->de_port_idx, ctx->de_pin_idx, 1); /* DE=高 */
        rs485_tx_kick(port);
    }

    return status_success;
}

hpm_stat_t rs485_send(rs485_port_t port, const uint8_t* data, uint32_t len)
{
    hpm_stat_t stat = rs485_send_dma(port, data, len);
    if (stat != status_success) {
        return stat;
    }

    /* 自旋等待管线排空 */
    rs485_flush_tx(port);

    return status_success;
}

bool rs485_is_tx_idle(rs485_port_t port)
{
    if (port >= RS485_PORT_MAX)
        return true;

    rs485_port_ctx_t* ctx = &s_ports[port];
    return !ctx->tx_running && kfifo_is_empty(&ctx->tx_fifo);
}

void rs485_flush_tx(rs485_port_t port)
{
    if (port >= RS485_PORT_MAX)
        return;

    rs485_port_ctx_t* ctx = &s_ports[port];

    /* 等待 TX 队列排空 */
    while (!kfifo_is_empty(&ctx->tx_fifo) || ctx->tx_running) {
        /* 自旋：队列未空 或 DMA 仍在运行 */
    }
}

uint32_t rs485_tx_pending(rs485_port_t port)
{
    if (port >= RS485_PORT_MAX)
        return 0;
    return kfifo_len(&s_ports[port].tx_fifo);
}
