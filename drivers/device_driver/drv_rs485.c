/*
 * Copyright (c) 2024 G1_Hand 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    drv_rs485.c
 * @brief   RS-485 半双工驱动实现 — 零拷贝 DMA 发送 + DMA 环形 RX
 */

#include "drv_rs485.h"
#include "bsp_gpio.h"
#include "bsp_uart.h"
#include "hpm_dmamux_src.h"
#include "hpm_gpio_drv.h"
#include "hpm_iomux.h"
#include "hpm_l1c_drv.h"
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
#define IOC_PC05 69
#define IOC_PC06 70
#define IOC_PC07 71

#define GPIO_PORT_PB 1
#define GPIO_PORT_PC 2 /* GPIO_DO_GPIOC */

#define PB00_PIN 0
#define PB01_PIN 1
#define PB02_PIN 2
#define PB24_PIN 24
#define PB25_PIN 25
#define PB26_PIN 26
#define PC05_PIN 5
#define PC06_PIN 6
#define PC07_PIN 7

/* --------------------------------------------------------------------------
 * DMA 通道分配
 * -------------------------------------------------------------------------- */

#define DMA_CH_UART1_TX  0
#define DMA_CH_UART1_RX  1
#define DMA_CH_UART14_TX 4
#define DMA_CH_UART14_RX 5
#define DMA_CH_UART8_TX  6
#define DMA_CH_UART8_RX  7


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

    /* 发送 — 零拷贝 DMA */
    const uint8_t *tx_buf;   /* 当前 DMA 源缓冲区（NULL = 空闲） */
    uint32_t tx_len;          /* 当前传输字节数 */
    bool tx_busy;             /* DMA 传输进行中 */
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

    /* 初始化接收环形缓冲区 */
    kfifo_init(&ctx->rx_fifo, ctx->rx_ring_storage, RS485_RX_RING_BUF_SIZE, NULL);
    ctx->tx_busy = false;
    ctx->tx_buf = NULL;
    ctx->tx_len = 0;

    /* 按端口配置 UART */
    switch (port) {
    case RS485_PORT_MOTOR1: /* UART1: TX=PC7, RX=PC6, DE=PC5 */
        ctx->uart_cfg.base = HPM_UART1;
        ctx->uart_cfg.baudrate = RS485_DEFAULT_BAUDRATE;
        ctx->uart_cfg.clk_name = clock_uart1;
        ctx->uart_cfg.irq_num = IRQn_UART1;
        ctx->uart_cfg.tx_ioc_pad = IOC_PC07;
        ctx->uart_cfg.tx_ioc_func = IOC_PC07_FUNC_CTL_UART1_TXD;
        ctx->uart_cfg.rx_ioc_pad = IOC_PC06;
        ctx->uart_cfg.rx_ioc_func = IOC_PC06_FUNC_CTL_UART1_RXD;
        ctx->uart_cfg.tx_dma_ch = DMA_CH_UART1_TX;
        ctx->uart_cfg.rx_dma_ch = DMA_CH_UART1_RX;
        ctx->uart_cfg.tx_dma_req = HPM_DMA_SRC_UART1_TX;
        ctx->uart_cfg.rx_dma_req = HPM_DMA_SRC_UART1_RX;
        ctx->de_port = HPM_GPIO0;
        ctx->de_port_idx = GPIO_PORT_PC;
        ctx->de_pin_idx = PC05_PIN;
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

    /* 注册 TX 完成回调（HDMA ISR → 拉低 DE） */
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
 * 发送 — 零拷贝 DMA（无软件队列）
 * ====================================================================== */

/**
 * @brief HDMA ISR 回调：DMA 发送完成
 *
 * 收尾：flush UART 移位寄存器 + 拉低 DE + 标记空闲。
 * 调用者通过 rs485_send_dma() 传入的 data 缓冲区此时可以安全复用。
 */
static void rs485_tx_dma_done(void *user_data)
{
    rs485_port_t port = (rs485_port_t)(uintptr_t)user_data;
    rs485_port_ctx_t *ctx = &s_ports[port];

    /* 等待移位寄存器排空，然后拉低 DE 回到接收模式 */
    bsp_uart_flush(&ctx->uart_cfg);
    bsp_gpio_write(ctx->de_port, ctx->de_port_idx, ctx->de_pin_idx, 0);

    ctx->tx_busy = false;
    ctx->tx_buf = NULL;
    ctx->tx_len = 0;
}

hpm_stat_t rs485_send_dma(rs485_port_t port, const uint8_t *data, uint32_t len)
{
    if (port >= RS485_PORT_MAX || !data || len == 0) {
        return status_fail;
    }

    rs485_port_ctx_t *ctx = &s_ports[port];

    /* DMA 忙则拒绝（RS-485 半双工：TX 期间不应有新请求） */
    if (ctx->tx_busy) {
        return status_fail;
    }

    /* 保存缓冲区引用（零拷贝：DMA 直接从 data 读取） */
    ctx->tx_buf = data;
    ctx->tx_len = len;
    ctx->tx_busy = true;

    /* 刷 D-Cache 确保 DMA 读取到 CPU 写入的最新数据 */
    {
        uint32_t flush_start = HPM_L1C_CACHELINE_ALIGN_DOWN((uint32_t)data);
        uint32_t flush_end   = HPM_L1C_CACHELINE_ALIGN_UP((uint32_t)data + len);
        l1c_dc_flush(flush_start, flush_end - flush_start);
    }

    /* 拉高 DE 进入发送模式，启动 DMA */
    bsp_gpio_write(ctx->de_port, ctx->de_port_idx, ctx->de_pin_idx, 1);
    bsp_uart_send_dma(&ctx->uart_cfg, data, len);

    return status_success;
}

hpm_stat_t rs485_send(rs485_port_t port, const uint8_t *data, uint32_t len)
{
    hpm_stat_t stat = rs485_send_dma(port, data, len);
    if (stat != status_success) {
        return stat;
    }
    return status_success;
}

bool rs485_is_tx_idle(rs485_port_t port)
{
    if (port >= RS485_PORT_MAX)
        return true;

    return !s_ports[port].tx_busy;
}
