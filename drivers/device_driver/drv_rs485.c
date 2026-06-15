/*
 * Copyright (c) 2026 G1_Hand 项目组
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
#define GPIO_PORT_PB 1

#define PB00_PIN 0
#define PB01_PIN 1
#define PB02_PIN 2
#define PB24_PIN 24
#define PB25_PIN 25
#define PB26_PIN 26
#define PB29_PIN 29

/* --------------------------------------------------------------------------
 * DMA 通道分配
 * -------------------------------------------------------------------------- */

#define DMA_CH_UART15_TX 0
#define DMA_CH_UART15_RX 1
#define DMA_CH_UART14_TX 4
#define DMA_CH_UART14_RX 5
#define DMA_CH_UART8_TX 6
#define DMA_CH_UART8_RX 7

/* --------------------------------------------------------------------------
 * 端口上下文
 * -------------------------------------------------------------------------- */

typedef struct {
    bsp_uart_config_t uart_cfg;
    GPIO_Type* de_port;
    uint32_t de_port_idx;
    uint8_t de_pin_idx;

    /* 接收 — 必须缓存行对齐，确保 l1c_dc_flush 不误伤相邻变量 */
    uint8_t rx_dma_buf[RS485_RX_CIRC_BUF_SIZE] __attribute__((aligned(64)));
    uint8_t rx_ring_storage[RS485_RX_RING_BUF_SIZE];
    kfifo_t rx_fifo;
    rs485_rx_callback_t rx_cb;

    /* 发送 — 零拷贝 DMA */
    const uint8_t* tx_buf; /* 当前 DMA 源缓冲区（NULL = 空闲） */
    uint32_t tx_len; /* 当前传输字节数 */
    bool tx_busy; /* 发送流程进行中 */
    volatile bool tx_dma_done; /* HDMA ISR 置位：DMA 搬运完毕 */
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
    case RS485_PORT_MOTOR1: /* UART15: TX=PB31, RX=PB30, DE=PB29 */
        ctx->uart_cfg.base = HPM_UART15;
        ctx->uart_cfg.baudrate = RS485_DEFAULT_BAUDRATE;
        ctx->uart_cfg.clk_name = clock_uart15;
        ctx->uart_cfg.irq_num = IRQn_UART15;
        ctx->uart_cfg.tx_ioc_pad = IOC_PAD_PB31;
        ctx->uart_cfg.tx_ioc_func = IOC_PB31_FUNC_CTL_UART15_TXD;
        ctx->uart_cfg.rx_ioc_pad = IOC_PAD_PB30;
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

    /* 配置引脚复用 (TXD, RXD) */
    HPM_IOC->PAD[ctx->uart_cfg.tx_ioc_pad].FUNC_CTL = ctx->uart_cfg.tx_ioc_func;
    HPM_IOC->PAD[ctx->uart_cfg.rx_ioc_pad].FUNC_CTL = ctx->uart_cfg.rx_ioc_func;

    /*
     * 为 RX 引脚开启内部上拉，防止 RS-485 收发器在发送模式（DE=HIGH）
     * 禁用接收器时 RO 引脚高阻态导致浮空。
     *
     * SIT3088ETK: DE=HIGH → 接收器禁用 → RO=高阻态
     * 若无上拉，MCU 引脚浮空 → UART 读到连续 LOW → 虚假 0x00 字节
     * PE=1 (上拉使能) + PS=1 (上拉) → 引脚保持 HIGH (空闲态)
     */
    HPM_IOC->PAD[ctx->uart_cfg.rx_ioc_pad].PAD_CTL |=
        IOC_PAD_PAD_CTL_PE_MASK | IOC_PAD_PAD_CTL_PS_MASK;

    /* 通用配置 */
    ctx->uart_cfg.dma_controller = HPM_HDMA;
    ctx->uart_cfg.dmamux_controller = HPM_DMAMUX;
    ctx->uart_cfg.rx_circ_buf = ctx->rx_dma_buf;
    ctx->uart_cfg.rx_circ_buf_size = RS485_RX_CIRC_BUF_SIZE;
    ctx->uart_cfg.rx_idle_threshold = RS485_RX_IDLE_THRESHOLD;
    ctx->uart_cfg.rx_callback = rs485_rx_callback;
    ctx->rx_cb = NULL;

    /* DMA 完成回调：标记 tx_dma_done，主循环轮询 TEMT 后拉低 DE */
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

    /* 每次查询接收数据时顺手收尾 TX — 主循环高频调用，DE 及时拉低 */
    rs485_poll_tx(port);

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
 * @brief HDMA ISR 回调：DMA 搬运完毕
 *
 * 仅标记 tx_dma_done，实际收尾由 TX 空闲回调完成。
 * 分离两个事件的原因：DMA TC 表示 FIFO 接收完毕，
 * TX 空闲表示移位寄存器已排空 + 线空闲，两者之间可能差数毫秒。
 */
static void rs485_tx_dma_done(void* user_data)
{
    rs485_port_t port = (rs485_port_t)(uintptr_t)user_data;
    s_ports[port].tx_dma_done = true;
}

/**
 * @brief TX 收尾轮询：检查 TEMT 并拉低 DE
 *
 * 在主循环中调用，非阻塞。当 DMA 完成且移位寄存器排空 (TEMT) 时拉低 DE。
 * TEMT = FIFO 空 + 移位寄存器空，硬件直接反映真实发送状态。
 */
void rs485_poll_tx(rs485_port_t port)
{
    rs485_port_ctx_t* ctx = &s_ports[port];

    if (!ctx->tx_busy || !ctx->tx_dma_done) {
        return;
    }

    if (ctx->uart_cfg.base->LSR & UART_LSR_TEMT_MASK) {
        bsp_gpio_write(ctx->de_port, ctx->de_port_idx, ctx->de_pin_idx, 0);
        ctx->tx_busy = false;
        ctx->tx_dma_done = false;
        ctx->tx_buf = NULL;
        ctx->tx_len = 0;
    }
}

hpm_stat_t rs485_send_dma(rs485_port_t port, const uint8_t* data, uint32_t len)
{
    if (port >= RS485_PORT_MAX || !data || len == 0) {
        return status_fail;
    }

    rs485_port_ctx_t* ctx = &s_ports[port];

    /* 先收尾上一次发送：若 TEMT 已置位则拉低 DE */
    rs485_poll_tx(port);

    /* DMA 忙则拒绝（RS-485 半双工：TX 期间不应有新请求） */
    if (ctx->tx_busy) {
        return status_fail;
    }

    /* 保存缓冲区引用（零拷贝：DMA 直接从 data 读取） */
    ctx->tx_buf = data;
    ctx->tx_len = len;
    ctx->tx_busy = true;
    ctx->tx_dma_done = false; /* 保险：TX 空闲回调必须等 DMA 先完成 */

    /* 刷 D-Cache 确保 DMA 读取到 CPU 写入的最新数据 */
    {
        uint32_t flush_start = HPM_L1C_CACHELINE_ALIGN_DOWN((uint32_t)data);
        uint32_t flush_end = HPM_L1C_CACHELINE_ALIGN_UP((uint32_t)data + len);
        l1c_dc_flush(flush_start, flush_end - flush_start);
    }

    /* 拉高 DE 进入发送模式，启动 DMA */
    bsp_gpio_write(ctx->de_port, ctx->de_port_idx, ctx->de_pin_idx, 1);
    bsp_uart_send_dma(&ctx->uart_cfg, data, len);

    return status_success;
}

hpm_stat_t rs485_send(rs485_port_t port, const uint8_t* data, uint32_t len)
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

    /* 先收尾：若 DMA 完成且移位寄存器已排空，拉低 DE */
    rs485_poll_tx(port);

    return !s_ports[port].tx_busy;
}
