/*
 * Copyright (c) 2026 G1_Hand 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    bsp_uart.c
 * @brief   BSP UART 抽象层实现 — 阻塞 / DMA / 中断驱动
 */

#include "bsp_uart.h"
#include "board.h"
#include "hpm_clock_drv.h"
#include "hpm_common.h"
#include "hpm_interrupt.h"
#include "hpm_iomux.h"
#include "hpm_soc.h"

#define BSP_UART_MAX_INSTANCES 4
#define BSP_UART_DESC_COUNT 2
#define BSP_UART_DESC_ALIGN __attribute__((aligned(8)))

/* 轮询间隔（微秒），用于阻塞收发超时轮询 */
#define BSP_UART_POLL_INTERVAL_US 10U

/**
 * @brief 实例结构体（内部使用）
 */
typedef struct {
    bsp_uart_config_t* cfg;
    dma_linked_descriptor_t rx_descriptors[BSP_UART_DESC_COUNT] BSP_UART_DESC_ALIGN;
    volatile bool tx_done; /* DMA 传输完成标志（ISR 写入） */
    volatile bool rx_idle;
    volatile bool hdma_registered; /* 是否已注册 HDMA ISR */
} bsp_uart_instance_t;

static bsp_uart_instance_t s_instances[BSP_UART_MAX_INSTANCES];
static uint32_t s_instance_count = 0; /** 当前实例数量 */

static bsp_uart_instance_t* find_instance(UART_Type* base)
{
    for (uint32_t i = 0; i < s_instance_count; i++) {
        if (s_instances[i].cfg->base == base) {
            return &s_instances[i];
        }
    }
    return NULL;
}

static bsp_uart_instance_t* find_by_tx_dma_ch(uint8_t ch)
{
    for (uint32_t i = 0; i < s_instance_count; i++) {
        if (s_instances[i].cfg->tx_dma_ch == ch) {
            return &s_instances[i];
        }
    }
    return NULL;
}

static void bsp_uart_init_pins(const bsp_uart_config_t* cfg)
{
    HPM_IOC->PAD[cfg->tx_ioc_pad].FUNC_CTL = cfg->tx_ioc_func;
    HPM_IOC->PAD[cfg->rx_ioc_pad].FUNC_CTL = cfg->rx_ioc_func;
}

/**
 * @brief 启动单次 DMA TX 传输（内部使用）
 */
static hpm_stat_t bsp_uart_start_tx_dma(bsp_uart_config_t* cfg,
    const uint8_t* data, uint32_t len)
{
    dma_channel_config_t tx_cfg = { 0 };
    dma_default_channel_config(cfg->dma_controller, &tx_cfg);
    tx_cfg.src_addr = core_local_mem_to_sys_address(BOARD_RUNNING_CORE,
        (uint32_t)data);
    tx_cfg.dst_addr = (uint32_t)&cfg->base->THR;
    tx_cfg.src_addr_ctrl = DMA_ADDRESS_CONTROL_INCREMENT;
    tx_cfg.dst_addr_ctrl = DMA_ADDRESS_CONTROL_FIXED;
    tx_cfg.src_width = DMA_TRANSFER_WIDTH_BYTE;
    tx_cfg.dst_width = DMA_TRANSFER_WIDTH_BYTE;
    tx_cfg.size_in_byte = len;
    tx_cfg.src_mode = DMA_HANDSHAKE_MODE_NORMAL;
    tx_cfg.src_burst_size = DMA_NUM_TRANSFER_PER_BURST_1T;
    /* 仅开启 TC 中断，屏蔽其他（& MASK_ALL 防止 ~ 污染 CTRL 其他位如 SRC_FIXBURST） */
    tx_cfg.interrupt_mask = DMA_INTERRUPT_MASK_ALL & ~DMA_INTERRUPT_MASK_TERMINAL_COUNT;

    return dma_setup_channel(cfg->dma_controller, cfg->tx_dma_ch, &tx_cfg, true);
}

/* --------------------------------------------------------------------------
 * HDMA 共享 ISR（所有通道共享 IRQn_HDMA = 56）
 * -------------------------------------------------------------------------- */

SDK_DECLARE_EXT_ISR_M(IRQn_HDMA, hdma_isr)
void hdma_isr(void)
{
    DMAV2_Type *dma = HPM_HDMA;

    /* 快照并清除所有通道的挂起中断（W1C），确保退出前中断线拉低 */
    uint32_t tc    = dma->INTTCSTS;
    uint32_t err   = dma->INTERRSTS;
    uint32_t abort = dma->INTABORTSTS;
    dma->INTTCSTS    = tc;
    dma->INTERRSTS   = err;
    dma->INTABORTSTS = abort;

    /* 仅处理我们关心的 TX TC 事件 */
    for (uint32_t i = 0; i < s_instance_count; i++) {
        bsp_uart_instance_t *inst = &s_instances[i];
        bsp_uart_config_t *cfg   = inst->cfg;

        if (tc & (1 << cfg->tx_dma_ch)) {
            inst->tx_done = true;
            if (cfg->tx_callback) {
                cfg->tx_callback(cfg->tx_user_data);
            }
        }
    }
}

/* --------------------------------------------------------------------------
 * UART 空闲检测 ISR（每路 UART 独立）
 * -------------------------------------------------------------------------- */

static void bsp_uart_rx_idle_handler(bsp_uart_instance_t* inst)
{
    bsp_uart_config_t* cfg = inst->cfg;

    if (uart_is_rxline_idle(cfg->base)) {
        uart_clear_rxline_idle_flag(cfg->base);

        uint32_t remaining = dma_get_remaining_transfer_size(cfg->dma_controller,
            cfg->rx_dma_ch);
        uint32_t rx_bytes = cfg->rx_circ_buf_size - remaining;

        if (rx_bytes > 0 && cfg->rx_callback) {
            cfg->rx_callback(cfg->rx_circ_buf, rx_bytes);
        }

        inst->rx_idle = true;
    }
}

SDK_DECLARE_EXT_ISR_M(IRQn_UART8, uart8_isr)
void uart8_isr(void)
{
    bsp_uart_instance_t* inst = find_instance(HPM_UART8);
    if (inst) {
        bsp_uart_rx_idle_handler(inst);
    }
}

SDK_DECLARE_EXT_ISR_M(IRQn_UART1, uart1_isr)
void uart1_isr(void)
{
    bsp_uart_instance_t* inst = find_instance(HPM_UART1);
    if (inst) {
        bsp_uart_rx_idle_handler(inst);
    }
}

SDK_DECLARE_EXT_ISR_M(IRQn_UART14, uart14_isr)
void uart14_isr(void)
{
    bsp_uart_instance_t* inst = find_instance(HPM_UART14);
    if (inst) {
        bsp_uart_rx_idle_handler(inst);
    }
}

SDK_DECLARE_EXT_ISR_M(IRQn_UART15, uart15_isr)
void uart15_isr(void)
{
    bsp_uart_instance_t* inst = find_instance(HPM_UART15);
    if (inst) {
        bsp_uart_rx_idle_handler(inst);
    }
}

void bsp_uart_init(bsp_uart_config_t* cfg)
{
    if (s_instance_count >= BSP_UART_MAX_INSTANCES) {
        return;
    }

    bsp_uart_instance_t* inst = &s_instances[s_instance_count++];
    inst->cfg = cfg;
    inst->tx_done = true;
    inst->rx_idle = false;
    inst->hdma_registered = false;

    /* 使能 UART 时钟 */
    clock_add_to_group(cfg->clk_name, 0);

    /* 配置引脚复用 */
    bsp_uart_init_pins(cfg);

    /* 初始化 UART */
    uart_config_t uart_config = { 0 };
    uart_default_config(cfg->base, &uart_config);
    uart_config.baudrate = cfg->baudrate;
    uart_config.dma_enable = true;
    uart_config.fifo_enable = true;
    uart_config.tx_fifo_level = uart_tx_fifo_trg_not_full;
    uart_config.rx_fifo_level = uart_rx_fifo_trg_not_empty;
    uart_config.src_freq_in_hz = clock_get_frequency(cfg->clk_name);

    /* 硬件接收空闲检测 */
    uart_config.rxidle_config.detect_enable = true;
    uart_config.rxidle_config.detect_irq_enable = true;
    uart_config.rxidle_config.idle_cond = uart_rxline_idle_cond_rxline_logic_one;
    uart_config.rxidle_config.threshold = cfg->rx_idle_threshold;

    if (uart_init(cfg->base, &uart_config) != status_success) {
        return;
    }

    /* 配置 DMAMUX */
    dmamux_config(cfg->dmamux_controller,
        DMA_SOC_CHN_TO_DMAMUX_CHN(cfg->dma_controller, cfg->rx_dma_ch),
        cfg->rx_dma_req, true);
    dmamux_config(cfg->dmamux_controller,
        DMA_SOC_CHN_TO_DMAMUX_CHN(cfg->dma_controller, cfg->tx_dma_ch),
        cfg->tx_dma_req, true);

    /* 构建 DMA 环形接收链表描述符 */
    dma_channel_config_t dma_ch_cfg = { 0 };
    dma_default_channel_config(cfg->dma_controller, &dma_ch_cfg);
    dma_ch_cfg.src_addr = (uint32_t)&cfg->base->RBR;
    dma_ch_cfg.src_addr_ctrl = DMA_ADDRESS_CONTROL_FIXED;
    dma_ch_cfg.dst_addr = core_local_mem_to_sys_address(BOARD_RUNNING_CORE,
        (uint32_t)cfg->rx_circ_buf);
    dma_ch_cfg.dst_addr_ctrl = DMA_ADDRESS_CONTROL_INCREMENT;
    dma_ch_cfg.interrupt_mask = DMA_INTERRUPT_MASK_ALL;  /* RX 环形 DMA 自维持，无需中断 */
    dma_ch_cfg.src_width = DMA_TRANSFER_WIDTH_BYTE;
    dma_ch_cfg.dst_width = DMA_TRANSFER_WIDTH_BYTE;
    dma_ch_cfg.size_in_byte = cfg->rx_circ_buf_size;
    dma_ch_cfg.src_mode = DMA_HANDSHAKE_MODE_HANDSHAKE;
    dma_ch_cfg.src_burst_size = DMA_NUM_TRANSFER_PER_BURST_1T;
    dma_ch_cfg.en_infiniteloop = false;

    for (int i = 0; i < BSP_UART_DESC_COUNT; i++) {
        if (dma_config_linked_descriptor(cfg->dma_controller,
                &inst->rx_descriptors[i],
                cfg->rx_dma_ch,
                &dma_ch_cfg)
            != status_success) {
            return;
        }
    }

    inst->rx_descriptors[0].linked_ptr = core_local_mem_to_sys_address(
        BOARD_RUNNING_CORE, (uint32_t)&inst->rx_descriptors[1]);
    inst->rx_descriptors[1].linked_ptr = core_local_mem_to_sys_address(
        BOARD_RUNNING_CORE, (uint32_t)&inst->rx_descriptors[0]);

    /* 使能 UART 空闲检测中断 */
    intc_m_enable_irq_with_priority(cfg->irq_num, 1);

    /* 注册 HDMA 中断（仅第一次） */
    if (!inst->hdma_registered) {
        intc_m_enable_irq_with_priority(IRQn_HDMA, 1);
        inst->hdma_registered = true;
    }

    /* 启动环形接收 DMA */
    bsp_uart_start_rx_dma(cfg);
}

hpm_stat_t bsp_uart_send_blocking(bsp_uart_config_t* cfg,
    const uint8_t* data, uint32_t len, uint32_t timeout_ms)
{
    if (!cfg || !data || len == 0) {
        return status_fail;
    }

    uint32_t elapsed_us = 0;
    uint32_t timeout_us = timeout_ms * 1000U;

    for (uint32_t i = 0; i < len; i++) {
        /* 等待 TX FIFO 有空位 */
        while (!(cfg->base->LSR & UART_LSR_THRE_MASK)) {
            if (timeout_ms > 0 && elapsed_us >= timeout_us) {
                return status_timeout;
            }
            clock_cpu_delay_us(BSP_UART_POLL_INTERVAL_US);
            elapsed_us += BSP_UART_POLL_INTERVAL_US;
        }
        cfg->base->THR = UART_THR_THR_SET(data[i]);
    }

    return status_success;
}

hpm_stat_t bsp_uart_recv_blocking(bsp_uart_config_t* cfg,
    uint8_t* data, uint32_t len, uint32_t timeout_ms)
{
    if (!cfg || !data || len == 0) {
        return status_fail;
    }

    uint32_t elapsed_us = 0;
    uint32_t timeout_us = timeout_ms * 1000U;

    for (uint32_t i = 0; i < len; i++) {
        /* 等待 RX 数据就绪 */
        while (!(cfg->base->LSR & UART_LSR_DR_MASK)) {
            if (timeout_ms > 0 && elapsed_us >= timeout_us) {
                return status_timeout;
            }
            clock_cpu_delay_us(BSP_UART_POLL_INTERVAL_US);
            elapsed_us += BSP_UART_POLL_INTERVAL_US;
        }
        data[i] = (uint8_t)(cfg->base->RBR & UART_RBR_RBR_MASK);
    }

    return status_success;
}

hpm_stat_t bsp_uart_send_dma(bsp_uart_config_t* cfg,
    const uint8_t* data, uint32_t len)
{
    bsp_uart_instance_t* inst = find_instance(cfg->base);
    if (!inst) {
        return status_fail;
    }

    inst->tx_done = false;
    return bsp_uart_start_tx_dma(cfg, data, len);
}

hpm_stat_t bsp_uart_send_dma_blocking(bsp_uart_config_t* cfg,
    const uint8_t* data, uint32_t len)
{
    hpm_stat_t stat = bsp_uart_send_dma(cfg, data, len);
    if (stat != status_success) {
        return stat;
    }

    while (bsp_uart_is_tx_busy(cfg)) {
        /* 自旋等待 DMA 完成 */
    }

    return status_success;
}

bool bsp_uart_is_tx_busy(bsp_uart_config_t* cfg)
{
    bsp_uart_instance_t* inst = find_instance(cfg->base);
    if (!inst) {
        return false;
    }

    /* tx_done 由 HDMA ISR 设置 */
    return !inst->tx_done;
}

void bsp_uart_flush(bsp_uart_config_t* cfg)
{
    uart_flush(cfg->base);
}

void bsp_uart_set_tx_callback(bsp_uart_config_t* cfg,
    bsp_uart_tx_callback_t cb, void* user_data)
{
    cfg->tx_callback = cb;
    cfg->tx_user_data = user_data;
}

void bsp_uart_start_rx_dma(bsp_uart_config_t* cfg)
{
    bsp_uart_instance_t* inst = find_instance(cfg->base);
    if (!inst) {
        return;
    }

    cfg->dma_controller->CHCTRL[cfg->rx_dma_ch].LLPOINTER = core_local_mem_to_sys_address(BOARD_RUNNING_CORE,
        (uint32_t)&inst->rx_descriptors[0]);
    dma_enable_channel(cfg->dma_controller, cfg->rx_dma_ch);
}

uint32_t bsp_uart_get_rx_count(bsp_uart_config_t* cfg)
{
    uint32_t remaining = dma_get_remaining_transfer_size(cfg->dma_controller,
        cfg->rx_dma_ch);
    return cfg->rx_circ_buf_size - remaining;
}
