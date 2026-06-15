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
#include <string.h>
#include "hpm_l1c_drv.h"

#define BSP_UART_MAX_INSTANCES 4
#define BSP_UART_DESC_COUNT 2
#define BSP_UART_DESC_ALIGN __attribute__((aligned(8)))

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
    DMAV2_Type* dma = HPM_HDMA;

    /* 快照并清除所有通道的挂起中断（W1C），确保退出前中断线拉低 */
    uint32_t tc = dma->INTTCSTS;
    uint32_t err = dma->INTERRSTS;
    uint32_t abort = dma->INTABORTSTS;
    dma->INTTCSTS = tc;
    dma->INTERRSTS = err;
    dma->INTABORTSTS = abort;

    /* 仅处理我们关心的 TX TC 事件 */
    for (uint32_t i = 0; i < s_instance_count; i++) {
        bsp_uart_instance_t* inst = &s_instances[i];
        bsp_uart_config_t* cfg = inst->cfg;

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

        /*
         * 双路径 RX：
         * 路径1 (DMA)：dma_get_remaining → 从 DMA 环形缓冲读
         * 路径2 (直读)：直接轮询 UART RBR（当 DMA 未捕获数据时的后备）
         *
         * 若 UART RX FIFO 中仍有数据而 DMA 未搬走（DMAMUX 配置问题等），
         * 路径2 直接读 RBR 确保数据不丢失。
         */
        uint32_t dma_remaining = dma_get_remaining_transfer_size(cfg->dma_controller,
            cfg->rx_dma_ch);
        uint32_t dma_rx_bytes = cfg->rx_circ_buf_size - dma_remaining;

        /* 路径2: 直读 UART RBR 中 DMA 未捕获的残留数据 */
        uint8_t direct_buf[64];
        uint32_t direct_len = 0;
        while ((cfg->base->LSR & UART_LSR_DR_MASK) && direct_len < sizeof(direct_buf)) {
            direct_buf[direct_len++] = (uint8_t)(cfg->base->RBR & UART_RBR_RBR_MASK);
        }

        /*
         * 优先使用直读 RBR（绕过 DMA），仅当 RBR 空时才用 DMA 数据。
         * DMA 搬回来的全是 0x00 而 RBR 有真实数据时，直读胜出。
         *
         * 注意：rx_callback (rs485_rx_callback) 通过 data 指针是否落在
         * rx_dma_buf 范围内来识别端口。直读数据在栈上，必须先拷入 DMA 缓冲
         * 区，否则回调中 ctx=NULL 导致数据被丢弃。
         */
        if (direct_len > 0) {
            if (direct_len <= cfg->rx_circ_buf_size) {
                memcpy(cfg->rx_circ_buf, direct_buf, direct_len);
                if (cfg->rx_callback) {
                    cfg->rx_callback(cfg->rx_circ_buf, direct_len);
                }
            }
        } else if (dma_rx_bytes > 0 && dma_rx_bytes <= cfg->rx_circ_buf_size) {
            uint32_t align_start = HPM_L1C_CACHELINE_ALIGN_DOWN((uint32_t)cfg->rx_circ_buf);
            uint32_t align_end = HPM_L1C_CACHELINE_ALIGN_UP((uint32_t)cfg->rx_circ_buf + dma_rx_bytes);
            l1c_dc_invalidate(align_start, align_end - align_start);
            if (cfg->rx_callback) {
                cfg->rx_callback(cfg->rx_circ_buf, dma_rx_bytes);
            }
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
    dma_ch_cfg.interrupt_mask = DMA_INTERRUPT_MASK_ALL; /* RX 环形 DMA 自维持，无需中断 */
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

    /* 刷新描述符 D-Cache，确保 DMA 读到最新的 linked_ptr 环（防止缓存行滞后导致链表断裂） */
    {
        uint32_t desc_align_start = HPM_L1C_CACHELINE_ALIGN_DOWN(
            (uint32_t)&inst->rx_descriptors[0]);
        uint32_t desc_align_end = HPM_L1C_CACHELINE_ALIGN_UP(
            (uint32_t)&inst->rx_descriptors[0] + sizeof(inst->rx_descriptors));
        l1c_dc_flush(desc_align_start, desc_align_end - desc_align_start);
    }

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

bool bsp_uart_is_tx_busy(bsp_uart_config_t* cfg)
{
    bsp_uart_instance_t* inst = find_instance(cfg->base);
    if (!inst) {
        return false;
    }

    /* tx_done 由 HDMA ISR 设置 */
    return !inst->tx_done;
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

    /*
     * RS-485 半双工：TX 期间接收器禁用，DMA 可能捕获噪声数据。
     * 必须完整复位通道 —— disable → clear_status → invalidate → flush_desc → restart。
     * 参照 uart0_rx_task 模式，但 uart0 从不重启（circle 永远运行），
     * RS-485 每次 TX 完成后需重新初始化 RX DMA。
     */

    /* 1. 停掉当前通道 */
    dma_disable_channel(cfg->dma_controller, cfg->rx_dma_ch);

    /* 2. 清除残留传输状态（TC/Error/Abort），防止干扰后续传输 */
    dma_clear_transfer_status(cfg->dma_controller, cfg->rx_dma_ch);

    /* 3. 使 DMA 缓冲区 D-Cache 失效（清除 TX 期间捕获的噪声/残留数据） */
    {
        uint32_t align_start = HPM_L1C_CACHELINE_ALIGN_DOWN(
            (uint32_t)cfg->rx_circ_buf);
        uint32_t align_end = HPM_L1C_CACHELINE_ALIGN_UP(
            (uint32_t)cfg->rx_circ_buf + cfg->rx_circ_buf_size);
        l1c_dc_invalidate(align_start, align_end - align_start);
    }

    /* 4. 刷新描述符 D-Cache，确保 DMA 读到最新的 linked_ptr 环 */
    {
        uint32_t align_start = HPM_L1C_CACHELINE_ALIGN_DOWN(
            (uint32_t)&inst->rx_descriptors[0]);
        uint32_t align_end = HPM_L1C_CACHELINE_ALIGN_UP(
            (uint32_t)&inst->rx_descriptors[0] + sizeof(inst->rx_descriptors));
        l1c_dc_flush(align_start, align_end - align_start);
    }

    /* 5. 重新加载第一个描述符并启动 circle DMA */
    cfg->dma_controller->CHCTRL[cfg->rx_dma_ch].LLPOINTER = core_local_mem_to_sys_address(
        BOARD_RUNNING_CORE, (uint32_t)&inst->rx_descriptors[0]);
    dma_enable_channel(cfg->dma_controller, cfg->rx_dma_ch);
}

uint32_t bsp_uart_get_rx_count(bsp_uart_config_t* cfg)
{
    uint32_t remaining = dma_get_remaining_transfer_size(cfg->dma_controller,
        cfg->rx_dma_ch);
    return cfg->rx_circ_buf_size - remaining;
}
