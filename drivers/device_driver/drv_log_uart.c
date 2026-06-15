/**
 * @file    drv_log_uart.c
 * @author  G1_Hand 项目组
 * @version V1.1.0
 * @date    2025-06-12
 * @brief   UART0 设备驱动实现（DMA 输出 + DMA circle 模式接收）
 * @attention
 *
 * Copyright (c) 2025 G1_Hand 项目组.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_log_uart.h"

#include "board.h"
#include "hpm_clock_drv.h"
#include "hpm_dmamux_drv.h"
#include "hpm_dmav2_drv.h"
#include "hpm_interrupt.h"
#include "hpm_iomux.h"
#include "hpm_l1c_drv.h"
#include "hpm_misc.h"
#include "hpm_soc.h"
#include "hpm_uart_drv.h"

#include "kfifo.h"

#include <string.h>

/* Private constants ---------------------------------------------------------*/

/** @brief DMA circle 接收缓冲区大小（字节） */
#define DRV_LOG_UART_RX_CIRC_BUF_SIZE   (128U)

/** @brief kfifo 缓冲区大小（字节，须为2的幂） */
#define DRV_LOG_UART_RX_FIFO_SIZE       (256U)

/** @brief 空闲检测阈值（位时间） */
#define DRV_LOG_UART_RX_IDLE_THRESHOLD  (10U)

/** @brief 链接描述符数量（成环最小需要2个） */
#define DRV_LOG_UART_RX_DESC_COUNT      (2U)

/* Private variables ---------------------------------------------------------*/

/** @brief DMA circle 缓冲区（64字节对齐，保证 DCache 操作不误伤相邻变量） */
static uint8_t s_rx_dma_buf[DRV_LOG_UART_RX_CIRC_BUF_SIZE] __attribute__((aligned(64)));

/** @brief kfifo 后端存储 */
static uint8_t s_rx_ring_storage[DRV_LOG_UART_RX_FIFO_SIZE];

/** @brief 接收 FIFO 实例 */
static kfifo_t s_rx_fifo;

/** @brief DMA 链接描述符（8字节对齐），成环：0→1→0 */
static dma_linked_descriptor_t s_rx_descriptors[DRV_LOG_UART_RX_DESC_COUNT]
    __attribute__((aligned(8)));

/** @brief RX 是否已初始化（防止重复初始化 ISR） */
static bool s_rx_initialized = false;

/* Private function prototypes -----------------------------------------------*/

/* (ISR 函数声明由 SDK_DECLARE_EXT_ISR_M 宏完成) */

/**
 * @brief UART0 空闲检测 ISR
 *
 * 当 UART0 RX 线空闲（持续高电平 ≥ threshold 位时间）时触发，
 * 从 DMA circle 缓冲区取出已接收数据并写入 kfifo。
 *
 * 双路径设计：
 *   路径1 (DMA)：从 s_rx_dma_buf 读取 DMA 已搬运的数据
 *   路径2 (RBR)：直接轮询 UART RBR 寄存器（兜底，防止 DMAMUX 延迟丢数据）
 */
SDK_DECLARE_EXT_ISR_M(IRQn_UART0, uart0_rx_isr)
void uart0_rx_isr(void)
{
    if (!uart_is_rxline_idle(HPM_UART0)) {
        return;
    }

    /* 清除空闲标志（W1C） */
    uart_clear_rxline_idle_flag(HPM_UART0);

    /*
     * 路径1：计算 DMA 已搬运的字节数
     *
     * dma_get_remaining_transfer_size 返回当前描述符剩余的传输次数。
     * 由于 src_width=BYTE(1字节)，remaining 单位就是字节。
     * DMA 已写字节 = 缓冲区大小 - 剩余字节
     */
    uint32_t dma_remaining = dma_get_remaining_transfer_size(HPM_HDMA,
        DRV_LOG_UART_DEFAULT_RX_DMA_CH);
    uint32_t dma_rx_bytes = DRV_LOG_UART_RX_CIRC_BUF_SIZE - dma_remaining;

    /*
     * 路径2：直读 UART RBR 寄存器（兜底）
     *
     * 当 DMAMUX 响应延迟导致 RBR FIFO 中仍有数据未被 DMA 搬走时，
     * 直接从 RBR 读出，防止数据丢失。
     */
    uint8_t direct_buf[64];
    uint32_t direct_len = 0;
    while ((HPM_UART0->LSR & UART_LSR_DR_MASK) && direct_len < sizeof(direct_buf)) {
        direct_buf[direct_len++] = (uint8_t)(HPM_UART0->RBR & UART_RBR_RBR_MASK);
    }

    if (direct_len > 0 && direct_len <= DRV_LOG_UART_RX_CIRC_BUF_SIZE) {
        /* 直读路径优先：将 RBR 数据复制到 DMA 缓冲区，再写入 kfifo */
        memcpy(s_rx_dma_buf, direct_buf, direct_len);
        kfifo_put(&s_rx_fifo, s_rx_dma_buf, direct_len);
    } else if (dma_rx_bytes > 0 && dma_rx_bytes <= DRV_LOG_UART_RX_CIRC_BUF_SIZE) {
        /*
         * DMA 路径：D-Cache 失效后读取 DMA 写入的数据
         *
         * DMA 通过系统总线写入 s_rx_dma_buf，CPU 可能持有该地址范围的
         * 陈旧缓存行。必须使失效后才能看到 DMA 写入的最新数据。
         */
        uint32_t align_start = HPM_L1C_CACHELINE_ALIGN_DOWN(
            (uint32_t)s_rx_dma_buf);
        uint32_t align_end = HPM_L1C_CACHELINE_ALIGN_UP(
            (uint32_t)s_rx_dma_buf + dma_rx_bytes);
        l1c_dc_invalidate(align_start, align_end - align_start);

        kfifo_put(&s_rx_fifo, s_rx_dma_buf, dma_rx_bytes);
    }
}

/* Exported functions --------------------------------------------------------*/

/**
 * @brief 初始化日志串口驱动
 * @param ctx 驱动上下文指针
 * @param config 配置结构体指针
 * @return 操作结果错误码
 */
drv_log_uart_error_t drv_log_uart_init(drv_log_uart_context_t* ctx,
    const drv_log_uart_config_t* config)
{
    if (!ctx || !config) {
        return DRV_LOG_UART_ERROR_NULL_PTR;
    }

    if (ctx->initialized) {
        drv_log_uart_deinit(ctx);
    }

    /* 保存配置 */
    ctx->config = *config;
    ctx->tx_busy = false;

    /* 配置 UART0 引脚: PA00=TX, PA01=RX */
    HPM_IOC->PAD[IOC_PAD_PA00].FUNC_CTL = IOC_PA00_FUNC_CTL_UART0_TXD;
    HPM_IOC->PAD[IOC_PAD_PA01].FUNC_CTL = IOC_PA01_FUNC_CTL_UART0_RXD;

    /* 使能 UART0 时钟 */
    clock_add_to_group(clock_uart0, 0);

    /* 初始化 UART0（DMA 模式） */
    uart_config_t uart_cfg = { 0 };
    uart_default_config(HPM_UART0, &uart_cfg);
    uart_cfg.src_freq_in_hz = clock_get_frequency(clock_uart0);
    uart_cfg.baudrate = config->baudrate;
    uart_cfg.dma_enable = true;
    uart_cfg.fifo_enable = true;
    uart_cfg.tx_fifo_level = uart_tx_fifo_trg_not_full;
    uart_cfg.num_of_stop_bits = stop_bits_1;
    uart_cfg.word_length = word_length_8_bits;
    uart_cfg.parity = parity_none;

    uart_init(HPM_UART0, &uart_cfg);

    /* 配置 DMAMUX：将 UART0 TX DMA 请求路由到指定 DMA 通道 */
    dmamux_config(HPM_DMAMUX,
        DMA_SOC_CHN_TO_DMAMUX_CHN(HPM_HDMA, config->tx_dma_ch),
        HPM_DMA_SRC_UART0_TX, true);

    /*
     * --- RX DMA circle 模式初始化 ---
     *
     * 依赖：UART0 基础初始化已完成（时钟、引脚、uart_init）
     * 说明：RX 仅初始化一次（s_rx_initialized 防重入），
     *       circle DMA 启动后永不停机，数据通过 kfifo 交给上层消费。
     */
    if (!s_rx_initialized) {
        /* RX kfifo 初始化 */
        kfifo_init(&s_rx_fifo, s_rx_ring_storage,
            DRV_LOG_UART_RX_FIFO_SIZE, NULL);

        /* 使能 UART0 接收空闲检测 */
        uart_rxline_idle_config_t idle_cfg;
        idle_cfg.detect_enable = true;
        idle_cfg.detect_irq_enable = true;
        idle_cfg.idle_cond = uart_rxline_idle_cond_rxline_logic_one;
        idle_cfg.threshold = DRV_LOG_UART_RX_IDLE_THRESHOLD;

        if (uart_init_rxline_idle_detection(HPM_UART0, idle_cfg) != status_success) {
            ctx->initialized = true; /* TX 仍可用 */
            return DRV_LOG_UART_OK;
        }

        /* 配置 DMAMUX：将 UART0 RX DMA 请求路由到 RX DMA 通道 */
        dmamux_config(HPM_DMAMUX,
            DMA_SOC_CHN_TO_DMAMUX_CHN(HPM_HDMA, config->rx_dma_ch),
            HPM_DMA_SRC_UART0_RX, true);

        /* 构建 DMA 环形接收链表描述符 (0→1→0 成环) */
        dma_channel_config_t dma_ch_cfg = { 0 };
        dma_default_channel_config(HPM_HDMA, &dma_ch_cfg);

        dma_ch_cfg.src_addr = (uint32_t)&HPM_UART0->RBR;
        dma_ch_cfg.src_addr_ctrl = DMA_ADDRESS_CONTROL_FIXED;
        dma_ch_cfg.dst_addr = core_local_mem_to_sys_address(BOARD_RUNNING_CORE,
            (uint32_t)s_rx_dma_buf);
        dma_ch_cfg.dst_addr_ctrl = DMA_ADDRESS_CONTROL_INCREMENT;
        dma_ch_cfg.src_width = DMA_TRANSFER_WIDTH_BYTE;
        dma_ch_cfg.dst_width = DMA_TRANSFER_WIDTH_BYTE;
        dma_ch_cfg.size_in_byte = DRV_LOG_UART_RX_CIRC_BUF_SIZE;
        dma_ch_cfg.src_mode = DMA_HANDSHAKE_MODE_HANDSHAKE;
        dma_ch_cfg.src_burst_size = DMA_NUM_TRANSFER_PER_BURST_1T;
        dma_ch_cfg.interrupt_mask = DMA_INTERRUPT_MASK_ALL;
        dma_ch_cfg.en_infiniteloop = false;

        for (int i = 0; i < DRV_LOG_UART_RX_DESC_COUNT; i++) {
            if (dma_config_linked_descriptor(HPM_HDMA,
                    &s_rx_descriptors[i],
                    config->rx_dma_ch,
                    &dma_ch_cfg) != status_success) {
                ctx->initialized = true; /* TX 仍可用 */
                return DRV_LOG_UART_OK;
            }
        }

        /* 链接成环：0 → 1 → 0 */
        s_rx_descriptors[0].linked_ptr = core_local_mem_to_sys_address(
            BOARD_RUNNING_CORE, (uint32_t)&s_rx_descriptors[1]);
        s_rx_descriptors[1].linked_ptr = core_local_mem_to_sys_address(
            BOARD_RUNNING_CORE, (uint32_t)&s_rx_descriptors[0]);

        /* 刷新描述符 D-Cache，确保 DMA 读到最新的 linked_ptr 环 */
        {
            uint32_t desc_align_start = HPM_L1C_CACHELINE_ALIGN_DOWN(
                (uint32_t)&s_rx_descriptors[0]);
            uint32_t desc_align_end = HPM_L1C_CACHELINE_ALIGN_UP(
                (uint32_t)&s_rx_descriptors[0] + sizeof(s_rx_descriptors));
            l1c_dc_flush(desc_align_start, desc_align_end - desc_align_start);
        }

        /* 注册 UART0 空闲检测中断 */
        intc_m_enable_irq_with_priority(IRQn_UART0, 1);

        /* 启动 RX circle DMA */
        HPM_HDMA->CHCTRL[config->rx_dma_ch].LLPOINTER =
            core_local_mem_to_sys_address(BOARD_RUNNING_CORE,
                (uint32_t)&s_rx_descriptors[0]);
        dma_enable_channel(HPM_HDMA, config->rx_dma_ch);

        s_rx_initialized = true;
    }

    ctx->initialized = true;

    return DRV_LOG_UART_OK;
}

/**
 * @brief 反初始化日志串口驱动
 * @param ctx 驱动上下文指针
 */
void drv_log_uart_deinit(drv_log_uart_context_t* ctx)
{
    if (!ctx) {
        return;
    }

    dma_disable_channel(HPM_HDMA, ctx->config.tx_dma_ch);
    ctx->tx_busy = false;
    ctx->initialized = false;
}

/**
 * @brief 检查驱动是否已初始化
 * @param ctx 驱动上下文指针
 * @return true表示已初始化，false表示未初始化
 */
bool drv_log_uart_is_initialized(const drv_log_uart_context_t* ctx)
{
    return ctx ? ctx->initialized : false;
}

/**
 * @brief 轮询处理：检查 TX DMA 是否完成
 * @param ctx 驱动上下文指针
 * @return 操作结果错误码
 */
drv_log_uart_error_t drv_log_uart_poll(drv_log_uart_context_t* ctx)
{
    if (!ctx) {
        return DRV_LOG_UART_ERROR_NULL_PTR;
    }

    if (!ctx->initialized) {
        return DRV_LOG_UART_ERROR_UNINITIALIZED;
    }

    /* 若 TX 忙碌、DMA 已停、且 UART 移位寄存器已空，则传输完成 */
    if (ctx->tx_busy
        && !dma_channel_is_enable(HPM_HDMA, ctx->config.tx_dma_ch)
        && uart_check_status(HPM_UART0, uart_stat_transmitter_empty)) {
        ctx->tx_busy = false;
    }

    return DRV_LOG_UART_OK;
}

/**
 * @brief 非阻塞 DMA 发送
 * @param ctx 驱动上下文指针
 * @param data 数据指针
 * @param len 数据长度
 * @return 操作结果错误码
 */
drv_log_uart_error_t drv_log_uart_send(drv_log_uart_context_t* ctx,
    const uint8_t* data, uint32_t len)
{
    if (!ctx || !data) {
        return DRV_LOG_UART_ERROR_NULL_PTR;
    }

    if (!ctx->initialized) {
        return DRV_LOG_UART_ERROR_UNINITIALIZED;
    }

    if (len == 0) {
        return DRV_LOG_UART_OK;
    }

    if (ctx->tx_busy) {
        return DRV_LOG_UART_ERROR_TX_BUSY;
    }

    /* 刷新 D-Cache，确保 DMA 读到 CPU 写入的数据 */
    {
        uint32_t flush_start = HPM_L1C_CACHELINE_ALIGN_DOWN((uint32_t)data);
        uint32_t flush_end = HPM_L1C_CACHELINE_ALIGN_UP((uint32_t)data + len);
        l1c_dc_flush(flush_start, flush_end - flush_start);
    }

    /* 配置 TX DMA：目标侧使用 HANDSHAKE 模式（UART FIFO 有空位才写） */
    dma_channel_config_t tx_cfg = { 0 };
    dma_default_channel_config(HPM_HDMA, &tx_cfg);

    tx_cfg.src_addr = core_local_mem_to_sys_address(BOARD_RUNNING_CORE,
        (uint32_t)data);
    tx_cfg.dst_addr = (uint32_t)&HPM_UART0->THR;
    tx_cfg.src_addr_ctrl = DMA_ADDRESS_CONTROL_INCREMENT;
    tx_cfg.dst_addr_ctrl = DMA_ADDRESS_CONTROL_FIXED;
    tx_cfg.src_width = DMA_TRANSFER_WIDTH_BYTE;
    tx_cfg.dst_width = DMA_TRANSFER_WIDTH_BYTE;
    tx_cfg.size_in_byte = len;
    tx_cfg.src_mode = DMA_HANDSHAKE_MODE_NORMAL;
    tx_cfg.dst_mode = DMA_HANDSHAKE_MODE_HANDSHAKE; /* ← 关键：外设控制流速 */
    tx_cfg.src_burst_size = DMA_NUM_TRANSFER_PER_BURST_1T;
    tx_cfg.interrupt_mask = DMA_INTERRUPT_MASK_ALL;

    dma_setup_channel(HPM_HDMA, ctx->config.tx_dma_ch, &tx_cfg, true);
    ctx->tx_busy = true;

    return DRV_LOG_UART_OK;
}

/**
 * @brief 查询 TX DMA 是否忙碌
 * @param ctx 驱动上下文指针
 * @return true表示正在发送
 */
bool drv_log_uart_is_tx_busy(const drv_log_uart_context_t* ctx)
{
    if (!ctx || !ctx->initialized) {
        return false;
    }

    return ctx->tx_busy;
}

/**
 * @brief 从接收 FIFO 读取数据
 * @param buf     目标缓冲区
 * @param max_len 最大读取字节数
 * @return 实际读取的字节数
 */
uint32_t drv_log_uart_rx_read(uint8_t* buf, uint32_t max_len)
{
    if (!buf || max_len == 0) {
        return 0;
    }

    return kfifo_get(&s_rx_fifo, buf, max_len);
}

/**
 * @brief 查询接收 FIFO 中可读字节数
 * @return 可读字节数
 */
uint32_t drv_log_uart_rx_available(void)
{
    return kfifo_len(&s_rx_fifo);
}

/* Private functions ---------------------------------------------------------*/

/* (无) */
