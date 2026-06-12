/**
 * @file    drv_log_uart.c
 * @author  G1_Hand 项目组
 * @version V1.0.0
 * @date    2025-06-12
 * @brief   日志串口设备驱动实现（UART0 DMA 输出）
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
#include "hpm_iomux.h"
#include "hpm_l1c_drv.h"
#include "hpm_misc.h"
#include "hpm_soc.h"
#include "hpm_uart_drv.h"

/* Private constants ---------------------------------------------------------*/

/* (无) */

/* Private variables ---------------------------------------------------------*/

/* (无) */

/* Private function prototypes -----------------------------------------------*/

/* (无) */

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

/* Private functions ---------------------------------------------------------*/

/* (无) */
