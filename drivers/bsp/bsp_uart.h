/*
 * Copyright (c) 2024 G1_Hand 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    bsp_uart.h
 * @brief   BSP UART 抽象层 — 完整 API（阻塞 / DMA / 中断驱动）
 *
 * 提供以下功能：
 * - 阻塞模式收发（轮询，无 DMA，适合短数据）
 * - DMA 非阻塞收发（中断驱动，回调通知完成）
 * - DMA 阻塞收发（启动 DMA 后自旋等待）
 * - DMA 环形接收 + 硬件空闲检测
 * - 中断驱动的 TX 完成通知
 */

#ifndef BSP_UART_H
#define BSP_UART_H

#include "hpm_clock_drv.h"
#include "hpm_dmamux_drv.h"
#include "hpm_dmav2_drv.h"
#include "hpm_uart_drv.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------------------------
 * 回调类型
 * -------------------------------------------------------------------------- */

/** @brief 接收帧回调 — 从 UART ISR 上下文调用 */
typedef void (*bsp_uart_rx_callback_t)(uint8_t* data, uint32_t len);

/** @brief 发送完成回调 — 从 HDMA ISR 上下文调用 */
typedef void (*bsp_uart_tx_callback_t)(void* user_data);

/* --------------------------------------------------------------------------
 * 配置结构体
 * -------------------------------------------------------------------------- */

typedef struct {
    UART_Type* base; /**< UART 基地址（例如 HPM_UART8） */
    uint32_t baudrate; /**< 波特率（例如 115200） */
    uint32_t clk_name; /**< 时钟名称（例如 clock_uart8） */
    uint32_t irq_num; /**< UART 中断号（例如 IRQn_UART8） */

    /* 引脚复用 — 使用方式：HPM_IOC->PAD[tx_ioc_pad].FUNC_CTL = tx_ioc_func */
    uint16_t tx_ioc_pad; /**< TX 引脚的 IOC 焊盘索引 */
    uint16_t tx_ioc_func; /**< TX 引脚的 IOC 功能值 */
    uint16_t rx_ioc_pad; /**< RX 引脚的 IOC 焊盘索引 */
    uint16_t rx_ioc_func; /**< RX 引脚的 IOC 功能值 */

    /* DMA 配置 */
    DMA_Type* dma_controller; /**< DMA 基地址（例如 HPM_HDMA） */
    DMAMUX_Type* dmamux_controller; /**< DMAMUX 基地址（例如 HPM_DMAMUX） */
    uint8_t tx_dma_ch; /**< 发送 DMA 通道号 */
    uint8_t rx_dma_ch; /**< 接收 DMA 通道号 */
    uint8_t tx_dma_req; /**< 发送 DMA 请求源 */
    uint8_t rx_dma_req; /**< 接收 DMA 请求源 */

    /* 接收环形缓冲区（调用者提供，生命周期 >= 驱动） */
    uint8_t* rx_circ_buf; /**< DMA 环形缓冲区 */
    uint32_t rx_circ_buf_size; /**< 环形缓冲区大小（字节数） */

    /* 空闲检测 */
    uint8_t rx_idle_threshold; /**< 接收空闲阈值（位时间，例如 10） */

    /* 回调 */
    bsp_uart_rx_callback_t rx_callback; /**< 接收帧回调（ISR 中调用） */
    bsp_uart_tx_callback_t tx_callback; /**< 发送完成回调（HDMA ISR 中调用） */
    void* tx_user_data; /**< 发送回调用户数据 */
} bsp_uart_config_t;

/* ======================================================================
 * 初始化
 * ====================================================================== */

/**
 * @brief 初始化 UART（时钟 + 引脚 + DMA 环形接收 + 空闲检测 + HDMA 中断）
 *
 * 执行：时钟使能 → 引脚复用 → UART 配置 → DMAMUX → DMA 链表描述符 → 中断使能 → 启动 RX DMA
 *
 * @param cfg 持久配置指针
 */
void bsp_uart_init(bsp_uart_config_t* cfg);

/* ======================================================================
 * DMA 发送
 * ====================================================================== */

/**
 * @brief DMA 非阻塞发送
 *
 * 启动 DMA → 立即返回。DMA 完成后 tx_callback 在 ISR 中被调用。
 * data 缓冲区在 DMA 运行期间必须保持有效。
 *
 * @param cfg  UART 配置
 * @param data 数据缓冲区（DMA 完成前必须有效）
 * @param len  字节数
 * @return     status_success / status_fail
 */
hpm_stat_t bsp_uart_send_dma(bsp_uart_config_t* cfg,
    const uint8_t* data, uint32_t len);

/* ======================================================================
 * 发送状态
 * ====================================================================== */

/**
 * @brief 检查 DMA 发送是否正在进行
 *
 * @param cfg UART 配置
 * @return    true 表示 DMA 仍在传输中
 */
bool bsp_uart_is_tx_busy(bsp_uart_config_t* cfg);

/* ======================================================================
 * 回调管理
 * ====================================================================== */

/**
 * @brief 注册 TX 完成回调
 *
 * @param cfg       UART 配置
 * @param cb        回调函数（NULL 取消注册）
 * @param user_data 回调用户数据
 */
void bsp_uart_set_tx_callback(bsp_uart_config_t* cfg,
    bsp_uart_tx_callback_t cb, void* user_data);

/* ======================================================================
 * 接收 DMA（环形 + 空闲检测）
 * ====================================================================== */

/**
 * @brief 启动 DMA 环形接收
 */
void bsp_uart_start_rx_dma(bsp_uart_config_t* cfg);

/**
 * @brief 获取 DMA 环形缓冲区中已接收字节数
 *
 * @param cfg UART 配置
 * @return    已接收字节数
 */
uint32_t bsp_uart_get_rx_count(bsp_uart_config_t* cfg);

#ifdef __cplusplus
}
#endif

#endif /* BSP_UART_H */
