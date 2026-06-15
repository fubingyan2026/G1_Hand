/**
 * @file    drv_log_uart.h
 * @author  G1_Hand 项目组
 * @version V1.1.0
 * @date    2025-06-12
 * @brief   UART0 设备驱动（DMA 输出 + DMA circle 模式接收）
 * @attention
 *
 * Copyright (c) 2025 G1_Hand 项目组.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 *
 */

#ifndef __DRV_LOG_UART_H
#define __DRV_LOG_UART_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/**
 * @brief UART0 驱动错误码枚举
 */
typedef enum {
    DRV_LOG_UART_OK = 0,              /**< 操作成功 */
    DRV_LOG_UART_ERROR_NULL_PTR,      /**< 空指针错误 */
    DRV_LOG_UART_ERROR_INVALID_PARAM, /**< 无效参数 */
    DRV_LOG_UART_ERROR_UNINITIALIZED, /**< 未初始化 */
    DRV_LOG_UART_ERROR_TX_BUSY,       /**< TX DMA 忙 */
} drv_log_uart_error_t;

/**
 * @brief UART0 驱动配置结构体
 */
typedef struct {
    uint32_t baudrate;             /**< 波特率（默认 2000000） */
    uint8_t tx_dma_ch;             /**< TX DMA 通道号 */
    uint8_t rx_dma_ch;             /**< RX DMA 通道号（circle模式） */
} drv_log_uart_config_t;

/**
 * @brief UART0 驱动上下文结构体
 */
typedef struct {
    drv_log_uart_config_t config;  /**< 配置参数 */
    volatile bool tx_busy;         /**< TX DMA 传输中标志 */
    bool initialized;              /**< 初始化标志 */
} drv_log_uart_context_t;

/* Exported constants --------------------------------------------------------*/

#define DRV_LOG_UART_DEFAULT_BAUDRATE    (2000000UL)
#define DRV_LOG_UART_DEFAULT_TX_DMA_CH   (2)
#define DRV_LOG_UART_DEFAULT_RX_DMA_CH   (3)

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/* --- 初始化 / 生命周期 --- */

drv_log_uart_error_t drv_log_uart_init(drv_log_uart_context_t* ctx,
    const drv_log_uart_config_t* config);
void drv_log_uart_deinit(drv_log_uart_context_t* ctx);
bool drv_log_uart_is_initialized(const drv_log_uart_context_t* ctx);

/* --- TX（日志输出） --- */

/**
 * @brief 轮询处理：检查 TX DMA 是否完成并更新内部状态
 * @param ctx 驱动上下文指针
 * @return 操作结果错误码
 */
drv_log_uart_error_t drv_log_uart_poll(drv_log_uart_context_t* ctx);

/**
 * @brief 非阻塞 DMA 发送（dst_mode=HANDSHAKE，UART 流控）
 * @param ctx 驱动上下文指针
 * @param data 数据指针
 * @param len 数据长度
 * @return 操作结果错误码
 */
drv_log_uart_error_t drv_log_uart_send(drv_log_uart_context_t* ctx,
    const uint8_t* data, uint32_t len);

bool drv_log_uart_is_tx_busy(const drv_log_uart_context_t* ctx);

/* --- RX（DMA circle + kfifo） --- */

/**
 * @brief 从接收 FIFO 读取数据
 * @param buf  目标缓冲区
 * @param max_len 最大读取字节数
 * @return 实际读取的字节数
 */
uint32_t drv_log_uart_rx_read(uint8_t* buf, uint32_t max_len);

/**
 * @brief 查询接收 FIFO 中可读字节数
 * @return 可读字节数
 */
uint32_t drv_log_uart_rx_available(void);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_LOG_UART_H */
