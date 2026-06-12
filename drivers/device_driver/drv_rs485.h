/*
 * Copyright (c) 2024 G1_Hand 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    drv_rs485.h
 * @brief   RS-485 半双工通信驱动
 *
 * 管理 G1_Hand 灵巧手控制器的三路 RS-485 端口：
 * - RS485_PORT_MOTOR1  (UART15, PB31/PB30, DE=PB29)
 * - RS485_PORT_MOTOR2  (UART14, PB24/PB25, DE=PB26)
 * - RS485_PORT_EXT     (UART8,  PB00/PB01, DE=PB02)
 *
 * 通信模式：
 *   TX: 拉高 DE → DMA 直接从调用者缓冲区发送 → 完成后 ISR 拉低 DE（零拷贝，无软件队列）
 *   RX: 持续 DMA 环形接收 + 硬件空闲检测 → 帧完成后触发回调
 */

#ifndef DRV_RS485_H
#define DRV_RS485_H

#include "hpm_common.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------------------------
 * 端口枚举
 * -------------------------------------------------------------------------- */

typedef enum {
    RS485_PORT_MOTOR1 = 0, /**< 电机通讯通道 1 (UART15) */
    RS485_PORT_MOTOR2, /**< 电机通讯通道 2 (UART14) */
    RS485_PORT_EXT, /**< 外部 RS-485 通讯 (UART8) */
    RS485_PORT_MAX
} rs485_port_t;

/* --------------------------------------------------------------------------
 * 默认配置
 * -------------------------------------------------------------------------- */

#define RS485_DEFAULT_BAUDRATE (2000000U)
#define RS485_RX_IDLE_THRESHOLD 10U /**< 接收空闲阈值（位时间） */
#define RS485_RX_CIRC_BUF_SIZE 128U /**< 每端口 DMA 环形缓冲区大小 */
#define RS485_RX_RING_BUF_SIZE 256U /**< 每端口接收环形缓冲区大小 */

/* --------------------------------------------------------------------------
 * 回调类型
 * -------------------------------------------------------------------------- */

/**
 * @brief 接收帧回调类型
 *
 * 当硬件空闲检测到完整帧时，在 UART ISR 上下文中调用。
 * @param port  接收到数据的端口
 * @param data  指向环形缓冲区中接收数据的指针
 * @param len   帧的字节数
 */
typedef void (*rs485_rx_callback_t)(rs485_port_t port, uint8_t* data, uint32_t len);

/* --------------------------------------------------------------------------
 * 初始化
 * -------------------------------------------------------------------------- */

void rs485_init(rs485_port_t port);
void rs485_init_all(void);

/* --------------------------------------------------------------------------
 * 接收
 * -------------------------------------------------------------------------- */

/** @brief 注册接收帧回调（ISR 上下文调用） */
void rs485_set_rx_callback(rs485_port_t port, rs485_rx_callback_t cb);

/** @brief 获取接收队列中可读字节数 */
uint32_t rs485_rx_available(rs485_port_t port);

/** @brief 从接收队列读取数据（应用层调用） */
uint32_t rs485_rx_read(rs485_port_t port, uint8_t* out, uint32_t max_len);

/**
 * @brief RS-485 非阻塞发送（零拷贝 DMA）
 *
 * 若 DMA 空闲则立即启动传输并返回；若 DMA 忙则返回 status_fail。
 * DMA 直接从 data 缓冲区读取，无中间拷贝。完成后 HDMA ISR 自动拉低 DE。
 *
 * @attention data 缓冲区在 DMA 传输期间必须保持有效！
 *            对于阻塞版本 rs485_send() 自动保证，非阻塞版本调用者负责。
 *
 * @param port 发送端口
 * @param data 数据指针（DMA 直接读取，不拷贝）
 * @param len  字节数
 * @return     status_success 若传输已启动，status_fail 若端口忙
 */
hpm_stat_t rs485_send_dma(rs485_port_t port, const uint8_t* data, uint32_t len);

/**
 * @brief RS-485 阻塞发送
 *
 * 调用 rs485_send_dma() 后自旋等待 DMA 完成 → 返回。
 *
 * @param port 发送端口
 * @param data 数据指针
 * @param len  字节数
 * @return     成功返回 status_success
 */
hpm_stat_t rs485_send(rs485_port_t port, const uint8_t* data, uint32_t len);

/**
 * @brief 检查发送管线是否空闲（无 DMA 运行）
 *
 * @param port 端口
 * @return     true 表示空闲
 */
bool rs485_is_tx_idle(rs485_port_t port);

#ifdef __cplusplus
}
#endif

#endif /* DRV_RS485_H */
