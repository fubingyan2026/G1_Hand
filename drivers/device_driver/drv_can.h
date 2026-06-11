/*
 * Copyright (c) 2026 G1_Hand 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    drv_can.h
 * @author  G1_Hand
 * @version 1.1.0
 * @brief   CAN-FD 设备驱动 — 多实例支持，中断接收 + kfifo 缓冲 + 阻塞/非阻塞发送
 *
 * @attention
 * - 使用 context/config 双结构体模式，所有 API 通过上下文指针操作实例
 * - 每个 MCAN 实例独立配置 (引脚、时钟、波特率、STB)
 * - 支持 CAN-FD (仲裁段/数据段双波特率 + BRS + TDC)
 */

#ifndef __DRV_CAN_H
#define __DRV_CAN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "hpm_clock_drv.h"
#include "hpm_common.h"
#include "hpm_mcan_drv.h"

/* Exported constants --------------------------------------------------------*/

/** @brief 最大 CAN 实例数 */
#define DRV_CAN_MAX_INSTANCES (6U)

/** @brief 默认接收 kfifo 大小 (字节, 必须为 2 的幂) */
#define DRV_CAN_DEFAULT_RX_FIFO_SIZE (256U)

/** @brief 默认 CAN-FD 仲裁段波特率 (1Mbps) */
#define DRV_CAN_DEFAULT_NOMINAL_BAUDRATE (1000000U)

/** @brief 默认 CAN-FD 数据段波特率 (5Mbps) */
#define DRV_CAN_DEFAULT_DATA_BAUDRATE (5000000U)

/* Exported types ------------------------------------------------------------*/

/**
 * @brief CAN 驱动上下文 (运行时状态, 不透明指针)
 */
typedef struct drv_can_context drv_can_context_t;

/**
 * @brief CAN 驱动配置结构体 (初始化参数, 初始化后不变)
 */
typedef struct {
    const char* name; /**< 实例名称 (调试用) */

    /* MCAN 硬件参数 */
    MCAN_Type* base; /**< MCAN 基地址 (如 HPM_MCAN4) */
    uint32_t irq_num; /**< 中断号 (如 IRQn_MCAN4) */
    clock_name_t clock_name; /**< 时钟名称 (如 clock_can4) */
    mcan_node_mode_t mode; /**< CAN 节点模式 (如 mcan_mode_normal) */

    /* 波特率 */
    uint32_t nominal_baudrate; /**< 仲裁段波特率 (bps) */
    uint32_t data_baudrate; /**< 数据段波特率 (bps, CAN-FD) */

    /* TXD 引脚 */
    uint16_t txd_ioc_pad; /**< TXD IOC 焊盘 (如 IOC_PAD_PA16) */
    uint32_t txd_ioc_func; /**< TXD IOC 功能 (如 IOC_PA16_FUNC_CTL_MCAN4_TXD) */

    /* RXD 引脚 */
    uint16_t rxd_ioc_pad; /**< RXD IOC 焊盘 (如 IOC_PAD_PA17) */
    uint32_t rxd_ioc_func; /**< RXD IOC 功能 (如 IOC_PA17_FUNC_CTL_MCAN4_RXD) */

    /* STB 引脚 (收发器待机控制, 可选) */
    bool has_stb; /**< 是否有 STB 引脚 */
    uint16_t stb_ioc_pad; /**< STB IOC 焊盘 (如 IOC_PAD_PA18) */
    uint32_t stb_ioc_func; /**< STB IOC GPIO 功能 (如 IOC_PA18_FUNC_CTL_GPIO_A_18) */
    GPIO_Type* stb_gpio; /**< STB GPIO 端口 (如 HPM_GPIO0) */
    uint32_t stb_gpio_port; /**< STB GPIO 端口索引 (如 GPIO_DO_GPIOA) */
    uint8_t stb_gpio_pin; /**< STB GPIO 引脚编号 (如 18) */
    uint8_t stb_active_level; /**< STB 有效电平 (0=正常, 1=待机) */

    /* TDC 配置 (CAN-FD 数据段发送器延迟补偿, 可选) */
    bool tdc_manual; /**< true=手动TDC, false=SDK自动 */
    uint8_t tdc_ssp_offset; /**< SSP 偏移 (TQ 数) */
    uint8_t tdc_filter_win; /**< TDC 滤波器窗口 (TQ 数) */

    /* 中断 */
    uint8_t irq_priority; /**< 中断优先级 */
} drv_can_config_t;

/**
 * @brief CAN 消息接收回调
 *
 * 在 MCAN ISR 上下文中调用，应尽快返回。
 * @param ctx    触发回调的 CAN 实例上下文
 * @param rx_msg 接收到的 CAN 消息指针 (指向 ISR 临时缓冲区，返回后可能被覆盖)
 */
typedef void (*drv_can_rx_callback_t)(drv_can_context_t* ctx,
    const mcan_rx_message_t* rx_msg);

/**
 * @brief CAN 控制器总线状态
 */
typedef enum {
    DRV_CAN_STATE_ERROR_ACTIVE = 0, /**< 错误主动 (正常) */
    DRV_CAN_STATE_ERROR_WARNING, /**< 错误警告 (TEC/REC > 96) */
    DRV_CAN_STATE_ERROR_PASSIVE, /**< 错误被动 (TEC/REC > 127) */
    DRV_CAN_STATE_BUS_OFF, /**< 总线关闭 (TEC > 255) */
} drv_can_state_t;

/**
 * @brief CAN 驱动错误码
 */
typedef enum {
    DRV_CAN_OK = 0, /**< 操作成功 */
    DRV_CAN_ERROR_NULL_PTR, /**< 空指针错误 */
    DRV_CAN_ERROR_UNINITIALIZED, /**< 未初始化 */
    DRV_CAN_ERROR_INVALID_PARAM, /**< 无效参数 */
    DRV_CAN_ERROR_NO_INSTANCE, /**< 无可用实例槽位 */
    DRV_CAN_ERROR_TX_BUSY, /**< 发送忙碌 (非阻塞发送时 TXFIFO 满) */
} drv_can_error_t;

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 根据配置初始化一个 CAN 实例
 *
 * 从静态上下文池中分配一个槽位，根据 config 完成全部硬件初始化：
 *   1. 配置引脚复用 (TXD, RXD, STB)
 *   2. 配置 STB 引脚为 GPIO 输出 (若 has_stb = true)
 *   3. 使能 CAN 时钟 (PLL1_CLK0 / 10 = 80MHz)
 *   4. 获取默认 MCAN 配置并覆盖为 CAN-FD 参数
 *   5. 配置 CAN-FD 消息 RAM 布局
 *   6. 配置中断 (RXFIFO0 新消息 + 消息丢失)
 *   7. 配置滤波器 (接收所有帧, 拒绝不匹配)
 *   8. 初始化 MCAN 控制器
 *   9. 初始化接收 kfifo
 *  10. 使能 MCAN 中断
 *
 * @param config 配置参数 (必须非空, 初始化后内部复制, 调用者可释放)
 * @return 上下文指针 (成功), NULL (失败)
 */
drv_can_context_t* drv_can_init(const drv_can_config_t* config);

/**
 * @brief 反初始化 CAN 实例
 *
 * 禁止中断、反初始化 MCAN、恢复 STB 为待机、释放上下文槽位。
 * @param ctx 上下文指针
 * @return DRV_CAN_OK 成功, 其他为错误码
 */
drv_can_error_t drv_can_deinit(drv_can_context_t* ctx);

/**
 * @brief 检查 CAN 实例是否已初始化
 * @param ctx 上下文指针
 * @return true 已初始化, false 未初始化
 */
bool drv_can_is_initialized(const drv_can_context_t* ctx);

/**
 * @brief 获取 CAN 实例的 MCAN 基地址
 * @param ctx 上下文指针
 * @return MCAN 基地址, NULL 表示无效
 */
MCAN_Type* drv_can_get_base(const drv_can_context_t* ctx);

/**
 * @brief 获取 CAN 控制器当前总线状态
 *
 * 通过 PSR 寄存器检测总线关闭/错误被动/错误警告状态。
 * @param ctx 上下文指针
 * @return 当前总线状态, 未初始化返回 BUS_OFF
 */
drv_can_state_t drv_can_get_state(const drv_can_context_t* ctx);

/**
 * @brief 从总线关闭状态恢复
 *
 * 清除 CCCR.INIT 启动恢复流程，MCAN 自动等待 128×11 个隐性位后重启参与总线。
 * 恢复后 TEC/REC 归零。
 * @param ctx 上下文指针
 */
void drv_can_recover(drv_can_context_t* ctx);

/**
 * @brief 注册接收消息回调
 *
 * 回调在 ISR 上下文中调用，应尽快返回。
 * @param ctx 上下文指针
 * @param cb  回调函数, 传入 NULL 取消注册
 */
void drv_can_set_rx_callback(drv_can_context_t* ctx, drv_can_rx_callback_t cb);

/**
 * @brief 获取接收 kfifo 中可读字节数
 * @param ctx 上下文指针
 * @return 可读字节数, 未初始化返回 0
 */
uint32_t drv_can_rx_available(const drv_can_context_t* ctx);

/**
 * @brief 从接收 kfifo 中读取一条 CAN 消息
 * @param ctx    上下文指针
 * @param rx_msg 输出缓冲区 (存放 mcan_rx_message_t)
 * @return DRV_CAN_OK 成功, 其他为错误码
 */
drv_can_error_t drv_can_rx_read(drv_can_context_t* ctx, mcan_rx_message_t* rx_msg);

/**
 * @brief 阻塞发送 CAN 消息
 *
 * 发送 CAN-FD 帧时, 调用者需在 tx_frame 中设置 canfd_frame=1，
 * 需要 BRS 时设置 bitrate_switch=1。
 *
 * @param ctx      上下文指针
 * @param tx_frame 待发送的 CAN 消息帧
 * @return DRV_CAN_OK 成功, 其他为错误码
 */
drv_can_error_t drv_can_tx_send(drv_can_context_t* ctx,
    const mcan_tx_frame_t* tx_frame);

/**
 * @brief 非阻塞发送 CAN 消息 (通过 TXFIFO)
 *
 * 若 TXFIFO 满则返回 DRV_CAN_ERROR_TX_BUSY。
 *
 * @param ctx        上下文指针
 * @param tx_frame   待发送的 CAN 消息帧
 * @param fifo_index 输出参数: 分配的 TXFIFO 索引 (可为 NULL)
 * @return DRV_CAN_OK 成功, 其他为错误码
 */
drv_can_error_t drv_can_tx_send_nonblocking(drv_can_context_t* ctx,
    const mcan_tx_frame_t* tx_frame, uint32_t* fifo_index);

#ifdef __cplusplus
}
#endif

#endif /* __DRV_CAN_H */
