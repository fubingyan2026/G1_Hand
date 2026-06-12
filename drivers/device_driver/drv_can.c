/*
 * Copyright (c) 2026 G1_Hand 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    drv_can.c
 * @author  G1_Hand
 * @version 1.1.0
 * @brief   CAN-FD 设备驱动实现 — 多实例支持 + 中断接收 + kfifo 缓冲 + 阻塞/非阻塞发送
 *
 * @attention
 * - 使用静态上下文池管理多实例, 每个 MCAN 实例独立配置
 * - 所有引脚复用、时钟、GPIO 配置均在驱动层内部完成
 * - 必须在 init_esc_pins() 之后调用 drv_can_init() (ESC 会将 PA16-PA18 设为模拟模式)
 * - 新增 CAN 实例只需定义 drv_can_config_t 并调用 drv_can_init(), 无需修改驱动代码
 */

/* Includes ------------------------------------------------------------------*/
#include "drv_can.h"

#include <string.h>

#include "hpm_clock_drv.h"
#include "hpm_gpio_drv.h"
#include "hpm_interrupt.h"
#include "hpm_iomux.h"
#include "hpm_soc.h"
#include "kfifo.h"

/* Private constants ---------------------------------------------------------*/

/** @brief 每个实例的接收 kfifo 大小 (字节, 必须为 2 的幂) */
#define RX_FIFO_SIZE (1024U)

/* Private types -------------------------------------------------------------*/

/**
 * @brief CAN 驱动上下文结构体 (运行时状态)
 *
 * 包含配置副本和全部运行时状态。通过静态池管理，不透明于外部调用者。
 */
struct drv_can_context {
    drv_can_config_t config; /**< 配置参数副本 */
    bool initialized; /**< 初始化标志 */
    uint32_t clock_freq; /**< 时钟频率 (Hz) */
    mcan_rx_message_t rx_msg; /**< ISR 临时接收缓冲区 */
    uint8_t rx_fifo_buf[RX_FIFO_SIZE]; /**< kfifo 数据缓冲区 */
    kfifo_t rx_fifo; /**< 接收 kfifo 实例 */
    drv_can_rx_callback_t rx_callback; /**< 用户接收回调 */
};

/* Private variables ---------------------------------------------------------*/

/** @brief 静态上下文池 */
static drv_can_context_t s_contexts[DRV_CAN_MAX_INSTANCES];

/** @brief 已使用上下文数量 */
static uint8_t s_instance_count = 0;

/* Private function prototypes -----------------------------------------------*/

/**
 * @brief 根据 MCAN 基地址查找已初始化的上下文
 * @param base MCAN 基地址
 * @return 上下文指针, 未找到返回 NULL
 */
static drv_can_context_t* can_find_by_base(MCAN_Type* base);

/**
 * @brief CAN 中断通用处理函数
 *
 * 由各 MCAN 实例的 ISR 调用, 根据基地址分发到对应上下文。
 * @param base MCAN 基地址
 */
static void can_isr_handler(MCAN_Type* base);

/**
 * @brief 配置 CAN 滤波器 (接收所有帧, 拒绝不匹配)
 * @param config MCAN 配置结构体指针
 */
static void can_config_filter(mcan_config_t* config);

/* ======================================================================
 * CAN 中断服务程序 (MCAN0 ~ MCAN7)
 * ====================================================================== */

SDK_DECLARE_EXT_ISR_M(IRQn_MCAN0, mcan0_isr)
void mcan0_isr(void) { can_isr_handler(HPM_MCAN0); }

SDK_DECLARE_EXT_ISR_M(IRQn_MCAN1, mcan1_isr)
void mcan1_isr(void) { can_isr_handler(HPM_MCAN1); }

SDK_DECLARE_EXT_ISR_M(IRQn_MCAN2, mcan2_isr)
void mcan2_isr(void) { can_isr_handler(HPM_MCAN2); }

SDK_DECLARE_EXT_ISR_M(IRQn_MCAN3, mcan3_isr)
void mcan3_isr(void) { can_isr_handler(HPM_MCAN3); }

SDK_DECLARE_EXT_ISR_M(IRQn_MCAN4, mcan4_isr)
void mcan4_isr(void) { can_isr_handler(HPM_MCAN4); }

SDK_DECLARE_EXT_ISR_M(IRQn_MCAN5, mcan5_isr)
void mcan5_isr(void) { can_isr_handler(HPM_MCAN5); }

SDK_DECLARE_EXT_ISR_M(IRQn_MCAN6, mcan6_isr)
void mcan6_isr(void) { can_isr_handler(HPM_MCAN6); }

SDK_DECLARE_EXT_ISR_M(IRQn_MCAN7, mcan7_isr)
void mcan7_isr(void) { can_isr_handler(HPM_MCAN7); }

/* ======================================================================
 * 公共 API
 * ====================================================================== */

drv_can_context_t* drv_can_init(const drv_can_config_t* config)
{
    drv_can_context_t* ctx = NULL;
    uint32_t freq;
    /* mcan_config_t ~500+ 字节, 必须 static 避免栈溢出 */
    static mcan_config_t mcan_cfg;

    /* 参数检查 */
    if (!config || !config->base) {
        return NULL;
    }

    /* 检查是否已存在同一 MCAN 实例 */
    ctx = can_find_by_base(config->base);
    if (ctx) {
        drv_can_deinit(ctx);
    }

    /* 从静态池中分配新槽位 */
    if (s_instance_count >= DRV_CAN_MAX_INSTANCES) {
        return NULL;
    }
    ctx = &s_contexts[s_instance_count];
    memset(ctx, 0, sizeof(drv_can_context_t));
    ctx->config = *config;

    /* 1. 配置引脚复用 (TXD, RXD) */
    HPM_IOC->PAD[config->txd_ioc_pad].FUNC_CTL = config->txd_ioc_func;
    HPM_IOC->PAD[config->rxd_ioc_pad].FUNC_CTL = config->rxd_ioc_func;

    /* 2. 配置 STB 引脚 (可选) */
    if (config->has_stb) {
        HPM_IOC->PAD[config->stb_ioc_pad].FUNC_CTL = config->stb_ioc_func;
        gpio_set_pin_output_with_initial(config->stb_gpio,
            config->stb_gpio_port, config->stb_gpio_pin,
            config->stb_active_level);
    }

    /* 3. 使能 CAN 时钟 (PLL1_CLK0 / 10 = 80MHz) */
    clock_add_to_group(config->clock_name, 0);
    clock_set_source_divider(config->clock_name, clk_src_pll1_clk0, 10);
    freq = clock_get_frequency(config->clock_name);
    ctx->clock_freq = freq;

    /* 清除可能残留的 MCAN 中断标志, 防止 PLIC 使能前误触发 */
    mcan_clear_interrupt_flags(config->base, ~0UL);

    /* 4. 初始化消息缓冲区属性 (必须在 mcan_get_default_config 之前) */
    {
        mcan_msg_buf_attr_t buf_attr;
        buf_attr.ram_base = MCAN_MSG_BUF_BASE_VALID_START;
        buf_attr.ram_size = MCAN_MSG_BUF_SIZE_MAX;
        mcan_set_msg_buf_attr(config->base, &buf_attr);
    }

    /* 5. 获取默认 MCAN 配置并覆盖为 CAN-FD 参数 */
    mcan_get_default_config(config->base, &mcan_cfg);
    mcan_cfg.use_lowlevel_timing_setting = false;
    mcan_cfg.mode = config->mode;
    mcan_cfg.baudrate = config->nominal_baudrate;
    mcan_cfg.baudrate_fd = config->data_baudrate;
    mcan_cfg.enable_canfd = true;
    mcan_cfg.enable_tdc = true;
    mcan_cfg.disable_auto_retransmission = true;

    /* 手动 TDC: 针对特定收发器调优 SSP 偏移 (默认 SDK 自动计算可能不匹配) */
    if (config->tdc_manual) {
        mcan_cfg.tdc_config.ssp_offset = config->tdc_ssp_offset;
        mcan_cfg.tdc_config.filter_window_length = config->tdc_filter_win;
    }

    /* 6. 覆盖 RAM 为 CAN-FD 布局 */
    mcan_get_default_ram_config(config->base, &mcan_cfg.ram_config, true);

    /* 7. 配置中断和滤波器 */
    mcan_cfg.interrupt_mask = MCAN_INT_RXFIFO0_NEW_MSG
        | MCAN_INT_RXFIFO0_MSG_LOST;
    can_config_filter(&mcan_cfg);

    /* 8. 初始化 MCAN 控制器 */
    if (mcan_init(config->base, &mcan_cfg, freq) != status_success) {
        return NULL;
    }

    /* 9. 初始化接收 kfifo */
    kfifo_init(&ctx->rx_fifo, ctx->rx_fifo_buf, RX_FIFO_SIZE, NULL);
    ctx->rx_callback = NULL;

    /* 10. 使能 MCAN 中断 */
    intc_m_enable_irq_with_priority(config->irq_num, config->irq_priority);

    ctx->initialized = true;
    s_instance_count++;

    return ctx;
}

drv_can_error_t drv_can_deinit(drv_can_context_t* ctx)
{
    uint8_t i;

    if (!ctx || !ctx->initialized) {
        return DRV_CAN_ERROR_UNINITIALIZED;
    }

    /* 禁止 MCAN 中断 */
    intc_m_disable_irq(ctx->config.irq_num);

    /* 反初始化 MCAN */
    mcan_deinit(ctx->config.base);

    /* 恢复 STB 为待机模式 */
    if (ctx->config.has_stb) {
        gpio_write_pin(ctx->config.stb_gpio, ctx->config.stb_gpio_port,
            ctx->config.stb_gpio_pin, (uint8_t)(!ctx->config.stb_active_level));
    }

    ctx->initialized = false;

    /* 压缩上下文池 (将后面的实例前移) */
    for (i = 0; i < s_instance_count; i++) {
        if (&s_contexts[i] == ctx) {
            /* 将最后一个有效实例移到当前位置 */
            s_instance_count--;
            if (i < s_instance_count) {
                memcpy(&s_contexts[i], &s_contexts[s_instance_count],
                    sizeof(drv_can_context_t));
            }
            memset(&s_contexts[s_instance_count], 0, sizeof(drv_can_context_t));
            break;
        }
    }

    return DRV_CAN_OK;
}

bool drv_can_is_initialized(const drv_can_context_t* ctx)
{
    return (ctx && ctx->initialized);
}

MCAN_Type* drv_can_get_base(const drv_can_context_t* ctx)
{
    if (!ctx || !ctx->initialized) {
        return NULL;
    }
    return ctx->config.base;
}

drv_can_state_t drv_can_get_state(const drv_can_context_t* ctx)
{
    uint32_t psr;

    if (!ctx || !ctx->initialized) {
        return DRV_CAN_STATE_BUS_OFF;
    }

    psr = ctx->config.base->PSR;

    if (psr & MCAN_PSR_BO_MASK) {
        return DRV_CAN_STATE_BUS_OFF;
    }
    if (psr & MCAN_PSR_EP_MASK) {
        return DRV_CAN_STATE_ERROR_PASSIVE;
    }
    if (psr & MCAN_PSR_EW_MASK) {
        return DRV_CAN_STATE_ERROR_WARNING;
    }
    return DRV_CAN_STATE_ERROR_ACTIVE;
}

void drv_can_recover(drv_can_context_t* ctx)
{
    MCAN_Type* base;

    if (!ctx || !ctx->initialized) {
        return;
    }

    base = ctx->config.base;

    /* 检查是否确实处于总线关闭状态 */
    if (!(base->PSR & MCAN_PSR_BO_MASK)) {
        return;
    }

    /*
     * 防止重复清除 INIT 导致恢复计数器被反复复位:
     * 若 INIT 已被清除 (上次 recover 调用已启动恢复流程),
     * 则说明恢复正在进行中, 不应再次写入 CCCR.INIT。
     * 每次清除 INIT 都会复位 128×11 隐性位计数器,
     * 重复清除会导致恢复永远无法完成。
     */
    if (!(base->CCCR & MCAN_CCCR_INIT_MASK)) {
        /* 恢复流程已在运行中, 等待完成 */
        return;
    }

    /*
     * 总线关闭恢复流程 (ISO 11898-1):
     * 1. MCAN 进入 bus-off 时已自动设置 CCCR.INIT
     * 2. 清除 CCCR.INIT, MCAN 开始等待 128×11 个隐性位
     * 3. 恢复序列完成后 MCAN 自动清除 INIT, TEC/REC 归零
     */
    base->CCCR &= ~MCAN_CCCR_INIT_MASK;
}

void drv_can_set_rx_callback(drv_can_context_t* ctx, drv_can_rx_callback_t cb)
{
    if (ctx && ctx->initialized) {
        ctx->rx_callback = cb;
    }
}

uint32_t drv_can_rx_available(const drv_can_context_t* ctx)
{
    if (!ctx || !ctx->initialized) {
        return 0;
    }
    return kfifo_len(&ctx->rx_fifo);
}

drv_can_error_t drv_can_rx_read(drv_can_context_t* ctx, mcan_rx_message_t* rx_msg)
{
    unsigned int read;

    if (!rx_msg) {
        return DRV_CAN_ERROR_NULL_PTR;
    }
    if (!ctx || !ctx->initialized) {
        return DRV_CAN_ERROR_UNINITIALIZED;
    }

    read = kfifo_get(&ctx->rx_fifo, (unsigned char*)rx_msg,
        sizeof(mcan_rx_message_t));
    if (read < sizeof(mcan_rx_message_t)) {
        return DRV_CAN_ERROR_INVALID_PARAM;
    }

    return DRV_CAN_OK;
}

drv_can_error_t drv_can_tx_send(drv_can_context_t* ctx,
    const mcan_tx_frame_t* tx_frame)
{
    hpm_stat_t stat;

    if (!tx_frame) {
        return DRV_CAN_ERROR_NULL_PTR;
    }
    if (!ctx || !ctx->initialized) {
        return DRV_CAN_ERROR_UNINITIALIZED;
    }

    stat = mcan_transmit_blocking(ctx->config.base, (mcan_tx_frame_t*)tx_frame);
    return (stat == status_success) ? DRV_CAN_OK : DRV_CAN_ERROR_TX_BUSY;
}

drv_can_error_t drv_can_tx_send_nonblocking(drv_can_context_t* ctx,
    const mcan_tx_frame_t* tx_frame, uint32_t* fifo_index)
{
    hpm_stat_t stat;
    uint32_t idx = 0;

    if (!tx_frame) {
        return DRV_CAN_ERROR_NULL_PTR;
    }
    if (!ctx || !ctx->initialized) {
        return DRV_CAN_ERROR_UNINITIALIZED;
    }

    stat = mcan_transmit_via_txfifo_nonblocking(ctx->config.base,
        (mcan_tx_frame_t*)tx_frame, &idx);
    if (fifo_index) {
        *fifo_index = idx;
    }
    return (stat == status_success) ? DRV_CAN_OK : DRV_CAN_ERROR_TX_BUSY;
}

/* ======================================================================
 * 私有函数
 * ====================================================================== */

static drv_can_context_t* can_find_by_base(MCAN_Type* base)
{
    uint8_t i;

    for (i = 0; i < s_instance_count; i++) {
        if (s_contexts[i].config.base == base && s_contexts[i].initialized) {
            return &s_contexts[i];
        }
    }
    return NULL;
}

static void can_isr_handler(MCAN_Type* base)
{
    drv_can_context_t* ctx = can_find_by_base(base);

    if (!ctx) {
        /* 未注册的 MCAN 中断, 清除所有标志后返回 */
        mcan_clear_interrupt_flags(base, ~0UL);
        return;
    }

    uint32_t ir = base->IR;

    /* 处理 RXFIFO0 新消息 */
    if (ir & MCAN_INT_RXFIFO0_NEW_MSG) {
        if (mcan_read_rxfifo(base, 0, &ctx->rx_msg) == status_success) {
            kfifo_put(&ctx->rx_fifo, (const unsigned char*)&ctx->rx_msg,
                sizeof(mcan_rx_message_t));

            if (ctx->rx_callback) {
                ctx->rx_callback(ctx, &ctx->rx_msg);
            }
        }
    }

    /* 可选: 记录 RXFIFO0 消息丢失统计 */
    (void)(ir & MCAN_INT_RXFIFO0_MSG_LOST);

    /* 清除已处理的中断标志 (W1C) */
    base->IR = ir;
}

static void can_config_filter(mcan_config_t* config)
{
    mcan_all_filters_config_t* filter_cfg = &config->all_filters_config;

    /* 标准帧和扩展帧滤波器使用 SDK 默认配置 (经典滤波器, filter_mask = 0, 全接受) */
    /* 无需修改 filter_elem_list 指针 */

    /* 全局滤波器: 拒绝不匹配的帧 */
    filter_cfg->global_filter_config.accept_non_matching_std_frame_option = MCAN_ACCEPT_NON_MATCHING_FRAME_OPTION_REJECT;
    filter_cfg->global_filter_config.accept_non_matching_ext_frame_option = MCAN_ACCEPT_NON_MATCHING_FRAME_OPTION_REJECT;
}
