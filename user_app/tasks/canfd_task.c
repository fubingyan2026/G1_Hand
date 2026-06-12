/*
 * Copyright (c) 2026 G1_Hand 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    canfd_task.c
 * @brief   CAN-FD 收发测试任务实现
 *
 * 使用 drv_can 多实例驱动，在 CAN4 上进行周期性 CAN-FD 帧发送和接收轮询。
 * - TX: 每 ~2 秒发送一条 CAN-FD 帧 (标准 ID=0x123, BRS, DLC=15, 64 字节递增数据)
 * - RX: 每轮轮询检查 kfifo 中是否有接收数据，有则打印
 */

/* Includes ------------------------------------------------------------------*/
#include "canfd_task.h"

#include <stdio.h>
#include <string.h>

#include "bsp_systick.h"
#include "drv_can.h"

/* Private constants ---------------------------------------------------------*/

/** @brief CAN-FD 测试帧标准 ID */
#define CANFD_TEST_STD_ID (0x123U)

/** @brief TX 间隔 (MCHTMR 周期数, ~2 秒 @ 24MHz) */
#define CANFD_TX_INTERVAL_MS (200U)

/* Private variables ---------------------------------------------------------*/

/** @brief drv_can 上下文指针 */
static drv_can_context_t* s_can_ctx;

/** @brief TX 计数器 (每次发送递增, 填入数据段) */
static uint8_t s_tx_counter;

/** @brief RX 接收计数 (用于打印统计) */
static uint32_t s_rx_count;

/** @brief 上次 TX 的毫秒时间戳 */
static uint32_t s_last_tx_ms;

/* Private function prototypes -----------------------------------------------*/

/**
 * @brief 获取当前 MCHTMR 计数值
 * @return 当前计数值
 */
static uint32_t canfd_task_get_ticks(void);

/**
 * @brief CAN 接收回调 (ISR 上下文)
 * @param ctx    触发回调的 CAN 实例上下文
 * @param rx_msg 接收到的消息指针
 */
static void canfd_task_rx_callback(drv_can_context_t* ctx,
    const mcan_rx_message_t* rx_msg);

/* ======================================================================
 * 公共 API
 * ====================================================================== */

void canfd_task_init(void)
{
    drv_can_context_t* ctx;

    /* CAN4 硬件配置 */
    const drv_can_config_t can4_config = {
        .name = "can4",
        .base = HPM_MCAN4,
        .irq_num = IRQn_MCAN4,
        .clock_name = clock_can4,
        .mode = mcan_mode_normal,
        .nominal_baudrate = DRV_CAN_DEFAULT_NOMINAL_BAUDRATE,
        .data_baudrate = DRV_CAN_DEFAULT_DATA_BAUDRATE,
        .txd_ioc_pad = IOC_PAD_PA16,
        .txd_ioc_func = IOC_PA16_FUNC_CTL_MCAN4_TXD,
        .rxd_ioc_pad = IOC_PAD_PA17,
        .rxd_ioc_func = IOC_PA17_FUNC_CTL_MCAN4_RXD,
        .has_stb = true,
        .stb_ioc_pad = IOC_PAD_PA18,
        .stb_ioc_func = IOC_PA18_FUNC_CTL_GPIO_A_18,
        .stb_gpio = HPM_GPIO0,
        .stb_gpio_port = GPIO_DO_GPIOA,
        .stb_gpio_pin = 18,
        .stb_active_level = 0,
        .tdc_manual = true,
        .tdc_ssp_offset = 10, /* TCAN1044 ~125ns → 10 TQ @80MHz */
        .tdc_filter_win = 10,
        .irq_priority = 1,
    };

    printf("\n[CANFD] Initializing CAN4 (PA16/PA17/PA18)...\n");

    /* 初始化 CAN4 实例 */
    ctx = drv_can_init(&can4_config);
    if (!ctx) {
        printf("[CANFD] ERROR: drv_can_init failed!\n");
        return;
    }

    /* 注册 ISR 接收回调 (仅做计数统计, 不打印) */
    drv_can_set_rx_callback(ctx, canfd_task_rx_callback);

    /* 记录初始时间 */
    s_last_tx_ms = canfd_task_get_ticks();
    s_tx_counter = 0;
    s_rx_count = 0;
    s_can_ctx = ctx;

    printf("[CANFD] CAN4 initialized, nominal=%u bps, data=%u bps\n",
        DRV_CAN_DEFAULT_NOMINAL_BAUDRATE, DRV_CAN_DEFAULT_DATA_BAUDRATE);
    printf("[CANFD] RX callback registered, TX interval=%u ms\n\n",
        CANFD_TX_INTERVAL_MS);
}

void canfd_task_poll(void)
{
    uint32_t now;

    if (!s_can_ctx) {
        return;
    }

    /* 1. 检查接收 kfifo 中是否有数据 */
    while (drv_can_rx_available(s_can_ctx) >= sizeof(mcan_rx_message_t)) {
        mcan_rx_message_t rx_msg;
        if (drv_can_rx_read(s_can_ctx, &rx_msg) == DRV_CAN_OK) {
            /* s_rx_count 由 ISR 回调递增，此处仅读取 */

            /* 打印接收信息 */
            if (rx_msg.use_ext_id) {
                printf("[CANFD] RX ext=0x%08lX dlc=%u fd=%u brs=%u\n",
                    (unsigned long)rx_msg.ext_id, rx_msg.dlc,
                    rx_msg.canfd_frame, rx_msg.bitrate_switch);
            } else {
                printf("[CANFD] RX std=0x%03lX dlc=%u fd=%u brs=%u\n",
                    (unsigned long)rx_msg.std_id, rx_msg.dlc,
                    rx_msg.canfd_frame, rx_msg.bitrate_switch);
            }
        }
    }

    /* 2. 检查总线状态, 总线关闭时自动恢复 */
    {
        drv_can_state_t state = drv_can_get_state(s_can_ctx);
        if (state == DRV_CAN_STATE_BUS_OFF) {
            printf("[CANFD] BUS-OFF detected, recovering...\n");
            drv_can_recover(s_can_ctx);
            s_last_tx_ms = canfd_task_get_ticks();
            return;
        }
        if (state == DRV_CAN_STATE_ERROR_PASSIVE) {
            static bool s_ep_warned;
            if (!s_ep_warned) {
                printf("[CANFD] WARNING: error-passive\n");
                s_ep_warned = true;
            }
        }
    }

    /* 3. 周期性发送 CAN-FD 测试帧 */
    now = canfd_task_get_ticks();
    if (now - s_last_tx_ms >= CANFD_TX_INTERVAL_MS) {
        s_last_tx_ms = now;

        mcan_tx_frame_t tx_frame;
        uint8_t i;

        memset(&tx_frame, 0, sizeof(tx_frame));

        /* 填充帧头 */
        tx_frame.std_id = CANFD_TEST_STD_ID;
        tx_frame.use_ext_id = 0; /* 标准 ID */
        tx_frame.canfd_frame = 1; /* CAN-FD 帧 */
        tx_frame.bitrate_switch = 1; /* 使能 BRS (数据段高速) */
        tx_frame.dlc = 15; /* DLC=15 → 64 字节数据 */

        /* 填充数据 (递增计数器) */
        for (i = 0; i < 64; i++) {
            tx_frame.data_8[i] = s_tx_counter + i;
        }

        /* 非阻塞发送 */
        drv_can_error_t err = drv_can_tx_send_nonblocking(s_can_ctx, &tx_frame, NULL);
        if (err == DRV_CAN_OK) {

        } else {
            printf("[CANFD] TX #%u: ERROR %d\n",
                (unsigned int)s_tx_counter, (int)err);
        }

        s_tx_counter++;
    }
}

/* ======================================================================
 * 私有函数
 * ====================================================================== */

static uint32_t canfd_task_get_ticks(void)
{
    return millis();
}

static void canfd_task_rx_callback(drv_can_context_t* ctx,
    const mcan_rx_message_t* rx_msg)
{
    /*
     * ISR 上下文 — 仅做轻量计数，严禁调用 printf 等不可重入函数！
     * 接收消息的打印由 canfd_task_poll() 中的 kfifo 轮询完成。
     */
    (void)ctx;
    (void)rx_msg;
    s_rx_count++;
}
