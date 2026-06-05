//
// Created by fubingyan on 25-9-20.
//

/**
 * @file    led.h
 * @brief   LED 控制模块 — 支持 ON/OFF/编码闪烁三种状态
 * @note    使用 FSM 管理状态转换，kfifo 异步命令队列接收外部指令。
 *          引脚操作通过 config 回调解耦，不直接依赖 HAL。
 *          实例链表使用侵入式 clist 管理。
 */

#ifndef __LED_H
#define __LED_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "clist.h"
#include "fsm.h"
#include "kfifo.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief LED 错误码
 */
typedef enum {
    LED_OK = 0, /**< 操作成功 */
    LED_OK_EXISTED = 1, /**< 已初始化 */
    LED_ERROR_INVALID_PARAM = -1, /**< 无效参数 */
    LED_ERROR_NO_MEMORY = -2, /**< 内存不足 */
    LED_ERROR_NOT_FOUND = -3, /**< 未找到实例 */
    LED_ERROR_ALREADY_EXIST = -4, /**< 同名实例已存在 */
    LED_ERROR_INTERNAL = -5, /**< 内部错误 */
} led_error_t;

/**
 * @brief LED 工作状态
 */
typedef enum __attribute__((packed)) {
    LED_STATE_NONE = 0, /**< 无状态（空闲） */
    LED_STATE_OFF, /**< 熄灭 */
    LED_STATE_ON, /**< 常亮 */
    LED_STATE_BLINK_CODE, /**< 编码闪烁（闪烁+间隔循环） */
    LED_STATE_MAX, /**< 状态总数 */
} led_state_t;

/**
 * @brief 闪烁阶段
 */
typedef enum __attribute__((packed)) {
    LED_BLINK_PHASE_BLINKING = 0, /**< 闪烁中：按 cycle_ms 切换亮灭 */
    LED_BLINK_PHASE_INTERVAL, /**< 间隔中：按 wait_ms 保持熄灭 */
} led_blink_phase_t;

/**
 * @brief LED 句柄前向声明
 */
typedef struct led_handle led_handle_t;

/**
 * @brief 系统时间获取回调
 * @return 毫秒时间戳
 */
typedef uint32_t (*led_get_time_cb_t)(void);

/**
 * @brief LED 状态变化回调
 * @param instance 触发回调的 LED 实例
 * @param new_state 新状态
 * @param user_data 用户数据
 */
typedef void (*led_state_change_cb_t)(led_handle_t* instance,
    led_state_t new_state, void* user_data);

/**
 * @brief 闪烁阶段变化回调
 * @param instance LED 实例
 * @param phase 当前闪烁阶段
 * @param user_data 用户数据
 */
typedef void (*led_blink_phase_cb_t)(led_handle_t* instance,
    led_blink_phase_t phase, void* user_data);

/**
 * @brief GPIO 边沿变化回调
 * @param instance LED 实例
 * @param rising true=上升沿(亮)，false=下降沿(灭)
 * @param user_data 用户数据
 */
typedef void (*led_edge_cb_t)(led_handle_t* instance, bool rising,
    void* user_data);

/**
 * @brief LED 配置
 * @note write_pin/read_pin 由用户实现，负责实际引脚操作
 */
typedef struct {
    const char* name; /**< LED 名称（唯一标识） */
    led_state_t init_state; /**< 初始状态 */
    void (*write_pin)(bool on); /**< 引脚写入回调：true=逻辑亮，false=逻辑灭 */
} led_config_t;

/**
 * @brief LED 异步命令
 */
typedef struct {
    led_state_t led_set_state; /**< 目标状态 */
    uint16_t led_blink_cycle_ms; /**< 闪烁间隔(ms) */
    uint16_t led_blink_wait_ms; /**< 等待间隔(ms) */
    uint16_t led_blink_code_counts; /**< 闪烁次数（0=无限循环） */
} led_cmd_t;

/**
 * @brief LED 控制句柄
 */
struct led_handle {
    led_config_t config; /**< 配置副本 */
    clist_head_t node; /**< clist 链表节点 */
    fsm_t fsm; /**< FSM 状态机上下文 */

    led_cmd_t current_cmd; /**< 当前命令参数 */
    uint32_t last_toggle_time; /**< 上次翻转时间戳 */
    uint32_t interval_start_time; /**< 间隔阶段起始时间戳 */

    uint16_t current_led_blink_code_counts; /**< 当前闪烁计数 */
    led_blink_phase_t blink_code_phase; /**< 当前闪烁阶段 */
    led_blink_phase_t blink_code_phase_last; /**< 上次闪烁阶段（用于检测变化） */
    bool blink_sw_on; /**< 软件跟踪 LED 亮灭状态，避免依赖 GPIO 读回 */

    bool is_static; /**< 静态分配标志 */
    bool initialized; /**< 初始化完成标志 */
    bool pending_blink_update; /**< 待处理的闪烁参数更新（等待 LED 熄灭） */

    kfifo_t* cmd_fifo; /**< 异步命令队列 */

    void* state_change_cb; /**< 状态变化回调 (led_state_change_cb_t) */
    void* blink_phase_cb; /**< 闪烁阶段变化回调 (led_blink_phase_cb_t) */
    void* edge_cb; /**< 引脚边沿回调 (led_edge_cb_t) */
    void* callback_user_data; /**< 回调用户数据 */
};

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

#define LED_IS_OK(err) ((err) >= 0) /**< 判断错误码是否表示成功 */
#define LED_IS_ERR(err) ((err) < 0) /**< 判断错误码是否表示失败 */

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief 初始化 LED 子系统
 * @param get_time_cb 系统毫秒时间获取回调
 * @return 错误码
 */
led_error_t led_init(led_get_time_cb_t get_time_cb);

/**
 * @brief 反初始化 LED 子系统，释放所有资源
 */
void led_deinit(void);

/**
 * @brief 动态注册 LED 实例
 * @param config 配置指针
 * @return 实例指针，失败返回 NULL
 */
led_handle_t* led_register(const led_config_t* config);

/**
 * @brief 静态注册 LED 实例（使用用户分配的内存）
 * @param config 配置指针
 * @param instance 用户提供的句柄内存
 * @return 错误码
 */
led_error_t led_register_static(const led_config_t* config,
    led_handle_t* instance);

/**
 * @brief 注销 LED 实例
 * @param name LED 名称
 * @return 错误码
 */
led_error_t led_unregister(const char* name);

/**
 * @brief 按名称查找 LED 实例
 * @param name LED 名称
 * @return 实例指针，未找到返回 NULL
 */
led_handle_t* led_get_instance(const char* name);

/**
 * @brief 获取 LED 链表头
 * @return clist 链表头指针，未初始化返回 NULL
 */
clist_head_t* led_get_head(void);

/**
 * @brief 设置 LED 状态（异步，通过命令队列发送）
 * @param instance LED 实例
 * @param state 目标状态
 */
void led_set_state(led_handle_t* instance, led_state_t state);

/**
 * @brief 设置 LED 闪烁参数
 * @param instance LED 实例
 * @param cmd 闪烁参数
 * @return 错误码
 */
led_error_t led_set_blink_interval(led_handle_t* instance,
    const led_cmd_t* cmd);

/**
 * @brief 获取当前闪烁阶段
 * @param instance LED 实例
 * @return 闪烁阶段
 */
led_blink_phase_t led_get_blink_phase(led_handle_t* instance);

/**
 * @brief 注册 LED 回调函数
 * @param instance LED 实例
 * @param state_cb 状态变化回调
 * @param blink_phase_cb 闪烁阶段变化回调
 * @param edge_cb 引脚边沿回调
 * @param user_data 用户数据，透传给回调
 */
void led_set_callbacks(led_handle_t* instance, led_state_change_cb_t state_cb,
    led_blink_phase_cb_t blink_phase_cb,
    led_edge_cb_t edge_cb, void* user_data);

/**
 * @brief LED 刷新任务
 * @note 需在主循环或定时器中定期调用。
 *       处理命令队列、执行 FSM 步进、检测闪烁阶段变化。
 */
void led_task_refresh(void);

#ifdef __cplusplus
}
#endif

#endif /* __LED_H */