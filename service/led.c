
/**
 * @file    led.c
 * @brief   LED 控制模块实现
 * @note    通过 FSM 管理 ON/OFF/BLINK_CODE 三种工作状态。
 *          命令通过 kfifo 异步队列传递，支持闪烁参数热更新。
 *          引脚操作通过 config 回调解耦。
 */

/* Includes ------------------------------------------------------------------*/
#include "led.h"

#include <string.h>

/* Private macros ------------------------------------------------------------*/
#define LED_CMD_FIFO_SIZE 4 /**< 异步命令队列深度（必须为2的幂） */

/* Private variables ---------------------------------------------------------*/

static clist_head_t s_led_head; /**< LED 实例链表头 */
static led_get_time_cb_t s_led_get_time; /**< 系统时间获取回调 */
static bool s_led_initialized; /**< 子系统初始化标志 */
static kfifo_t s_cmd_fifo;

/* Private function prototypes -----------------------------------------------*/

static inline uint32_t led_time_diff(uint32_t new_time, uint32_t old_time);
static inline uint32_t led_get_time_now(void);
static void led_phys_write(led_handle_t* handle, bool on);
static fsm_state_t led_fsm_none_handler(fsm_t* ctx);
static fsm_state_t led_fsm_off_handler(fsm_t* ctx);
static fsm_state_t led_fsm_on_handler(fsm_t* ctx);
static fsm_state_t led_fsm_blink_handler(fsm_t* ctx);
static void led_fsm_on_entry(fsm_t* ctx, fsm_state_t state);
static void led_fsm_on_exit(fsm_t* ctx, fsm_state_t state);
static void led_process_cmds(led_handle_t* handle);
static void led_check_blink_phase_change(led_handle_t* handle);

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 安全计算32位时间差（处理溢出回绕）
 */
static inline uint32_t led_time_diff(uint32_t new_time, uint32_t old_time)
{
    return (uint32_t)(new_time - old_time);
}

/**
 * @brief 获取系统时间
 */
static inline uint32_t led_get_time_now(void)
{
    return s_led_get_time ? s_led_get_time() : 0;
}

/**
 * @brief 物理写引脚电平
 * @param handle LED 实例
 * @param on true=逻辑亮，false=逻辑灭
 * @note 写之前读取当前状态，检测边沿变化并触发 edge_cb
 */
static void led_phys_write(led_handle_t* handle, bool on)
{
    handle->config.write_pin(on);

    if (handle->edge_cb) {
        ((led_edge_cb_t)handle->edge_cb)(handle, on, handle->callback_user_data);
    }
}

/* FSM handlers --------------------------------------------------------------*/

/**
 * @brief FSM NONE 状态处理：保持空闲
 */
static fsm_state_t led_fsm_none_handler(fsm_t* ctx)
{
    (void)ctx;
    return LED_STATE_NONE;
}

/**
 * @brief FSM OFF 状态处理：输出低电平
 */
static fsm_state_t led_fsm_off_handler(fsm_t* ctx)
{
    led_handle_t* handle = (led_handle_t*)fsm_user_data(ctx);
    led_phys_write(handle, false);
    return LED_STATE_OFF;
}

/**
 * @brief FSM ON 状态处理：输出高电平
 */
static fsm_state_t led_fsm_on_handler(fsm_t* ctx)
{
    led_handle_t* handle = (led_handle_t*)fsm_user_data(ctx);
    led_phys_write(handle, true);
    return LED_STATE_ON;
}

/**
 * @brief FSM BLINK_CODE 状态处理：编码闪烁
 * @note  闪烁由两个阶段循环组成：
 *        BLINKING — 按 led_blink_cycle_ms 间隔翻转引脚
 *        INTERVAL — 保持熄灭 led_blink_wait_ms
 *        当 led_blink_code_counts==0 时无限循环，
 *        否则达到指定次数后自动切换到 OFF 状态。
 */
static fsm_state_t led_fsm_blink_handler(fsm_t* ctx)
{
    led_handle_t* handle = (led_handle_t*)fsm_user_data(ctx);
    uint32_t now = led_get_time_now();

    if (handle->blink_code_phase == LED_BLINK_PHASE_BLINKING) {
        /* 闪烁阶段：按 cycle_ms 翻转 — 使用软件状态跟踪，不依赖 GPIO 读回 */
        if (led_time_diff(now, handle->last_toggle_time) >= handle->current_cmd.led_blink_cycle_ms) {
            handle->last_toggle_time = now;
            handle->blink_sw_on = !handle->blink_sw_on;
            led_phys_write(handle, handle->blink_sw_on);
            /* 下降沿（从亮到灭）计数一次 */
            if (!handle->blink_sw_on) {
                handle->current_led_blink_code_counts++;
                if (handle->current_cmd.led_blink_code_counts == 0 || handle->current_led_blink_code_counts >= handle->current_cmd.led_blink_code_counts) {
                    /* 进入间隔阶段 */
                    handle->blink_code_phase_last = handle->blink_code_phase;
                    handle->blink_code_phase = LED_BLINK_PHASE_INTERVAL;
                    handle->interval_start_time = now;
                    handle->current_led_blink_code_counts = 0;
                    /* 指定次数模式：完成后自动关闭 */
                    if (handle->current_cmd.led_blink_code_counts > 0) {
                        return LED_STATE_OFF;
                    }
                }
            }
        }
    } else {
        /* 间隔阶段：等待 wait_ms 后继续闪烁 */
        if (led_time_diff(now, handle->interval_start_time) >= handle->current_cmd.led_blink_wait_ms) {
            handle->blink_code_phase = LED_BLINK_PHASE_BLINKING;
            handle->last_toggle_time = now;
        }
    }
    return LED_STATE_BLINK_CODE;
}

/* FSM entry / exit ----------------------------------------------------------*/

/**
 * @brief FSM 进入状态回调
 * @note  根据不同状态执行初始化操作：
 *        ON — 立即输出高电平
 *        OFF — 立即输出低电平
 *        BLINK_CODE — 重置计数，进入间隔阶段
 */
static void led_fsm_on_entry(fsm_t* ctx, fsm_state_t state)
{
    led_handle_t* handle = (led_handle_t*)fsm_user_data(ctx);
    uint32_t now = led_get_time_now();

    if (handle->state_change_cb) {
        ((led_state_change_cb_t)handle->state_change_cb)(handle, state,
            handle->callback_user_data);
    }

    switch (state) {
    case LED_STATE_ON:
        led_phys_write(handle, true);
        break;
    case LED_STATE_OFF:
        led_phys_write(handle, false);
        break;
    case LED_STATE_BLINK_CODE:
        handle->current_led_blink_code_counts = 0;
        handle->blink_code_phase_last = handle->blink_code_phase;
        handle->blink_code_phase = LED_BLINK_PHASE_INTERVAL;
        handle->interval_start_time = now;
        handle->blink_sw_on = false;
        led_phys_write(handle, false);
        break;
    default:
        break;
    }
}

/**
 * @brief FSM 退出状态回调（当前无操作）
 */
static void led_fsm_on_exit(fsm_t* ctx, fsm_state_t state)
{
    (void)ctx;
    (void)state;
}

/* Command processing --------------------------------------------------------*/

/**
 * @brief 处理 LED 异步命令队列
 * @note  从 kfifo 中取出命令并执行：
 *        - 状态切换命令直接触发 fsm_goto
 *        - BLINK_CODE 命令支持热更新：如果 LED 当前亮着，延迟到熄灭后更新
 */
static void led_process_cmds(led_handle_t* handle)
{
    led_cmd_t cmd;
    while (kfifo_get(handle->cmd_fifo, (unsigned char*)&cmd, sizeof(led_cmd_t)) == sizeof(led_cmd_t)) {
        if (cmd.led_set_state != LED_STATE_NONE) {
            fsm_goto(&handle->fsm, cmd.led_set_state);
        }

        if (cmd.led_set_state == LED_STATE_BLINK_CODE) {
            /* 热更新：LED 亮着时暂不修改参数 */
            if (handle->pending_blink_update && fsm_current_state(&handle->fsm) == LED_STATE_BLINK_CODE) {
                if (handle->blink_sw_on) {
                    kfifo_put(handle->cmd_fifo, (unsigned char*)&cmd, sizeof(led_cmd_t));
                    break;
                }
                handle->pending_blink_update = false;
            }

            if (cmd.led_blink_cycle_ms > 0)
                handle->current_cmd.led_blink_cycle_ms = cmd.led_blink_cycle_ms;
            if (cmd.led_blink_wait_ms > 0)
                handle->current_cmd.led_blink_wait_ms = cmd.led_blink_wait_ms;
            if (cmd.led_blink_code_counts > 0)
                handle->current_cmd.led_blink_code_counts = cmd.led_blink_code_counts;

            /* 已经在闪烁模式则重置到间隔阶段 */
            if (fsm_current_state(&handle->fsm) == LED_STATE_BLINK_CODE) {
                handle->current_led_blink_code_counts = 0;
                handle->blink_code_phase = LED_BLINK_PHASE_INTERVAL;
                handle->interval_start_time = led_get_time_now();
                handle->blink_sw_on = false;
                led_phys_write(handle, false);
            }
        }
    }
}

/**
 * @brief 检测闪烁阶段变化并触发回调
 */
static void led_check_blink_phase_change(led_handle_t* handle)
{
    if (!handle || !handle->blink_phase_cb)
        return;

    if (handle->blink_code_phase != handle->blink_code_phase_last) {
        ((led_blink_phase_cb_t)handle->blink_phase_cb)(
            handle, handle->blink_code_phase, handle->callback_user_data);
        handle->blink_code_phase_last = handle->blink_code_phase;
    }
}

/* Exported functions --------------------------------------------------------*/

led_error_t led_init(led_get_time_cb_t get_time_cb)
{
    if (get_time_cb == NULL)
        return LED_ERROR_INVALID_PARAM;
    if (s_led_initialized)
        return LED_OK_EXISTED;

    s_led_get_time = get_time_cb;
    clist_init(&s_led_head);
    s_led_initialized = true;
    return LED_OK;
}

void led_deinit(void)
{
    if (!s_led_initialized)
        return;

    /* 安全遍历释放所有实例 */
    clist_head_t *pos, *tmp;
    clist_for_each_safe(pos, tmp, &s_led_head)
    {
        led_handle_t* h = clist_entry(pos, led_handle_t, node);
        clist_del(pos);
        if (h->cmd_fifo)
            kfifo_free((kfifo_t*)h->cmd_fifo);
        if (!h->is_static)
            free(h);
    }

    clist_init(&s_led_head);
    s_led_initialized = false;
}

led_handle_t* led_get_instance(const char* name)
{
    if (name == NULL || !s_led_initialized)
        return NULL;

    led_handle_t* h;
    clist_for_each_entry(h, &s_led_head, node)
    {
        if (strcmp(h->config.name, name) == 0)
            return h;
    }
    return NULL;
}

clist_head_t* led_get_head(void)
{
    return s_led_initialized ? &s_led_head : NULL;
}

led_error_t led_register_static(const led_config_t* config,
    led_handle_t* instance)
{
    if (config == NULL || instance == NULL || config->name == NULL || config->write_pin == NULL) {
        return LED_ERROR_INVALID_PARAM;
    }
    if (!s_led_initialized)
        return LED_ERROR_INTERNAL;
    if (led_get_instance(config->name))
        return LED_ERROR_ALREADY_EXIST;

    memset(instance, 0, sizeof(led_handle_t));
    memcpy(&instance->config, config, sizeof(led_config_t));

    /* 初始化 FSM */
    static const char* names[] = { "NONE", "OFF", "ON", "BLINK" };
    static fsm_handler_t handlers[LED_STATE_MAX];
    static fsm_guard_t transitions[LED_STATE_MAX * LED_STATE_MAX];
    memset(handlers, 0, sizeof(handlers));
    memset(transitions, 0, sizeof(transitions));

    handlers[LED_STATE_NONE] = led_fsm_none_handler;
    handlers[LED_STATE_OFF] = led_fsm_off_handler;
    handlers[LED_STATE_ON] = led_fsm_on_handler;
    handlers[LED_STATE_BLINK_CODE] = led_fsm_blink_handler;

    fsm_config_t fsm_cfg = {
        .handlers = handlers,
        .transitions = transitions,
        .state_count = LED_STATE_MAX,
        .entry_cb = led_fsm_on_entry,
        .exit_cb = led_fsm_on_exit,
        .state_names = names,
        .user_data = instance,
    };
    fsm_fill(&fsm_cfg, fsm_always_true);
    fsm_init(&instance->fsm, config->init_state, &fsm_cfg);

    static uint8_t led_cmd_buffer[LED_CMD_FIFO_SIZE * sizeof(led_cmd_t)] = {0};
    /* 分配命令队列 */
    kfifo_init(&s_cmd_fifo,led_cmd_buffer,sizeof(led_cmd_buffer), NULL);
    instance->cmd_fifo = &s_cmd_fifo;
    if (!instance->cmd_fifo)
        return LED_ERROR_NO_MEMORY;

    instance->is_static = true;
    instance->initialized = true;

    clist_add_tail(&s_led_head, &instance->node);
    return LED_OK;
}

led_handle_t* led_register(const led_config_t* config)
{
    if (config == NULL || config->name == NULL)
        return NULL;

    led_handle_t* instance = (led_handle_t*)malloc(sizeof(led_handle_t));
    if (!instance)
        return NULL;

    led_error_t err = led_register_static(config, instance);
    if (LED_IS_ERR(err)) {
        free(instance);
        return NULL;
    }
    instance->is_static = false;
    return instance;
}

led_error_t led_unregister(const char* name)
{
    if (name == NULL || !s_led_initialized)
        return LED_ERROR_INVALID_PARAM;

    led_handle_t *h, *tmp;
    clist_for_each_entry_safe(h, tmp, &s_led_head, node)
    {
        if (strcmp(h->config.name, name) == 0) {
            clist_del(&h->node);
            if (h->cmd_fifo)
                kfifo_free((kfifo_t*)h->cmd_fifo);
            if (!h->is_static)
                free(h);
            return LED_OK;
        }
    }
    return LED_ERROR_NOT_FOUND;
}

void led_set_state(led_handle_t* instance, led_state_t state)
{
    if (instance == NULL)
        return;

    led_cmd_t cmd = { .led_set_state = state };
    kfifo_put(instance->cmd_fifo, (unsigned char*)&cmd,
        sizeof(led_cmd_t));
}

led_error_t led_set_blink_interval(led_handle_t* instance,
    const led_cmd_t* cmd)
{
    if (instance == NULL || cmd == NULL)
        return LED_ERROR_INVALID_PARAM;

    /* 参数未变化则跳过 */
    bool changed = (instance->current_cmd.led_set_state != LED_STATE_BLINK_CODE) || (instance->current_cmd.led_blink_cycle_ms != cmd->led_blink_cycle_ms) || (instance->current_cmd.led_blink_wait_ms != cmd->led_blink_wait_ms) || (instance->current_cmd.led_blink_code_counts != cmd->led_blink_code_counts);
    if (!changed)
        return LED_OK;

    /* 如果在闪烁中且 LED 亮着，延迟更新 */
    if (fsm_current_state(&instance->fsm) == LED_STATE_BLINK_CODE) {
        if (instance->blink_sw_on) {
            memcpy(&instance->current_cmd, cmd, sizeof(led_cmd_t));
            instance->current_cmd.led_set_state = LED_STATE_BLINK_CODE;
            instance->pending_blink_update = true;
            return kfifo_put(instance->cmd_fifo,
                       (unsigned char*)&instance->current_cmd,
                       sizeof(led_cmd_t))
                    == sizeof(led_cmd_t)
                ? LED_OK
                : LED_ERROR_INTERNAL;
        }
        /* LED 已熄灭，立即更新参数 */
        memcpy(&instance->current_cmd, cmd, sizeof(led_cmd_t));
        instance->pending_blink_update = false;
        instance->current_led_blink_code_counts = 0;
        instance->blink_code_phase = LED_BLINK_PHASE_INTERVAL;
        instance->interval_start_time = led_get_time_now();
        instance->blink_sw_on = false;
        led_phys_write(instance, false);
        return LED_OK;
    }

    /* 不在闪烁状态，直接更新参数 */
    memcpy(&instance->current_cmd, cmd, sizeof(led_cmd_t));
    instance->pending_blink_update = false;
    return LED_OK;
}

led_blink_phase_t led_get_blink_phase(led_handle_t* instance)
{
    return instance ? (led_blink_phase_t)instance->blink_code_phase
                    : LED_BLINK_PHASE_INTERVAL;
}

void led_set_callbacks(led_handle_t* instance, led_state_change_cb_t state_cb,
    led_blink_phase_cb_t blink_phase_cb,
    led_edge_cb_t edge_cb, void* user_data)
{
    if (instance == NULL)
        return;
    instance->state_change_cb = (void*)state_cb;
    instance->blink_phase_cb = (void*)blink_phase_cb;
    instance->edge_cb = (void*)edge_cb;
    instance->callback_user_data = user_data;
}

/**
 * @brief LED 刷新任务
 * @note 遍历所有 LED 实例，依次处理命令队列、执行 FSM 步进、检测阶段变化。
 *       需在主循环或定时器中以固定周期调用。
 */
void led_task_refresh(void)
{
    if (!s_led_initialized || clist_empty(&s_led_head))
        return;

    led_handle_t* h;
    clist_for_each_entry(h, &s_led_head, node)
    {
        if (!h->initialized)
            continue;
        led_process_cmds(h);
        fsm_step(&h->fsm);
        led_check_blink_phase_change(h);
    }
}