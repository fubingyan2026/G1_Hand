# LED 控制模块 (led)

<p align='right'>版本: 2.0.0 | 作者: max </p>

支持常亮、熄灭、编码闪烁三种工作状态的 LED 控制模块。采用 **FSM 状态机** 管理状态转换，**kfifo 异步命令队列** 接收外部指令，**clist 侵入式链表** 管理实例。

---

## 核心特性

- **异步命令架构** — 内置 `kfifo` 命令队列，状态切换和参数修改均为异步执行，不阻塞调用者。
- **FSM 状态管理** — 通过状态机优雅管理 ON/OFF/BLINK_CODE 状态转换，支持进入/退出回调。
- **硬件解耦** — 引脚操作通过 `write_pin`/`read_pin` 回调注入，模块不直接依赖 HAL。
- **闪烁参数热更新** — 闪烁过程中可动态修改频率/次数，亮着时延迟到熄灭后生效，避免毛刺。
- **内存安全** — 支持**静态分配**与**动态分配**，反初始化时自动识别并安全回收资源。

---

## 配置结构体

### `led_config_t` — LED 基础配置

| 字段 | 类型 | 说明 |
| :--- | :--- | :--- |
| `name` | `const char*` | LED 唯一名称，用于 `led_get_instance()` 查找 |
| `init_state` | `led_state_t` | 注册后的初始状态 |
| `write_pin` | `void (*)(bool on)` | 引脚写入回调：`true`=逻辑亮，`false`=逻辑灭 |
| `read_pin` | `bool (*)(void)` | 引脚读取回调：返回逻辑电平 |

写入回调用户自行实现，内部处理 GPIO 端口/引脚和有效电平映射。

### `led_cmd_t` — 异步命令

| 字段 | 类型 | 说明 |
| :--- | :--- | :--- |
| `led_set_state` | `led_state_t` | 目标状态 |
| `led_blink_cycle_ms` | `uint16_t` | 闪烁间隔 (ms) |
| `led_blink_wait_ms` | `uint16_t` | 等待间隔 (ms) |
| `led_blink_code_counts` | `uint16_t` | 闪烁次数（`0`=无限循环） |

---

## 使用指南

### 1. 初始化系统

```c
#include "led.h"

// 注入系统毫秒计数函数
led_init(HAL_GetTick);
```

### 2. 实现引脚回调

```c
static void led_write_pin(bool on) {
    // GPIOB PIN9: 低电平有效
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9,
                      on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static bool led_read_pin(void) {
    return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_RESET;
}
```

### 3. 注册 LED 实例

**静态注册（推荐）:**

```c
static led_handle_t s_led;

const led_config_t cfg = {
    .name = "LED0",
    .init_state = LED_STATE_BLINK_CODE,
    .write_pin = led_write_pin,
    .read_pin = led_read_pin,
};
led_register_static(&cfg, &s_led);
```

**动态注册:**

```c
led_handle_t* led = led_register(&cfg);
```

### 4. 注册回调

```c
void on_state_change(led_handle_t* h, led_state_t s, void* ud) { ... }
void on_blink_phase(led_handle_t* h, led_blink_phase_t p, void* ud) { ... }
void on_edge(led_handle_t* h, bool rising, void* ud) { ... }

led_set_callbacks(led, on_state_change, on_blink_phase, on_edge, NULL);
```

### 5. 控制 LED

```c
led_set_state(led, LED_STATE_ON);      // 常亮
led_set_state(led, LED_STATE_OFF);     // 熄灭
led_set_state(led, LED_STATE_BLINK_CODE);  // 编码闪烁
```

**配置闪烁参数（异步命令）:**

```c
// 闪烁 3 次，每次间隔 100ms，每轮间隔 1000ms，完成后自动关闭
led_set_blink_interval(led, &(led_cmd_t){
    .led_blink_cycle_ms = 100,
    .led_blink_wait_ms = 1000,
    .led_blink_code_counts = 3,
});
led_set_state(led, LED_STATE_BLINK_CODE);
```

### 6. 任务刷新

```c
// 需在主循环或定时器中定期调用（建议周期 ≤ 10ms）
led_task_refresh();
```

---

## 编码闪烁行为

闪烁由两个阶段构成循环：

- **BLINKING** — 按 `led_blink_cycle_ms` 间隔翻转引脚电平，每个下降沿（亮→灭）计数一次
- **INTERVAL** — 保持熄灭 `led_blink_wait_ms`，等待结束后继续下一轮闪烁

| `led_blink_code_counts` | 行为 |
| :---------------------- | :--- |
| `0` | 无限循环，不会自动关闭 |
| `> 0` | 闪烁指定次数后自动切换到 `LED_STATE_OFF` |

当闪烁参数需要热更新时，如果 LED 正处于亮状态，模块会延迟到熄灭后才应用新参数，避免引脚产生异常脉冲。

---

## API 参考

| 函数 | 说明 |
| :--- | :--- |
| `led_init(cb)` | 初始化 LED 子系统，注入时间回调 |
| `led_deinit()` | 反初始化，释放所有资源 |
| `led_register(cfg)` | 动态注册 LED 实例 |
| `led_register_static(cfg, h)` | 静态注册 LED，使用预分配内存 |
| `led_unregister(name)` | 注销并释放指定名称的 LED |
| `led_get_instance(name)` | 根据名称获取 LED 句柄 |
| `led_get_head()` | 获取 LED 链表头 |
| `led_set_state(h, state)` | 异步设置 LED 运行状态 |
| `led_set_blink_interval(h, cmd)` | 异步配置闪烁参数 |
| `led_get_blink_phase(h)` | 获取当前闪烁阶段 |
| `led_set_callbacks(...)` | 注册状态/闪烁阶段/边沿回调 |
| `led_task_refresh()` | **核心任务**，需在主循环中定时调用 |

---

## 注意事项

1. **刷新频率**: `led_task_refresh()` 的调用频率决定了闪烁精度，建议周期 ≤ 10ms。
2. **引脚初始化**: `led_register` 前需要调用者自行完成 GPIO 初始化（模式、速度、默认电平）。
3. **FIFO 深度**: 命令队列深度为 8，若在极短时间内发送超过 8 条命令，旧指令将被覆盖。
4. **空指针安全**: 所有公共 API 均对 `NULL` 参数做保护处理。