# G1_Hand CAN-FD 电机控制通信协议
**版本**: V1.0.0 | **日期**: 2026-06-12 | **作者**: maximilian
---
## 1. 概述
G1_Hand 灵巧手控制器通过 **CAN-FD** 总线接收上位机指令，控制 **9 个手指电机**。电机通过两路 RS-485 总线连接（PORT1: 电机1-5, PORT2: 电机6-9）。
CAN-FD 物理层参数:
| 项目 | 参数 |
|------|------|
| 仲裁段波特率 | 1 Mbps |
| 数据段波特率 | 5 Mbps (BRS) |
| 收发器 | TCAN1044 |
| MCU 引脚 | PA16(TXD), PA17(RXD), PA18(STB) |
---
## 2. CAN ID 分配
| CAN ID (11-bit Std) | 方向 | 用途 |
|---------------------|------|------|
| `0x100` | Host → MCU | 控制命令 / 查询命令 |
| `0x101` | MCU → Host | 应答帧 / 心跳帧 |
---
## 3. 协议帧结构
### 3.1 帧格式
**固定长度**（查询 6 字节，普通命令 24 字节，SET_POSITION_SPEED 42 字节），所有多字节字段为**小端序 (Little-Endian)**。
| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 1 | `header` | 帧头，固定值 `'s'` (0x73) |
| 1 | 1 | `cmd` | 命令码（见 §4） |
| 2 | 1 | `flags` | 标志位（见 §5） |
| 3 | 1 | `datalen` | data 段字节数: 0 (查询), 18 (普通命令/应答), 36 (SET_POSITION_SPEED) |
| 4 | N | `data[]` | 电机数据 (N = datalen, 见 §6) |
| 4+N | 1 | `crc_check` | 累加和校验 (data[0..N-1] 字节之和的低 8 位) |
| 5+N | 1 | `ender` | 帧尾，固定值 `'e'` (0x65) |
**帧长度规则**: `frame_bytes = 6 + datalen`
| 帧类型 | datalen | 帧长 |
|--------|---------|------|
| 查询命令 | 0 | 6 bytes |
| 普通控制命令 / 应答 | 18 | 24 bytes |
| SET_POSITION_SPEED | 36 | 42 bytes |
```
帧布局示意:
┌────────┬─────┬───────┬─────────┬──────────────────┬───────────┬───────┐
│ header │ cmd │ flags │ datalen │    data[0..N-1]  │ crc_check │ ender │
│  1B    │ 1B  │  1B   │   1B    │      N bytes     │    1B     │  1B   │
│  's'   │     │       │         │  (N = datalen)   │  累加和   │  'e'  │
└────────┴─────┴───────┴─────────┴──────────────────┴───────────┴───────┘
```
### 3.2 C 语言结构体定义
```c
#define MOTOR_NUMS  9
typedef struct __attribute__((packed)) {
    uint8_t header;              // 帧头，固定 's' (0x73)
    uint8_t cmd;                 // 命令码
    uint8_t flags;               // 标志位
    uint8_t datalen;             // data 段实际字节数 (0-36)
    uint8_t data[4 * MOTOR_NUMS]; // 最大 36 字节 (9电机×2B 或 9电机×4B)
    uint8_t crc_check;            // 累加和校验（取低 8 位）
    uint8_t ender;               // 帧尾，固定 'e' (0x65)
} canfd_protocol_t;
### 3.3 帧校验规则
1. `header` 必须等于 `'s'` (0x73)
2. `datalen` 必须为 0 (查询)、18 (普通命令/应答) 或 36 (SET_POSITION_SPEED)
3. `ender` 必须等于 `'e'` (0x65)
4. CAN-FD DLC 对应字节数必须等于 `6 + datalen`
5. `crc_check` 必须等于 `(data[0] + data[1] + ... + data[datalen-1]) & 0xFF`（累加和取低 8 位）。仅覆盖 data 段，不包含 header/cmd/flags/datalen。
6. 以上任一条件不满足，帧将被丢弃
---
## 4. 命令码 (CMD) 定义
### 4.1 控制命令 (Host → MCU, CAN ID 0x100)
| CMD | 助记符 | data[2 × id] 格式 | 说明 |
|-----|--------|-------------------|------|
| `0x01` | SET_SPEED | `int16 LE` | 目标速度 (RPM)，bit15=1 正转 |
| `0x02` | SET_POSITION | `uint16 LE` | 目标位置 (度 × 10)，范围 0–3600 |
| `0x03` | SET_CURRENT | `uint16 LE` | 电流/力矩限制 (mA 或原始值 0–16384) |
| `0x04` | START | 任意非零 | 启动电机 (非零即执行) |
| `0x05` | STOP | 任意非零 | 急停电机 (非零即执行) |
| `0x06` | CLEAR_FAULT | 任意非零 | 清除电机故障 (非零即执行) |
| `0x07` | SET_POSITION_SPEED | `uint16 LE` + `int16 LE` | 位置+速度闭环 (4B/电机: 前2B=pos, 后2B=speed) |
> **注意**: `data[2×id]` 为零表示**跳过**该电机。控制命令仅对 data 非零的电机生效。
### 4.2 查询命令 (Host → MCU, CAN ID 0x100)
| CMD | 助记符 | data 段 | 说明 |
|-----|--------|---------|------|
| `0x10` | QUERY_STATUS | 忽略 | 查询电机状态 |
| `0x11` | QUERY_POSITION | 忽略 | 查询电角度位置 |
| `0x12` | QUERY_SPEED | 忽略 | 查询当前速度 |
| `0x13` | QUERY_CURRENT | 忽略 | 查询当前电流 |
查询命令**立即返回缓存值**（不发起 RS-485 通讯）。
### 4.3 应答命令 (MCU → Host, CAN ID 0x101)
**控制应答** — CMD 与原始控制命令相同 (0x01–0x06), datalen=18，始终包含全部 9 电机:
| data[2×id] | 格式 | 说明 |
|------------|------|------|
| `uint16 LE` | 错误码 | 操作的电机位置为 `0x0000` (OK) 或错误码，未操作电机位置为 `0x0000` |
**查询应答** — CMD 与查询命令相同 (0x10–0x13):
| CMD | data[2×id] 格式 | 说明 |
|-----|-----------------|------|
| `0x10` RESP_STATUS | `uint16 LE` | 状态字: [motor_status:8][mode:4][fault:4] |
| `0x11` RESP_POSITION | `uint16 LE` | 电角度 (度 × 10)，范围 0–3599 |
| `0x12` RESP_SPEED | `int16 LE` | 当前速度 (RPM) |
| `0x13` RESP_CURRENT | `uint16 LE` | 当前电流 (mA 或原始值) |
### 4.4 心跳帧 (MCU → Host, CAN ID 0x101)
| CMD | data[0..1] | data[2..3] | data[4..17] |
|-----|------------|------------|-------------|
| `0xF0` | `[bus_state:u8][motor_count:u8]` | `fault_bitmap:u16 LE` | 保留 (0x00) |
**bus_state**: 0=ErrorActive, 1=ErrorWarning, 2=ErrorPassive, 3=BusOff
**fault_bitmap**: bit i 对应电机 i+1 有故障 (i = 0..8)
**发送周期**: 100 ms
---
## 5. 标志位 (flags) 定义
| Bit | 名称 | 值 | 说明 |
|-----|------|---|------|
| 0 | ACK_REQUEST | `0x01` | 主机需要此命令的应答帧 |
| 1–7 | — | 0 | 保留，必须为 0 |
**规则**:
- `ACK_REQUEST = 1`: MCU 在 RS-485 应答完成后构建控制应答帧 (CAN ID 0x101)，CMD 与原始命令相同
- `ACK_REQUEST = 0`: 单向命令，MCU 不发送应答 (仅更新本地缓存)
- 查询命令默认可视为 `ACK_REQUEST = 1`（总是需要应答）
---
## 6. data 段编址
data 段固定 18 或 36 字节，始终包含全部 9 个电机，按电机 ID 编址。
**普通命令 (2 字节/电机, CMD 0x01-0x06, 0x10-0x13)**: datalen = 18
| 电机 ID | data 偏移 | 说明 |
|---------|----------|------|
| 1 | `data[0..1]` | 低字节+高字节, uint16 LE |
| N | `data[(N-1)*2..(N-1)*2+1]` | |
**SET_POSITION_SPEED (4 字节/电机, CMD 0x07)**: datalen = 36
| 电机 ID | data 偏移 | 说明 |
|---------|----------|------|
| 1 | `data[0..1]`=pos, `data[2..3]`=speed | uint16 LE + int16 LE |
| N | `data[(N-1)*4..(N-1)*4+3]` | |
---
## 7. 错误码 (Error Code)
| 值 | 名称 | 说明 |
|----|------|------|
| `0x0000` | MOTOR_CONTROL_OK | 操作成功 |
| `0x0001` | MOTOR_CONTROL_ERROR_MOTOR_NOT_FOUND | 电机未找到 |
| `0x0002` | MOTOR_CONTROL_ERROR_MOTOR_BUSY | 电机忙 (前一条命令尚未完成) |
| `0x0003` | MOTOR_CONTROL_ERROR_INVALID_PARAM | 无效参数 |
| `0x0004` | MOTOR_CONTROL_ERROR_TIMEOUT | RS-485 应答超时 |
| `0x0005` | MOTOR_CONTROL_ERROR_RS485_FAIL | RS-485 通讯失败 |
| `0x0006` | MOTOR_CONTROL_ERROR_MOTOR_FAULT | 电机故障 |
---
## 8. 通信时序
### 8.1 控制命令 (无应答)
| 步骤 | 方向 | 内容 |
|------|------|------|
| 1 | Host → MCU | CAN ID `0x100`, CMD=`0x01` SET_SPEED, motor1=3000, flags=0 |
| 2 | MCU → Motor | `finger_set_speed()` 通过 RS-485 |
| 3 | Motor → MCU | RS-485 应答帧 |
| 4 | MCU (内部) | 更新本地缓存，不应答 CAN-FD |
### 8.2 控制命令 (有应答)
| 步骤 | 方向 | 内容 |
|------|------|------|
| 1 | Host → MCU | CAN ID `0x100`, CMD=`0x02` SET_POSITION, flags=`0x01` |
| 2 | MCU → Motor | `finger_set_position()` 通过 RS-485 |
| 3 | Motor → MCU | RS-485 应答帧 |
| 4 | MCU (内部) | 更新缓存，构建 ACK 帧 |
| 5 | MCU → Host | CAN ID `0x101`, CMD=`0x02`, data[motor_id]=`0x0000` (OK) |
### 8.3 查询命令
| 步骤 | 方向 | 内容 |
|------|------|------|
| 1 | Host → MCU | CAN ID `0x100`, CMD=`0x10` QUERY_STATUS |
| 2 | MCU (内部) | 读取本地缓存 |
| 3 | MCU → Host | CAN ID `0x101`, CMD=`0x10`, data[1..9] = 全部电机状态 |
---
## 9. 示例
### 9.1 设置全部电机速度 (正转 3000 RPM)
**CAN ID**: 0x100, **datalen**: 18, **帧长**: 24 bytes
| Offset | Field | Value | Description |
|--------|-------|-------|-------------|
| 0 | header | `0x73` | `'s'` |
| 1 | cmd | `0x01` | SET_SPEED |
| 2 | flags | `0x00` | 不应答 |
| 3 | datalen | `0x12` | 18 (9 电机 × 2 字节) |
| 4–21 | data | `B8 0B B8 0B ...` | 每电机: 3000 RPM = 0x0BB8 (LE) |
| 22 | crc_check | `0xDB` | sum(data[0..17]) & 0xFF |
| 23 | ender | `0x65` | `'e'` |
```
完整帧: 73 01 00 12 B8 0B B8 0B B8 0B B8 0B B8 0B B8 0B B8 0B B8 0B B8 0B DB 65
```
### 9.2 设置电机1位置 90.0°，其余电机不变
**CAN ID**: 0x100, **datalen**: 18, **帧长**: 24 bytes
| Offset | Field | Value | Description |
|--------|-------|-------|-------------|
| 0 | header | `0x73` | `'s'` |
| 1 | cmd | `0x02` | SET_POSITION |
| 2 | flags | `0x01` | 需要应答 |
| 3 | datalen | `0x12` | 18 (固定 9 电机 × 2 字节) |
| 4–5 | data[0..1] | `84 03` | 电机1: 900 (90.0° × 10) = 0x0384 (LE) |
| 6–21 | data[2..17] | `00 00 ...` | 电机2–9: 0x0000 (跳过，不改变) |
| 22 | crc_check | `0x87` | sum(data[0..17]) = 0x84+0x03+0x00×16 = 0x87 |
| 23 | ender | `0x65` | `'e'` |
```
完整帧: 73 02 01 12 84 03 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 87 65
```
### 9.3 查询全部电机状态
**查询帧** (datalen=0, 6 bytes):
| Offset | 0 | 1 | 2 | 3 | 4 | 5 |
|--------|---|---|---|---|---|---|
| Field | header | cmd | flags | datalen | crc_check | ender |
| Value | `0x73` | `0x10` | `0x00` | `0x00` | `0x00` | `0x65` |
crc_check = 0x00（datalen=0，无 data 字节，校验和为 0）
**应答帧** (datalen=18, 24 bytes):
| Offset | 0 | 1 | 2 | 3 | 4–21 | 22 | 23 |
|--------|---|---|---|---|------|----|----|
| Field | header | cmd | flags | datalen | data[0..17] | crc_check | ender |
| Value | `0x73` | `0x10` | `0x00` | `0x12` | 9×uint16 LE | XX | `0x65` |
### 9.4 心跳帧
**CAN ID**: 0x101, **datalen**: 4, **帧长**: 10 bytes
| Offset | Field | Value | Description |
|--------|-------|-------|-------------|
| 0 | header | `0x73` | `'s'` |
| 1 | cmd | `0xF0` | HEARTBEAT |
| 2 | flags | `0x00` | — |
| 3 | datalen | `0x04` | 4 |
| 4 | data[0] | `0x00` | bus_state (0=OK) |
| 5 | data[1] | `0x09` | motor_count (9) |
| 6–7 | data[2..3] | `00 00` | fault_bitmap (LE) |
| 8 | crc_check | `0x09` | sum(data[0..3]) = 0x00+0x09+0x00+0x00 = 0x09 |
| 9 | ender | `0x65` | `'e'` |
---
## 10. 限制与约束
1. **每电机单命令**: 同一电机同一时间只能有一个待处理 RS-485 命令。主机需等待应答（或超时）后再发下一条。
2. **RS-485 应答超时**: 50 ms（由 finger_task 管理）
3. **RS-485 命令间隔**: ≥ 5 ms（由 finger_task FSM 管理）
4. **心跳周期**: 100 ms（固定）
5. **查询非实时**: 查询命令返回本地缓存，不发起 RS-485 通讯。如需最新数据，先发控制命令触发 RS-485 交换。
6. **SET_CURRENT 限制**: finger 服务无独立电流控制 API，SET_CURRENT 通过复用 `finger_set_position()` 的力矩限制参数实现。
---
## 11. 协议优缺点
### 优点
1. **单帧多电机**: 利用 CAN-FD 64 字节容量，一帧可同时控制全部 9 个电机，总线利用率高。
2. **固定帧长, 解析简单**: 查询 6 字节、普通命令 24 字节、SET_POSITION_SPEED 42 字节。解析器无需处理可变长度边界，O(1) 定位帧尾。
3. **简洁帧结构**: 仅 7 个字段，无复杂的嵌套 TLV 或可变长度段。解析器通过 `datalen` 直接计算帧尾位置，O(1) 定位。
4. **硬件 CRC + 软件累加和双重保护**: CAN-FD 硬件 CRC 保证链路层完整性，协议层累加和校验覆盖 data 段，防御软件编解码错误。
5. **统一的 CAN ID 方案**: 仅 2 个 CAN ID (`0x100` 下行, `0x101` 上行)，无需 ID 查表。CMD 字节区分帧类型，扩展性好 (0x00-0xFF)。
6. **ACK 可选**: `flags bit0` 控制是否需要应答。高频控制命令可设为单向 (flags=0)，减少总线负载。
7. **复用现有中间件**: 使用 `protocol_packer`/`protocol_parser` 统一帧打包/解析，与 RS-485 电机协议共用基础设施。
### 缺点
1. **固定电机编址**: 电机 ID 必须连续 (1-9)，不支持稀疏 ID 或动态拓扑。新增电机需扩展 `data[]` 数组。
2. **无时间戳/序列号**: 帧中无 timestamp 或 sequence 字段，上位机无法判断帧是否乱序或重传 (CAN-FD 硬件层面保证顺序，但应用层无法追踪)。
3. **查询读缓存**: 查询命令返回本地缓存而非实时 RS-485 数据。缓存仅在 RS-485 应答后更新，查询结果可能滞后。
4. **无紧急抢占**: 无优先级机制。若某电机正在等待 RS-485 应答 (最长 50ms)，新的控制命令对该电机返回 BUSY，需主机重试。
