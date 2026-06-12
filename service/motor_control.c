/*
 * Copyright (c) 2026 G1_Hand 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    motor_control.c
 * @author  maximilian
 * @version V2.0.0
 * @date    2026-06-11
 * @brief   CAN-FD 电机控制桥接服务层实现
 *
 * 协议帧: [header='s'][cmd][flags][datalen][data:N][crc_check(累加和)][ender='e']
 * 可变长度: 6+N 字节 (N=datalen, 0/18/36)
 */

/* Includes ------------------------------------------------------------------*/
#include "motor_control.h"

#include "log.h"

#include <string.h>

/* Private constants ---------------------------------------------------------*/

/* (无) */

/* Private variables ---------------------------------------------------------*/

/** @brief 实例链表头 */
static clist_head_t s_motor_control_head;

/** @brief 子系统初始化标志 */
static bool s_motor_control_initialized;

/* Private function prototypes -----------------------------------------------*/

/**
 * @brief 处理单个电机的控制命令
 * @param handle 电机句柄
 * @param cmd 命令码
 * @param val 2 字节数据值（小端序已解析）
 * @return 错误码
 */
static motor_control_error_t motor_control_dispatch_control(
    motor_control_handle_t* handle, uint8_t cmd, uint16_t val);

/**
 * @brief 构建查询应答帧（填充全部 9 个电机的缓存值）
 * @param cmd 查询命令码
 * @param p_response 输出协议帧
 */
static void motor_control_build_query_response(uint8_t cmd,
    canfd_protocol_t* p_response);

/**
 * @brief 在 canfd_protocol_t 的 data 段设置 uint16 值
 * @param proto 协议帧
 * @param motor_id 电机 ID (1-9)
 * @param val 16-bit 值
 */
static inline void motor_control_set_data(canfd_protocol_t* proto,
    uint8_t motor_id, uint16_t val)
{
    MOTOR_CONTROL_SET_DATA_U16(proto, motor_id, val);
}

/*
 * ============================================================================
 * 公共 API — 子系统生命周期
 * ============================================================================
 */

motor_control_error_t motor_control_init(void)
{
    if (s_motor_control_initialized) {
        return MOTOR_CONTROL_OK;
    }

    clist_init(&s_motor_control_head);
    s_motor_control_initialized = true;

    return MOTOR_CONTROL_OK;
}

void motor_control_deinit(void)
{
    motor_control_handle_t* h;
    motor_control_handle_t* tmp;

    if (!s_motor_control_initialized) {
        return;
    }

    clist_for_each_entry_safe(h, tmp, &s_motor_control_head, node)
    {
        clist_del(&h->node);
        h->initialized = false;
        h->finger = NULL;
    }

    clist_init(&s_motor_control_head);
    s_motor_control_initialized = false;
}

bool motor_control_is_initialized(void)
{
    return s_motor_control_initialized;
}

/*
 * ============================================================================
 * 公共 API — 实例管理
 * ============================================================================
 */

motor_control_error_t motor_control_register_static(
    const motor_control_config_t* config,
    motor_control_handle_t* instance)
{
    if (!config || !instance) {
        return MOTOR_CONTROL_ERROR_NULL_PTR;
    }

    if (!s_motor_control_initialized) {
        return MOTOR_CONTROL_ERROR_UNINITIALIZED;
    }

    if (config->motor_id == 0 || config->motor_id > 9) {
        return MOTOR_CONTROL_ERROR_INVALID_PARAM;
    }

    if (motor_control_get_by_id(config->motor_id)) {
        return MOTOR_CONTROL_ERROR_ALREADY_EXIST;
    }

    if (instance->initialized) {
        clist_del(&instance->node);
        instance->initialized = false;
        instance->finger = NULL;
    }

    instance->config = *config;

    finger_handle_t* finger = finger_get_by_id(config->finger_motor_id,
        config->finger_port);
    if (!finger) {
        LOG_E("motor_ctrl", "未找到手指电机, motor_id=%u, 端口=%u",
            (unsigned int)config->finger_motor_id,
            (unsigned int)config->finger_port);
        return MOTOR_CONTROL_ERROR_NOT_FOUND;
    }
    instance->finger = finger;

    instance->status = 0;
    instance->position = 0;
    instance->mode = FINGER_MODE_IDLE;
    instance->speed = 0;
    instance->current = 0;
    instance->fault_flags = 0;
    instance->has_pending_response = false;
    memset(&instance->response_frame, 0, sizeof(canfd_protocol_t));
    instance->pending_cmd = 0;
    instance->command_in_flight = false;
    instance->ack_requested = false;

    clist_add_tail(&s_motor_control_head, &instance->node);
    instance->initialized = true;

    LOG_I("motor_ctrl", "已注册电机 %u → finger_id=%u, 端口=%u",
        (unsigned int)config->motor_id,
        (unsigned int)config->finger_motor_id,
        (unsigned int)config->finger_port);

    return MOTOR_CONTROL_OK;
}

motor_control_error_t motor_control_unregister(const char* name)
{
    motor_control_handle_t* h;

    if (!name) {
        return MOTOR_CONTROL_ERROR_NULL_PTR;
    }

    if (!s_motor_control_initialized) {
        return MOTOR_CONTROL_ERROR_UNINITIALIZED;
    }

    clist_for_each_entry(h, &s_motor_control_head, node)
    {
        if (strcmp(h->config.name, name) == 0) {
            clist_del(&h->node);
            h->initialized = false;
            h->finger = NULL;
            return MOTOR_CONTROL_OK;
        }
    }

    return MOTOR_CONTROL_ERROR_NOT_FOUND;
}

motor_control_handle_t* motor_control_get_by_id(uint8_t motor_id)
{
    motor_control_handle_t* h;

    if (!s_motor_control_initialized) {
        return NULL;
    }

    clist_for_each_entry(h, &s_motor_control_head, node)
    {
        if (h->config.motor_id == motor_id) {
            return h;
        }
    }

    return NULL;
}

clist_head_t* motor_control_get_head(void)
{
    if (!s_motor_control_initialized) {
        return NULL;
    }

    return &s_motor_control_head;
}

/*
 * ============================================================================
 * 公共 API — CAN-FD 帧处理
 * ============================================================================
 */

motor_control_error_t motor_control_process_rx_frame(
    const canfd_protocol_t* protocol)
{
    if (!protocol) {
        return MOTOR_CONTROL_ERROR_NULL_PTR;
    }

    if (!s_motor_control_initialized) {
        return MOTOR_CONTROL_ERROR_UNINITIALIZED;
    }

    /*
     * 帧头/帧尾/CRC 已由 protocol_parser 层完成校验，
     * 此处仅校验 header（偏移量 0 对所有帧大小均正确）做最后防御。
     */
    if (protocol->header != MOTOR_CONTROL_HEADER_VAL) {
        return MOTOR_CONTROL_ERROR_BAD_FRAME;
    }

    uint8_t cmd = protocol->cmd;
    uint8_t flags = protocol->flags;
    uint8_t datalen = protocol->datalen;
    bool ack_requested = (flags & MOTOR_CONTROL_FLAG_ACK_REQUEST) != 0;

    /* 判断命令类型 */
    bool is_control = (cmd >= MOTOR_CONTROL_CMD_SET_SPEED
        && cmd <= MOTOR_CONTROL_CMD_SET_POSITION_SPEED);
    bool is_query = (cmd >= MOTOR_CONTROL_CMD_QUERY_STATUS
        && cmd <= MOTOR_CONTROL_CMD_QUERY_CURRENT);

    /* 校验 datalen — 固定长度: 查询=0, 2B命令=18, 4B命令=36 */
    bool is_4byte_cmd = (cmd == MOTOR_CONTROL_CMD_SET_POSITION_SPEED);
    uint8_t expected_datalen = is_4byte_cmd ? MOTOR_CONTROL_DATA_MAX
                                            : (is_query ? 0U : MOTOR_CONTROL_DATA_BYTES);

    if (datalen != expected_datalen) {
        LOG_W("motor_ctrl", "帧 datalen 不匹配: cmd=%s(0x%02X) datalen=%u 期望=%u",
            motor_control_cmd_name(cmd), (unsigned int)cmd,
            (unsigned int)datalen, (unsigned int)expected_datalen);
        return MOTOR_CONTROL_ERROR_BAD_FRAME;
    }

    /* 打印接收到的 CAN 命令 */
    LOG_I("motor_ctrl", "收到 CAN 帧 → %s(0x%02X) flags=0x%02X datalen=%u 应答=%s",
        motor_control_cmd_name(cmd), (unsigned int)cmd,
        (unsigned int)flags, (unsigned int)datalen,
        ack_requested ? "是" : "否");

    motor_control_error_t result = MOTOR_CONTROL_OK;

    if (is_control) {
        if (is_4byte_cmd) {
            /* SET_POSITION_SPEED: 固定 36 字节，4 字节/电机 [pos:u16][speed:i16] */
            LOG_I("motor_ctrl", "  → 位置+速度闭环命令, 遍历9电机...");
            for (uint8_t motor_id = 1; motor_id <= MOTOR_CONTROL_MOTOR_NUMS; motor_id++) {
                uint16_t pos = MOTOR_CONTROL_DATA_POS(protocol, motor_id);
                int16_t speed_val = MOTOR_CONTROL_DATA_SPD(protocol, motor_id);

                if (pos == 0 && speed_val == 0) {
                    continue;
                }

                motor_control_handle_t* handle = motor_control_get_by_id(motor_id);
                if (!handle) {
                    LOG_W("motor_ctrl", "  → 电机%u 未注册, 跳过", (unsigned int)motor_id);
                    result = MOTOR_CONTROL_ERROR_NOT_FOUND;
                    continue;
                }
                if (handle->command_in_flight) {
                    LOG_W("motor_ctrl", "  → 电机%u 忙(上一条命令未完成), 跳过",
                        (unsigned int)motor_id);
                    result = MOTOR_CONTROL_ERROR_MOTOR_BUSY;
                    continue;
                }

                LOG_I("motor_ctrl", "  → RS-485 [电机%u] 位置+速度闭环: pos=%.1f° speed=%d RPM",
                    (unsigned int)motor_id, (float)pos / 10.0f, (int)speed_val);

                finger_error_t ferr = finger_set_position_speed(
                    handle->finger, (uint32_t)pos, (uint32_t)(int32_t)speed_val);
                if (FINGER_IS_OK(ferr)) {
                    handle->position = pos;
                    handle->speed = speed_val;
                    handle->command_in_flight = true;
                    handle->pending_cmd = cmd;
                    handle->ack_requested = ack_requested;
                    handle->has_pending_response = false;
                } else {
                    LOG_E("motor_ctrl", "  → RS-485 [电机%u] 位置+速度命令失败, 错误码=%d",
                        (unsigned int)motor_id, (int)ferr);
                    result = MOTOR_CONTROL_ERROR_RS485_FAIL;
                }
            }
        } else {
            /* 普通控制命令：固定 18 字节，2 字节/电机 × 9 */
            LOG_I("motor_ctrl", "  → %s, 遍历9电机...", motor_control_cmd_name(cmd));
            for (uint8_t motor_id = 1; motor_id <= MOTOR_CONTROL_MOTOR_NUMS; motor_id++) {
                uint16_t val = MOTOR_CONTROL_DATA_U16(protocol, motor_id);

                if (val == 0) {
                    continue;
                }

                motor_control_handle_t* handle = motor_control_get_by_id(motor_id);
                if (!handle) {
                    LOG_W("motor_ctrl", "  → 电机%u 未注册, 跳过", (unsigned int)motor_id);
                    result = MOTOR_CONTROL_ERROR_NOT_FOUND;
                    continue;
                }
                if (handle->command_in_flight) {
                    LOG_W("motor_ctrl", "  → 电机%u 忙(上一条命令未完成), 跳过",
                        (unsigned int)motor_id);
                    result = MOTOR_CONTROL_ERROR_MOTOR_BUSY;
                    continue;
                }

                motor_control_error_t dispatch_err = motor_control_dispatch_control(
                    handle, cmd, val);
                if (MOTOR_CONTROL_IS_OK(dispatch_err)) {
                    handle->command_in_flight = true;
                    handle->pending_cmd = cmd;
                    handle->ack_requested = ack_requested;
                    handle->has_pending_response = false;
                } else {
                    result = dispatch_err;
                }
            }
        }
    } else if (is_query) {
        /* 查询命令：从缓存构建应答帧 */
        LOG_I("motor_ctrl", "  → 查询命令, 从缓存构建应答帧...");
        canfd_protocol_t response;
        motor_control_build_query_response(cmd, &response);

        /* 存入第一个有效的 handle（task 层统一扫描发送） */
        motor_control_handle_t* first = clist_first_entry(
            &s_motor_control_head, motor_control_handle_t, node);
        if (first) {
            first->response_frame = response;
            first->has_pending_response = true;
            LOG_I("motor_ctrl", "  → 查询应答帧已就绪, 待发送");
        }
    } else {
        LOG_W("motor_ctrl", "  → 未知命令类型 cmd=0x%02X", (unsigned int)cmd);
        result = MOTOR_CONTROL_ERROR_INVALID_PARAM;
    }

    if (MOTOR_CONTROL_IS_ERR(result)) {
        LOG_W("motor_ctrl", "  → 帧处理完成, 有错误: %d", (int)result);
    }

    return result;
}

bool motor_control_pop_response(motor_control_handle_t* handle,
    canfd_protocol_t* p_response)
{
    if (!handle || !handle->initialized) {
        return false;
    }

    if (!handle->has_pending_response) {
        return false;
    }

    if (p_response) {
        *p_response = handle->response_frame;
    }

    LOG_D("motor_ctrl", "弹出应答帧: 电机%u cmd=0x%02X(%s) datalen=%u",
        (unsigned int)handle->config.motor_id,
        (unsigned int)handle->response_frame.cmd,
        motor_control_cmd_name(handle->response_frame.cmd),
        (unsigned int)handle->response_frame.datalen);

    handle->has_pending_response = false;

    return true;
}

/*
 * ============================================================================
 * 公共 API — 状态查询（本地缓存）
 * ============================================================================
 */

uint8_t motor_control_get_status(const motor_control_handle_t* handle)
{
    if (!handle || !handle->initialized) {
        return 0;
    }
    return handle->status;
}

uint16_t motor_control_get_position(const motor_control_handle_t* handle)
{
    if (!handle || !handle->initialized) {
        return 0;
    }
    return handle->position;
}

finger_mode_t motor_control_get_mode(const motor_control_handle_t* handle)
{
    if (!handle || !handle->initialized) {
        return FINGER_MODE_IDLE;
    }
    return handle->mode;
}

uint16_t motor_control_get_current(const motor_control_handle_t* handle)
{
    if (!handle || !handle->initialized) {
        return 0;
    }
    return handle->current;
}

int16_t motor_control_get_speed(const motor_control_handle_t* handle)
{
    if (!handle || !handle->initialized) {
        return 0;
    }
    return handle->speed;
}

/*
 * ============================================================================
 * 公共 API — 批量操作
 * ============================================================================
 */

motor_control_error_t motor_control_emergency_stop_all(void)
{
    motor_control_handle_t* h;

    if (!s_motor_control_initialized) {
        return MOTOR_CONTROL_ERROR_UNINITIALIZED;
    }

    LOG_W("motor_ctrl", "紧急停止全部电机!");

    clist_for_each_entry(h, &s_motor_control_head, node)
    {
        if (h->finger) {
            (void)finger_emergency_stop(h->finger);
        }
    }

    return MOTOR_CONTROL_OK;
}

/*
 * ============================================================================
 * 公共 API — finger 应答回调（内部接口）
 * ============================================================================
 */

void motor_control_on_finger_response(finger_handle_t* finger_handle,
    finger_cmd_type_t cmd_type, bool success, void* user_data)
{
    (void)cmd_type;

    if (!finger_handle || !user_data) {
        return;
    }

    motor_control_handle_t* handle = (motor_control_handle_t*)user_data;

    if (!handle->initialized) {
        return;
    }

    /* 1. 更新缓存 */
    handle->status = finger_get_status(finger_handle);
    handle->position = finger_get_position(finger_handle);
    handle->mode = finger_get_mode(finger_handle);

    LOG_D("motor_ctrl", "电机%u RS-485 应答: %s, 状态=0x%02X, 位置=%u",
        (unsigned int)handle->config.motor_id,
        success ? "成功" : "失败",
        (unsigned int)handle->status,
        (unsigned int)handle->position);

    /* 2. 若主机未请求应答，直接清除飞行标志 */
    if (!handle->ack_requested) {
        handle->command_in_flight = false;
        return;
    }

    /* 3. 构建控制应答帧，error_code 写入 data[(motor_id-1)*2] 位置 */
    canfd_protocol_t resp;
    memset(&resp, 0, sizeof(resp));
    resp.cmd = handle->pending_cmd;
    resp.flags = 0;
    resp.datalen = MOTOR_CONTROL_DATA_BYTES; /* 固定 18 字节 = 9 电机 × 2 字节 */

    uint16_t error_code = success ? (uint16_t)MOTOR_CONTROL_OK
                                  : (uint16_t)MOTOR_CONTROL_ERROR_MOTOR_FAULT;
    motor_control_set_data(&resp, handle->config.motor_id, error_code);

    LOG_I("motor_ctrl", "电机%u 应答帧已构建: cmd=%s error=0x%04X",
        (unsigned int)handle->config.motor_id,
        motor_control_cmd_name(handle->pending_cmd),
        (unsigned int)error_code);

    /* 4. 存入应答帧 */
    handle->response_frame = resp;
    handle->has_pending_response = true;
    handle->command_in_flight = false;
    handle->ack_requested = false;
}

/*
 * ============================================================================
 * 公共工具函数 — 命令码转中文助记符
 * ============================================================================
 */

const char* motor_control_cmd_name(uint8_t cmd)
{
    switch (cmd) {
    case MOTOR_CONTROL_CMD_SET_SPEED:
        return "设置速度";
    case MOTOR_CONTROL_CMD_SET_POSITION:
        return "设置位置";
    case MOTOR_CONTROL_CMD_SET_CURRENT:
        return "设置电流";
    case MOTOR_CONTROL_CMD_START:
        return "启动";
    case MOTOR_CONTROL_CMD_STOP:
        return "急停";
    case MOTOR_CONTROL_CMD_CLEAR_FAULT:
        return "清除故障";
    case MOTOR_CONTROL_CMD_SET_POSITION_SPEED:
        return "位置+速度闭环";
    case MOTOR_CONTROL_CMD_QUERY_STATUS:
        return "查询状态";
    case MOTOR_CONTROL_CMD_QUERY_POSITION:
        return "查询位置";
    case MOTOR_CONTROL_CMD_QUERY_SPEED:
        return "查询速度";
    case MOTOR_CONTROL_CMD_QUERY_CURRENT:
        return "查询电流";
    case MOTOR_CONTROL_CMD_HEARTBEAT:
        return "心跳";
    default:
        return "未知命令";
    }
}

/*
 * ============================================================================
 * 私有函数
 * ============================================================================
 */

static motor_control_error_t motor_control_dispatch_control(
    motor_control_handle_t* handle, uint8_t cmd, uint16_t val)
{
    if (!handle || !handle->finger) {
        return MOTOR_CONTROL_ERROR_NULL_PTR;
    }

    finger_error_t finger_err = FINGER_OK;

    switch (cmd) {
    case MOTOR_CONTROL_CMD_SET_SPEED: {
        /* val = int16 speed (RPM)，需转换为 finger 速度格式 */
        /* finger 速度: bit31=方向, 低31位=RPM */
        int16_t speed = (int16_t)val;
        uint32_t finger_speed;
        if (speed >= 0) {
            finger_speed = 0x80000000UL | (uint32_t)speed;
        } else {
            finger_speed = (uint32_t)(-speed);
        }
        handle->speed = speed;
        LOG_I("motor_ctrl", "  → RS-485 [电机%u] 设置速度=%d RPM (raw=0x%04X)",
            (unsigned int)handle->config.motor_id, (int)speed, (unsigned int)val);
        finger_err = finger_set_speed(handle->finger, finger_speed);
        break;
    }

    case MOTOR_CONTROL_CMD_SET_POSITION: {
        /* val = uint16 角度×10 → 转换为 finger position */
        /* finger position = 角度(度) × 减速比 × 极对数 */
        /* 从 val (度×10) 恢复角度: val/10，再乘以减速比和极对数 */
        uint32_t angle_tenth = val; /* 度 × 10 */
        uint32_t position = angle_tenth * 1080UL * 2UL / 10UL; /* 简化: ×216 */
        handle->position = val; /* 缓存原始角度×10 */
        LOG_I("motor_ctrl", "  → RS-485 [电机%u] 设置位置=%.1f° (raw=0x%04X)",
            (unsigned int)handle->config.motor_id,
            (float)val / 10.0f, (unsigned int)val);
        finger_err = finger_set_position(handle->finger, position, handle->current);
        break;
    }

    case MOTOR_CONTROL_CMD_SET_CURRENT: {
        /* val = uint16 电流 (mA 或原始值) */
        /* finger 无独立电流 API，通过 finger_set_position 的 torque 参数实现 */
        handle->current = val;
        LOG_I("motor_ctrl", "  → RS-485 [电机%u] 设置电流=%u mA",
            (unsigned int)handle->config.motor_id, (unsigned int)val);
        finger_err = finger_set_position(handle->finger,
            0, /* 使用默认位置 */
            val);
        break;
    }

    case MOTOR_CONTROL_CMD_START:
        LOG_I("motor_ctrl", "  → RS-485 [电机%u] 启动",
            (unsigned int)handle->config.motor_id);
        finger_err = finger_start(handle->finger);
        break;

    case MOTOR_CONTROL_CMD_STOP:
        LOG_I("motor_ctrl", "  → RS-485 [电机%u] 急停",
            (unsigned int)handle->config.motor_id);
        finger_err = finger_emergency_stop(handle->finger);
        break;

    case MOTOR_CONTROL_CMD_CLEAR_FAULT:
        LOG_I("motor_ctrl", "  → RS-485 [电机%u] 清除故障",
            (unsigned int)handle->config.motor_id);
        finger_err = finger_clear_fault(handle->finger);
        break;

    default:
        LOG_W("motor_ctrl", "  → RS-485 [电机%u] 未知命令 0x%02X",
            (unsigned int)handle->config.motor_id, (unsigned int)cmd);
        return MOTOR_CONTROL_ERROR_INVALID_PARAM;
    }

    if (FINGER_IS_ERR(finger_err)) {
        LOG_E("motor_ctrl", "  → RS-485 [电机%u] 命令失败, 错误码=%d",
            (unsigned int)handle->config.motor_id, (int)finger_err);
        return MOTOR_CONTROL_ERROR_RS485_FAIL;
    }

    return MOTOR_CONTROL_OK;
}

static void motor_control_build_query_response(uint8_t cmd,
    canfd_protocol_t* p_response)
{
    if (!p_response) {
        return;
    }

    memset(p_response, 0, sizeof(canfd_protocol_t));
    p_response->cmd = cmd;
    p_response->flags = 0;
    p_response->datalen = MOTOR_CONTROL_DATA_BYTES; /* 9 电机 × 2 字节 = 18 */

    /* 遍历 9 个电机，填充缓存值 */
    for (uint8_t motor_id = 1; motor_id <= MOTOR_CONTROL_MOTOR_NUMS; motor_id++) {
        motor_control_handle_t* handle = motor_control_get_by_id(motor_id);
        if (!handle) {
            continue;
        }

        uint16_t val = 0;

        switch (cmd) {
        case MOTOR_CONTROL_CMD_QUERY_STATUS:
            /* status word: [status:8][mode:4][fault:4] */
            val = (uint16_t)handle->status
                | ((uint16_t)(handle->mode & 0x0FU) << 8)
                | ((uint16_t)(handle->fault_flags & 0x0FU) << 12);
            break;

        case MOTOR_CONTROL_CMD_QUERY_POSITION:
            val = handle->position; /* 度 × 10 */
            break;

        case MOTOR_CONTROL_CMD_QUERY_SPEED:
            val = (uint16_t)(handle->speed); /* int16 → uint16 保持位模式 */
            break;

        case MOTOR_CONTROL_CMD_QUERY_CURRENT:
            val = handle->current;
            break;

        default:
            break;
        }

        motor_control_set_data(p_response, motor_id, val);
    }

    LOG_I("motor_ctrl", "查询应答帧已构建: cmd=%s(0x%02X) datalen=%u",
        motor_control_cmd_name(cmd), (unsigned int)cmd,
        (unsigned int)MOTOR_CONTROL_DATA_BYTES);
}
