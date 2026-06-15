/**
 * @file    finger.c
 * @author  maximilian
 * @version V1.0.0
 * @date    2026-06-08
 * @brief   手指电机控制模块实现
 * @attention
 *
 * Copyright (c) 2026 G1_Hand 项目组.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "finger.h"

#include <string.h>

#include "log.h"

/* Private constants ---------------------------------------------------------*/

/** @brief 指令类型：读取控制表 */
#define FINGER_CMD_READ 0x01U
/** @brief 指令类型：写取控制表 */
#define FINGER_CMD_WRITE 0x02U
/** @brief 指令类型：指令控制模式 */
#define FINGER_CMD_CONTROL 0x03U

/** @brief 控制表：电机ID */
#define FINGER_CTL_ID 0x01U
/** @brief 控制表：减速比（读） */
#define FINGER_CTL_REDUCTION_RATIO 0x02U
/** @brief 控制表：极对数（读） */
#define FINGER_CTL_POLE_PAIRS 0x03U
/** @brief 控制表：电机位置 */
#define FINGER_CTL_POSITION 0x07U
/** @brief 控制表：电机状态 */
#define FINGER_CTL_STATUS 0x08U
/** @brief 控制表：输出轴角度 */
#define FINGER_CTL_OUTPUT_ANGLE 0x0AU
/** @brief 控制表：软件版本号 */
#define FINGER_CTL_VERSION 0x0BU

/** @brief 控制表：写电机ID */
#define FINGER_CTL_SET_ID 0x15U
/** @brief 控制表：写减速比 */
#define FINGER_CTL_SET_REDUCTION 0x16U
/** @brief 控制表：写极对数 */
#define FINGER_CTL_SET_POLE_PAIRS 0x17U

/** @brief 控制表：启动电机 */
#define FINGER_CTL_START 0x29U
/** @brief 控制表：急停 */
#define FINGER_CTL_ESTOP 0x2AU
/** @brief 控制表：暂停/启动 */
#define FINGER_CTL_PAUSE 0x2BU
/** @brief 控制表：参数装订 */
#define FINGER_CTL_SAVE_FLASH 0x2CU
/** @brief 控制表：清除故障 */
#define FINGER_CTL_CLEAR_FAULT 0x2DU
/** @brief 控制表：速度模式 */
#define FINGER_CTL_SPEED 0x2FU
/** @brief 控制表：位置模式（无反馈） */
#define FINGER_CTL_POSITION_NOFB 0x31U
/** @brief 控制表：位置+速度闭环（无反馈） */
#define FINGER_CTL_POS_SPEED_NOFB 0x4BU

/** @brief 指令帧头 */
static const uint8_t s_finger_cmd_header[] = { 0x55, 0xAA };
#define FINGER_HEADER_LEN 2U

/* Private variables ---------------------------------------------------------*/

static clist_head_t s_finger_head; /**< 手指电机实例链表头 */
static bool s_finger_initialized; /**< 子系统初始化标志 */

/* Private function prototypes -----------------------------------------------*/

static protocol_packer_error_t finger_checksum_callback(const uint8_t* data,
    uint16_t len, uint8_t* checksum_out, uint16_t* checksum_len);
static protocol_packer_error_t finger_fill_len_callback(uint8_t* buffer,
    uint16_t payload_len);
static finger_error_t finger_packer_init(finger_handle_t* handle);
static void finger_build_data_segment(finger_handle_t* handle, uint8_t* data,
    uint16_t* p_len);
static finger_error_t finger_parse_response(const finger_handle_t* handle,
    const uint8_t* frame, uint16_t len);

/* Private functions ---------------------------------------------------------*/

/**
 * @brief 校验和回调：累加所有字节取低 8 位
 * @param data 数据缓冲区（帧头+数据段）
 * @param len 数据长度
 * @param checksum_out 输出校验码
 * @param checksum_len 输出校验码长度
 * @return 操作结果错误码
 */
static protocol_packer_error_t finger_checksum_callback(const uint8_t* data,
    uint16_t len, uint8_t* checksum_out, uint16_t* checksum_len)
{
    uint32_t sum = 0;

    for (uint16_t i = 0; i < len; i++) {
        sum += data[i];
    }

    checksum_out[0] = (uint8_t)(sum & 0xFFU);
    *checksum_len = 1;

    return PROTOCOL_PACKER_OK;
}

/**
 * @brief 帧长度填充回调：在字节 2 填入总帧长度
 * @param buffer 帧缓冲区（含帧头和数据段）
 * @param payload_len 数据段长度
 * @return 操作结果错误码
 * @note 总帧长 = 帧头(2) + 数据段(len) + 校验(1)
 */
static protocol_packer_error_t finger_fill_len_callback(uint8_t* buffer,
    uint16_t payload_len)
{
    /* buffer[0..1] = 帧头, buffer[2] = 长度字段位置 */
    buffer[FINGER_HEADER_LEN] = (uint8_t)(FINGER_HEADER_LEN + payload_len + 1U);
    return PROTOCOL_PACKER_OK;
}

/**
 * @brief 初始化电机内嵌协议打包器
 * @param handle 电机句柄
 * @return 错误码
 */
static finger_error_t finger_packer_init(finger_handle_t* handle)
{
    protocol_packer_config_t packer_cfg;

    packer_cfg.name = handle->config.name;
    packer_cfg.header = s_finger_cmd_header;
    packer_cfg.header_len = FINGER_HEADER_LEN;
    packer_cfg.footer = NULL;
    packer_cfg.footer_len = 0;
    packer_cfg.checksum_cb = finger_checksum_callback;
    packer_cfg.fill_len_cb = finger_fill_len_callback;
    packer_cfg.checksum_len = 1;
    packer_cfg.output_buffer = handle->config.tx_buffer;
    packer_cfg.output_buffer_len = handle->config.tx_buffer_len;

    protocol_packer_error_t err = protocol_packer_init(&handle->packer, &packer_cfg);
    if (err != PROTOCOL_PACKER_OK) {
        return FINGER_ERROR_PACKER_ERROR;
    }

    return FINGER_OK;
}

/**
 * @brief 构建数据段（帧头之后、校验之前的部分）
 * @param handle 电机句柄
 * @param data 输出数据缓冲区
 * @param p_len 输出数据长度
 * @note 数据段格式: [长度占位(1)] [ID(1)] [指令(1)] [控制表(1)] [负载数据(N)]
 */
static void finger_build_data_segment(finger_handle_t* handle, uint8_t* data,
    uint16_t* p_len)
{
    uint16_t offset = 0;

    /* 长度占位字节（由 fill_len_cb 填充） */
    data[offset++] = 0x00;

    /* ID */
    data[offset++] = handle->config.motor_id;

    /* 指令类型和控制表 */
    switch (handle->pending_cmd) {
    case FINGER_CMD_READ_POSITION:
        data[offset++] = FINGER_CMD_READ;
        data[offset++] = FINGER_CTL_POSITION;
        break;

    case FINGER_CMD_READ_STATUS:
        data[offset++] = FINGER_CMD_READ;
        data[offset++] = FINGER_CTL_STATUS;
        break;

    case FINGER_CMD_READ_OUTPUT_ANGLE:
        data[offset++] = FINGER_CMD_READ;
        data[offset++] = FINGER_CTL_OUTPUT_ANGLE;
        break;

    case FINGER_CMD_READ_VERSION:
        data[offset++] = FINGER_CMD_READ;
        data[offset++] = FINGER_CTL_VERSION;
        break;

    case FINGER_CMD_READ_ID:
        data[offset++] = FINGER_CMD_READ;
        data[offset++] = FINGER_CTL_ID;
        break;

    case FINGER_CMD_SET_ID:
        data[offset++] = FINGER_CMD_WRITE;
        data[offset++] = FINGER_CTL_SET_ID;
        data[offset++] = handle->pending_new_id;
        break;

    case FINGER_CMD_SET_REDUCTION_RATIO:
        data[offset++] = FINGER_CMD_WRITE;
        data[offset++] = FINGER_CTL_SET_REDUCTION;
        data[offset++] = (uint8_t)(handle->pending_position & 0xFFU);
        break;

    case FINGER_CMD_SET_POLE_PAIRS:
        data[offset++] = FINGER_CMD_WRITE;
        data[offset++] = FINGER_CTL_SET_POLE_PAIRS;
        data[offset++] = (uint8_t)(handle->pending_position & 0xFFU);
        break;

    case FINGER_CMD_START:
        data[offset++] = FINGER_CMD_CONTROL;
        data[offset++] = FINGER_CTL_START;
        data[offset++] = 0x00;
        break;

    case FINGER_CMD_STOP:
        data[offset++] = FINGER_CMD_CONTROL;
        data[offset++] = FINGER_CTL_ESTOP;
        data[offset++] = 0x00;
        break;

    case FINGER_CMD_PAUSE_RESUME:
        data[offset++] = FINGER_CMD_CONTROL;
        data[offset++] = FINGER_CTL_PAUSE;
        data[offset++] = 0x00;
        break;

    case FINGER_CMD_SAVE_FLASH:
        data[offset++] = FINGER_CMD_CONTROL;
        data[offset++] = FINGER_CTL_SAVE_FLASH;
        data[offset++] = 0x00;
        break;

    case FINGER_CMD_CLEAR_FAULT:
        data[offset++] = FINGER_CMD_CONTROL;
        data[offset++] = FINGER_CTL_CLEAR_FAULT;
        data[offset++] = 0x00;
        break;

    case FINGER_CMD_SET_SPEED:
        data[offset++] = FINGER_CMD_CONTROL;
        data[offset++] = FINGER_CTL_SPEED;
        /* 速度 4 字节，大端序，最高位为方向 */
        data[offset++] = (uint8_t)((handle->pending_speed >> 24) & 0xFFU);
        data[offset++] = (uint8_t)((handle->pending_speed >> 16) & 0xFFU);
        data[offset++] = (uint8_t)((handle->pending_speed >> 8) & 0xFFU);
        data[offset++] = (uint8_t)(handle->pending_speed & 0xFFU);
        break;

    case FINGER_CMD_SET_POSITION:
        data[offset++] = FINGER_CMD_CONTROL;
        data[offset++] = FINGER_CTL_POSITION_NOFB;
        /* 角度 4 字节 + 预留 2 字节 + 力矩 2 字节 */
        data[offset++] = (uint8_t)((handle->pending_position >> 24) & 0xFFU);
        data[offset++] = (uint8_t)((handle->pending_position >> 16) & 0xFFU);
        data[offset++] = (uint8_t)((handle->pending_position >> 8) & 0xFFU);
        data[offset++] = (uint8_t)(handle->pending_position & 0xFFU);
        data[offset++] = 0x10; /* 预留 */
        data[offset++] = 0x00;
        data[offset++] = (uint8_t)((handle->pending_torque >> 8) & 0xFFU);
        data[offset++] = (uint8_t)(handle->pending_torque & 0xFFU);
        break;

    case FINGER_CMD_SET_POSITION_SPEED:
        data[offset++] = FINGER_CMD_CONTROL;
        data[offset++] = FINGER_CTL_POS_SPEED_NOFB;
        /* 角度 4 字节 + 速度 4 字节 */
        data[offset++] = (uint8_t)((handle->pending_position >> 24) & 0xFFU);
        data[offset++] = (uint8_t)((handle->pending_position >> 16) & 0xFFU);
        data[offset++] = (uint8_t)((handle->pending_position >> 8) & 0xFFU);
        data[offset++] = (uint8_t)(handle->pending_position & 0xFFU);
        data[offset++] = (uint8_t)((handle->pending_speed >> 24) & 0xFFU);
        data[offset++] = (uint8_t)((handle->pending_speed >> 16) & 0xFFU);
        data[offset++] = (uint8_t)((handle->pending_speed >> 8) & 0xFFU);
        data[offset++] = (uint8_t)(handle->pending_speed & 0xFFU);
        break;

    default:
        break;
    }

    *p_len = offset;
}

/**
 * @brief 解析应答帧数据并更新电机状态
 * @param handle 电机句柄
 * @param frame 完整应答帧
 * @param len 帧长度
 * @return 错误码
 */
static finger_error_t finger_parse_response(const finger_handle_t* handle,
    const uint8_t* frame, uint16_t len)
{
    /*
     * 应答帧结构: [0xAA][0x55][长度][ID][指令][控制表][数据...][校验]
     * 最小帧长: 帧头(2)+长度(1)+ID(1)+指令(1)+控制表(1)+校验(1) = 7
     */
    if (len < 7) {
        return FINGER_ERROR_INVALID_PARAM;
    }

    uint8_t cmd = frame[4];
    uint8_t ctl = frame[5];
    (void)cmd;

    /*
     * 处理应答数据：
     * - 读取指令 (0x01)：数据段包含查询结果
     * - 写入指令 (0x02)：通常回显所写数据
     * - 控制指令 (0x03)：通常回显控制表和数据
     */
    switch (ctl) {
    case FINGER_CTL_POSITION:
        /* 电机电角度 0-359°，占 2 字节 */
        if (len >= 8) {
            /* cast away const: only used internally to cache response data */
            *(uint16_t*)&((finger_handle_t*)handle)->position = (uint16_t)(((uint16_t)frame[6] << 8) | frame[7]);
            LOG_I("finger", "ID=0x%02X 位置=%u°", handle->config.motor_id,
                (unsigned int)((finger_handle_t*)handle)->position);
        }
        break;

    case FINGER_CTL_STATUS:
        /* 电机状态信息，占 1 字节 */
        if (len >= 7) {
            uint8_t status = frame[6];
            *(uint8_t*)&((finger_handle_t*)handle)->status = status;
            if (status <= FINGER_MODE_SPEED_POSITION) {
                *(finger_mode_t*)&((finger_handle_t*)handle)->mode = (finger_mode_t)status;
            }
            LOG_I("finger", "ID=0x%02X 状态=0x%02X 模式=%d",
                handle->config.motor_id, status,
                (int)((finger_handle_t*)handle)->mode);
        }
        break;

    case FINGER_CTL_OUTPUT_ANGLE:
        /* 输出轴角度，占 2 字节，单位 0.0555° */
        if (len >= 8) {
            *(uint32_t*)&((finger_handle_t*)handle)->output_angle_raw = (uint32_t)(((uint16_t)frame[6] << 8) | frame[7]);
            LOG_I("finger", "ID=0x%02X 输出轴角度=0x%04lX (%.1f°)",
                handle->config.motor_id,
                (unsigned long)((finger_handle_t*)handle)->output_angle_raw,
                (double)((finger_handle_t*)handle)->output_angle_raw * 0.0555);
        }
        break;

    default:
        /* 其他控制表：写确认或控制确认，无需额外处理 */
        LOG_I("finger", "ID=0x%02X ctl=0x%02X cmd=0x%02X 应答确认",
            handle->config.motor_id, ctl, cmd);
        break;
    }

    return FINGER_OK;
}

/* Exported functions --------------------------------------------------------*/

/* --- 子系统生命周期 --- */

finger_error_t finger_init(void)
{
    if (s_finger_initialized) {
        return FINGER_OK_EXISTED;
    }

    clist_init(&s_finger_head);
    s_finger_initialized = true;

    return FINGER_OK;
}

void finger_deinit(void)
{
    if (!s_finger_initialized) {
        return;
    }

    /* 安全遍历释放所有实例 */
    clist_head_t *pos, *tmp;
    clist_for_each_safe(pos, tmp, &s_finger_head)
    {
        finger_handle_t* h = clist_entry(pos, finger_handle_t, node);
        clist_del(pos);
        (void)protocol_packer_deinit(&h->packer);
    }

    clist_init(&s_finger_head);
    s_finger_initialized = false;
}

bool finger_is_initialized(void)
{
    return s_finger_initialized;
}

/* --- 实例管理 --- */

finger_error_t finger_register_static(const finger_config_t* config,
    finger_handle_t* instance)
{
    if (!config || !instance) {
        return FINGER_ERROR_NULL_PTR;
    }

    if (!config->name || !config->tx_buffer) {
        return FINGER_ERROR_NULL_PTR;
    }

    if (config->motor_id == 0 || config->motor_id > 254) {
        return FINGER_ERROR_INVALID_PARAM;
    }

    if (config->tx_buffer_len < FINGER_TX_BUFFER_SIZE) {
        return FINGER_ERROR_INVALID_PARAM;
    }

    if (!s_finger_initialized) {
        return FINGER_ERROR_UNINITIALIZED;
    }

    if (finger_get_instance(config->name)) {
        return FINGER_ERROR_ALREADY_EXIST;
    }

    /* 清零句柄内存 */
    memset(instance, 0, sizeof(finger_handle_t));

    /* 复制配置 */
    memcpy(&instance->config, config, sizeof(finger_config_t));

    /* 设置默认值 */
    if (instance->config.reduction_ratio == 0) {
        instance->config.reduction_ratio = 1080;
    }
    if (instance->config.pole_pairs == 0) {
        instance->config.pole_pairs = 2;
    }

    /* 初始化内嵌协议打包器 */
    finger_error_t err = finger_packer_init(instance);
    if (FINGER_IS_ERR(err)) {
        return err;
    }

    /* 初始化状态 */
    instance->mode = FINGER_MODE_IDLE;
    instance->position = 0;
    instance->output_angle_raw = 0;
    instance->status = 0;
    instance->pending_cmd = FINGER_CMD_NONE;
    instance->response_pending = false;
    instance->initialized = true;

    /* 加入全局链表 */
    clist_add_tail(&s_finger_head, &instance->node);

    return FINGER_OK;
}

finger_error_t finger_unregister(const char* name)
{
    if (!name || !s_finger_initialized) {
        return FINGER_ERROR_NULL_PTR;
    }

    finger_handle_t *h, *tmp;
    clist_for_each_entry_safe(h, tmp, &s_finger_head, node)
    {
        if (strcmp(h->config.name, name) == 0) {
            clist_del(&h->node);
            (void)protocol_packer_deinit(&h->packer);
            return FINGER_OK;
        }
    }

    return FINGER_ERROR_NOT_FOUND;
}

finger_handle_t* finger_get_instance(const char* name)
{
    if (!name || !s_finger_initialized) {
        return NULL;
    }

    finger_handle_t* h;
    clist_for_each_entry(h, &s_finger_head, node)
    {
        if (strcmp(h->config.name, name) == 0) {
            return h;
        }
    }

    return NULL;
}

finger_handle_t* finger_get_by_id(uint8_t motor_id, rs485_port_t port)
{
    if (!s_finger_initialized) {
        return NULL;
    }

    finger_handle_t* h;
    clist_for_each_entry(h, &s_finger_head, node)
    {
        if (h->config.motor_id == motor_id && h->config.rs485_port == port) {
            return h;
        }
    }

    return NULL;
}

clist_head_t* finger_get_head(void)
{
    return s_finger_initialized ? &s_finger_head : NULL;
}

/* --- 电机控制（异步命令） --- */

finger_error_t finger_start(finger_handle_t* handle)
{
    if (!handle) {
        return FINGER_ERROR_NULL_PTR;
    }
    if (!handle->initialized) {
        return FINGER_ERROR_UNINITIALIZED;
    }

    handle->pending_cmd = FINGER_CMD_START;
    handle->response_pending = true;

    return FINGER_OK;
}

finger_error_t finger_emergency_stop(finger_handle_t* handle)
{
    if (!handle) {
        return FINGER_ERROR_NULL_PTR;
    }
    if (!handle->initialized) {
        return FINGER_ERROR_UNINITIALIZED;
    }

    handle->pending_cmd = FINGER_CMD_STOP;
    handle->response_pending = true;

    return FINGER_OK;
}

finger_error_t finger_pause_resume(finger_handle_t* handle)
{
    if (!handle) {
        return FINGER_ERROR_NULL_PTR;
    }
    if (!handle->initialized) {
        return FINGER_ERROR_UNINITIALIZED;
    }

    handle->pending_cmd = FINGER_CMD_PAUSE_RESUME;
    handle->response_pending = true;

    return FINGER_OK;
}

finger_error_t finger_save_flash(finger_handle_t* handle)
{
    if (!handle) {
        return FINGER_ERROR_NULL_PTR;
    }
    if (!handle->initialized) {
        return FINGER_ERROR_UNINITIALIZED;
    }

    handle->pending_cmd = FINGER_CMD_SAVE_FLASH;
    handle->response_pending = true;

    return FINGER_OK;
}

finger_error_t finger_clear_fault(finger_handle_t* handle)
{
    if (!handle) {
        return FINGER_ERROR_NULL_PTR;
    }
    if (!handle->initialized) {
        return FINGER_ERROR_UNINITIALIZED;
    }

    handle->pending_cmd = FINGER_CMD_CLEAR_FAULT;
    handle->response_pending = true;

    return FINGER_OK;
}

finger_error_t finger_set_speed(finger_handle_t* handle, uint32_t speed)
{
    if (!handle) {
        return FINGER_ERROR_NULL_PTR;
    }
    if (!handle->initialized) {
        return FINGER_ERROR_UNINITIALIZED;
    }

    handle->pending_speed = speed;
    handle->pending_cmd = FINGER_CMD_SET_SPEED;
    handle->response_pending = true;

    return FINGER_OK;
}

finger_error_t finger_set_position(finger_handle_t* handle,
    uint32_t position, uint16_t torque)
{
    if (!handle) {
        return FINGER_ERROR_NULL_PTR;
    }
    if (!handle->initialized) {
        return FINGER_ERROR_UNINITIALIZED;
    }

    handle->pending_position = position;
    handle->pending_torque = torque;
    handle->pending_cmd = FINGER_CMD_SET_POSITION;
    handle->response_pending = true;

    return FINGER_OK;
}

finger_error_t finger_set_position_speed(finger_handle_t* handle,
    uint32_t position, uint32_t speed)
{
    if (!handle) {
        return FINGER_ERROR_NULL_PTR;
    }
    if (!handle->initialized) {
        return FINGER_ERROR_UNINITIALIZED;
    }

    handle->pending_position = position;
    handle->pending_speed = speed;
    handle->pending_cmd = FINGER_CMD_SET_POSITION_SPEED;
    handle->response_pending = true;

    return FINGER_OK;
}

/* --- 查询命令 --- */

finger_error_t finger_read_position(finger_handle_t* handle)
{
    if (!handle) {
        return FINGER_ERROR_NULL_PTR;
    }
    if (!handle->initialized) {
        return FINGER_ERROR_UNINITIALIZED;
    }

    handle->pending_cmd = FINGER_CMD_READ_POSITION;
    handle->response_pending = true;

    return FINGER_OK;
}

finger_error_t finger_read_status(finger_handle_t* handle)
{
    if (!handle) {
        return FINGER_ERROR_NULL_PTR;
    }
    if (!handle->initialized) {
        return FINGER_ERROR_UNINITIALIZED;
    }

    handle->pending_cmd = FINGER_CMD_READ_STATUS;
    handle->response_pending = true;

    return FINGER_OK;
}

finger_error_t finger_read_output_angle(finger_handle_t* handle)
{
    if (!handle) {
        return FINGER_ERROR_NULL_PTR;
    }
    if (!handle->initialized) {
        return FINGER_ERROR_UNINITIALIZED;
    }

    handle->pending_cmd = FINGER_CMD_READ_OUTPUT_ANGLE;
    handle->response_pending = true;

    return FINGER_OK;
}

finger_error_t finger_read_version(finger_handle_t* handle)
{
    if (!handle) {
        return FINGER_ERROR_NULL_PTR;
    }
    if (!handle->initialized) {
        return FINGER_ERROR_UNINITIALIZED;
    }

    handle->pending_cmd = FINGER_CMD_READ_VERSION;
    handle->response_pending = true;

    return FINGER_OK;
}

finger_error_t finger_read_id(finger_handle_t* handle)
{
    if (!handle) {
        return FINGER_ERROR_NULL_PTR;
    }
    if (!handle->initialized) {
        return FINGER_ERROR_UNINITIALIZED;
    }

    handle->pending_cmd = FINGER_CMD_READ_ID;
    handle->response_pending = true;

    return FINGER_OK;
}

/* --- ID 及参数配置 --- */

finger_error_t finger_set_id(finger_handle_t* handle, uint8_t new_id)
{
    if (!handle) {
        return FINGER_ERROR_NULL_PTR;
    }
    if (!handle->initialized) {
        return FINGER_ERROR_UNINITIALIZED;
    }

    if (new_id == 0 || new_id > 254) {
        return FINGER_ERROR_INVALID_PARAM;
    }

    handle->pending_new_id = new_id;
    handle->pending_cmd = FINGER_CMD_SET_ID;
    handle->response_pending = true;

    return FINGER_OK;
}

/* --- 状态查询（本地缓存，不发起通讯） --- */

uint16_t finger_get_position(const finger_handle_t* handle)
{
    return (handle && handle->initialized) ? handle->position : 0;
}

uint8_t finger_get_status(const finger_handle_t* handle)
{
    return (handle && handle->initialized) ? handle->status : 0;
}

finger_mode_t finger_get_mode(const finger_handle_t* handle)
{
    return (handle && handle->initialized) ? handle->mode : FINGER_MODE_IDLE;
}

/* --- 回调设置 --- */

void finger_set_response_callback(finger_handle_t* handle,
    finger_response_cb_t cb, void* user_data)
{
    if (!handle) {
        return;
    }

    handle->response_cb = cb;
    handle->callback_user_data = user_data;
}

/* --- 内部接口（供 finger_task 调用） --- */

finger_error_t finger_build_command(finger_handle_t* handle,
    uint8_t** p_out_frame, uint16_t* p_out_len)
{
    if (!handle || !p_out_frame || !p_out_len) {
        return FINGER_ERROR_NULL_PTR;
    }

    /* 初始化输出参数 */
    *p_out_frame = NULL;
    *p_out_len = 0;

    if (!handle->initialized) {
        return FINGER_ERROR_UNINITIALIZED;
    }

    if (handle->pending_cmd == FINGER_CMD_NONE) {
        return FINGER_OK;
    }

    if (!handle->response_pending) {
        return FINGER_OK;
    }

    /* 构建数据段 */
    uint8_t data[32];
    uint16_t data_len = 0;
    finger_build_data_segment(handle, data, &data_len);

    if (data_len == 0) {
        return FINGER_ERROR_INVALID_PARAM;
    }

    /* 使用打包器构建完整帧 */
    protocol_packer_error_t err = protocol_packer_pack(
        &handle->packer, data, data_len, p_out_frame, p_out_len);
    if (err != PROTOCOL_PACKER_OK) {
        return FINGER_ERROR_PACKER_ERROR;
    }

    return FINGER_OK;
}

finger_error_t finger_process_response(finger_handle_t* handle,
    const uint8_t* frame, uint16_t len)
{
    if (!handle || !frame) {
        return FINGER_ERROR_NULL_PTR;
    }

    if (!handle->initialized) {
        return FINGER_ERROR_UNINITIALIZED;
    }

    finger_cmd_type_t completed_cmd = handle->pending_cmd;

    /* 解析应答数据 */
    finger_error_t parse_err = finger_parse_response(handle, frame, len);

    /* 清除待发送状态 */
    handle->pending_cmd = FINGER_CMD_NONE;
    handle->response_pending = false;

    /* 触发回调 */
    if (handle->response_cb) {
        handle->response_cb(handle, completed_cmd,
            (parse_err == FINGER_OK), handle->callback_user_data);
    }

    return parse_err;
}
