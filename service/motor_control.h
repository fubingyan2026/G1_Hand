/*
 * Copyright (c) 2026 G1_Hand 项目组
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    motor_control.h
 * @author  maximilian
 * @version V2.0.0
 * @date    2026-06-11
 * @brief   CAN-FD 电机控制桥接服务层
 *
 * 通过 CAN-FD 接收上位机指令，转换为 RS-485 电机控制指令（调用 finger 服务），
 * 并将电机应答通过 CAN-FD 回传给上位机。
 *
 * 协议帧格式 (固定 22 字节):
 *   [header:1B='s'][cmd:1B][flags:1B][data:18B=9电机×2字节][ender:1B='e']
 *
 * @note
 * - 采用 context/config 双结构体模式，支持多实例（每电机一个实例）
 * - 每实例封装一个 finger_handle_t，通过 finger 回调异步接收应答
 * - 不再依赖 protocol_parser/ protocol_packer，直接校验帧头帧尾
 */

#ifndef __MOTOR_CONTROL_H
#define __MOTOR_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "clist.h"
#include "drv_rs485.h"
#include "finger.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief CAN-FD 协议帧结构体（可变长度，packed）
 *
 * 每帧可携带 0-9 个电机的数据，每个电机 2 或 4 字节（小端序）。
 * datalen 指定 data 段实际使用的字节数。
 * 普通命令: 2 字节/电机 (datalen ≤ 18, 必须为偶数)
 * 复合命令: 4 字节/电机 (datalen ≤ 36, 必须为 4 的倍数)
 * 帧总字节数 = 6 + datalen (最小 6, 最大 42)
 */
typedef struct __attribute__((packed)) {
    uint8_t header;                     /**< 帧头，固定 's' (0x73) */
    uint8_t cmd;                        /**< 命令码 */
    uint8_t flags;                      /**< 标志位 */
    uint8_t datalen;                    /**< data 段实际字节数 (0-36) */
    uint8_t data[36];                   /**< 电机数据 (最大 36 字节), 小端序 */
    uint8_t crc_check;                  /**< XOR 校验 (覆盖 cmd..data[datalen-1]) */
    uint8_t ender;                      /**< 帧尾，固定 'e' (0x65) */
} canfd_protocol_t;

/**
 * @brief 电机控制桥接错误码枚举
 */
typedef enum {
    MOTOR_CONTROL_OK = 0,               /**< 操作成功 */
    MOTOR_CONTROL_ERROR_NULL_PTR,       /**< 空指针错误 */
    MOTOR_CONTROL_ERROR_INVALID_PARAM,  /**< 无效参数 */
    MOTOR_CONTROL_ERROR_UNINITIALIZED,  /**< 未初始化 */
    MOTOR_CONTROL_ERROR_NOT_FOUND,      /**< 电机未找到 */
    MOTOR_CONTROL_ERROR_ALREADY_EXIST,  /**< 同名实例已存在 */
    MOTOR_CONTROL_ERROR_MOTOR_BUSY,     /**< 电机忙（有待处理命令） */
    MOTOR_CONTROL_ERROR_MOTOR_FAULT,    /**< 电机故障 */
    MOTOR_CONTROL_ERROR_TIMEOUT,        /**< RS-485 应答超时 */
    MOTOR_CONTROL_ERROR_RS485_FAIL,     /**< RS-485 通讯失败 */
    MOTOR_CONTROL_ERROR_BAD_FRAME,      /**< 帧头/帧尾校验失败 */
} motor_control_error_t;

/**
 * @brief CAN-FD 协议命令码枚举
 */
typedef enum {
    /* --- 控制命令 (0x01-0x0F) --- */
    MOTOR_CONTROL_CMD_SET_SPEED = 0x01,     /**< 设置速度 (int16 LE, RPM) */
    MOTOR_CONTROL_CMD_SET_POSITION = 0x02,  /**< 设置位置 (uint16 LE, 度×10) */
    MOTOR_CONTROL_CMD_SET_CURRENT = 0x03,   /**< 设置电流 (uint16 LE, mA) */
    MOTOR_CONTROL_CMD_START = 0x04,         /**< 启动电机 (data != 0 → 执行) */
    MOTOR_CONTROL_CMD_STOP = 0x05,          /**< 急停电机 (data != 0 → 执行) */
    MOTOR_CONTROL_CMD_CLEAR_FAULT = 0x06,   /**< 清除故障 (data != 0 → 执行) */
    MOTOR_CONTROL_CMD_SET_POSITION_SPEED = 0x07, /**< 位置+速度闭环 (4B/电机: pos:u16+speed:i16 LE) */

    /* --- 查询命令 (0x10-0x1F) --- */
    MOTOR_CONTROL_CMD_QUERY_STATUS = 0x10,  /**< 查询状态 */
    MOTOR_CONTROL_CMD_QUERY_POSITION = 0x11, /**< 查询位置 */
    MOTOR_CONTROL_CMD_QUERY_SPEED = 0x12,   /**< 查询速度 */
    MOTOR_CONTROL_CMD_QUERY_CURRENT = 0x13, /**< 查询电流 */

    /* --- 心跳 (0xF0) --- */
    MOTOR_CONTROL_CMD_HEARTBEAT = 0xF0,     /**< 心跳帧（MCU → Host） */
} motor_control_cmd_t;

/**
 * @brief 电机控制桥接实例配置结构体
 */
typedef struct {
    const char* name;                   /**< 实例名称 */
    uint8_t motor_id;                   /**< CAN-FD 协议中的电机 ID（1-9） */
    uint8_t finger_motor_id;            /**< 对应 finger 模块的 RS-485 地址 */
    rs485_port_t finger_port;           /**< 对应 finger 模块的 RS-485 端口 */
} motor_control_config_t;

/* 前向声明 */
typedef struct motor_control_handle motor_control_handle_t;

/**
 * @brief 电机控制桥接实例句柄（运行时上下文）
 */
struct motor_control_handle {
    motor_control_config_t config;      /**< 配置副本 */

    finger_handle_t* finger;            /**< 绑定的 finger 电机实例 */

    /* 状态缓存（由 finger 应答回调更新） */
    uint8_t status;                     /**< 最近查询的电机状态字节 */
    uint16_t position;                  /**< 最近查询的电角度（度×10） */
    finger_mode_t mode;                 /**< 当前工作模式 */
    int16_t speed;                      /**< 最近设置/查询的速度 (RPM) */
    uint16_t current;                   /**< 最近设置/查询的电流/力矩值 */
    uint8_t fault_flags;                /**< 故障标志位 */

    /* CAN-FD 应答帧缓冲（每电机单应答槽，固定 22 字节） */
    bool has_pending_response;          /**< 有待发送的 CAN-FD 应答帧 */
    canfd_protocol_t response_frame;    /**< 应答帧数据（固定 22 字节） */

    /* RS-485 命令追踪 */
    uint8_t pending_cmd;               /**< 发起操作的原 CAN 命令码 */
    bool command_in_flight;             /**< RS-485 命令已发出，等待应答 */
    bool ack_requested;                 /**< 主机请求了应答 */

    clist_head_t node;                  /**< 链表节点 */
    bool initialized;                   /**< 初始化完成标志 */
};

/* Exported constants --------------------------------------------------------*/

/** @brief 协议帧开销字节数 (header + cmd + flags + datalen + crc_check + ender) */
#define MOTOR_CONTROL_FRAME_OVERHEAD    6U

/** @brief data 段最大字节数 (SET_POSITION_SPEED: 9 电机 × 4 字节) */
#define MOTOR_CONTROL_DATA_MAX          36U

/** @brief 帧头值 */
#define MOTOR_CONTROL_HEADER_VAL        's'     /* 0x73 */

/** @brief 帧尾值 */
#define MOTOR_CONTROL_ENDER_VAL         'e'     /* 0x65 */

/** @brief 电机总数 */
#define MOTOR_CONTROL_MOTOR_NUMS        9U

/** @brief data 段字节数 */
#define MOTOR_CONTROL_DATA_BYTES        18U

/** @brief CAN-FD 下行命令帧 CAN ID（Host → MCU） */
#define MOTOR_CONTROL_CAN_ID_CMD        0x100U

/** @brief CAN-FD 上行应答帧 CAN ID（MCU → Host） */
#define MOTOR_CONTROL_CAN_ID_RESP       0x101U

/** @brief flags: bit0 = 主机需要应答 */
#define MOTOR_CONTROL_FLAG_ACK_REQUEST  0x01U

/* Exported macro ------------------------------------------------------------*/

/** @brief 判断电机控制错误码是否成功 */
#define MOTOR_CONTROL_IS_OK(err)    ((err) >= 0)

/** @brief 判断电机控制错误码是否失败 */
#define MOTOR_CONTROL_IS_ERR(err)   ((err) < 0)

/**
 * @brief 获取电机 ID 在 data 段中的 16-bit 值（小端序）
 * @param proto 协议帧指针
 * @param motor_id 电机 ID (1-9)
 * @return uint16_t 值
 */
#define MOTOR_CONTROL_DATA_U16(proto, motor_id) \
    ((uint16_t)((proto)->data[((motor_id) - 1) * 2]) \
    | ((uint16_t)((proto)->data[((motor_id) - 1) * 2 + 1]) << 8))

/**
 * @brief 设置电机 ID 在 data 段中的 16-bit 值（小端序）
 * @param proto 协议帧指针
 * @param motor_id 电机 ID (1-9)
 * @param val uint16_t 值
 */
#define MOTOR_CONTROL_SET_DATA_U16(proto, motor_id, val) \
    do { \
        (proto)->data[((motor_id) - 1) * 2] = (uint8_t)((val) & 0xFFU); \
        (proto)->data[((motor_id) - 1) * 2 + 1] = (uint8_t)(((val) >> 8) & 0xFFU); \
    } while (0)

/**
 * @brief 获取电机 ID 在 4 字节 data 段中的位置值 (uint16 LE, 前 2 字节)
 */
#define MOTOR_CONTROL_DATA_POS(proto, motor_id) \
    ((uint16_t)((proto)->data[((motor_id) - 1) * 4]) \
    | ((uint16_t)((proto)->data[((motor_id) - 1) * 4 + 1]) << 8))

/**
 * @brief 获取电机 ID 在 4 字节 data 段中的速度值 (int16 LE, 后 2 字节)
 */
#define MOTOR_CONTROL_DATA_SPD(proto, motor_id) \
    ((int16_t)((uint16_t)((proto)->data[((motor_id) - 1) * 4 + 2]) \
    | ((uint16_t)((proto)->data[((motor_id) - 1) * 4 + 3]) << 8)))

/* Exported functions prototypes ---------------------------------------------*/

/* --- 子系统生命周期 --- */

motor_control_error_t motor_control_init(void);
void motor_control_deinit(void);
bool motor_control_is_initialized(void);

/* --- 实例管理 --- */

motor_control_error_t motor_control_register_static(
    const motor_control_config_t* config,
    motor_control_handle_t* instance);

motor_control_error_t motor_control_unregister(const char* name);
motor_control_handle_t* motor_control_get_by_id(uint8_t motor_id);
clist_head_t* motor_control_get_head(void);

/* --- CAN-FD 帧处理（供 motor_control_task 调用） — 帧头/帧尾/CRC 由 protocol_packer/parser 处理 --- */

/**
 * @brief 处理通过 CAN-FD 接收到的完整协议帧
 *
 * 调用前应由 task 层完成 header/ender 校验。
 * 根据 cmd 和 flags 遍历 9 个电机的 data 段，转换为 finger API 调用或缓存查询。
 *
 * @param protocol 指向已校验的协议帧
 * @return 错误码
 */
motor_control_error_t motor_control_process_rx_frame(
    const canfd_protocol_t* protocol);

/**
 * @brief 弹出待发送的 CAN-FD 应答帧
 *
 * @param handle 电机句柄
 * @param p_response 输出: 协议帧副本
 * @return true 有应答帧，false 无
 */
bool motor_control_pop_response(motor_control_handle_t* handle,
    canfd_protocol_t* p_response);

/* --- 状态查询（本地缓存，不发起 RS-485 通讯） --- */

uint8_t motor_control_get_status(const motor_control_handle_t* handle);
uint16_t motor_control_get_position(const motor_control_handle_t* handle);
finger_mode_t motor_control_get_mode(const motor_control_handle_t* handle);
uint16_t motor_control_get_current(const motor_control_handle_t* handle);
int16_t motor_control_get_speed(const motor_control_handle_t* handle);

/* --- 批量操作 --- */

motor_control_error_t motor_control_emergency_stop_all(void);

/* --- 内部接口（供 finger 回调使用） --- */

void motor_control_on_finger_response(finger_handle_t* finger_handle,
    finger_cmd_type_t cmd_type, bool success, void* user_data);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_CONTROL_H */
