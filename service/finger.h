//
// Created by maximilian on 2026-06-08.
//

/**
 * @file    finger.h
 * @brief   手指电机控制模块 — 基于 RS-485 总线协议的手指电机服务层
 * @note    每个电机实例内嵌 protocol_packer 用于构建指令帧，
 *          实例通过侵入式 clist 管理，支持多实例（最多 254 个）。
 *          电机控制 API 用于组装待发送命令，实际通讯由 finger_task 调度。
 */

#ifndef __FINGER_H
#define __FINGER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "clist.h"
#include "drv_rs485.h"
#include "protocol_packer.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 手指电机错误码枚举
 */
typedef enum {
    FINGER_OK = 0, /**< 操作成功 */
    FINGER_OK_EXISTED, /**< 已初始化 */
    FINGER_ERROR_NULL_PTR, /**< 空指针错误 */
    FINGER_ERROR_INVALID_PARAM, /**< 无效参数 */
    FINGER_ERROR_UNINITIALIZED, /**< 未初始化 */
    FINGER_ERROR_NOT_FOUND, /**< 未找到实例 */
    FINGER_ERROR_ALREADY_EXIST, /**< 同名实例已存在 */
    FINGER_ERROR_PACKER_ERROR, /**< 打包器出错 */
    FINGER_ERROR_BUSY, /**< 电机忙（有待处理命令） */
} finger_error_t;

/**
 * @brief 手指电机工作模式枚举（对应电机状态返回值）
 */
typedef enum __attribute__((packed)) {
    FINGER_MODE_IDLE = 0, /**< 空闲模式 */
    FINGER_MODE_SPEED, /**< 速度模式 */
    FINGER_MODE_POSITION, /**< 位置模式 */
    FINGER_MODE_TORQUE, /**< 力矩模式 */
    FINGER_MODE_CURRENT, /**< 电流模式 */
    FINGER_MODE_ZERO, /**< 0°校准模式 */
    FINGER_MODE_OPEN, /**< 开环模式 */
    FINGER_MODE_FR, /**< 正反转运动模式 */
    FINGER_MODE_SPEED_POSITION, /**< 位置+速度闭环模式 */
} finger_mode_t;

/**
 * @brief 手指电机待发送命令类型枚举（内部使用）
 */
typedef enum __attribute__((packed)) {
    FINGER_CMD_NONE = 0, /**< 无待处理命令 */
    FINGER_CMD_START, /**< 启动电机 */
    FINGER_CMD_STOP, /**< 急停 */
    FINGER_CMD_PAUSE_RESUME, /**< 暂停/恢复 */
    FINGER_CMD_SAVE_FLASH, /**< 参数装订（保存至Flash） */
    FINGER_CMD_CLEAR_FAULT, /**< 清除故障 */
    FINGER_CMD_SET_SPEED, /**< 速度模式 */
    FINGER_CMD_SET_POSITION, /**< 位置模式（带力矩） */
    FINGER_CMD_SET_POSITION_SPEED, /**< 位置+速度闭环模式 */
    FINGER_CMD_READ_POSITION, /**< 查询电机电角度 */
    FINGER_CMD_READ_STATUS, /**< 查询电机状态 */
    FINGER_CMD_READ_OUTPUT_ANGLE, /**< 查询输出轴角度 */
    FINGER_CMD_SET_ID, /**< 修改电机ID */
    FINGER_CMD_SET_REDUCTION_RATIO, /**< 写减速比 */
    FINGER_CMD_SET_POLE_PAIRS, /**< 写极对数 */
    FINGER_CMD_READ_VERSION, /**< 读取软件版本号 */
    FINGER_CMD_READ_ID, /**< 读取电机ID */
} finger_cmd_type_t;

/* 前向声明 ----------------------------------------------------------------*/

typedef struct finger_handle finger_handle_t;

/**
 * @brief 应答回调函数类型
 * @param handle 触发回调的电机实例
 * @param cmd_type 对应的命令类型
 * @param success 命令是否执行成功
 * @param user_data 用户数据
 */
typedef void (*finger_response_cb_t)(finger_handle_t* handle,
    finger_cmd_type_t cmd_type, bool success, void* user_data);

/**
 * @brief 手指电机配置结构体
 */
typedef struct {
    const char* name; /**< 电机名称（唯一标识） */
    uint8_t motor_id; /**< 电机 RS-485 地址（1-254） */
    rs485_port_t rs485_port; /**< 所属 RS-485 总线端口 */
    uint8_t* tx_buffer; /**< 发送帧缓冲区（packer 输出用） */
    uint16_t tx_buffer_len; /**< 发送帧缓冲区大小（建议 >= 32 字节） */
    uint16_t reduction_ratio; /**< 减速比（默认 1080） */
    uint8_t pole_pairs; /**< 电机极对数（默认 2） */
} finger_config_t;

/**
 * @brief 手指电机控制句柄
 */
struct finger_handle {
    finger_config_t config; /**< 配置副本 */
    clist_head_t node; /**< clist 链表节点 */
    protocol_packer_context_t packer; /**< 内嵌协议打包器 */

    /* 运行时状态 */
    finger_mode_t mode; /**< 当前工作模式 */
    uint16_t position; /**< 最近一次查询的电角度（0-359°） */
    uint32_t output_angle_raw; /**< 最近一次查询的输出轴角度原始值 */
    uint8_t status; /**< 最近一次查询的状态信息 */

    /* 待发送命令 */
    finger_cmd_type_t pending_cmd; /**< 待发送的命令类型 */
    uint32_t pending_position; /**< 目标位置值 */
    uint32_t pending_speed; /**< 目标速度值 */
    uint16_t pending_torque; /**< 目标力矩值 */
    uint8_t pending_new_id; /**< 新 ID（修改 ID 命令） */

    /* 回调 */
    finger_response_cb_t response_cb; /**< 应答回调 */
    void* callback_user_data; /**< 回调透传数据 */

    bool initialized; /**< 初始化完成标志 */
    bool response_pending; /**< 等待应答标志 */
};

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/** @brief 判断手指电机错误码是否表示成功 */
#define FINGER_IS_OK(err) ((err) >= 0)

/** @brief 判断手指电机错误码是否表示失败 */
#define FINGER_IS_ERR(err) ((err) < 0)

/** @brief 默认帧缓冲区大小 */
#define FINGER_TX_BUFFER_SIZE 32

/* Exported functions prototypes ---------------------------------------------*/

/* --- 子系统生命周期 --- */

/**
 * @brief 初始化手指电机服务子系统
 * @return 错误码
 */
finger_error_t finger_init(void);

/**
 * @brief 反初始化手指电机服务子系统，释放所有实例
 */
void finger_deinit(void);

/**
 * @brief 检查手指电机子系统是否已初始化
 * @return true 已初始化，false 未初始化
 */
bool finger_is_initialized(void);

/* --- 实例管理 --- */

/**
 * @brief 静态注册手指电机实例
 * @param config 配置结构体指针
 * @param instance 用户提供的句柄内存（需已清零）
 * @return 错误码
 */
finger_error_t finger_register_static(const finger_config_t* config,
    finger_handle_t* instance);

/**
 * @brief 注销手指电机实例
 * @param name 电机名称
 * @return 错误码
 */
finger_error_t finger_unregister(const char* name);

/**
 * @brief 按名称查找手指电机实例
 * @param name 电机名称
 * @return 实例指针，未找到返回 NULL
 */
finger_handle_t* finger_get_instance(const char* name);

/**
 * @brief 按电机 ID 和端口查找手指电机实例
 * @param motor_id 电机地址
 * @param port RS-485 端口
 * @return 实例指针，未找到返回 NULL
 */
finger_handle_t* finger_get_by_id(uint8_t motor_id, rs485_port_t port);

/**
 * @brief 获取手指电机实例链表头
 * @return clist 链表头指针，未初始化返回 NULL
 */
clist_head_t* finger_get_head(void);

/* --- 电机控制（异步命令） --- */

/**
 * @brief 启动电机
 * @param handle 电机句柄
 * @return 错误码
 */
finger_error_t finger_start(finger_handle_t* handle);

/**
 * @brief 急停
 * @param handle 电机句柄
 * @return 错误码
 */
finger_error_t finger_emergency_stop(finger_handle_t* handle);

/**
 * @brief 暂停/恢复电机运行
 * @param handle 电机句柄
 * @return 错误码
 */
finger_error_t finger_pause_resume(finger_handle_t* handle);

/**
 * @brief 参数装订（保存至 Flash）
 * @note 请勿在高速运动时使用，保存过程芯片会短暂无法响应
 * @param handle 电机句柄
 * @return 错误码
 */
finger_error_t finger_save_flash(finger_handle_t* handle);

/**
 * @brief 清除故障
 * @param handle 电机句柄
 * @return 错误码
 */
finger_error_t finger_clear_fault(finger_handle_t* handle);

/**
 * @brief 设置速度模式（无反馈）
 * @param handle 电机句柄
 * @param speed 速度值（最高位为方向，bit31=1 正转，bit31=0 反转）
 * @return 错误码
 */
finger_error_t finger_set_speed(finger_handle_t* handle, uint32_t speed);

/**
 * @brief 设置位置模式（无反馈，带力矩限制）
 * @param handle 电机句柄
 * @param position 目标位置值（= 角度(度) × 减速比 × 极对数）
 * @param torque 力矩值（0-16384，最大不超过 16384）
 * @return 错误码
 */
finger_error_t finger_set_position(finger_handle_t* handle,
    uint32_t position, uint16_t torque);

/**
 * @brief 设置位置+速度闭环模式（无反馈）
 * @param handle 电机句柄
 * @param position 目标位置值（最高位为方向）
 * @param speed 速度值
 * @return 错误码
 */
finger_error_t finger_set_position_speed(finger_handle_t* handle,
    uint32_t position, uint32_t speed);

/* --- 查询命令 --- */

/**
 * @brief 查询电机电角度位置（异步，结果通过回调返回）
 * @param handle 电机句柄
 * @return 错误码
 */
finger_error_t finger_read_position(finger_handle_t* handle);

/**
 * @brief 查询电机状态信息（异步，结果通过回调返回）
 * @param handle 电机句柄
 * @return 错误码
 */
finger_error_t finger_read_status(finger_handle_t* handle);

/**
 * @brief 查询输出轴角度（异步，结果通过回调返回）
 * @note 只有校准过输出轴才能正确读取
 * @param handle 电机句柄
 * @return 错误码
 */
finger_error_t finger_read_output_angle(finger_handle_t* handle);

/**
 * @brief 读取软件版本号（异步，结果通过回调返回）
 * @param handle 电机句柄
 * @return 错误码
 */
finger_error_t finger_read_version(finger_handle_t* handle);

/**
 * @brief 读取电机ID（异步，结果通过回调返回）
 * @param handle 电机句柄
 * @return 错误码
 */
finger_error_t finger_read_id(finger_handle_t* handle);

/* --- ID 及参数配置 --- */

/**
 * @brief 修改电机 ID（立即生效，需 finger_save_flash 才能断电保存）
 * @param handle 电机句柄
 * @param new_id 新 ID（1-254）
 * @return 错误码
 */
finger_error_t finger_set_id(finger_handle_t* handle, uint8_t new_id);

/* --- 状态查询（本地缓存，不发起通讯） --- */

/**
 * @brief 获取本地缓存的电机位置
 * @param handle 电机句柄
 * @return 电角度（0-359°），未初始化返回 0
 */
uint16_t finger_get_position(const finger_handle_t* handle);

/**
 * @brief 获取本地缓存的电机状态
 * @param handle 电机句柄
 * @return 状态值，未初始化返回 0
 */
uint8_t finger_get_status(const finger_handle_t* handle);

/**
 * @brief 获取本地缓存的电机模式
 * @param handle 电机句柄
 * @return 工作模式，未初始化返回 FINGER_MODE_IDLE
 */
finger_mode_t finger_get_mode(const finger_handle_t* handle);

/* --- 回调设置 --- */

/**
 * @brief 注册应答回调函数
 * @param handle 电机句柄
 * @param cb 应答回调
 * @param user_data 透传用户数据
 */
void finger_set_response_callback(finger_handle_t* handle,
    finger_response_cb_t cb, void* user_data);

/* --- 内部接口（供 finger_task 调用） --- */

/**
 * @brief 构建待发送命令帧（内部接口）
 * @param handle 电机句柄
 * @param p_out_frame 输出参数，返回完整帧指针
 * @param p_out_len 输出参数，返回帧长度
 * @return 错误码（FINGER_OK 有帧待发送，FINGER_ERROR_BUSY 表示电机忙）
 */
finger_error_t finger_build_command(finger_handle_t* handle,
    uint8_t** p_out_frame, uint16_t* p_out_len);

/**
 * @brief 处理应答帧数据（内部接口）
 * @param handle 电机句柄
 * @param frame 完整应答帧（含帧头帧尾校验）
 * @param len 帧长度
 * @return 错误码
 */
finger_error_t finger_process_response(finger_handle_t* handle,
    const uint8_t* frame, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* __FINGER_H */
