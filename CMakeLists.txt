# Copyright (c) 2021 HPMicro
# Copyright (c) 2026 G1_Hand 项目组
# SPDX-License-Identifier: BSD-3-Clause

cmake_minimum_required(VERSION 3.13)


# the following lines before "### END ###" are added during project localization
### START ###
set(ENV{HPM_SDK_BASE} "${CMAKE_CURRENT_SOURCE_DIR}/hpm_sdk_localized_for_hpm6e00_ethercat_slave")
set(LOCALIZED_BOARD "hpm6e00_ethercat_slave")
if(BOARD)
  if(NOT ${BOARD} MATCHES ${LOCALIZED_BOARD})
    message(FATAL_ERROR "ABORT:\n hpm sdk has been localized for ${LOCALIZED_BOARD} already.")
  endif()
endif()
set(BOARD ${LOCALIZED_BOARD})
### END ###


find_package(hpm-sdk REQUIRED HINTS $ENV{HPM_SDK_BASE})

project(G1_Hand)

# ============================================================================
# 应用源文件
#
# 按架构分层注册所有应用源文件。
# 注意：generate_ide_projects() 会为 Segger Embedded Studio / IAR EW 自动
# 发现 .c/.h 文件，以下 sdk_app_src() 确保 Ninja/CMake 命令行构建也能完整编译。
# ============================================================================

# --- 应用入口 ---
sdk_app_src(user_app/app_main.c)

# --- 任务模块 (user_app/tasks) ---
sdk_app_src(user_app/tasks/uart_task.c)
sdk_app_src(user_app/tasks/led_task.c)
sdk_app_src(user_app/tasks/log_task.c)
sdk_app_src(user_app/tasks/finger_task.c)
sdk_app_src(user_app/tasks/motor_control_task.c)
sdk_app_src(user_app/tasks/motor_link_task.c)

# --- 服务层 (service) ---
sdk_app_src(service/led.c)
sdk_app_src(service/finger.c)
sdk_app_src(service/motor_control.c)

# --- 中间件：数据结构与算法 (middlewares/utils) ---
sdk_app_src(middlewares/utils/kfifo.c)
sdk_app_src(middlewares/utils/clist.c)
# --- 中间件：有限状态机 (middlewares/fsm) ---
sdk_app_src(middlewares/fsm/fsm.c)
# --- 中间件：协议工具 (middlewares/protocol_tools) ---
sdk_app_src(middlewares/protocol_tools/protocol_parser.c)
sdk_app_src(middlewares/protocol_tools/protocol_packer.c)
# --- 中间件：日志模块 (middlewares/log) ---
sdk_app_src(middlewares/log/log.c)

# --- 设备驱动层 (drivers/device_driver) ---
sdk_app_src(drivers/device_driver/drv_rs485.c)
sdk_app_src(drivers/device_driver/drv_led.c)
sdk_app_src(drivers/device_driver/drv_can.c)
sdk_app_src(drivers/device_driver/drv_log_uart.c)

# --- BSP 层 (drivers/bsp) ---
sdk_app_src(drivers/bsp/bsp_uart.c)
sdk_app_src(drivers/bsp/bsp_gpio.c)
sdk_app_src(drivers/bsp/bsp_systick.c)

# ============================================================================
# 头文件搜索路径
#
# 为跨目录引用提供 include 路径。使用 PRIVATE 作用域，因为这些头文件
# 仅由 app 目标内部使用，不对外暴露。
# ============================================================================

target_include_directories(app PRIVATE
    # 应用层
    ${CMAKE_CURRENT_SOURCE_DIR}/user_app/tasks

    # 服务层
    ${CMAKE_CURRENT_SOURCE_DIR}/service

    # 中间件
    ${CMAKE_CURRENT_SOURCE_DIR}/middlewares/utils
    ${CMAKE_CURRENT_SOURCE_DIR}/middlewares/fsm
    ${CMAKE_CURRENT_SOURCE_DIR}/middlewares/protocol_tools
    ${CMAKE_CURRENT_SOURCE_DIR}/middlewares/log

    # BSP 层
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/device_driver
    ${CMAKE_CURRENT_SOURCE_DIR}/drivers/bsp
)

# ============================================================================
# IDE 项目生成
#
# 为 Segger Embedded Studio 和 IAR EW 生成 IDE 项目文件。
# IDE 项目会自动发现并注册所有 .c/.h 源文件，无需手动维护文件列表。
# ============================================================================

generate_ide_projects()
