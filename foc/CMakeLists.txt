# Copyright (c) 2021 HPMicro
# SPDX-License-Identifier: BSD-3-Clause

cmake_minimum_required(VERSION 3.13)

# 硬件单精度浮点 ABI（必须在 find_package 之前设置，SDK 电机类例程均如此）
set(RV_ABI "ilp32f")
set(RV_ARCH "rv32imafc")

# 产物名，缺省为 demo（SDK application.cmake: if(NOT APP_NAME) set(APP_NAME demo)）
set(APP_NAME foc)

# SEGGER RTT：printf 走 J-Link，不占用 UART0 引脚
set(CONFIG_SEGGER_RTT 1)


# the following lines before "### END ###" are added during project localization
### START ###
set(ENV{HPM_SDK_BASE} "${CMAKE_CURRENT_SOURCE_DIR}/hpm_sdk_localized_for_hpm5e00evk")
set(LOCALIZED_BOARD "hpm5e00evk")
if(BOARD)
  if(NOT ${BOARD} MATCHES ${LOCALIZED_BOARD})
    message(FATAL_ERROR "ABORT:\n hpm sdk has been localized for ${LOCALIZED_BOARD} already.")
  endif()
endif()
set(BOARD ${LOCALIZED_BOARD})
### END ###


find_package(hpm-sdk REQUIRED HINTS $ENV{HPM_SDK_BASE})

project(foc)

# 关闭 UART console：否则它的 _write 与 RTT_Syscalls_GCC.c 的 _write 重定义冲突。
# 副作用：board.h 不再提供 BOARD_CONSOLE_UART_* 宏，sys.c 里对应的初始化已移除。
sdk_compile_definitions(-DCONFIG_NDEBUG_CONSOLE=1)

# SPI 组件的非阻塞(DMA)收发接口被 #if USE_DMA_MGR 包着，不开这个宏就没有声明
sdk_compile_definitions(-DUSE_DMA_MGR=1)

# USB 虚拟串口 (CherryUSB CDC ACM)
set(CONFIG_CHERRYUSB 1)
set(CONFIG_USB_DEVICE 1)
set(CONFIG_USB_DEVICE_CDC_ACM 1)

sdk_inc(core/inner/inc)
sdk_inc(core/outer/inc)

sdk_app_src(core/inner/src/main.c)
sdk_app_src(core/inner/src/sys.c)
sdk_app_src(core/inner/src/adc.c)
sdk_app_src(core/inner/src/tim.c)
sdk_app_src(core/inner/src/spi.c)
sdk_app_src(core/inner/src/dma.c)
sdk_app_src(core/inner/src/cdc_acm.c)
sdk_app_src(core/outer/src/foc.c)
sdk_app_src(core/outer/src/foc_task.c)
sdk_app_src(core/outer/src/encoder.c)
sdk_app_src(core/outer/src/config.c)
sdk_app_src(core/outer/src/util.c)
sdk_app_src(core/outer/src/vofa.c)
sdk_app_src(core/outer/src/usb_driver.c)
generate_ide_projects()
