# Copyright (c) 2021 HPMicro
# SPDX-License-Identifier: BSD-3-Clause

cmake_minimum_required(VERSION 3.13)

# 硬件单精度浮点 ABI（必须在 find_package 之前设置，SDK 电机类例程均如此）
set(RV_ABI "ilp32f")
set(RV_ARCH "rv32imafc")

# 产物名，缺省为 demo（SDK application.cmake: if(NOT APP_NAME) set(APP_NAME demo)）
set(APP_NAME foc)


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

sdk_inc(src)
sdk_app_src(src/main.c)
generate_ide_projects()
