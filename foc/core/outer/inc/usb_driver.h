#ifndef __USB_DRIVER_H__
#define __USB_DRIVER_H__

#include <stdint.h>
#include "foc.h"

#define USB_CMD_RESET_ALL_CONFIG 0x3B
#define USB_CMD_SAVE_CONFIG 0x3F

#define USB_CMD_SET_OP_MODE 0x20

#define USB_CMD_CALIB_START 0x27
#define USB_CMD_ANTICOGGING_START 0x28

#define USB_CMD_SET_TORQUE 0x23
#define USB_CMD_SET_VELOCITY 0x24
#define USB_CMD_SET_POSITION 0x25

#define USB_CMD_MOTOR_ENABLE 0x21
#define USB_CMD_MOTOR_DISABLE 0x22

#define USB_CMD_SET_HOME 0x2D

/* MATLAB 生成模型 (SMO + I/F 强拖) */
#define USB_CMD_MODEL_START 0x30
#define USB_CMD_MODEL_STOP 0x31
#define USB_CMD_SET_MODEL_SPEED 0x32 /* float, 机械 rps */

#define USB_CMD_SET_CUR_KP 0x40
#define USB_CMD_SET_CUR_KI 0x41
#define USB_CMD_SET_CUR_BW 0x58

#define USB_CMD_SET_VEL_KP 0x42
#define USB_CMD_SET_VEL_KI 0x43

#define USB_CMD_SET_POS_KP 0x44 /* float 位置环 Kp */

/* I/F 强拖参数 (float; 改完可用 0x3F 存 Flash) */
#define USB_CMD_SET_IF_CURRENT 0x50 /* 强拖电流 (A) */
#define USB_CMD_SET_IF_ACCEL 0x51   /* 频率加速度 (rps/s) */
#define USB_CMD_SET_IF_SWITCH 0x52  /* 交接转速 (rps) */
#define USB_CMD_SET_IF_BLEND 0x53   /* 交接平滑时间 (ms) */
#define USB_CMD_SET_IF_STALL 0x54   /* 失步角 (电角度) */
#define USB_CMD_SET_IF_HOLD 0x55    /* 交接前保持时间 (ms) */

/* 保护阈值 (float, 单位 V; 改完可用 0x3F 存 Flash) */
#define USB_CMD_SET_UNDER_VOLTAGE 0x56
#define USB_CMD_SET_OVER_VOLTAGE 0x57

/* 读参数: 固件回一行 ASCII($FOCFG,...), 由 vofa 任务整包发出 */
#define USB_CMD_GET_CONFIG 0x3E

typedef struct {
	uint8_t id;
	uint8_t data[4];
} UsbFrame;

void USB_parse_frame(UsbFrame *frame);

#endif
