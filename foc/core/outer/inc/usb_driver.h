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

#define USB_CMD_SET_CUR_KP 0x40
#define USB_CMD_SET_CUR_KI 0x41

/* 调试用临时命令 (定位 FOC 角度/方向问题后可删) */
#define USB_CMD_DEBUG_PHASE_OFFSET 0x50 // float 电角度附加偏移 (rad)
#define USB_CMD_DEBUG_ID 0x51		// float d 轴电流目标 (A)
#define USB_CMD_DEBUG_OPENLOOP_V 0x52	// float 开环固定矢量幅值 (V), 0=关闭
#define USB_CMD_DEBUG_OPENLOOP_ANGLE 0x53 // float 开环固定矢量电角度 (rad)
#define USB_CMD_DEBUG_FF 0x54		  // float 1=开/0=关 反电势前馈+解耦

#define USB_CMD_SET_VEL_KP 0x42
#define USB_CMD_SET_VEL_KI 0x43

#define USB_CMD_SET_POS_KP 0x44 // float 位置环 Kp

typedef struct {
	uint8_t id;
	uint8_t data[4];
} UsbFrame;

void USB_parse_frame(UsbFrame *frame);

#endif
