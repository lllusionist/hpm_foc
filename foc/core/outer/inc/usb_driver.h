#ifndef __USB_DRIVER_H__
#define __USB_DRIVER_H__

#include <stdint.h>
#include "foc.h"

#define USB_CMD_RESET_ALL_CONFIG 0x3B
#define USB_CMD_SAVE_CONFIG 0x3F

#define USB_CMD_SET_OP_MODE 0x20

#define USB_CMD_CALIB_START 0x27
#define USB_CMD_ANTICOGGING_START 0x28
#define USB_CMD_SET_ANTICOG 0x2A    /* float 0/1: 齿槽补偿在线开关 (A/B 用) */
#define USB_CMD_SET_FRICTION_FF 0x2B /* float 0/1: 摩擦前馈在线开关 */
#define USB_CMD_DUMP_COGGING 0x2C    /* 无参: 分块打印 512 点齿槽表 */
#define USB_CMD_ANTICOGGING_SWEEP 0x2E /* 无参: 恒速扫描法齿槽标定 (0x2D 已被 SET_HOME 占用) */

#define USB_CMD_SET_TORQUE 0x23
#define USB_CMD_SET_VELOCITY 0x24
#define USB_CMD_SET_POSITION 0x25

#define USB_CMD_MOTOR_ENABLE 0x21
#define USB_CMD_MOTOR_DISABLE 0x22

#define USB_CMD_SET_HOME 0x2D

#define USB_CMD_SET_CUR_KP 0x40
#define USB_CMD_SET_CUR_KI 0x41
#define USB_CMD_SET_CUR_BW 0x58

#define USB_CMD_SET_VEL_KP 0x42
#define USB_CMD_SET_VEL_KI 0x43

#define USB_CMD_SET_POS_KP 0x44 /* float 位置环 Kp */

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
