#ifndef __USB_DRIVER_H__
#define __USB_DRIVER_H__

#include <stdint.h>

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

#define USB_CMD_SET_VEL_KP 0x42
#define USB_CMD_SET_VEL_KI 0x43

typedef struct {
	uint8_t id;
	uint8_t data[4];
} UsbFrame;

void USB_parse_frame(UsbFrame *frame);

#endif
