#include "usb_driver.h"
#include "util.h"
#include "foc_task.h"

void USB_parse_frame(UsbFrame *frame)
{
	uint8_t id = frame->id;

	switch (id) {
	// 现在就能用: 在线调电流环 PI
	case USB_CMD_SET_CUR_KP: {
		float Cur_Kp = data_to_float(&frame->data[0]);
		FOC.current_ctrl_p_gain = Cur_Kp;
		break;
	}

	case USB_CMD_SET_CUR_KI: {
		float Cur_Ki = data_to_float(&frame->data[0]);
		FOC.current_ctrl_i_gain = Cur_Ki;
		break;
	}

	// 下面这些要等控制层移植过来
	case USB_CMD_RESET_ALL_CONFIG:  // CONFIG_clear_all_settings()
	case USB_CMD_SAVE_CONFIG:       // CONFIG_save_to_flash()
	case USB_CMD_SET_OP_MODE:       // CONTROLLER_set_op_mode(mode)
	case USB_CMD_CALIB_START:
		foc_request_stage(CALIBRATION);
		break;
	case USB_CMD_ANTICOGGING_START: // MCT_set_state(ANTICOGGING)
	case USB_CMD_SET_TORQUE:        // Controller.input_current_buffer
	case USB_CMD_SET_VELOCITY:      // Controller.input_velocity_buffer
	case USB_CMD_SET_POSITION:      // Controller.input_position_buffer
	case USB_CMD_MOTOR_ENABLE:      // MCT_set_state(RUN)
	case USB_CMD_MOTOR_DISABLE:     // MCT_set_state(IDLE)
	case USB_CMD_SET_HOME:          // CONTROLLER_set_home()
	case USB_CMD_SET_VEL_KP:        // Config.vel_p_gain
	case USB_CMD_SET_VEL_KI:        // Config.vel_i_gain
	default:
		break;
	}
}
