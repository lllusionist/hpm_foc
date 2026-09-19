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

	// 配置持久化 (擦写期间会关中断, 电机使能时禁止)
	case USB_CMD_SAVE_CONFIG: {
		if (FOC.is_armed) {
			printf("cfg save: motor armed, refused\n");
			break;
		}
		uint8_t ret = CONFIG_save_to_flash();
		printf("cfg save: %s\n", (ret == 0) ? "ok" : "fail");
		break;
	}

	case USB_CMD_RESET_ALL_CONFIG:
		if (FOC.is_armed) {
			printf("cfg reset: motor armed, refused\n");
			break;
		}
		config_init();
		CONFIG_save_to_flash();
		printf("cfg reset\n");
		break;

	case USB_CMD_SET_OP_MODE: // CONTROLLER_set_op_mode(mode)
		break;

	case USB_CMD_CALIB_START:
		foc_request_stage(CALIBRATION);
		break;

	case USB_CMD_SET_TORQUE: {
		float iq = data_to_float(&frame->data[0]);
		FOC.Iq_set = iq;
		break;
	}

	case USB_CMD_MOTOR_ENABLE:
		foc_request_stage(RUN);
		break;

	case USB_CMD_MOTOR_DISABLE:
		foc_request_stage(IDLE);
		break;

	// 调试用: 定位 FOC 角度/方向问题
	case USB_CMD_DEBUG_PHASE_OFFSET:
		FOC.phase_offset = data_to_float(&frame->data[0]);
		break;

	case USB_CMD_DEBUG_ID: {
		float id = data_to_float(&frame->data[0]);
		FOC.Id_set = id;
		break;
	}

	case USB_CMD_DEBUG_OPENLOOP_V:
		FOC.openloop_voltage = data_to_float(&frame->data[0]);
		break;

	case USB_CMD_DEBUG_OPENLOOP_ANGLE:
		FOC.openloop_angle = data_to_float(&frame->data[0]);
		break;

	case USB_CMD_ANTICOGGING_START: // MCT_set_state(ANTICOGGING)
	case USB_CMD_SET_VELOCITY:      // Controller.input_velocity_buffer
	case USB_CMD_SET_POSITION:      // Controller.input_position_buffer
	case USB_CMD_SET_HOME:          // CONTROLLER_set_home()
	case USB_CMD_SET_VEL_KP:        // Config.vel_p_gain
	case USB_CMD_SET_VEL_KI:        // Config.vel_i_gain
	default:
		break;
	}
}
