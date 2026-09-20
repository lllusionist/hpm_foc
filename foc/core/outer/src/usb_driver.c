#include "usb_driver.h"
#include "util.h"
#include "foc_task.h"
#include "foc_ctrl.h"

void USB_parse_frame(UsbFrame *frame)
{
	uint8_t id = frame->id;

	switch (id) {
	// 配置持久化 (延迟到任务里执行: 擦写要关中断几十 ms)
	case USB_CMD_SAVE_CONFIG:
		Controller.config_cmd_pending = 1;
		break;

	case USB_CMD_RESET_ALL_CONFIG:
		Controller.config_cmd_pending = 2;
		break;

	// 中断里只记请求, 由 1kHz 控制任务执行
	case USB_CMD_SET_OP_MODE: {
		tControlMode mode = (tControlMode)frame->data[0];
		if (mode <= CONTROL_MODE_POSITION_PROFILE) {
			Controller.op_mode_req     = mode;
			Controller.op_mode_pending = true;
		}
		break;
	}

	case USB_CMD_CALIB_START:
		foc_request_stage(CALIBRATION);
		break;

	case USB_CMD_SET_TORQUE: {
		float current = data_to_float(&frame->data[0]);

		if (current < -Config.current_limit) current = -Config.current_limit;
		if (current > Config.current_limit) current = Config.current_limit;

		if (Config.invert_motor_dir) {
			Controller.input_current_buffer = -current;
		} else {
			Controller.input_current_buffer = current;
		}
		CONTROLLER_sync_callback();
		break;
	}

	case USB_CMD_SET_VELOCITY: {
		float velocity = data_to_float(&frame->data[0]);

		if (velocity < -Config.velocity_limit) velocity = -Config.velocity_limit;
		if (velocity > Config.velocity_limit) velocity = Config.velocity_limit;

		if (Config.invert_motor_dir) {
			Controller.input_velocity_buffer = -velocity;
		} else {
			Controller.input_velocity_buffer = velocity;
		}
		CONTROLLER_sync_callback();
		break;
	}

	case USB_CMD_SET_POSITION: {
		float position = data_to_float(&frame->data[0]);

		if (position < 0.0f) position = 0.0f;
		if (position > 2.0f * 3.14159265f) position = 2.0f * 3.14159265f;

		if (Config.invert_motor_dir) {
			Controller.input_position_buffer = -position;
		} else {
			Controller.input_position_buffer = position;
		}
		CONTROLLER_sync_callback();
		break;
	}

	case USB_CMD_MOTOR_ENABLE:
		if (!Config.calib_valid) {
			printf("enable refused: not calibrated\n");
			break;
		}
		foc_request_stage(RUN);
		break;

	case USB_CMD_MOTOR_DISABLE:
		foc_request_stage(IDLE);
		break;

	case USB_CMD_SET_HOME:
		Controller.home_pending = true;
		break;

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

	case USB_CMD_SET_VEL_KP: {
		Config.vel_p_gain = data_to_float(&frame->data[0]);
		break;
	}

	case USB_CMD_SET_VEL_KI: {
		Config.vel_i_gain = data_to_float(&frame->data[0]);
		break;
	}

	case USB_CMD_SET_POS_KP: {
		Config.pos_p_gain = data_to_float(&frame->data[0]);
		break;
	}

	// 保护阈值 (改完用 0x3F 存 Flash)
	case USB_CMD_SET_UNDER_VOLTAGE: {
		Config.protect_under_voltage = data_to_float(&frame->data[0]);
		break;
	}

	case USB_CMD_SET_OVER_VOLTAGE: {
		Config.protect_over_voltage = data_to_float(&frame->data[0]);
		break;
	}

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

	case USB_CMD_DEBUG_FF:
		FOC.ff_enable = (data_to_float(&frame->data[0]) > 0.5f);
		break;

	case USB_CMD_ANTICOGGING_START: // 未移植
	default:
		break;
	}
}
