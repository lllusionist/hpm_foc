#include "usb_driver.h"
#include "util.h"
#include "foc_task.h"
#include "foc_ctrl.h"
#include "vofa.h"

void USB_parse_frame(UsbFrame *frame)
{
	uint8_t id = frame->id;

	switch (id) {
	/* 配置持久化 (延迟到任务里执行: 擦写要关中断几十 ms) */
	case USB_CMD_SAVE_CONFIG:
		Controller.config_cmd_pending = 1;
		break;

	case USB_CMD_RESET_ALL_CONFIG:
		Controller.config_cmd_pending = 2;
		break;

	/* 中断里只记请求, 由 1kHz 控制任务执行 */
	case USB_CMD_SET_OP_MODE: {
		int mode = (int)data_to_float(&frame->data[0]);
		if (mode >= 0 && mode <= CONTROL_MODE_POSITION_PROFILE) {
			Controller.op_mode_req = (tControlMode)mode;
			Controller.op_mode_pending = true;
			Config.default_op_mode = mode;
		}
		break;
	}

	case USB_CMD_CALIB_START:
		foc_request_stage(CALIBRATION);
		break;

	case USB_CMD_SET_TORQUE: {
		float current = data_to_float(&frame->data[0]);

		if (current < -Config.current_limit)
			current = -Config.current_limit;
		if (current > Config.current_limit)
			current = Config.current_limit;

		if (Config.invert_motor_dir)
			Controller.input_current_buffer = -current;
		else
			Controller.input_current_buffer = current;
		CONTROLLER_sync_callback();
		break;
	}

	case USB_CMD_SET_VELOCITY: {
		float velocity = data_to_float(&frame->data[0]);

		if (velocity < -Config.velocity_limit)
			velocity = -Config.velocity_limit;
		if (velocity > Config.velocity_limit)
			velocity = Config.velocity_limit;

		if (Config.invert_motor_dir)
			Controller.input_velocity_buffer = -velocity;
		else
			Controller.input_velocity_buffer = velocity;
		CONTROLLER_sync_callback();
		break;
	}

	case USB_CMD_SET_POSITION: {
		float position = data_to_float(&frame->data[0]);

		/* 位置为单圈机械角(rad), 允许正负, 限幅到 [-π, π] */
		if (position > 3.14159265f)
			position = 3.14159265f;
		if (position < -3.14159265f)
			position = -3.14159265f;

		if (Config.invert_motor_dir)
			Controller.input_position_buffer = -position;
		else
			Controller.input_position_buffer = position;
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

	/* 现在就能用: 在线调电流环 PI */
	case USB_CMD_SET_CUR_KP: {
		float Cur_Kp = data_to_float(&frame->data[0]);
		if (Cur_Kp < 0.0f)
			Cur_Kp = 0.0f;
		if (Cur_Kp > 1.0e5f)
			Cur_Kp = 1.0e5f;
		FOC.current_ctrl_p_gain = Cur_Kp;
		Config.current_kp = Cur_Kp;
		Config.current_gain_magic = CONFIG_MAGIC;
		break;
	}

	case USB_CMD_SET_CUR_KI: {
		float Cur_Ki = data_to_float(&frame->data[0]);
		if (Cur_Ki < 0.0f)
			Cur_Ki = 0.0f;
		if (Cur_Ki > 1.0e7f)
			Cur_Ki = 1.0e7f;
		FOC.current_ctrl_i_gain = Cur_Ki;
		Config.current_ki = Cur_Ki;
		Config.current_gain_magic = CONFIG_MAGIC;
		break;
	}

	case USB_CMD_SET_CUR_BW: {
		float bw = data_to_float(&frame->data[0]);
		if (bw < 50.0f)
			bw = 50.0f;
		if (bw > 2000.0f)
			bw = 2000.0f;
		Config.current_ctrl_bw = bw;
		FOC_update_current_ctrl_gain(bw);
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

	/* 保护阈值 (改完用 0x3F 存 Flash) */
	case USB_CMD_SET_UNDER_VOLTAGE: {
		Config.protect_under_voltage = data_to_float(&frame->data[0]);
		break;
	}

	case USB_CMD_SET_OVER_VOLTAGE: {
		Config.protect_over_voltage = data_to_float(&frame->data[0]);
		break;
	}

	case USB_CMD_GET_CONFIG:
		vofa_request_config();
		break;

	/* A/B 调试开关: 本函数在 USB 中断里跑, 不打 printf */
	case USB_CMD_SET_ANTICOG:
		Config.anticogging_enable =
		    (data_to_float(&frame->data[0]) > 0.5f) ? 1 : 0;
		break;

	case USB_CMD_SET_FRICTION_FF:
		CONTROLLER_set_friction_ff(
		    (data_to_float(&frame->data[0]) > 0.5f) ? 1U : 0U);
		break;

	case USB_CMD_ANTICOGGING_START:
		foc_request_stage(ANTICOGGING);
		break;

	default:
		break;
	}
}
