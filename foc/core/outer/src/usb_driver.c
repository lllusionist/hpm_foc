#include "usb_driver.h"
#include "util.h"
#include "foc_task.h"
#include "foc_ctrl.h"
#include "foc_matlab.h"
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
		if (mode >= 0 && mode <= CONTROL_MODE_IF) {
			Controller.op_mode_req = (tControlMode)mode;
			Controller.op_mode_pending = true;
			/* IF 只作启动过程模式, 不写进默认模式 */
			if (mode != CONTROL_MODE_IF)
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

	/* MATLAB 生成模型: 启动/停止/目标机械转速 (模型内部自带 I/F 与 SMO) */
	case USB_CMD_MODEL_START:
		foc_request_stage(MATLAB_RUN);
		break;

	case USB_CMD_MODEL_STOP:
		foc_request_stage(IDLE);
		break;

	case USB_CMD_SET_MODEL_SPEED: {
		float rps = CLAMP(data_to_float(&frame->data[0]),
				  -Config.velocity_limit,
				  Config.velocity_limit);
		FOC_MATLAB_set_speed_rps(rps);
		break;
	}

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

	case USB_CMD_ANTICOGGING_START:
		foc_request_stage(ANTICOGGING);
		break;

	/* I/F 强拖参数: 先发速度指令(0x24)定方向和交接目标, 再切模式 0x20=4 */
	case USB_CMD_SET_IF_CURRENT:
		Config.if_current = CLAMP(data_to_float(&frame->data[0]), 0.05f,
					  Config.current_limit);
		break;

	case USB_CMD_SET_IF_ACCEL:
		Config.if_accel_rps2 = CLAMP(data_to_float(&frame->data[0]),
					     0.5f, 200.0f);
		break;

	case USB_CMD_SET_IF_SWITCH:
		Config.if_switch_rps = CLAMP(data_to_float(&frame->data[0]),
					     0.2f, Config.velocity_limit);
		break;

	case USB_CMD_SET_IF_BLEND:
		Config.if_blend_ms = CLAMP(data_to_float(&frame->data[0]), 0.0f,
					   1000.0f);
		break;

	case USB_CMD_SET_IF_STALL:
		Config.if_stall_deg = CLAMP(data_to_float(&frame->data[0]),
					    10.0f, 120.0f);
		break;

	case USB_CMD_SET_IF_HOLD:
		Config.if_hold_ms = CLAMP(data_to_float(&frame->data[0]), 0.0f,
					  2000.0f);
		break;

	default:
		break;
	}
}
