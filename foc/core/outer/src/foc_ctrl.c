#include "foc_ctrl.h"
#include "config.h"
#include "encoder.h"
#include "foc_task.h"
#include <stdio.h>

tController Controller;

#define CTRL_PERIOD (1.0f / 5000.0f) // Task_FOC 周期 (5kHz)

// 低速摩擦前馈与速度增益调度; 电流值来自 Config 随电机档位变化
// ENGAGE/RELEASE 速度阈值保持固定 (与电机无关)
#define FRICTION_FF_ENGAGE_VEL 0.010f
#define FRICTION_FF_RELEASE_VEL 0.050f
#define VEL_BOOST_GAIN 1.0f
#define VEL_BOOST_FULL_VEL 0.5f
#define VEL_BOOST_END_VEL 1.0f

static bool friction_boost;
static bool friction_ff_en = true;

static const char *profile_name(int32_t p)
{
	if (p == MOTOR_PROFILE_5210)
		return "5210";
	if (p == MOTOR_PROFILE_2312)
		return "2312";
	return "4310";
}

void CONTROLLER_update_input_pos_filter_gain(float bw)
{
	float bandwidth = bw * M_2PI;
	Controller.input_pos_filter_ki = 2.0f * bandwidth;
	Controller.input_pos_filter_kp =
	    0.25f * SQ(Controller.input_pos_filter_ki);
}

void CONTROLLER_init(void)
{
	Controller.ctrl_mode = (tControlMode)Config.default_op_mode;
	Controller.target_reached = false;

	Controller.op_mode_pending = false;
	Controller.home_pending = false;
	Controller.config_cmd_pending = 0;
	Controller.motor_profile_pending = -1;

	CONTROLLER_update_input_pos_filter_gain(Config.position_filter_bw);
}

void CONTROLLER_reset(void)
{
	taskENTER_CRITICAL();

	float th_meas =
	    (float)encoder.count_aligned * (6.2831853f / ENCODER_CPR_F);

	Controller.input_position = th_meas;
	Controller.input_velocity = 0.0f;
	Controller.input_current = 0.0f;

	Controller.input_position_buffer = th_meas;
	Controller.input_velocity_buffer = 0.0f;
	Controller.input_current_buffer = 0.0f;

	Controller.pos_setpoint = th_meas;
	Controller.vel_setpoint = 0.0f;
	Controller.cur_setpoint = 0.0f;

	Controller.vel_integrator = 0;

	friction_boost = false;

	Controller.input_updated = false;
	Controller.target_reached = false;

	FOC.current_ctrl_integral_d = 0;
	FOC.current_ctrl_integral_q = 0;

	taskEXIT_CRITICAL();
}

int CONTROLLER_set_op_mode(tControlMode mode)
{
	Controller.ctrl_mode = mode;
	CONTROLLER_reset();

	return 0;
}

int CONTROLLER_set_home(void)
{
	int ret = 0;

	if (foc_get_stage() == IDLE) {
		taskENTER_CRITICAL();
		encoder.shadow_count = 0;
		taskEXIT_CRITICAL();
	} else if (foc_get_stage() == RUN) {
		/* pos_setpoint 和位置误差都用 count_aligned 换算 (单圈弧度),
		 * 不能用 encoder.pos (多圈圈数, 单位不同, L8 修复) */
		float th_meas =
		    (float)encoder.count_aligned * (6.2831853f / ENCODER_CPR_F);
		float pos_err = Controller.pos_setpoint - th_meas;
		while (pos_err > M_PI)
			pos_err -= M_2PI;
		while (pos_err < -M_PI)
			pos_err += M_2PI;

		if ((ABS(encoder.vel) < 0.5f) && (ABS(pos_err) < 0.01f)) {
			taskENTER_CRITICAL();

			encoder.shadow_count = 0;
			encoder.pos = 0;
			encoder.pll_pos = (float)encoder.count_aligned;
			encoder.pll_vel = 0;
			encoder.count_in_cpr_prev = encoder.count_in_cpr;

			Controller.input_position = 0;
			Controller.input_position_buffer = 0;
			Controller.pos_setpoint = 0;
			Controller.vel_setpoint = 0;
			Controller.cur_setpoint = 0;

			taskEXIT_CRITICAL();
		} else {
			ret = -1;
		}
	} else {
		ret = -1;
	}

	return ret;
}

// 命令写入缓冲后立即锁存 (USB/CAN 调用); 状态不满足则只存缓冲
void CONTROLLER_sync_callback(void)
{
	if (foc_get_stage() != RUN)
		return;

	switch (Controller.ctrl_mode) {
	case CONTROL_MODE_CURRENT_RAMP:
		Controller.input_current = Controller.input_current_buffer;
		break;

	case CONTROL_MODE_VELOCITY_RAMP:
		Controller.input_velocity = Controller.input_velocity_buffer;
		break;

	case CONTROL_MODE_POSITION_FILTER:
	case CONTROL_MODE_POSITION_PROFILE:
		Controller.input_position = Controller.input_position_buffer;
		break;

	default:
		break;
	}

	Controller.target_reached = false;
}

// 5 kHz 外环生成电流给定；电流环在 ADC 中断中执行。
void CONTROLLER_loop(void)
{
	float iq_set = 0;
	float v_err = 0.0f;
	tFSMState stage = foc_get_stage();

	if (Controller.enable_refused_pending) {
		Controller.enable_refused_pending = false;
		printf("enable refused: not calibrated\n");
	}

	// 在任务上下文处理从中断延后的请求。
	if ((stage == IDLE) || (stage == RUN)) {
		if (Controller.op_mode_pending) {
			Controller.op_mode_pending = false;
			CONTROLLER_set_op_mode(Controller.op_mode_req);
		}
		if (Controller.home_pending) {
			Controller.home_pending = false;
			CONTROLLER_set_home();
		}
	}

	// 仅在失能时切换电机配置档。
	if (Controller.motor_profile_pending >= 0) {
		int32_t p = Controller.motor_profile_pending;
	Controller.motor_profile_pending = -1;
	Controller.enable_refused_pending = false;
		if (FOC.is_armed) {
			printf("motor profile: armed, refused\n");
		} else {
			CONFIG_apply_motor_profile(p);
			FOC_update_current_ctrl_gain(Config.current_ctrl_bw);
			uint8_t r = CONFIG_save_to_flash();
			printf("motor profile: %s Rs=%.3f L=%.1fuH Pn=%ld "
			       "lam=%.5f CLIM=%.1fA VLIM=%.1frps bw=%.0fHz "
			       "velkp=%.3f velki=%.2f dragmin=%.2fV "
			       "fluxiq=%.2fA jiq=%.2fA save=%s\n",
			       profile_name(p),
			       (double)Config.phase_resistance,
			       (double)(Config.phase_inductance * 1e6f),
			       (long)Config.pole_pairs,
			       (double)Config.flux_linkage,
			       (double)Config.current_limit,
			       (double)Config.velocity_limit,
			       (double)Config.current_ctrl_bw,
			       (double)Config.vel_p_gain,
			       (double)Config.vel_i_gain,
			       (double)Config.calib_drag_v_min,
			       (double)(Config.calib_current *
					Config.calib_flux_iq_gain),
			       (double)(Config.calib_current *
					Config.calib_j_iq_gain),
			       (r == 0U) ? "ok" : "fail");
			printf("motor profile: calibration invalidated, "
			       "run 0x27 again before use");
			if (Config.pole_pairs <= 0)
				printf(" (Pn/lambda unknown -> 0x27 dir_pp "
				       "will measure them)");
			printf("\n");
		}
	}

	if (Controller.config_cmd_pending != 0U) {
		uint8_t cmd = Controller.config_cmd_pending;
		Controller.config_cmd_pending = 0;
		if (FOC.is_armed) {
			printf("cfg cmd: motor armed, refused\n");
		} else if (cmd == 1U) {
			printf("cfg save: %s\n",
			       (CONFIG_save_to_flash() == 0) ? "ok" : "fail");
		} else if (cmd == 2U) {
			config_init();
			FOC_update_current_ctrl_gain(Config.current_ctrl_bw);
			CONFIG_save_to_flash();
			printf("cfg reset (profile=%ld, bw=%.0fHz)\n",
			       (long)Config.motor_profile,
			       (double)Config.current_ctrl_bw);
		}
	}

	if ((stage != RUN) && (stage != ANTICOGGING))
		return;

	const float vel_meas = encoder.vel; // rps
	float vel_des = 0.0f;

	switch (Controller.ctrl_mode) {
	case CONTROL_MODE_CURRENT_RAMP: {
		float max_step_size =
		    ABS(CTRL_PERIOD * Config.current_ramp_rate);
		float full_step =
		    Controller.input_current - Controller.cur_setpoint;
		float step = CLAMP(full_step, -max_step_size, max_step_size);
		Controller.cur_setpoint += step;
	} break;

	case CONTROL_MODE_VELOCITY_RAMP: {
		float max_step_size =
		    ABS(CTRL_PERIOD * Config.velocity_ramp_rate);
		float full_step =
		    Controller.input_velocity - Controller.vel_setpoint;
		float step = CLAMP(full_step, -max_step_size, max_step_size);
		Controller.vel_setpoint += step;
		Controller.cur_setpoint =
		    Config.current_ff_gain * (step / CTRL_PERIOD);

		if (!Controller.target_reached) {
			if (ABS(Controller.input_velocity - vel_meas) <
			    Config.target_velcity_window) {
				Controller.target_reached = true;
			}
		}
	} break;

	case CONTROL_MODE_POSITION_FILTER:
	case CONTROL_MODE_POSITION_PROFILE: {
		// 单圈机械角 (用对齐计数, 零位与校准一致), 单位 rad
		float th_meas =
		    (float)encoder.count_aligned * (6.2831853f / ENCODER_CPR_F);
		float perr = Controller.input_position - th_meas;
		while (perr > 3.14159265f)
			perr -= 6.2831853f; // 最短路径卷绕到 [-π, π]
		while (perr < -3.14159265f)
			perr += 6.2831853f;

		Controller.cur_setpoint = 0.0f;
		Controller.vel_setpoint = 0.0f;
		Controller.pos_setpoint = th_meas;
		vel_des = Config.pos_p_gain * perr; // 位置 P -> 速度目标

		if (!Controller.target_reached) {
			if (ABS(perr) < Config.target_position_window)
				Controller.target_reached = true;
		}
	} break;

	default:
		break;
	}

	// 速度模式用轨迹速度; 位置模式的 vel_des 已在上面按单圈角度 P 给出
	if (Controller.ctrl_mode < CONTROL_MODE_POSITION_FILTER)
		vel_des = Controller.vel_setpoint;

	// 速度限幅
	vel_des =
	    CLAMP(vel_des, -Config.velocity_limit, +Config.velocity_limit);

	// 速度环 PI
	iq_set = Controller.cur_setpoint;

	float vel_kp = Config.vel_p_gain;
	float vel_ki = Config.vel_i_gain;
	bool low_speed_assist = (foc_get_stage() != ANTICOGGING);

	if (Controller.ctrl_mode >= CONTROL_MODE_VELOCITY_RAMP) {
		// 低速时调度速度环增益。
		float av = fabsf(vel_meas);
		float boost = 1.0f;
		if (low_speed_assist) {
			if (av < VEL_BOOST_FULL_VEL)
				boost = VEL_BOOST_GAIN;
			else if (av < VEL_BOOST_END_VEL)
				boost = 1.0f + (VEL_BOOST_GAIN - 1.0f) *
						   (VEL_BOOST_END_VEL - av) /
						   (VEL_BOOST_END_VEL -
						    VEL_BOOST_FULL_VEL);
		}
		vel_kp *= boost;
		vel_ki *= boost;

		v_err = vel_des - vel_meas;
		iq_set += vel_kp * v_err;
		iq_set += Controller.vel_integrator;
	}

	// 电流模式下按速度限值约束 Iq 给定。
	if (Controller.ctrl_mode < CONTROL_MODE_VELOCITY_RAMP) {
		float Imax =
		    (+Config.velocity_limit - vel_meas) * Config.vel_p_gain;
		float Imin =
		    (-Config.velocity_limit - vel_meas) * Config.vel_p_gain;
		iq_set = CLAMP(iq_set, Imin, Imax);
	}

	// 根据静摩擦或动摩擦补偿前馈电流，USB 0x2B 可在线关闭。
	if (friction_ff_en &&
	    (Controller.ctrl_mode >= CONTROL_MODE_VELOCITY_RAMP) &&
	    low_speed_assist) {
		float av = fabsf(vel_meas);
		if (av < FRICTION_FF_ENGAGE_VEL)
			friction_boost = true;
		else if (av > FRICTION_FF_RELEASE_VEL)
			friction_boost = false;

		float ff =
		    friction_boost ? Config.friction_break_a
				   : Config.friction_move_a;
		if (vel_des > FRICTION_FF_ENGAGE_VEL)
			iq_set += ff;
		else if (vel_des < -FRICTION_FF_ENGAGE_VEL)
			iq_set -= ff;
	}

	// 对齿槽补偿表相邻点线性插值。
	if ((Controller.ctrl_mode >= CONTROL_MODE_VELOCITY_RAMP) &&
	    Config.anticogging_enable && Config.cogging_map_valid &&
	    (foc_get_stage() != ANTICOGGING)) {
		float fidx = (float)COGGING_MAP_NUM *
			     (float)encoder.count_aligned / ENCODER_CPR_F;
		int i0 = (int)fidx;
		float frac = fidx - (float)i0;
		if (i0 >= (int)COGGING_MAP_NUM)
			i0 = 0;
		int i1 = (i0 + 1 < (int)COGGING_MAP_NUM) ? i0 + 1 : 0;
		float cg = (1.0f - frac) * (float)Config.cogging_map[i0] +
			   frac * (float)Config.cogging_map[i1];
		iq_set += cg / COGGING_SCALE;
	}

	// 电流限幅
	bool limited = false;
	if (iq_set > +Config.current_limit) {
		limited = true;
		iq_set = +Config.current_limit;
	}
	if (iq_set < -Config.current_limit) {
		limited = true;
		iq_set = -Config.current_limit;
	}

	FOC.Id_set = 0.0f;
	FOC.Iq_set = iq_set;

	// 速度积分器 (抗饱和 + 限幅)
	if (Controller.ctrl_mode < CONTROL_MODE_VELOCITY_RAMP)
		Controller.vel_integrator = 0.0f;
	else if (limited)
		Controller.vel_integrator *= 0.99f;
	else
		Controller.vel_integrator += (vel_ki * CTRL_PERIOD) * v_err;
	// 齿槽标定期间放宽速度积分限幅; 正常运行用配置值 (随电机档位变化)
	float vint_lim =
	    (foc_get_stage() == ANTICOGGING) ? 4.0f
					     : Config.vel_integrator_limit;
	Controller.vel_integrator =
	    CLAMP(Controller.vel_integrator, -vint_lim, vint_lim);
}

// 读取摩擦前馈开关状态，供标定流程保存和恢复。
uint8_t CONTROLLER_get_friction_ff(void)
{
	return friction_ff_en ? 1U : 0U;
}

void CONTROLLER_set_friction_ff(uint8_t en)
{
	friction_ff_en = (en != 0U);
	if (!friction_ff_en)
		friction_boost = false;
}
