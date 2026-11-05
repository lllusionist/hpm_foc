#include "foc_ctrl.h"
#include "config.h"
#include "encoder.h"
#include "foc_task.h"
#include <stdio.h>

tController Controller;

#define CTRL_PERIOD (1.0f / 5000.0f) /* Task_FOC 周期 (5kHz) */

/* 低速摩擦前馈 + 增益调度 (4310 实测: 静摩擦约 0.05A, 动摩擦约 0.03A).
 * 把 MOVE/BREAK 设为 0 可关闭前馈; VEL_BOOST_GAIN 设为 1 可关闭调度.
 */
#define FRICTION_FF_MOVE_A 0.030f
#define FRICTION_FF_BREAK_A 0.050f
#define FRICTION_FF_ENGAGE_VEL 0.010f
#define FRICTION_FF_RELEASE_VEL 0.050f
/* 基础增益已到 0.17/5.3, 低速不再额外放大 */
#define VEL_BOOST_GAIN 1.0f
#define VEL_BOOST_FULL_VEL 0.5f
#define VEL_BOOST_END_VEL 1.0f

static bool friction_boost;

/* ---------------- I/F 强拖 (CONTROL_MODE_IF) ----------------
 * 只换角度来源: 强制电角度按频率斜坡推进, 电流仍由电流环锁定
 * (禁止用 FOC_voltage() 强拖, 见 docs/CODE_STYLE.md 功率级安全)。
 * 两段交接 (参考无感工程的分段切换):
 *   DRAG: 强制角斜坡 -> 到 if_switch_rps 并保持 if_hold_ms
 *   CUT : 角度源切到编码器+偏置, 电流保持强拖值, 偏置收到 0
 *   最后交给速度环 (设定点取实测, 积分器预置强拖电流)
 */
#define IF_TICKS_PER_MS (uint32_t)(1.0f / (CTRL_PERIOD * 1000.0f)) /* 5 */
#define IF_START_MAX_RPS 0.5f	/* 起转前转速上限, 超过则拒绝强拖 */
#define IF_SWITCH_MIN_RPS 0.2f	/* 交接转速下限 (rps) */
#define IF_CUR_MIN_A 0.05f	/* 强拖电流下限 (A) */
#define IF_ACCEL_MIN_RPS2 0.5f	/* 频率加速度下限 (rps/s) */
#define IF_STALL_MIN_DEG 10.0f	/* 失步角下限 (电角度) */
#define IF_STALL_MAX_DEG 120.0f /* 失步角上限 (电角度) */
#define IF_START_W_RATIO 0.1f	/* 起始角速度 = 交接角速度的 10% */
#define IF_TIMEOUT_MS 3000U	/* 强拖总超时 */
#define IF_PRINT_MS 500U	/* 状态打印周期 (5kHz 里不能每拍 printf) */
#define IF_LOSS_SETTLE_MS 20U	/* 失步去抖 */
#define IF_WIN_RATIO 0.5f	/* 交接窗口: 负载角/转速差共用 */
#define IF_CUT_MIN_RATIO 0.5f	/* 角度交接期转速不得低于该比例 */
#define IF_BLEND_MAX_TICKS 5000U /* 交接平滑上限 (1000ms) */
#define VEL_INT_LIMIT 0.5f	/* Controller.vel_integrator 全局限幅 */

/* if_step() 返回 */
enum {
	IF_STEP_RUN = 0, /* 仍在强拖, 本拍外环已被接管 */
	IF_STEP_DONE,	 /* 已完成 (交接成功, 或拒绝启动后落回速度环) */
	IF_STEP_ABORT,	 /* 失败停机, 本拍不要再走外环 */
};

static float if_wrap_pi(float a)
{
	while (a > M_PI)
		a -= M_2PI;
	while (a < -M_PI)
		a += M_2PI;
	return a;
}

/* 清强拖状态 (进出该模式时调用; 在临界区里跑, 不打印) */
static void if_reset(void)
{
	Controller.if_stage = IF_STAGE_OFF;
	Controller.if_tick = 0;
	Controller.if_hold_cnt = 0;
	Controller.if_loss_cnt = 0;
	Controller.if_blend_left = 0;
	Controller.if_drag_i = 0.0f;
	Controller.if_omega_sw = 0.0f;
	Controller.if_stall_rad = 0.0f;
	Controller.if_blend_step = 0.0f;

	FOC.if_mode = false;
	FOC.if_angle = 0.0f;
	FOC.if_phase_vel = 0.0f;
	FOC.if_iq = 0.0f;
	FOC.phase_offset = 0.0f;
}

/* 启动强拖; 条件不满足时退回速度模式并返回 false */
static bool if_start(void)
{
	float p = (float)Config.pole_pairs;
	float sw;
	float dir;

	if (p < 1.0f) {
		printf("IF refused: pole pairs not calibrated\n");
		Controller.ctrl_mode = CONTROL_MODE_VELOCITY_RAMP;
		return false;
	}

	/* 静止才能强拖: 已经在转就拒绝, 直接落回速度环 */
	if (fabsf(encoder.vel) > IF_START_MAX_RPS) {
		printf("IF refused: spinning %.2f rps\n",
		       (double)encoder.vel);
		Controller.ctrl_mode = CONTROL_MODE_VELOCITY_RAMP;
		return false;
	}

	sw = CLAMP(Config.if_switch_rps, IF_SWITCH_MIN_RPS,
		   Config.velocity_limit);
	Controller.if_drag_i =
	    CLAMP(Config.if_current, 0.0f, Config.current_limit);
	if (Controller.if_drag_i < IF_CUR_MIN_A) {
		printf("IF refused: current %.3f A too low\n",
		       (double)Controller.if_drag_i);
		Controller.ctrl_mode = CONTROL_MODE_VELOCITY_RAMP;
		return false;
	}
	Controller.if_stall_rad = CLAMP(Config.if_stall_deg, IF_STALL_MIN_DEG,
					IF_STALL_MAX_DEG) *
				  DEG_TO_RAD;

	/* 方向跟随速度指令缓冲的符号 (0x24 下发), 缺省正转 */
	dir = (Controller.input_velocity_buffer < 0.0f) ? -1.0f : 1.0f;
	Controller.if_omega_sw = M_2PI * p * sw * dir;
	Controller.if_tick = 0;
	Controller.if_hold_cnt = 0;
	Controller.if_loss_cnt = 0;
	Controller.if_blend_left = 0;

	/* 强制角从当前转子电角度起步; 起始角速度给一点先行量,
	 * 避免矢量正好压在 d 轴上打滑 */
	FOC.if_angle = encoder.phase;
	FOC.if_phase_vel = IF_START_W_RATIO * Controller.if_omega_sw;
	FOC.if_iq = 0.0f;
	FOC.if_mode = true; /* 最后置位: 中断只能看到完整参数 */
	Controller.if_stage = IF_STAGE_DRAG;

	printf("IF: start cur=%.2fA acc=%.1frps/s sw=%.2frps dir=%d\n",
	       (double)Controller.if_drag_i, (double)Config.if_accel_rps2,
	       (double)sw, (int)dir);
	return true;
}

/* 失败停机: 立刻断强拖电流, 退回 IDLE (故障码由保护逻辑给) */
static uint8_t if_abort(const char *why)
{
	float dlt = if_wrap_pi(encoder.phase - FOC.if_angle);
	float we = FOC.if_phase_vel;
	float iq = FOC.if_iq;

	FOC.if_mode = false;
	FOC.if_phase_vel = 0.0f;
	FOC.if_iq = 0.0f;
	FOC.phase_offset = 0.0f;
	Controller.if_blend_left = 0;
	Controller.if_hold_cnt = 0;
	Controller.if_stage = IF_STAGE_OFF;
	Controller.ctrl_mode = CONTROL_MODE_VELOCITY_RAMP;

	printf("IF abort: %s (we=%.1frad/s d=%.0fdeg iq=%.2fA)\n", why,
	       (double)we, (double)(dlt * RAD_TO_DEG), (double)iq);
	foc_request_stage(IDLE);
	return IF_STEP_ABORT;
}

/* 角度段交接: 角度源从强制角切到编码器+偏置, 电流保持强拖值 */
static void if_cut_begin(void)
{
	float off = if_wrap_pi(FOC.if_angle - encoder.phase);
	float we = FOC.if_phase_vel;
	uint32_t n = (uint32_t)(Config.if_blend_ms *
				(float)IF_TICKS_PER_MS);

	if (n < 1U)
		n = 1U;
	if (n > IF_BLEND_MAX_TICKS)
		n = IF_BLEND_MAX_TICKS;

	/* 先写偏置再关强拖: 顺序反了会有 1 拍角度跳变 */
	FOC.phase_offset = off;
	FOC.if_mode = false;
	FOC.if_phase_vel = 0.0f;

	Controller.if_blend_step = off / (float)n;
	Controller.if_blend_left = (uint16_t)n;
	Controller.if_stage = IF_STAGE_CUT;

	printf("IF cut: d=%.0fdeg we=%.1f blend=%.0fms\n",
	       (double)(off * RAD_TO_DEG), (double)we,
	       (double)Config.if_blend_ms);
}

/* 速度段交接: 角度已是编码器, 速度环接管, 强拖电流转成积分器初值 */
static uint8_t if_speed_handover(void)
{
	float iq = FOC.if_iq;

	FOC.if_iq = 0.0f;
	FOC.phase_offset = 0.0f;
	Controller.if_stage = IF_STAGE_OFF;

	Controller.ctrl_mode = CONTROL_MODE_VELOCITY_RAMP;
	Controller.vel_setpoint = encoder.vel;
	Controller.vel_integrator = CLAMP(iq, -VEL_INT_LIMIT, VEL_INT_LIMIT);
	Controller.cur_setpoint = 0.0f;

	printf("IF done: rps=%.2f -> vel mode\n", (double)encoder.vel);
	return IF_STEP_DONE;
}

/* 5kHz 推进一步; 强拖期间由 CONTROLLER_loop 直接接管外环 */
static uint8_t if_step(void)
{
	const float p = (float)Config.pole_pairs;
	float step = ABS(Config.current_ramp_rate * CTRL_PERIOD);
	float we_step = M_2PI * p *
			MAX(Config.if_accel_rps2, IF_ACCEL_MIN_RPS2) *
			CTRL_PERIOD;
	float hold_ticks = CLAMP(Config.if_hold_ms, 0.0f, 2000.0f) *
			   (float)IF_TICKS_PER_MS;
	float dlt, w_meas;

	if (Controller.if_stage == IF_STAGE_OFF) {
		if (!if_start())
			return IF_STEP_DONE;
	}
	if (!FOC.is_armed)
		return if_abort("disarmed");

	Controller.if_tick++;
	if (Controller.if_tick > (IF_TIMEOUT_MS * IF_TICKS_PER_MS))
		return if_abort("timeout");

	w_meas = encoder.vel * M_2PI * p;

	if (Controller.if_stage == IF_STAGE_DRAG) {
		/* q 轴电流斜坡 */
		if (FOC.if_iq < Controller.if_drag_i) {
			FOC.if_iq += step;
			if (FOC.if_iq > Controller.if_drag_i)
				FOC.if_iq = Controller.if_drag_i;
		}

		/* 电角速度斜坡, 方向由 if_omega_sw 的符号决定 */
		if (fabsf(FOC.if_phase_vel) <
		    fabsf(Controller.if_omega_sw)) {
			if (Controller.if_omega_sw > 0.0f)
				FOC.if_phase_vel += we_step;
			else
				FOC.if_phase_vel -= we_step;
		}
		if (fabsf(FOC.if_phase_vel) > fabsf(Controller.if_omega_sw))
			FOC.if_phase_vel = Controller.if_omega_sw;

		/* 失步: 负载角 = 真实转子电角度 - 强制矢量电角度 */
		dlt = if_wrap_pi(encoder.phase - FOC.if_angle);
		if (fabsf(dlt) > Controller.if_stall_rad) {
			if (++Controller.if_loss_cnt >=
			    (IF_LOSS_SETTLE_MS * IF_TICKS_PER_MS))
				return if_abort("loss of sync");
		} else {
			Controller.if_loss_cnt = 0;
		}

		/* 到切换转速后先保持 if_hold_ms, 稳定才进角度交接 */
		if ((fabsf(FOC.if_phase_vel) >=
		     fabsf(Controller.if_omega_sw)) &&
		    (fabsf(dlt) <
		     IF_WIN_RATIO * Controller.if_stall_rad) &&
		    (fabsf(w_meas - FOC.if_phase_vel) <
		     IF_WIN_RATIO * fabsf(FOC.if_phase_vel))) {
			if (++Controller.if_hold_cnt >= (uint16_t)hold_ticks) {
				if_cut_begin();
				return IF_STEP_RUN;
			}
		} else {
			Controller.if_hold_cnt = 0;
		}

		if ((Controller.if_tick %
		     (IF_PRINT_MS * IF_TICKS_PER_MS)) == 0U)
			printf("IF: we=%.1f wm=%.1f d=%.0fdeg iq=%.2fA\n",
			       (double)FOC.if_phase_vel, (double)w_meas,
			       (double)(dlt * RAD_TO_DEG), (double)FOC.if_iq);

		return IF_STEP_RUN;
	}

	/* IF_STAGE_CUT: 角度已交给编码器+偏置, 电流保持强拖值 */
	if (Controller.if_blend_left > 0U) {
		Controller.if_blend_left--;
		if (Controller.if_blend_left == 0U)
			FOC.phase_offset = 0.0f;
		else
			FOC.phase_offset -= Controller.if_blend_step;
	}
	FOC.if_iq = Controller.if_drag_i;

	/* 角度交接期间转速塌了 (负载过重/滑极) 就停机 */
	if (fabsf(w_meas) <
	    IF_CUT_MIN_RATIO * fabsf(Controller.if_omega_sw))
		return if_abort("speed collapse");

	if (Controller.if_blend_left == 0U)
		return if_speed_handover();

	if ((Controller.if_tick % (IF_PRINT_MS * IF_TICKS_PER_MS)) == 0U)
		printf("IF cut: off=%.1fdeg wm=%.1f iq=%.2fA\n",
		       (double)(FOC.phase_offset * RAD_TO_DEG),
		       (double)w_meas, (double)FOC.if_iq);

	return IF_STEP_RUN;
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
	/* IF 是启动过程模式, 不能作为上电默认模式 (上电即强拖) */
	if (Controller.ctrl_mode == CONTROL_MODE_IF)
		Controller.ctrl_mode = CONTROL_MODE_VELOCITY_RAMP;
	Controller.target_reached = false;

	Controller.op_mode_pending = false;
	Controller.home_pending = false;
	Controller.config_cmd_pending = 0;

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

	/* 切模式必须清干净强拖状态, 避免残留强制矢量 */
	if (Controller.ctrl_mode == CONTROL_MODE_IF) {
		if_reset();
	} else {
		FOC.if_mode = false;
		FOC.if_phase_vel = 0.0f;
		FOC.if_iq = 0.0f;
		FOC.phase_offset = 0.0f;
		Controller.if_stage = IF_STAGE_OFF;
		Controller.if_blend_left = 0;
	}

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
		float pos_err = Controller.pos_setpoint - encoder.pos;
		if ((ABS(encoder.vel) < 0.5f) && (ABS(pos_err) < 0.01f)) {
			taskENTER_CRITICAL();

			encoder.shadow_count = 0;
			encoder.pos = 0;
			encoder.pll_pos =
			    (float)encoder
				.count_aligned; /* PLL 对齐当前单圈位置 */
			encoder.pll_vel = 0;
			/* 防止产生错误 delta */
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

/* 命令写入缓冲后立即锁存 (USB/CAN 调用); 状态不满足则只存缓冲 */
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

	/* 强拖期间仍接收速度指令: 交接后就是速度环的目标 */
	case CONTROL_MODE_IF:
		Controller.input_velocity = Controller.input_velocity_buffer;
		break;

	default:
		break;
	}

	Controller.target_reached = false;
}

/* 1kHz 外环: 生成 FOC.Iq_set, 电流环仍在 ADC 中断里执行 */
void CONTROLLER_loop(void)
{
	float iq_set = 0;
	float v_err = 0.0f;
	tFSMState stage = foc_get_stage();

	/* 处理中断里提交的请求 (必须在任务上下文执行) */
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
			CONFIG_save_to_flash();
			printf("cfg reset\n");
		}
	}

	if ((stage != RUN) && (stage != ANTICOGGING))
		return;

	/* I/F 强拖: 独占外环 (不走速度/位置级联, 也不做摩擦/齿槽前馈) */
	if (Controller.ctrl_mode == CONTROL_MODE_IF) {
		uint8_t st = if_step();

		if (st == IF_STEP_RUN) {
			/* 中断按 FOC.if_iq 输出, 这两个值只为上位机观测 */
			FOC.Id_set = 0.0f;
			FOC.Iq_set = FOC.if_iq;
			return;
		}
		if (st == IF_STEP_ABORT) {
			FOC.Id_set = 0.0f;
			FOC.Iq_set = 0.0f;
			return;
		}
		/* IF_STEP_DONE: 已交接成速度模式, 本拍继续走下面的速度环 */
	}

	const float vel_meas = encoder.vel; /* rps */
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
		/* 单圈机械角 (用对齐计数, 零位与校准一致), 单位 rad */
		float th_meas =
		    (float)encoder.count_aligned * (6.2831853f / ENCODER_CPR_F);
		float perr = Controller.input_position - th_meas;
		while (perr > 3.14159265f)
			perr -= 6.2831853f; /* 最短路径卷绕到 [-π, π] */
		while (perr < -3.14159265f)
			perr += 6.2831853f;

		Controller.cur_setpoint = 0.0f;
		Controller.vel_setpoint = 0.0f;
		Controller.pos_setpoint = th_meas;
		vel_des = Config.pos_p_gain * perr; /* 位置 P -> 速度目标 */

		if (!Controller.target_reached) {
			if (ABS(perr) < Config.target_position_window)
				Controller.target_reached = true;
		}
	} break;

	default:
		break;
	}

	/* 速度模式用轨迹速度; 位置模式的 vel_des 已在上面按单圈角度 P 给出 */
	if (Controller.ctrl_mode < CONTROL_MODE_POSITION_FILTER)
		vel_des = Controller.vel_setpoint;

	/* 速度限幅 */
	vel_des =
	    CLAMP(vel_des, -Config.velocity_limit, +Config.velocity_limit);

	/* 速度环 PI */
	iq_set = Controller.cur_setpoint;

	float vel_kp = Config.vel_p_gain;
	float vel_ki = Config.vel_i_gain;
	bool low_speed_assist = (foc_get_stage() != ANTICOGGING);

	if (Controller.ctrl_mode >= CONTROL_MODE_VELOCITY_RAMP) {
		/* 低速时静摩擦占主导, 平滑提升增益; 1rps 以上恢复原值 */
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

	/* 电流模式下的速度限幅 */
	if (Controller.ctrl_mode < CONTROL_MODE_VELOCITY_RAMP) {
		float Imax =
		    (+Config.velocity_limit - vel_meas) * Config.vel_p_gain;
		float Imin =
		    (-Config.velocity_limit - vel_meas) * Config.vel_p_gain;
		iq_set = CLAMP(iq_set, Imin, Imax);
	}

	/* 摩擦前馈: 粘住时给突破电流, 转起来后回到动摩擦偏置 */
	if ((Controller.ctrl_mode >= CONTROL_MODE_VELOCITY_RAMP) &&
	    low_speed_assist) {
		float av = fabsf(vel_meas);
		if (av < FRICTION_FF_ENGAGE_VEL)
			friction_boost = true;
		else if (av > FRICTION_FF_RELEASE_VEL)
			friction_boost = false;

		float ff =
		    friction_boost ? FRICTION_FF_BREAK_A : FRICTION_FF_MOVE_A;
		if (vel_des > FRICTION_FF_ENGAGE_VEL)
			iq_set += ff;
		else if (vel_des < -FRICTION_FF_ENGAGE_VEL)
			iq_set -= ff;
	}

	/* 齿槽转矩前馈补偿 (单圈机械角索引; 位置保持法测得保持电流, */
	/* 与齿槽转矩同向, 前馈取正) */
	if ((Controller.ctrl_mode >= CONTROL_MODE_VELOCITY_RAMP) &&
	    Config.anticogging_enable && Config.cogging_map_valid &&
	    (foc_get_stage() != ANTICOGGING)) {
		/* 相邻两点线性插值, 降低 512 点查表的量化毛刺 */
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

	/* 电流限幅 */
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

	/* 速度积分器 (抗饱和 + 限幅) */
	if (Controller.ctrl_mode < CONTROL_MODE_VELOCITY_RAMP)
		Controller.vel_integrator = 0.0f;
	else if (limited)
		Controller.vel_integrator *= 0.99f;
	else
		Controller.vel_integrator += (vel_ki * CTRL_PERIOD) * v_err;
	Controller.vel_integrator =
	    CLAMP(Controller.vel_integrator, -VEL_INT_LIMIT, VEL_INT_LIMIT);
}
