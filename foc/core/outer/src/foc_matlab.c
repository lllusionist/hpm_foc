#include "foc_matlab.h"
#include "FOC_Model.h"
#include "config.h"
#include "encoder.h"
#include "foc.h"
#include "foc_task.h"
#include "tim.h"
#include <math.h>
#include <stdio.h>

/* 生成代码里的 I/F 参数 (见 FOC_Model.c 的 [HPM 补丁]); 由本文件按目标转速设置 */
extern real32_T HPM_if_ramp_rps_s;
extern real32_T HPM_if_hold_rps;
extern uint16_T HPM_if_open_cnt;
extern real32_T HPM_if_iq_align;
extern real32_T HPM_if_iq_open;
extern real32_T HPM_if_iq_hold;
extern real32_T HPM_if_iq_preload;

/* 生成模型 TS = 100us (10kHz); 我们的电流环 20kHz, 每 2 拍跑一次 */
#define MODEL_DECIM 2U
/* 生成代码 SVPWM 输出按 ARR+1 = 8000 计数 */
#define MODEL_PWM_FULL 8000.0f
/* 模型速度单位是机械 rpm */
#define MODEL_RPS_TO_RPM 60.0f
/* 开环强拖终速: 必须与 FOC_Model.c 里 HPM_IF_HOLD_RPS 一致.
 * RunStage 的速度环以 Speed_ref 为目标, 交接时给同一个值才不突跳.
 * 24V 母线后可交接在 6 rps (反电势 ~7V, SMO 精度明显好于 12V/3rps) */
#define MODEL_IF_HOLD_RPS 3.5f
/* 无感可用的目标转速范围: 低于 2 rps 反电势 <2V, SMO 锁不住; 高于 8.5 rps
 * 24V 母线电压不够 (EMF 已 ~10V) */
#define MODEL_MIN_RPS 2.0f
#define MODEL_MAX_RPS 8.5f
/* 模型电流环带宽: 生成代码默认 kp=0.017 V/A, ki=35 V/(A·s) 只有 ~1 Hz,
 * 对位 0.3 s 电流才爬到 0.25 A. 这里按 motor.L/Rs 重算 (Ts=100us, 留裕量) */
#define MODEL_CUR_BW_HZ 150.0f

/* 模型自己没有失步保护: 电压长期打满或电流超限就主动停机 */
#define MODEL_TICKS_PER_MS 10U		 /* 模型 10 kHz */
#define MODEL_TRIP_MS 50U		 /* 去抖时间 */
#define MODEL_VMAG_TRIP 12.4f		 /* 生成代码饱和限幅 ±12.47 V */
#define MODEL_IPK_TRIP 1.5f		 /* 相电流峰值上限 (A) */
/* 观测器 vs 编码器转速差超过该值并持续 MODEL_SYNC_TRIP_MS 就算失步 */
#define MODEL_SYNC_TRIP_RPS 0.8f
#define MODEL_SYNC_TRIP_MS 100U

/* 速度环增益 (单位: A/rpm, A/(rpm·s)):
 * 生成值 kp=0.0034 / ki=1.44e-4 在纯 P 下有 ~13.5 Hz 极限环;
 * 2026-11-05 试过直接套用本工程 0.17/5.3 A/rps (÷60), ki 放大 600 倍,
 * 结果交接瞬间积分饱和 -> Vq 打满 ±12.47 V -> 触发保护停机. 已回退.
 * 实测 3.5 rps 稳态静差 0.9% (3.47 vs 3.5), Iq 需 0.033 A, 由 P 项维持;
 * ki 从 0.002 提到 0.006: 积分约 0.15 s 补上摩擦电流, 消掉静差,
 * 同时 kp 仍保持低值 (机械在 ~7 Hz 有一阶共振, 环路带宽要压在 2 Hz 内) */
#define MODEL_SPD_KP 0.0015f
#define MODEL_SPD_KI 0.0060f
/* 由 J 推导速度环增益时使用的设计带宽 (Hz): 要低于机械共振 (~8~10 Hz) */
#define MODEL_SPD_BW_HZ 6.0f

static uint8_t model_div;
static uint16_t model_trip_cnt;
static uint16_t model_sync_cnt;
/* 摩擦电流自学习 (闭环稳速时的 |Iq| 慢平均), <0 表示还没初值 */
static float fric_learn = -1.0f;
static bool enc_seen_moving; /* 编码器至少真实转过一次才启用失步校验 */
/* 上一拍 20 kHz 电流采样, 与当前拍平均后喂给 10 kHz 模型 */
static float model_ia_prev, model_ib_prev, model_ic_prev;
/* 上位机请求的模型转速 (rps): 启动前下发也能保存, 模型初始化时用它当目标 */
static float model_speed_req = MODEL_IF_HOLD_RPS;

void FOC_MATLAB_init(void)
{
	/* 用标定值覆盖生成代码里的电机参数; 未标定则保留生成时的默认值
	 * (生成代码的默认值已按 4310 改成 L=3mH / Pn=14 / Rs=6.8Ω) */
	if (Config.calib_valid && (Config.pole_pairs > 0)) {
		motor.Rs = Config.phase_resistance;
		motor.L = Config.phase_inductance;
		motor.Pn = (real32_T)Config.pole_pairs;
	}

	/* 电流环增益: 生成代码的 0.017/35 是按别的电机配的, 只有 ~1 Hz 带宽,
	 * 对位 0.3 s 电流只爬到 0.25 A. 按我们的 L/R 重算 (Ts=100us, 150 Hz) */
	float wc = 6.2831853f * MODEL_CUR_BW_HZ;
	curr_kpki.curr_d_kp = motor.L * wc;
	curr_kpki.curr_d_ki = motor.Rs * wc;

	/* 速度环增益: 见 MODEL_SPD_KP/KI 注释. 之前"跟随上位机速度环 Kp/Ki"
	 * 的实现已回退 —— 上位机默认 5.3 换算后是 0.088, 比生成值大 600 倍,
	 * 交接时积分瞬间打满, 反而把电机送进保护 */
	float lam = (Config.calib_valid && (Config.flux_linkage > 1e-4f))
			? Config.flux_linkage
			: 0.0133f;
	float kt = 1.5f * (float)motor.Pn * lam;

	spd_kpki.spd_kp = MODEL_SPD_KP;
	spd_kpki.spd_ki = MODEL_SPD_KI;
	/* 有实测 J 时按"二阶系统 ζ=1、带宽 6 Hz"推导速度环增益
	 * (6 Hz 是为了避开这台电机 ~8~10 Hz 的机械共振):
	 *   Kp = 4πζωbw·J/Kt   [A/(rad/s)]
	 *   Ki = 2πJ·ωbw²/Kt    [A/(rad/s)/s]
	 * 模型的速度误差单位是 rpm, 所以再除以 2π×60 */
	if ((Config.inertia > 1e-7f) && (Config.inertia < 1e-1f) && (kt > 0.01f)) {
		float wbw = 6.2831853f * MODEL_SPD_BW_HZ;
		float kp = 4.0f * 3.14159265f * wbw * Config.inertia / kt;
		float ki = 6.2831853f * Config.inertia * wbw * wbw / kt;

		spd_kpki.spd_kp = kp / (6.2831853f * 60.0f);
		spd_kpki.spd_ki = ki / (6.2831853f * 60.0f);
	}

	FOC_Model_initialize();
	rtU.MotorOnOff = 0.0f;

	/* ---- 由标定出的电机参数自动推导 I/F 与转速区间 ----
	 * 换电机时只需要重新标定 (R/L/λ/PP/J 全自动测), 这里按公式得到参数:
	 *   反电势系数 ke = 2π·Pn·λ (V/rps)
	 *   最高转速: 线性区相电压 0.9·Vbus/√3 的 75% 用来发反电势
	 *   最低转速: 反电势不低于 2 V (SMO 才有可用信号)
	 *   I/F 对位/开环电流: 额定电流(current_limit)的 13% / 23%
	 *   保持/预置电流: 实测摩擦电流 (闭环稳态自学习, 未知时 0.04 A) */
	float ke = 6.2831853f * (float)motor.Pn * motor.L; /* 占位, 下面重算 */
	float ke_emf = 6.2831853f * (float)motor.Pn *
		       ((Config.calib_valid && (Config.flux_linkage > 1e-4f))
			? Config.flux_linkage
			: 0.0133f);
	float v_avail = 0.9f * FOC.Vbus_filt / 1.7320508f;
	float n_max = 0.75f * v_avail / ke_emf;
	float n_min = 2.0f / ke_emf;
	float fric = Config.friction_current;

	(void)ke;
	if (n_max > 8.5f)
		n_max = 8.5f;
	if (n_min < 2.0f)
		n_min = 2.0f;
	if (fric < 0.02f)
		fric = 0.04f; /* 还没学到, 用经验值 */

	/* I/F 开环终速跟随目标转速: 这样交接瞬间速度环误差≈0.
	 * 目标低于开环终速时速度环要从 3.5 刹到目标, 低转速 SMO 无 EMF
	 * 会直接失锁 (实测目标 0.8 rps 时反向跑到 -10 rps). */
	float tgt = fabsf(model_speed_req);

	if (tgt < n_min)
		tgt = n_min;
	if (tgt > n_max)
		tgt = n_max;
	HPM_if_hold_rps = tgt;
	HPM_if_ramp_rps_s = (tgt > 3.0f) ? 3.0f : 2.0f;
	HPM_if_open_cnt = (uint16_T)(tgt / HPM_if_ramp_rps_s * 10000.0f + 0.5f);
	HPM_if_iq_align = 0.13f * Config.current_limit;
	HPM_if_iq_open = 0.23f * Config.current_limit;
	HPM_if_iq_hold = fric;
	HPM_if_iq_preload = fric;
	/* 交接后速度环的目标 = 开环终速 = 实际目标 (钳位后) */
	rtU.Speed_ref = tgt * MODEL_RPS_TO_RPM;
	model_div = 0;
	model_trip_cnt = 0;
	model_sync_cnt = 0;

	printf("matlab model: Rs=%.2f L=%.2fmH Pn=%d lam=%.4f ke=%.3fV/rps\n",
	       (double)motor.Rs, (double)(motor.L * 1000.0f), (int)motor.Pn,
	       (double)(ke_emf / (6.2831853f * (float)motor.Pn)),
	       (double)ke_emf);
	printf("  deriv: n=%.2f~%.2f rps (目标 %.2f) ramp=%.1f cnt=%u\n",
	       (double)n_min, (double)n_max, (double)tgt,
	       (double)HPM_if_ramp_rps_s, (unsigned)HPM_if_open_cnt);
	printf("  I/F: align=%.2fA open=%.2fA hold=%.3fA | curkp=%.2f "
	       "curki=%.0f spdkp=%.5f spdki=%.5f\n",
	       (double)HPM_if_iq_align, (double)HPM_if_iq_open,
	       (double)HPM_if_iq_hold, (double)curr_kpki.curr_d_kp,
	       (double)curr_kpki.curr_d_ki, (double)spd_kpki.spd_kp,
	       (double)spd_kpki.spd_ki);
}

void FOC_MATLAB_tick(void)
{
	if (++model_div < MODEL_DECIM)
		return;
	model_div = 0;

	/* 模型跑 10 kHz, ADC 是 20 kHz: 原来只取最新一拍, 丢掉另一拍.
	 * 静止时电流恒定, 采样点固定 -> 干净; 转动时占空比每拍都在变,
	 * PWM 三角纹波(24V/3mH/20kHz 约 0.1App)在采样点的位置跟着变,
	 * 电流反馈就出现 0.1~0.2 A 的"毛刺", 电流环跟着动作 -> 电压/电流抖动.
	 * 这里把两拍 20 kHz 采样做平均 (零额外延时, 纹波近似抵消一半) */
	{
		float ia_now = (FOC.Ia - FOC.Ia_midpoint) * FOC_CURRENT_SCALE;
		float ib_now = (FOC.Ib - FOC.Ib_midpoint) * FOC_CURRENT_SCALE;
		float ic_now = (FOC.Ic - FOC.Ic_midpoint) * FOC_CURRENT_SCALE;

		rtU.ia = 0.5f * (ia_now + model_ia_prev);
		rtU.ib = 0.5f * (ib_now + model_ib_prev);
		rtU.ic = 0.5f * (ic_now + model_ic_prev);
		model_ia_prev = ia_now;
		model_ib_prev = ib_now;
		model_ic_prev = ic_now;
	}
	rtU.VDC = FOC.Vbus_filt;

	FOC_Model_step();

	/* 观测器角度下的 dq 量 -> 占位 VOFA 通道 (只影响显示, 不参与控制)
	 * 角度正确时: Vd 应≈ -ωL·iq (很小), Vq ≈ ωλ + R·iq;
	 * 角度有偏差 Δ 时, 反电势会漏到 d 轴: Vd 多出 -ωλ·sinΔ, 可据此判断观测角 */
	{
		float c = cosf(SMOTheta);
		float s = sinf(SMOTheta);
		float id = ialpha * c + ibeta * s;
		float iq = ibeta * c - ialpha * s;

		FOC.Id = id;
		FOC.Iq = iq;
		UTILS_LP_FAST(FOC.Id_filt, id, 0.01f);
		UTILS_LP_FAST(FOC.Iq_filt, iq, 0.01f);
		FOC.Vd_out = ualpha * c + ubeta * s;
		FOC.Vq_out = ubeta * c - ualpha * s;
		FOC.Vmag = sqrtf(ualpha * ualpha + ubeta * ubeta);
		if (Config.calib_valid) {
			FOC.torque = 1.5f * (float)Config.pole_pairs *
				     Config.flux_linkage * iq;
		}
	}

	/* 摩擦电流自学习: RunStage 稳速时把 |Iq| 慢速平均, 写进
	 * Config.friction_current —— 换电机后不用再手填 I/F 保持电流 */
	{
		float w_err = fabsf(encoder.vel - (SMOWm / MODEL_RPS_TO_RPM));

		if ((FOC_MATLAB_state() >= 4.5f) && (w_err < 0.15f) &&
		    (fabsf(encoder.vel) > 1.0f)) {
			float iq_abs = fabsf(FOC.Iq);

			if (fric_learn < 0.0f)
				fric_learn = iq_abs;
			else
				fric_learn += 0.0002f * (iq_abs - fric_learn);
			if ((fric_learn > 0.01f) && (fric_learn < 1.0f))
				Config.friction_current = fric_learn;
		}
	}

	/* 模型失步/失控保护: 电压打满 (模型内部 ±12.47 V 饱和) 或相电流超限,
	 * 持续 50 ms 就关模型并回 IDLE —— 生成代码只按时间表切状态, 不判断失步 */
	{
		float ipk = fmaxf(fabsf(rtU.ia),
				  fmaxf(fabsf(rtU.ib), fabsf(rtU.ic)));

		if ((FOC.Vmag > MODEL_VMAG_TRIP) || (ipk > MODEL_IPK_TRIP)) {
			if (++model_trip_cnt >=
			    (MODEL_TRIP_MS * MODEL_TICKS_PER_MS)) {
				model_trip_cnt = 0;
				FOC.fault_code = FOC_FAULT_PHASE_OC;
				FOC_MATLAB_set_enable(false);
				foc_request_stage(IDLE);
			}
		} else {
			model_trip_cnt = 0;
		}
	}

	/* 失步交叉校验 (编码器已接入): 观测器与编码器转速差太大说明已经失步.
	 * 注意两个门槛:
	 *   1) 只看 RunStage(状态 5): Align/Open 段角度是强制给的, 而且 SMO
	 *      在 0 转速附近速度估计会漂(实测 -2 rps), 早开一定误跳;
	 *   2) 观测器和编码器至少要有一边真在转, 否则静止时噪声也满足条件.
	 * 之前只判 state>=3 且差 >0.8 rps, 结果对位/开环刚开始就误跳, 电机起不来. */
	{
		float w_model = SMOWm / MODEL_RPS_TO_RPM;
		bool moving = (fabsf(w_model) > 0.5f) ||
			      (fabsf(encoder.vel) > 0.5f);

		if (!enc_seen_moving && (fabsf(encoder.vel) > 1.0f))
			enc_seen_moving = true;

		if (enc_seen_moving && (FOC_MATLAB_state() >= 4.5f) && moving &&
		    (fabsf(encoder.vel - w_model) > MODEL_SYNC_TRIP_RPS)) {
			if (++model_sync_cnt >=
			    (MODEL_SYNC_TRIP_MS * MODEL_TICKS_PER_MS)) {
				model_sync_cnt = 0;
				FOC.fault_code = FOC_FAULT_PHASE_OC;
				FOC_MATLAB_set_enable(false);
				foc_request_stage(IDLE);
			}
		} else {
			model_sync_cnt = 0;
		}
	}

	pwm_set_duty(rtY.Tcmp1 / MODEL_PWM_FULL,
		     rtY.Tcmp2 / MODEL_PWM_FULL,
		     rtY.Tcmp3 / MODEL_PWM_FULL);
}

void FOC_MATLAB_set_enable(bool on)
{
	rtU.MotorOnOff = on ? 1.0f : 0.0f;
}

void FOC_MATLAB_set_speed_rps(float rps)
{
	/* 启动前也能存: FOC_MATLAB_init 会用它当 Speed_ref, 避免被默认值覆盖.
	 * 无感只在 [2.0, 8.5] rps 可用 (低于 2 rps 反电势 <2V SMO 锁不住,
	 * 高于 8.5 rps 24V 母线电压不够), 这里统一钳位, 负向暂不支持 */
	rps = fabsf(rps);
	if (rps < MODEL_MIN_RPS)
		rps = MODEL_MIN_RPS;
	if (rps > MODEL_MAX_RPS)
		rps = MODEL_MAX_RPS;
	model_speed_req = rps;
	rtU.Speed_ref = rps * MODEL_RPS_TO_RPM;
}

float FOC_MATLAB_speed_rpm(void)
{
	return SMOWm;
}

/* 模型速度环的目标值 (rpm): 与观测转速一起看, 一旦不一致就是调速环没生效 */
float FOC_MATLAB_speed_ref_rpm(void)
{
	return rtU.Speed_ref;
}

float FOC_MATLAB_theta(void)
{
	return SMOTheta;
}

/* 观测角与编码器角之差, 卷绕到 ±π; 用来在波形上直接看观测器误差,
 * 避免 2π 锯齿造成的"切换瞬间抖动"假象 */
float FOC_MATLAB_theta_error(void)
{
	float e = SMOTheta - encoder.phase;

	e -= 6.28318548f * floorf(e / 6.28318548f);
	if (e > 3.14159265f)
		e -= 6.28318548f;
	return e;
}

float FOC_MATLAB_state(void)
{
	return state;
}
