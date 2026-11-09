#include <string.h>
#include "foc_cal.h"
#include "encoder.h"
#include "foc_task.h"
#include "foc_ctrl.h"

#define CAL_LOOP_HZ (1000U)
/* 1kHz 标定循环 -> 每机械圈 2240 点 */
#define SAMPLES_PER_PPAIR (160U)
#define LUT_WINDOW (128U)	   /* offset LUT 滑动平均窗口 */
#define MAX_MOTOR_POLE_PAIRS (20U) /* 最大支持的极对数 */

/* 磁链: 开环拖动, 两转速点矢量差分消掉 R 项 */
#define FLUX_SPEED1 (2.0f * M_PI * 20.0f) /* 低速点 (电角速度 rad/s) */
/* 高速点: 30Hz 时 0.7A 会电压饱和 */
#define FLUX_SPEED2 (2.0f * M_PI * 80.0f) /* 高速点: Δω=377, λ 信号 x3.3 */
#define FLUX_SETTLE_TIME (1.5f) /* 每个转速稳定时间 (s) */
/* 每点采样点数 (1.5s, 低速点也够转一圈) */
#define FLUX_MEAS_SAMPLES (1500U)
/* 拖动电流系数 (云台 0.7A 以上易饱和) */
#define FLUX_IQ_GAIN (0.8f)
#define FLUX_RTEST_RATIO (0.5f) /* 同转速校验的降流比例 */

/* 电阻: 两点伏安法(相减抵消死区); 电感: 1kHz 正弦注入, Z²=R²+(ωL)² */
#define L_CAL_CYCLES (CAL_LOOP_HZ / 2U) /* 电感: 每轮正弦注入时长 0.5s */
/* 步长按量程 10% 派生, 不写死绝对值 */
#define CALIB_R_STEP_V (Config.calib_voltage * 0.10f)
#define R_CAL_WINDOW (CAL_LOOP_HZ / 5U)		     /* 每个工作点停留 200ms */
#define CALIB_L_MIN_V (Config.calib_voltage * 0.20f) /* 电感注入幅值下限 (V) */

static int encoder_error_arr[MAX_MOTOR_POLE_PAIRS * SAMPLES_PER_PPAIR];

static tCalibStep mCalibStep = CS_NULL;
static uint32_t loop_count;	 /* 1kHz 时间基准 */
static float phase_set;		 /* 电角度设定值 */
static int16_t sample_count;	 /* 采样点下标 */
static float next_sample_time;	 /* 下一个采样时刻 (秒) */
static int64_t start_count_long; /* 极对数标定起点计数 */
static bool dir_pp_runin;	 /* 极对数标定: 预跑阶段是否已完成 */

static float flux_v_sum, flux_i_sum; /* 电压/电流模长累加 */
static float flux_v1, flux_i1, flux_v2, flux_i2;
static float flux_vel1, flux_vel2; /* 两个转速点的实测机械转速 (rps) */
static float flux_vel_sum;	   /* 转速窗口累加 (取平均, 避免撞上噪声尖峰) */
static float flux_vq_sum;	   /* Vq 窗口累加 (同转速变电流测 R 用) */
static float flux_vq_a;		   /* 高电流点的 Vq 均值 */
static float flux_i_a;		   /* 高电流点的电流均值 */
static float flux_r_est;	   /* 由 ΔVq 差算出的 R (反电势项抵消) */
static float flux_vq_b;		   /* 第二转速点的 Vq 均值 */
static float flux_vd_sum;	   /* Vd 窗口累加 (矢量差分法用) */
static float flux_vd_a, flux_vd_b; /* 两个转速点的 Vd 均值 */

/* J 标定: 加速->自由滑行, 摩擦相减: J = Kt·I/(π·(a+ - a-)) */
#define J_CAL_IQ (Config.calib_current * 0.15f) /* 加速测试电流 (A) */
#define J_CAL_SAMPLES (300U)		       /* 每段采样点数 (1kHz -> 0.3s) */
#define J_SPINUP_RPS (2.0f) /* 起转目标 (rps): 太高会让 ACC 段冲到过流 */
#define J_SPINUP_MAX_RPS 12.0f /* 加速段转速上限: 超过就放弃, 防过流 */
static float j_sum_t, j_sum_v, j_sum_tv, j_sum_tt;
static uint32_t j_cnt;
static float j_slope_acc, j_slope_dec;
static float j_vel_limit_save; /* 标定期间需放开速度限幅 */

/* 最小二乘拟合 vel(t) 斜率 (rps/s) */
static float j_slope(void)
{
	float n = (float)j_cnt;
	float den = n * j_sum_tt - j_sum_t * j_sum_t;
	if (fabsf(den) < 1e-6f)
		return 0.0f;
	return (n * j_sum_tv - j_sum_t * j_sum_v) / den * (float)CAL_LOOP_HZ;
}

static uint32_t flux_cnt;
static bool flux_sat, flux_sat1, flux_sat2;

/* 电阻: 自适应两点伏安法 */
static float r_volt;	/* 当前注入电压 ures (V) */
static float r_i1;	/* 上一个工作点的稳态电流 Ires1 (A) */
static float r_isum;	/* 窗口内电流累加 */
static uint32_t r_icnt; /* 窗口内累加计数 */
static uint32_t r_s1;	/* 升压次数 */
/* 多点平均: 每级 ΔI 累加, 抗单点噪声 */
static float r_di_sum;
static uint32_t r_di_cnt;

/* 电感: 幅值自适应 */
static float l_v_dc, l_v_amp; /* 注入的直流偏置 / 正弦幅值 */
static uint8_t l_pass;	      /* 0 = 探测轮, 1 = 测量轮 */

void calibration_start(void)
{
	if (!FOC.adc_ready) {
		FOC.fault_code = FOC_FAULT_ADC;
		foc_request_stage(IDLE);
		mCalibStep = CS_NULL;
		return;
	}

	Config.encoder_dir = +1;
	Config.calib_valid = false;
	FOC.calib_mode = false;
	FOC.calib_vinj_mode = false;

	loop_count = 0;
	phase_set = 0;
	sample_count = 0;
	next_sample_time = 0;
	memset(encoder_error_arr, 0, sizeof(encoder_error_arr));

	/* 跨标定残留会混入 Vq1(实测 0.2985→0.6011), 每次必须清零 */
	flux_v_sum = flux_i_sum = flux_vq_sum = flux_vd_sum = 0.0f;
	flux_vel_sum = 0.0f;
	flux_cnt = 0;

	/* 回显实际生效参数, 便于确认配置已下发 */
	printf("calib: cur=%.2fA vol=%.2fV step=%.3fV R0=%.4fOhm "
	       "drag_min=%.2fV flux_iq=%.2fA j_iq=%.2fA\n",
	       Config.calib_current, Config.calib_voltage, CALIB_R_STEP_V,
	       Config.phase_resistance, (double)Config.calib_drag_v_min,
	       (double)(Config.calib_current * Config.calib_flux_iq_gain),
	       (double)(Config.calib_current * Config.calib_j_iq_gain));

	mCalibStep = CS_MOTOR_R_START;
}

void calibration_end(void)
{
	FOC.calib_mode = false;
	FOC.calib_vinj_mode = false;
	FOC.calib_adv_angle = false;
	foc_disarm();

	/* 没走到最后的状态 -> 本次标定作废(calib_valid=0), 打印停在哪一步 */
	if (!Config.calib_valid) {
		printf("calib: ABORTED at step %d (calib_valid=0, "
		       "\u5148\u8dd1\u5b8c 0x27 \u624d\u80fd\u7528\u9f7f\u69fd/\u95ed\u73af)\n",
		       (int)mCalibStep);
	}

	mCalibStep = CS_NULL;

	/* 编码器零位重算后旧齿槽表相位失效, 必须作废 */
	if (Config.cogging_map_valid) {
		Config.cogging_map_valid = 0;
		printf("cogging: map invalidated by calibration, redo 0x28\n");
	}
}

int calibration_step(void)
{
	return (int)mCalibStep;
}

void calibration_loop(void)
{
	float time = (float)loop_count / (float)CAL_LOOP_HZ;

	const float ia_meas = (FOC.Ia - FOC.Ia_midpoint) * FOC_CURRENT_SCALE;

	/* 拖动电压: 高阻电机按 R·I 算, 低阻电机(0.03Ω)必须用电压下限兜底 */
	float voltage_calc = Config.phase_resistance * Config.calib_current;
	float drag_v = voltage_calc;
	if (drag_v < Config.calib_drag_v_min)
		drag_v = Config.calib_drag_v_min;
	if (drag_v > (Config.calib_voltage * 0.9f))
		drag_v = Config.calib_voltage * 0.9f;

	float calib_phase_vel = 12.0f * M_PI;
	float total_calib_cycles = 14.0f;

	/* 开环拖动阶段每 300ms 打一次: 实际电压/电流/转速 —— 判断"拖不动"到底是
	 * 电流不够还是相序/方向/极对数不对 */
	if ((mCalibStep == CS_DIR_PP_LOOP) ||
	    (mCalibStep == CS_ENCODER_CW_LOOP) ||
	    (mCalibStep == CS_ENCODER_CCW_LOOP)) {
		static uint32_t drag_dbg_cnt;

		if (++drag_dbg_cnt >= 300U) {
			drag_dbg_cnt = 0;
			printf("drag: v=%.3fV I=%.2fA vel=%.3f rps "
			       "elec=%.2f rad\n",
			       (double)drag_v, (double)ia_meas,
			       (double)encoder.vel, (double)phase_set);
		}
	}

	switch (mCalibStep) {
	case CS_MOTOR_R_START:
		loop_count = 0;
		r_volt = CALIB_R_STEP_V; /* ures = Δu2 */
		r_i1 = 0.0f;		 /* Ires1 */
		r_isum = 0.0f;
		r_icnt = 0;
		r_s1 = 0; /* 升压次数 */
		r_di_sum = 0.0f;
		r_di_cnt = 0U;
		FOC.calib_mode = false;
		FOC.calib_vinj_mode = false;
		foc_arm();
		mCalibStep = CS_MOTOR_R_LOOP;
		break;

	case CS_MOTOR_R_LOOP: {
		FOC_voltage(r_volt, 0, 0); /* d 轴注入恒定电压 (开环) */

		/* 200ms 窗口平均 = 该工作点的稳态电流 */
		r_isum += ia_meas;
		r_icnt++;

		if (loop_count >= R_CAL_WINDOW) {
			float Is = fabsf(r_isum / (float)r_icnt);
			r_isum = 0.0f;
			r_icnt = 0;
			loop_count = 0;

			if ((Is < Config.calib_current) &&
			    ((r_volt + CALIB_R_STEP_V) <=
			     (Config.calib_voltage * 0.9f))) {
				/* 未达阈值: 累加本级 ΔI, 继续升压 */
				/* 1A 以下死区误差占电压几十 %, 差分 ΔI 偏小 -> 跳过 */
				if ((r_s1 > 0U) && (Is > 1.0f) && (r_i1 > 1.0f)) {
					r_di_sum += (Is - r_i1);
					r_di_cnt++;
				}
				r_i1 = Is;
				r_volt += CALIB_R_STEP_V;
				r_s1++;
			} else {
				/* Rs = Δu / 平均(ΔI) */
				if ((r_s1 > 0U) && (Is > 1.0f) && (r_i1 > 1.0f)) {
					r_di_sum += (Is - r_i1);
					r_di_cnt++;
				}
				float di_avg = (r_di_cnt > 0U)
						   ? (r_di_sum / (float)r_di_cnt)
						   : (Is - r_i1);
				if (fabsf(di_avg) > 1e-4f) {
					Config.phase_resistance =
					    CALIB_R_STEP_V / di_avg;
				}
				printf("res: %.4f Ohm (ures=%.3f I1=%.4f "
				       "I2=%.4f steps=%u di_avg=%.5f)\n",
				       Config.phase_resistance, r_volt, r_i1,
				       Is, (unsigned)r_s1, (double)di_avg);
				mCalibStep = CS_MOTOR_R_END;
			}
		}
	} break;

	case CS_MOTOR_R_END: {
		if (Config.phase_resistance < 0.001f)
			Config.phase_resistance = 0.001f;
		loop_count = 0;
		mCalibStep = CS_MOTOR_L_START;
	} break;

	case CS_MOTOR_L_START: {
		l_v_dc = Config.calib_current * Config.phase_resistance;
		l_pass = 0; /* 第 0 轮 = 探测轮 */

		/* 自适应幅值: u_amp = I_target×R, 保证不过流 */
		l_v_amp = Config.calib_current * Config.phase_resistance;
		if (l_v_amp < CALIB_L_MIN_V)
			l_v_amp = CALIB_L_MIN_V;

		if ((l_v_dc + l_v_amp) > Config.calib_voltage * 0.9f) {
			l_v_amp = (Config.calib_voltage * 0.9f) - l_v_dc;
			if (l_v_amp < CALIB_L_MIN_V)
				l_v_amp = CALIB_L_MIN_V;
		}

		/* 1kHz 正弦注入(更高频在 1kHz 循环里会退化成直流) */
		FOC.vinj_omega = 2.0f * M_PI * Config.current_ctrl_bw;
		FOC.vinj_amp = l_v_amp;
		FOC.vinj_dc = l_v_dc;
		FOC.inj_phase = 0;
		FOC.vinj_sum_sin = 0;
		FOC.vinj_sum_cos = 0;
		FOC.vinj_samples = 0;
		FOC.vinj_acc = false;

		loop_count = 0;
		FOC.calib_vinj_mode = true;
		foc_arm();
		mCalibStep = CS_MOTOR_L_LOOP;
	} break;

	case CS_MOTOR_L_LOOP:
		/* 丢弃起始瞬态后解调 */
		if (!FOC.vinj_acc && (loop_count >= 100U))
			FOC.vinj_acc = true;
		if (loop_count >= L_CAL_CYCLES) {
			FOC.vinj_acc = false;
			/* 注入保持, 由 L_END 决定是否复测 */
			mCalibStep = CS_MOTOR_L_END;
		}
		break;

	case CS_MOTOR_L_END: {
		float omega = 2.0f * M_PI * Config.current_ctrl_bw;
		int valid_samples = (int)FOC.vinj_samples;
		float I_amp =
		    (valid_samples > 0)
			? (2.0f / (float)valid_samples) *
			      sqrtf(SQ(FOC.vinj_sum_sin) + SQ(FOC.vinj_sum_cos))
			: 0.0f;
		float Z = (I_amp > 0.005f) ? (l_v_amp / I_amp) : 9999.0f;

		/* 探测轮: 按 |Z| 修正幅值后再测一轮 */
		if (l_pass == 0) {
			float new_amp = Config.calib_current * Z;
			if (new_amp < CALIB_L_MIN_V)
				new_amp = CALIB_L_MIN_V;
			if ((l_v_dc + new_amp) > Config.calib_voltage * 0.9f) {
				new_amp =
				    (Config.calib_voltage * 0.9f) - l_v_dc;
				if (new_amp < CALIB_L_MIN_V)
					new_amp = CALIB_L_MIN_V;
			}

			printf(
			    "ind-probe: Iamp=%.3f Z=%.3f Vamp %.3f -> %.3f\n",
			    I_amp, Z, l_v_amp, new_amp);

			l_v_amp = new_amp;
			FOC.vinj_amp = new_amp;
			FOC.inj_phase = 0;
			FOC.vinj_sum_sin = 0;
			FOC.vinj_sum_cos = 0;
			FOC.vinj_samples = 0;
			FOC.vinj_acc = false;
			l_pass = 1;
			loop_count = 0;
			mCalibStep = CS_MOTOR_L_LOOP;
			break;
		}

		/* ---- 测量轮: 计算电感 ---- */
		float sys_R = Config.phase_resistance;

		float X_L = 0.0f;
		if (Z > sys_R)
			X_L = sqrtf(Z * Z - sys_R * sys_R);
		else
			X_L = 0.001f; /* 测不出感抗时给极小值 */

		float L_eq = X_L / omega;
		Config.phase_inductance = L_eq * 1.0f;

		FOC.calib_vinj_mode = false;
		pwm_output_low_sides();
		FOC_update_current_ctrl_gain(Config.current_ctrl_bw);

		printf("ind: %.6f H (Iamp=%.3f Z=%.3f Vdc=%.3f Vamp=%.3f)\n",
		       Config.phase_inductance, I_amp, Z, l_v_dc, l_v_amp);

		phase_set = 0;
		loop_count = 0;
		mCalibStep = CS_DIR_PP_START;
	} break;

	case CS_DIR_PP_START:
		if (loop_count == 0)
			foc_arm();
		FOC_voltage((drag_v * time / 2.0f), 0, 0.0f);

		if (time >= 2.0f) {
			encoder.shadow_count = 0;
			start_count_long = 0;
			dir_pp_runin = false;
			loop_count = 0;
			phase_set = 0;
			mCalibStep = CS_DIR_PP_LOOP;
		}
		break;
	case CS_DIR_PP_LOOP:
		phase_set += calib_phase_vel / (float)CAL_LOOP_HZ;
		FOC_voltage(drag_v, 0, phase_set);

		if (!dir_pp_runin) {
			/* 预跑一圈, 让转子牵入同步, 避免圈数偏少 */
			if (phase_set >= total_calib_cycles * M_2PI) {
				encoder.shadow_count = 0;
				start_count_long = 0;
				dir_pp_runin = true;
			}
		} else if (phase_set >= 2.0f * total_calib_cycles * M_2PI) {
			pwm_output_low_sides();
			mCalibStep = CS_DIR_PP_END;
		}
		break;
	case CS_DIR_PP_END: {
		int64_t diff = encoder.shadow_count - start_count_long;

		if (abs((int)diff) < 500) {
			Config.calib_valid = false;
		} else {
			Config.encoder_dir = (diff > 0) ? 1 : -1;
			float mech_revs = fabsf((float)diff / ENCODER_CPR_F);
			float pp_raw = total_calib_cycles / mech_revs;
			Config.pole_pairs = (int)roundf(pp_raw);
			/* pp_raw 应非常接近整数; 若像 14.93 说明仍有瞬态残差 */
			printf("dir_pp: diff=%lld mech_revs=%.5f pp_raw=%.4f "
			       "-> %d\n",
			       (long long)diff, mech_revs, pp_raw,
			       (int)Config.pole_pairs);
		}

		/* 极对数无效则直接结束, 避免 encoder_error_arr 越界 */
		if ((Config.pole_pairs <= 0) ||
		    (Config.pole_pairs > (int)MAX_MOTOR_POLE_PAIRS)) {
			Config.calib_valid = false;
			loop_count = 0;
			mCalibStep = CS_NULL;
			foc_request_stage(IDLE);
			break;
		}

		mCalibStep = CS_ENCODER_START;
	} break;

	case CS_FLUX_START:
		Config.flux_linkage = 0;
		/* 角度改由 20kHz 中断推进, 消除 1kHz 阶梯误差 */
		FOC.calib_adv_angle = true;
		FOC.calib_id = 0;
		FOC.calib_iq =
		    Config.calib_current * Config.calib_flux_iq_gain;
		FOC.calib_phase_vel = 0;
		FOC.calib_angle = 0;
		FOC.calib_mode = true;
		loop_count = 0;
		mCalibStep = CS_FLUX_RAMP1;
		break;

	case CS_FLUX_RAMP1:
		/* 缓缓升到 FLUX_SPEED1, 让转子平稳同步 */
		FOC.calib_phase_vel +=
		    FLUX_SPEED1 / (FLUX_SETTLE_TIME * (float)CAL_LOOP_HZ);
		if (FOC.calib_phase_vel > FLUX_SPEED1)
			FOC.calib_phase_vel = FLUX_SPEED1;
		if (FOC.calib_angle >= M_2PI)
			FOC.calib_angle -= M_2PI;
		if (time > FLUX_SETTLE_TIME) {
			flux_v_sum = 0;
			flux_i_sum = 0;
			flux_vel_sum = 0.0f;
			flux_cnt = 0;
			flux_sat = false;
			loop_count = 0;
			mCalibStep = CS_FLUX_MEAS1;
		}
		break;

	case CS_FLUX_MEAS1:
	case CS_FLUX_MEAS2: {
		if (FOC.calib_angle >= M_2PI)
			FOC.calib_angle -= M_2PI;

		flux_v_sum += sqrtf(SQ(FOC.Vd_out) + SQ(FOC.Vq_out));
		flux_i_sum += sqrtf(SQ(FOC.Id) + SQ(FOC.Iq));
		flux_vq_sum += FOC.Vq_out;
		flux_vd_sum += FOC.Vd_out;
		flux_vel_sum += encoder.vel;
		if (FOC.saturated)
			flux_sat = true;
		flux_cnt++;

		if (flux_cnt >= FLUX_MEAS_SAMPLES) {
			float v_avg = flux_v_sum / (float)flux_cnt;
			float i_avg = flux_i_sum / (float)flux_cnt;

			if (mCalibStep == CS_FLUX_MEAS1) {
				flux_v1 = v_avg;
				flux_i1 = i_avg;
				flux_sat1 = flux_sat;
				flux_vel1 = flux_vel_sum / (float)flux_cnt;
				flux_vq_a = flux_vq_sum / (float)flux_cnt;
				flux_vd_a = flux_vd_sum / (float)flux_cnt;
				flux_i_a = i_avg;
				loop_count = 0;
				flux_vel_sum = 0.0f;
				flux_vq_sum = 0.0f;
				flux_vd_sum = 0.0f;
				flux_v_sum = 0.0f;
				flux_i_sum = 0.0f;
				flux_cnt = 0;
				FOC.calib_iq = Config.calib_current *
					       Config.calib_flux_iq_gain *
					       Config.calib_flux_rtest_ratio;
				mCalibStep = CS_FLUX_RTEST;
			} else {
				flux_v2 = v_avg;
				flux_i2 = i_avg;
				flux_sat2 = flux_sat;
				flux_vel2 = flux_vel_sum / (float)flux_cnt;
				flux_vq_b = flux_vq_sum / (float)flux_cnt;
				flux_vd_b = flux_vd_sum / (float)flux_cnt;
				mCalibStep = CS_FLUX_END;
			}
		}
	} break;

	case CS_FLUX_RTEST: {
		/* 同转速变电流: ΔVq = R·ΔIq, 反电势项抵消, 用于校验 R */
		if (FOC.calib_angle >= M_2PI)
			FOC.calib_angle -= M_2PI;

		flux_vq_sum += FOC.Vq_out;
		flux_i_sum += sqrtf(SQ(FOC.Id) + SQ(FOC.Iq));
		flux_vel_sum += encoder.vel;
		flux_cnt++;

		if (flux_cnt >= FLUX_MEAS_SAMPLES) {
			float vq_b = flux_vq_sum / (float)flux_cnt;
			float i_b = flux_i_sum / (float)flux_cnt;
			float di = flux_i_a - i_b;
			if (fabsf(di) > 0.1f)
				flux_r_est = (flux_vq_a - vq_b) / di;
			printf("flux Rtest: I %.3f->%.3f A, Vq %.4f->%.4f V, "
			       "vel %.3f rps, R_est=%.4f Ohm\n",
			       flux_i_a, i_b, flux_vq_a, vq_b,
			       flux_vel_sum / (float)flux_cnt, flux_r_est);

			FOC.calib_iq =
		    Config.calib_current * Config.calib_flux_iq_gain;
			flux_v_sum = flux_i_sum = flux_vq_sum = flux_vel_sum =
			    0.0f;
			flux_cnt = 0;
			loop_count = 0;
			mCalibStep = CS_FLUX_RAMP2;
		}
	} break;

	case CS_FLUX_RAMP2:
		/* 从 FLUX_SPEED1 升到 FLUX_SPEED2 */
		FOC.calib_phase_vel += (FLUX_SPEED2 - FLUX_SPEED1) /
				       (FLUX_SETTLE_TIME * (float)CAL_LOOP_HZ);
		if (FOC.calib_phase_vel > FLUX_SPEED2)
			FOC.calib_phase_vel = FLUX_SPEED2;
		if (FOC.calib_angle >= M_2PI)
			FOC.calib_angle -= M_2PI;
		if (time > FLUX_SETTLE_TIME) {
			flux_v_sum = 0;
			flux_i_sum = 0;
			flux_vel_sum = 0.0f;
			flux_cnt = 0;
			flux_sat = false;
			loop_count = 0;
			mCalibStep = CS_FLUX_MEAS2;
		}
		break;

	case CS_FLUX_END: {
		FOC.calib_mode = false;
		FOC.calib_adv_angle = false;
		pwm_output_low_sides();

		/* 拖动没跟上时 λ 一定是错的: 用实测转速和指令转速对比 */
		bool vel_ok = true;
		if (Config.pole_pairs > 0) {
			float exp1 = FLUX_SPEED1 /
				     (M_2PI * (float)Config.pole_pairs);
			float exp2 = FLUX_SPEED2 /
				     (M_2PI * (float)Config.pole_pairs);
			if ((fabsf(flux_vel1 - exp1) > 0.25f * exp1) ||
			    (fabsf(flux_vel2 - exp2) > 0.25f * exp2)) {
				vel_ok = false;
				printf("flux: 拖动跟不上 (vel %.3f/期望 %.3f, "
				       "%.3f/期望 %.3f rps), 拒绝 λ\n",
				       (double)flux_vel1, (double)exp1,
				       (double)flux_vel2, (double)exp2);
			}
		}
		if (vel_ok && !flux_sat1 && !flux_sat2 && (flux_i1 > 0.05f) &&
		    (flux_i2 > 0.05f)) {
			/* y = R·i + ω·λ: 扣掉 ωLi 平方项, 两个转速解 R 和 λ */
			float t1 =
			    SQ(flux_v1) -
			    SQ(FLUX_SPEED1 * Config.phase_inductance * flux_i1);
			float t2 =
			    SQ(flux_v2) -
			    SQ(FLUX_SPEED2 * Config.phase_inductance * flux_i2);
			float y1 = sqrtf((t1 > 0.0f) ? t1 : 0.0f);
			float y2 = sqrtf((t2 > 0.0f) ? t2 : 0.0f);

			float den =
			    FLUX_SPEED2 * flux_i1 - FLUX_SPEED1 * flux_i2;
			float lambda = (y2 * flux_i1 - y1 * flux_i2) / den;
			float res = (y1 - FLUX_SPEED1 * lambda) / flux_i1;
			float lambda_simple =
			    (flux_v2 - flux_v1) / (FLUX_SPEED2 - FLUX_SPEED1);

			/* 两个转速点的实测值 (vel = w_e/(2π·p)) */
			printf("flux pts: V1=%.4f I1=%.3f vel1=%.3f | V2=%.4f "
			       "I2=%.3f vel2=%.3f\n",
			       flux_v1, flux_i1, flux_vel1, flux_v2, flux_i2,
			       flux_vel2);
			printf("flux cmd: we1=%.1f we2=%.1f rad/s (预期 "
			       "vel1=%.3f vel2=%.3f rps)\n",
			       FLUX_SPEED1, FLUX_SPEED2,
			       FLUX_SPEED1 / (M_2PI * (float)Config.pole_pairs),
			       FLUX_SPEED2 /
				   (M_2PI * (float)Config.pole_pairs));
			printf(
			    "flux: R=%.3f Ohm lambda=%.6f Wb (simple %.6f)\n",
			    res, lambda, lambda_simple);

			/* 用 Vq 分量与 R_est 解 lambda, 两转速点互相校验 */
			{
				float rr = (flux_r_est > 0.001f)
					       ? flux_r_est
					       : Config.phase_resistance;
				float lv1 =
				    (flux_vq_a - rr * flux_i1) / FLUX_SPEED1;
				float lv2 =
				    (flux_vq_b - rr * flux_i2) / FLUX_SPEED2;
				/* 两点法(仅 Vq): 消掉 R·Iq 与恒定电压偏置 */
				float lvp = ((flux_vq_b - flux_vq_a) -
					     rr * (flux_i2 - flux_i1)) /
					    (FLUX_SPEED2 - FLUX_SPEED1);
				/* 用 |ΔV| 而非 Δ|V|: 近似垂直时模差会低估 */
				float dvd = flux_vd_b - flux_vd_a;
				float dvq = flux_vq_b - flux_vq_a;
				float dvm = sqrtf(SQ(dvd) + SQ(dvq));
				float lam_vec =
				    dvm / (FLUX_SPEED2 - FLUX_SPEED1);

				printf("flux Vq: Vq1=%.4f Vq2=%.4f Vd1=%.4f "
				       "Vd2=%.4f (R=%.4f)\n",
				       flux_vq_a, flux_vq_b, flux_vd_a,
				       flux_vd_b, rr);
				printf("flux Vq: lam1=%.6f lam2=%.6f (单点, "
				       "差%.0f%%)  两点法=%.6f\n",
				       lv1, lv2,
				       100.0f * (lv1 - lv2) /
					   ((lv1 + lv2) * 0.5f + 1e-9f),
				       lvp);
				printf("flux vec: |dV|=%.4f (模差仅%.4f) -> "
				       "lambda_vec=%.6f Wb\n",
				       dvm, flux_v2 - flux_v1, lam_vec);

				/* 矢量差分法作为最终磁链; 无效时回退到模差法 */
				if ((lam_vec > 0.00005f) && (lam_vec < 1.0f))
					Config.flux_linkage = lam_vec;
				else if ((lambda > 0.00005f) && (lambda < 1.0f))
					Config.flux_linkage = lambda;
			}

			/* 磁链标定的 R 只在接近电阻标定值时才采纳 */
			if ((res > 0.01f) && (res < 50.0f) &&
			    (fabsf(res - Config.phase_resistance) <
			     0.2f * Config.phase_resistance)) {
				Config.phase_resistance = res;
			} else {
				printf("flux: R rejected (%.4f vs %.4f), keep "
				       "%.4f\n",
				       res, Config.phase_resistance,
				       Config.phase_resistance);
			}
			FOC_update_current_ctrl_gain(Config.current_ctrl_bw);
		} else {
			printf("flux measure invalid (saturated or current too "
			       "small)\n");
		}

		phase_set = 0;
		loop_count = 0;
		mCalibStep = CS_MOTOR_J_START;
	} break;

	case CS_ENCODER_START:
		phase_set = 0;
		loop_count = 0;
		sample_count = 0;
		next_sample_time = 0;
		mCalibStep = CS_ENCODER_CW_LOOP;
		break;
	case CS_ENCODER_CW_LOOP:
		if (sample_count <
		    (int32_t)(Config.pole_pairs * SAMPLES_PER_PPAIR)) {
			float mech_phase = phase_set / (float)Config.pole_pairs;
			float mech_phase_wrapped = fmodf(mech_phase, M_2PI);
			if (mech_phase_wrapped < 0)
				mech_phase_wrapped += M_2PI;
			int count_ref =
			    (int)(mech_phase_wrapped * ENCODER_CPR_F / M_2PI);
			int actual_count =
			    (Config.encoder_dir == 1)
				? encoder.raw
				: (ENCODER_CPR - 1 - encoder.raw);
			int error = actual_count - count_ref;
			while (error > ENCODER_CPR / 2)
				error -= ENCODER_CPR;
			while (error < -ENCODER_CPR / 2)
				error += ENCODER_CPR;
			encoder_error_arr[sample_count] = error;
			sample_count++;
			phase_set += M_2PI / (float)SAMPLES_PER_PPAIR;
		} else {
			phase_set -= M_2PI / (float)SAMPLES_PER_PPAIR;
			loop_count = 0;
			sample_count--;
			mCalibStep = CS_ENCODER_CCW_LOOP;
		}
		FOC_voltage(drag_v, 0, phase_set);
		break;
	case CS_ENCODER_CCW_LOOP:
		if (sample_count >= 0) {
			float mech_phase = phase_set / (float)Config.pole_pairs;
			float mech_phase_wrapped = fmodf(mech_phase, M_2PI);
			if (mech_phase_wrapped < 0)
				mech_phase_wrapped += M_2PI;
			int count_ref =
			    (int)(mech_phase_wrapped * ENCODER_CPR_F / M_2PI);
			int actual_count =
			    (Config.encoder_dir == 1)
				? encoder.raw
				: (ENCODER_CPR - 1 - encoder.raw);
			int error = actual_count - count_ref;
			while (error > ENCODER_CPR / 2)
				error -= ENCODER_CPR;
			while (error < -ENCODER_CPR / 2)
				error += ENCODER_CPR;
			encoder_error_arr[sample_count] =
			    (encoder_error_arr[sample_count] + error) / 2;
			sample_count--;
			phase_set -= M_2PI / (float)SAMPLES_PER_PPAIR;
			FOC_voltage(drag_v, 0, phase_set);
		} else {
			pwm_output_low_sides();
			sample_count = 0;
			loop_count = 0;
			mCalibStep = CS_ENCODER_END;
		}
		break;
	case CS_ENCODER_END: {
		if ((Config.pole_pairs <= 0) ||
		    (Config.pole_pairs > (int32_t)MAX_MOTOR_POLE_PAIRS)) {
			/* 极对数异常则置无效并退出, 避免卡在 CALIBRATION */
			Config.calib_valid = false;
			mCalibStep = CS_NULL;
			foc_request_stage(IDLE);
			break;
		}

		uint32_t total_samples =
		    (uint32_t)Config.pole_pairs * SAMPLES_PER_PPAIR;
		int window = (int)LUT_WINDOW;

		if (sample_count == 0) {
			int64_t sum = 0;
			for (uint32_t i = 0; i < total_samples; i++)
				sum += encoder_error_arr[i];
			Config.encoder_offset =
			    (int32_t)(sum / (int64_t)total_samples);
		}

		if (sample_count < (int32_t)OFFSET_LUT_NUM) {
			int64_t window_sum = 0;
			for (int j = (-window) / 2; j < (window) / 2; j++) {
				int index = (int)((int32_t)sample_count *
						  (int32_t)total_samples /
						  (int32_t)OFFSET_LUT_NUM) +
					    j;
				while (index < 0)
					index += (int)total_samples;
				while (index >= (int)total_samples)
					index -= (int)total_samples;
				window_sum += encoder_error_arr[index];
			}
			int32_t window_avg = (int32_t)(window_sum / window);
			int lut_offset = (int)((int64_t)encoder_error_arr[0] *
					       OFFSET_LUT_NUM / ENCODER_CPR);
			int lut_index =
			    (lut_offset + sample_count) % (int)OFFSET_LUT_NUM;
			if (lut_index < 0)
				lut_index += (int)OFFSET_LUT_NUM;
			Config.offset_lut[lut_index] =
			    window_avg - Config.encoder_offset;
			sample_count++;
		} else {
			loop_count = 0;
			sample_count = 0;
			next_sample_time = 0;
			mCalibStep = CS_FLUX_START;
		}
	} break;

	case CS_MOTOR_J_START:
		Controller.ctrl_mode = CONTROL_MODE_CURRENT_RAMP;
		/* CURRENT_RAMP 以 input_current 为目标 */
		Controller.input_current = 0.0f;
		Controller.cur_setpoint = 0.0f;
		FOC.calib_mode = false;
		FOC.calib_vinj_mode = false;
		FOC.calib_id = 0.0f; /* 清掉磁链标定残留的 7A 拖动电流 */
		FOC.calib_iq = 0.0f;
		foc_arm();
		/* 标定要 2A 加速电流, 临时放开速度限幅 */
		j_vel_limit_save = Config.velocity_limit;
		Config.velocity_limit = 10000.0f;
		j_slope_acc = 0.0f;
		j_slope_dec = 0.0f;
		loop_count = 0;
		mCalibStep = CS_MOTOR_J_SPINUP;
		break;

	case CS_MOTOR_J_SPINUP:
	case CS_MOTOR_J_ACC:
	case CS_MOTOR_J_DEC: {
		/* 直接驱动(绕开 Controller 指令路径), 角度/角速度用编码器实测 */
		/* 减速段断电流自由滑行: 反制动 7ms 就过零, 1kHz 采样不够 */
		float j_iq = Config.calib_current * Config.calib_j_iq_gain;
		float iq = (mCalibStep == CS_MOTOR_J_DEC) ? 0.0f : j_iq;
		
		/* 转速上限保护: 小惯量电机 0.3s 加速会冲很高, 过流前先收手 */
		if ((mCalibStep != CS_MOTOR_J_SPINUP) &&
		    (fabsf(encoder.vel) > J_SPINUP_MAX_RPS)) {
			FOC.calib_iq = 0.0f;
			printf("J: 转速 %.2f rps 超过 %.0f, 放弃本次 J\n",
			       (double)encoder.vel, (double)J_SPINUP_MAX_RPS);
			j_slope_acc = 0.0f;
			j_slope_dec = 0.0f;
			mCalibStep = CS_MOTOR_J_END;
			break;
		}
		/* 直接驱动: 角度/角速度用编码器实测, 转子才能自由加速 */
		FOC.calib_mode = true;
		FOC.calib_id = 0.0f;
		FOC.calib_iq = iq;
		FOC.calib_angle = encoder.phase;
		FOC.calib_phase_vel =
		    encoder.vel * M_2PI * (float)Config.pole_pairs;

		if (mCalibStep == CS_MOTOR_J_SPINUP) {
			if ((encoder.vel >= J_SPINUP_RPS) || (time > 3.0f)) {
				printf("J spinup: t=%.2f vel=%.3f Iq=%.3f "
				       "set=%.3f ciq=%.2f armed=%d\n",
				       (double)time, encoder.vel,
				       (double)FOC.Iq, (double)FOC.Iq_set,
				       (double)FOC.calib_iq, (int)FOC.is_armed);
				j_sum_t = j_sum_v = j_sum_tv = j_sum_tt = 0.0f;
				j_cnt = 0;
				loop_count = 0;
				mCalibStep = CS_MOTOR_J_ACC;
			}
			break;
		}

		j_sum_t += (float)j_cnt;
		j_sum_v += encoder.vel;
		j_sum_tv += (float)j_cnt * encoder.vel;
		j_sum_tt += (float)j_cnt * (float)j_cnt;
		j_cnt++;

		/* |a-|/a+ 恒 >1(实测 2.35x), 接近零点提前收工, 至少 30 点 */
		if ((mCalibStep == CS_MOTOR_J_DEC) && (j_cnt > 30U) &&
		    (encoder.vel < 0.3f)) {
			j_slope_dec = j_slope();
			mCalibStep = CS_MOTOR_J_END;
			break;
		}

		if (j_cnt >= J_CAL_SAMPLES) {
			if (mCalibStep == CS_MOTOR_J_ACC) {
				j_slope_acc = j_slope();
				j_sum_t = j_sum_v = j_sum_tv = j_sum_tt = 0.0f;
				j_cnt = 0;
				mCalibStep = CS_MOTOR_J_DEC;
			} else {
				j_slope_dec = j_slope();
				mCalibStep = CS_MOTOR_J_END;
			}
		}
	} break;

	case CS_MOTOR_J_END: {
		Controller.input_current = 0.0f;
		Controller.cur_setpoint = 0.0f;
		Config.velocity_limit = j_vel_limit_save;
		pwm_output_low_sides();

		float da = j_slope_acc - j_slope_dec;
		float kt =
		    1.5f * (float)Config.pole_pairs * Config.flux_linkage;
		float J = 0.0f;
		if (fabsf(da) > 1e-3f)
			J = kt * (Config.calib_current * Config.calib_j_iq_gain) /
		    (3.14159265f * da);
		printf("J: a+=%.3f a-=%.3f rps/s  Kt=%.5f -> J=%.3e kg.m2\n",
		       j_slope_acc, j_slope_dec, kt, J);
		if ((J > 1e-7f) && (J < 1e-1f)) {
			/* 由 J 反算速度环初值: Kp=4πζωbw·J/Kt, Ki=2πJωbw²/Kt */
			const float wbw = 2.0f * 3.14159265f * 10.0f;
			printf(
			    "J: 建议速度环 Kp=%.2f Ki=%.1f (zeta=1, bw=10Hz)\n",
			    4.0f * 3.14159265f * wbw * J / kt,
			    2.0f * 3.14159265f * J * wbw * wbw / kt);
		}
		mCalibStep = CS_REPORT_OFFSET_LUT;
	} break;

	/* 停留一会儿供上位机读回偏移表 */
	case CS_REPORT_OFFSET_LUT:
		if (sample_count < (int32_t)OFFSET_LUT_NUM) {
			if (time > next_sample_time) {
				next_sample_time += 0.001f;
				sample_count++;
			}
		} else {
			/* ===== 标定结果汇总 ===== */
			float R = Config.phase_resistance;
			float L = Config.phase_inductance;
			float lm = Config.flux_linkage;
			float wc =
			    M_2PI *
			    Config.current_ctrl_bw; /* 电流环设计带宽 rad/s */

			int32_t lut_min = 0, lut_max = 0;
			int32_t cg_min = 0, cg_max = 0;
			for (uint32_t i = 0; i < OFFSET_LUT_NUM; i++) {
				int32_t v = Config.offset_lut[i];
				if (i == 0U) {
					lut_min = v;
					lut_max = v;
				}
				if (v < lut_min)
					lut_min = v;
				if (v > lut_max)
					lut_max = v;
			}
			for (uint32_t i = 0; i < COGGING_MAP_NUM; i++) {
				int32_t v = (int32_t)Config.cogging_map[i];
				if (i == 0U) {
					cg_min = v;
					cg_max = v;
				}
				if (v < cg_min)
					cg_min = v;
				if (v > cg_max)
					cg_max = v;
			}

			Config.calib_valid = true;

			printf("\n===== CALIB RESULT =====\n");
			printf(" R      = %.5f Ohm\n", R);
			printf(" L      = %.6f H\n", L);
			printf(" lambda = %.6f Wb\n", lm);
			printf(" Kt     = %.5f N.m/A\n",
			       1.5f * (float)Config.pole_pairs * lm);
			printf(" curKp  = %.4f (L*wc)  curKi = %.1f (R*wc)\n",
			       L * wc, R * wc);
			printf(" poles  = %d pairs\n", (int)Config.pole_pairs);
			printf(" enc    = dir %d offset %d cnt (%.3f deg)\n",
			       (int)Config.encoder_dir,
			       (int)Config.encoder_offset,
			       (float)Config.encoder_offset * 360.0f /
				   ENCODER_CPR_F);
			printf(" offset_lut : min %d max %d pkpk %d\n",
			       (int)lut_min, (int)lut_max,
			       (int)(lut_max - lut_min));
			printf(" cogging_map: min %d max %d pkpk %d\n",
			       (int)cg_min, (int)cg_max,
			       (int)(cg_max - cg_min));
			printf(" valid  = %d\n", (int)Config.calib_valid);
			printf("========================\n");

			loop_count = 0;
			sample_count = 0;
			next_sample_time = 0;
			mCalibStep = CS_NULL;
			CONFIG_save_to_flash();
			foc_request_stage(IDLE);
		}
		break;

	default:
		break;
	}
	loop_count++;
}

/* 齿槽标定(位置保持法): 每点停稳后多点平均, 正反两遍消摩擦 */
#define CG_GAP_STEPS 100      /* 起始/换向空走步数 */
#define CG_GAP_MS 50U	      /* 空走每步时间 */
#define CG_SETTLE_POS 0.0025f /* 到位窗口 (rad, 约 0.14° 机械 = 1/5 bin);
                               * 原来 0.006 (半个 bin) 会让相邻点索引乱跳 */
#define CG_SETTLE_VEL 0.05f   /* 停稳速度窗口 (rps) */
#define CG_SETTLE_TICKS 15U   /* 连续 15ms 满足才算停稳 (5ms 还在余振里) */
#define CG_SETTLE_MAX_MS 300U /* 单点最长等待 */
#define CG_AVG_SAMPLES 64U    /* 每点平均样本数 (1kHz -> 64ms):
                               * 单点电流噪声 ~9mA, 取 8 点时均方误差 3.2mA,
                               * 已占齿槽表幅值(14.5mA)的 22%; 取 64 点降到 1.1mA */
#define CG_TIMEOUT_S 300U     /* 总超时: 512点x2方向, 每点最多 300ms 停稳 */
#define CG_LP_BINS 1U	      /* 1 = 不做圆周低通 (避免 84 阶相位滞后) */

/* 恒速扫描法(0x2E): 匀速转 N 圈, 按机械角分箱平均;
 * 齿槽位置锁定(同向叠加), 摩擦随方向反号(相消) */
/* 扫描转速要让齿槽频率落在速度环带宽内: 0.1rps 时 84 阶=8.4Hz 可信;
 * 0.3rps 时 25Hz 相位滞后, 实测该表对 84 阶无效 */
#define CG_SW_RPS 0.10f	    /* 扫描转速 (rps). 试过 0.05: 84 阶只从 -29% 改善到 -40%,
				 * 但会粘滑(vel 频繁掉到 0)、14 阶反而从 -84% 退到 -64%,
				 * 总 std 仅 -19.2% -> -20.7% (在测量噪声内). 回到 0.1 更稳也更快 */
#define CG_SW_REVS 4.0f	    /* 每方向累加圈数 (0.1rps: 4.5圈x2 = 90s, 每箱 ~78 点) */
#define CG_SW_RAMP 1.5f	    /* 速度指令斜坡 (rps/s) */
#define CG_SW_SKIP_REV 0.5f /* 每方向先空转半圈再开始累加 */

static uint8_t cg_method = 0U; /* 0 = 位置保持(0x28), 1 = 恒速扫描(0x2E) */
static uint8_t cg_ff_save = 1U; /* 扫描前保存摩擦前馈开关, 结束后恢复 */
static float cg_sw_prev;  /* 上一拍机械角 */
static float cg_sw_travel; /* 本方向累计转角 (rad) */
static float cg_sw_dir;	   /* +1 / -1 */
static float cg_sw_target; /* 当前速度指令 */
static float cg_sw_last_travel; /* 上次记录的位置 (判停滞) */
static uint16_t cg_sw_nomove_ms; /* 连续不动计数 (ms) */
static uint8_t cg_sw_phase; /* 0=加减速 1=累加 2=收尾 */
static int32_t cg_sw_sum[2][COGGING_MAP_NUM];
static uint16_t cg_sw_cnt[2][COGGING_MAP_NUM];


typedef enum {
	CG_PH_GAP_FWD,
	CG_PH_SCAN_FWD,
	CG_PH_GAP_REV,
	CG_PH_SCAN_REV,
	CG_PH_DONE
} tCgPhase;

typedef enum { CG_ST_MOVE, CG_ST_SETTLE, CG_ST_AVG } tCgState;

static int cg_number; /* 已走步数 (诊断) */
static uint32_t cg_total_ticks;
static bool cg_valid;
static bool cg_running;
static int16_t cg_tmp[COGGING_MAP_NUM];
static float cg_pos_p_save;
static tCgPhase cg_phase;
static tCgState cg_state;
static int cg_step_idx;	 /* 当前阶段内步数 */
static uint32_t cg_tick; /* 当前步内 1kHz 计数 */
static uint32_t cg_settle_cnt;
static uint32_t cg_settle_timeouts; /* 停稳超时点数 (整次标定的质量指标) */
static int32_t cg_iq_sum;
static uint32_t cg_iq_n;
static uint8_t cg_written[(COGGING_MAP_NUM + 7U) / 8U];
/* 齿槽表 dump 游标: >= COGGING_MAP_NUM 表示空闲 */
static uint32_t cg_dump_idx = COGGING_MAP_NUM;

/* 位置指令前进/后退一步 (单圈机械角, 循环卷绕) */
static void cg_step(float dir)
{
	const float delta = dir * (M_2PI / (float)COGGING_MAP_NUM);
	float pos_ref = Controller.input_position + delta;
	while (pos_ref >= M_2PI)
		pos_ref -= M_2PI;
	while (pos_ref < 0.0f)
		pos_ref += M_2PI;
	Controller.input_position = pos_ref;
}

/* 单圈机械角 -> 补偿表下标 (0 ~ COGGING_MAP_NUM-1) */
static int cg_index(float pos)
{
	int idx = (int)nearbyintf((float)COGGING_MAP_NUM * pos / M_2PI);
	if (idx >= (int)COGGING_MAP_NUM)
		idx = 0;
	if (idx < 0)
		idx = 0;
	return idx;
}

/* 当前机械角 (rad, 单圈), 用对齐计数保证与校准零位一致 */
static float cg_pos(void)
{
	return (float)encoder.count_aligned * (M_2PI / ENCODER_CPR_F);
}

/* 位置误差卷绕到 [-π, π] */
static float cg_pos_err(void)
{
	float perr = Controller.input_position - cg_pos();
	while (perr > M_PI)
		perr -= M_2PI;
	while (perr < -M_PI)
		perr += M_2PI;
	return perr;
}

/* 静止判据: 位置误差和速度都进入窗口 */
static bool cg_settled(void)
{
	return (fabsf(cg_pos_err()) < CG_SETTLE_POS) &&
	       (fabsf(encoder.vel) < CG_SETTLE_VEL);
}

static int16_t cg_to_i16(int32_t raw)
{
	if (raw > 32767)
		raw = 32767;
	if (raw < -32768)
		raw = -32768;
	return (int16_t)raw;
}

static int cg_wrap(int idx)
{
	while (idx < 0)
		idx += (int)COGGING_MAP_NUM;
	while (idx >= (int)COGGING_MAP_NUM)
		idx -= (int)COGGING_MAP_NUM;
	return idx;
}

static void cg_mark(int idx)
{
	cg_written[idx >> 3] |= (uint8_t)(1U << (idx & 7));
}

static bool cg_is_written(int idx)
{
	return (cg_written[idx >> 3] & (uint8_t)(1U << (idx & 7))) != 0U;
}

void anticogging_start(void)
{
	cg_pos_p_save = Config.pos_p_gain;

	if (!Config.calib_valid || (Config.pole_pairs <= 0)) {
		printf("cogging: refused, calib_valid=%d pole_pairs=%ld "
		       "-> run 0x27 to completion first\n",
		       (int)Config.calib_valid, (long)Config.pole_pairs);
		cg_running = false;
		return;
	}

	foc_arm();
	FOC.calib_mode = false;
	FOC.calib_vinj_mode = false;

	Config.cogging_map_valid = 0;
	memset(Config.cogging_map, 0, sizeof(Config.cogging_map));
	memset(cg_written, 0, sizeof(cg_written));

	cg_number = 0;
	cg_total_ticks = 0;
	cg_valid = false;
	cg_running = true;
	cg_iq_sum = 0;
	cg_iq_n = 0;
	cg_settle_timeouts = 0;

	if (cg_method == 1U) {
		Controller.ctrl_mode = CONTROL_MODE_VELOCITY_RAMP;
		Controller.target_reached = false;
		CONTROLLER_reset();
		cg_ff_save = CONTROLLER_get_friction_ff();
		CONTROLLER_set_friction_ff(0U); /* 前馈换挡会注入台阶, 扫描期间关 */
		memset(cg_sw_sum, 0, sizeof(cg_sw_sum));
		memset(cg_sw_cnt, 0, sizeof(cg_sw_cnt));
		cg_sw_prev = cg_pos();
		cg_sw_travel = 0.0f;
		cg_sw_dir = 1.0f;
		cg_sw_target = 0.0f;
		cg_sw_phase = 0U;
		cg_sw_last_travel = 0.0f;
		cg_sw_nomove_ms = 0U;
		printf("cogging: sweep start, %.2f rps x %.0f revs x2 dirs "
		       "(skip %.1f rev), timeout=%us\n",
		       (double)CG_SW_RPS, (double)CG_SW_REVS,
		       (double)CG_SW_SKIP_REV, (unsigned)CG_TIMEOUT_S);
		return;
	}

	Controller.ctrl_mode = CONTROL_MODE_POSITION_FILTER;
	Controller.target_reached = false;
	CONTROLLER_reset();
	Config.pos_p_gain = 50.0f;
	cg_phase = CG_PH_GAP_FWD;
	cg_state = CG_ST_MOVE;
	cg_step_idx = 0;
	cg_tick = 0;
	cg_settle_cnt = 0;
	printf("cogging: start, %u pts x2 dirs, pos_p=%.1f, timeout=%us\n",
	       (unsigned)COGGING_MAP_NUM, (double)Config.pos_p_gain,
	       (unsigned)CG_TIMEOUT_S);
}

/* 扫完一遍后处理: 去均值 + 圆周低通 */
static void cg_finish_map(void)
{
	int32_t sum = 0;
	int32_t mean;

	/* 漏采的 bin 用两侧最近有效点插值 */
	uint32_t missing = 0;
	for (uint32_t i = 0; i < COGGING_MAP_NUM; i++) {
		if (cg_is_written((int)i))
			continue;
		missing++;
		int lo_d = 1;
		while ((lo_d < (int)COGGING_MAP_NUM) &&
		       !cg_is_written(cg_wrap((int)i - lo_d)))
			lo_d++;
		int hi_d = 1;
		while ((hi_d < (int)COGGING_MAP_NUM) &&
		       !cg_is_written(cg_wrap((int)i + hi_d)))
			hi_d++;
		if ((lo_d >= (int)COGGING_MAP_NUM) ||
		    (hi_d >= (int)COGGING_MAP_NUM))
			continue;
		int32_t vl = Config.cogging_map[cg_wrap((int)i - lo_d)];
		int32_t vh = Config.cogging_map[cg_wrap((int)i + hi_d)];
		Config.cogging_map[i] =
		    (int16_t)(vl + (vh - vl) * lo_d / (lo_d + hi_d));
		cg_mark((int)i);
	}
	if (missing)
		printf("cogging: filled %u missing bins\n", (unsigned)missing);

	for (uint32_t i = 0; i < COGGING_MAP_NUM; i++)
		sum += Config.cogging_map[i];
	mean = sum / (int32_t)COGGING_MAP_NUM;
	for (uint32_t i = 0; i < COGGING_MAP_NUM; i++)
		Config.cogging_map[i] =
		    cg_to_i16((int32_t)Config.cogging_map[i] - mean);

	/* 圆周低通: 1 bin = 不滤波(避免高阶相位滞后) */
	for (uint32_t i = 0; i < COGGING_MAP_NUM; i++) {
		int32_t s = 0;
		for (int k = -(int)(CG_LP_BINS / 2U);
		     k <= (int)(CG_LP_BINS / 2U); k++) {
			int j = (int)i + k;
			while (j < 0)
				j += (int)COGGING_MAP_NUM;
			while (j >= (int)COGGING_MAP_NUM)
				j -= (int)COGGING_MAP_NUM;
			s += Config.cogging_map[j];
		}
		cg_tmp[i] = (int16_t)(s / (int32_t)CG_LP_BINS);
	}
	for (uint32_t i = 0; i < COGGING_MAP_NUM; i++)
		Config.cogging_map[i] = cg_tmp[i];
}

/* 谐波自检: 判断表里是齿槽(84/168 阶)还是摩擦/编码器误差 */
static void cg_report_harmonics(void)
{
	int best_h[3] = { 0, 0, 0 };
	float best_a[3] = { 0.0f, 0.0f, 0.0f };

	for (int h = 1; h <= 200; h++) {
		float wr = cosf(6.2831853f * (float)h / (float)COGGING_MAP_NUM);
		float wi = -sinf(6.2831853f * (float)h / (float)COGGING_MAP_NUM);
		float cr = 1.0f, ci = 0.0f;
		float sr = 0.0f, si = 0.0f;

		for (uint32_t i = 0; i < COGGING_MAP_NUM; i++) {
			float v = (float)Config.cogging_map[i];

			sr += v * cr;
			si += v * ci;
			float t = cr * wr - ci * wi;

			ci = cr * wi + ci * wr;
			cr = t;
		}
		float a = sqrtf(sr * sr + si * si) * 2.0f /
			  (float)COGGING_MAP_NUM;

		for (int k = 0; k < 3; k++) {
			if (a > best_a[k]) {
				for (int j = 2; j > k; j--) {
					best_a[j] = best_a[j - 1];
					best_h[j] = best_h[j - 1];
				}
				best_a[k] = a;
				best_h[k] = h;
				break;
			}
		}
	}
	printf("cogging: top harm h=%d(%.1fmA) h=%d(%.1fmA) h=%d(%.1fmA)\n",
	       best_h[0], (double)(best_a[0] * 1000.0f / COGGING_SCALE),
	       best_h[1], (double)(best_a[1] * 1000.0f / COGGING_SCALE),
	       best_h[2], (double)(best_a[2] * 1000.0f / COGGING_SCALE));
	/* 物理意义阶次: 14=极对数, 28=极数, 24=槽数, 84/168=齿槽周期 */
	{
		static const int probe[] = { 6, 12, 14, 24, 28, 42, 84, 168 };
		printf("cogging: harm of interest ");
		for (uint32_t k = 0; k < sizeof(probe) / sizeof(probe[0]); k++) {
			int h = probe[k];
			float wr = cosf(6.2831853f * (float)h /
					(float)COGGING_MAP_NUM);
			float wi = -sinf(6.2831853f * (float)h /
					 (float)COGGING_MAP_NUM);
			float cr = 1.0f, ci = 0.0f, sr = 0.0f, si = 0.0f;
			for (uint32_t i = 0; i < COGGING_MAP_NUM; i++) {
				float v = (float)Config.cogging_map[i];
				sr += v * cr;
				si += v * ci;
				float t = cr * wr - ci * wi;
				ci = cr * wi + ci * wr;
				cr = t;
			}
			float a = sqrtf(sr * sr + si * si) * 2.0f /
				  (float)COGGING_MAP_NUM;
			printf("%d:%.1fmA ", h,
			       (double)(a * 1000.0f / COGGING_SCALE));
		}
		printf("\n");
	}
}

/* 空走: 每 CG_GAP_MS 一步 */
static void cg_gap_step(float dir)
{
	if (++cg_tick < CG_GAP_MS)
		return;
	cg_tick = 0;
	cg_step(dir);
	cg_number++;
	if (++cg_step_idx < CG_GAP_STEPS)
		return;

	cg_step_idx = 0;
	if (cg_phase == CG_PH_GAP_FWD)
		cg_phase = CG_PH_SCAN_FWD;
	else
		cg_phase = CG_PH_SCAN_REV;
	cg_state = CG_ST_MOVE;
}

/* 采样: 移动 -> 等停稳 -> 多点平均 */
static void cg_scan_step(float dir)
{
	switch (cg_state) {
	case CG_ST_MOVE:
		cg_step(dir);
		cg_number++;
		cg_state = CG_ST_SETTLE;
		cg_settle_cnt = 0;
		cg_tick = 0;
		break;

	case CG_ST_SETTLE:
		cg_tick++;
		if (cg_settled()) {
			if (++cg_settle_cnt >= CG_SETTLE_TICKS) {
				cg_iq_sum = 0;
				cg_iq_n = 0;
				cg_state = CG_ST_AVG;
			}
		} else {
			cg_settle_cnt = 0;
		}
		if (cg_tick >= CG_SETTLE_MAX_MS) {
			/* 个别点被齿槽夹住就就地采样并计数, 超时超 20% 才判失败 */
			cg_settle_timeouts++;
			printf("cogging: settle timeout #%u at step %d\n",
			       (unsigned)cg_settle_timeouts, cg_number);
			cg_iq_sum = 0;
			cg_iq_n = 0;
			cg_state = CG_ST_AVG;
		}
		break;

	case CG_ST_AVG: {
		cg_iq_sum += (int32_t)(FOC.Iq_filt * COGGING_SCALE);
		if (++cg_iq_n < CG_AVG_SAMPLES)
			break;

		int16_t v = cg_to_i16(cg_iq_sum / (int32_t)cg_iq_n);
		/* 索引用指令位置: 每步正好一个 bin, 实测位置会落进相邻 bin */
		int idx = cg_index(Controller.input_position);
		if ((cg_phase == CG_PH_SCAN_FWD) || !cg_is_written(idx)) {
			Config.cogging_map[idx] = v;
			cg_mark(idx);
		} else {
			Config.cogging_map[idx] =
			    (Config.cogging_map[idx] + v) / 2;
		}

		cg_state = CG_ST_MOVE;
		if (++cg_step_idx < (int)COGGING_MAP_NUM)
			break;

		cg_step_idx = 0;
		if (cg_phase == CG_PH_SCAN_FWD)
			cg_phase = CG_PH_GAP_REV;
		else
			cg_phase = CG_PH_DONE;
		cg_state = CG_ST_MOVE;
		cg_tick = 0;
	} break;
	}
}

/* 0 = 位置保持(0x28), 1 = 恒速扫描(0x2E) */
void anticogging_set_method(uint8_t m)
{
	cg_method = (m != 0U) ? 1U : 0U;
}

/* 建表: 正反两方向平均 */
static void cg_sweep_build(void)
{
	uint32_t both = 0, single = 0;
	float fric_sum = 0.0f;
	uint32_t fric_n = 0;

	for (uint32_t i = 0; i < COGGING_MAP_NUM; i++) {
		bool hf = cg_sw_cnt[0][i] > 0U;
		bool hr = cg_sw_cnt[1][i] > 0U;
		float f = hf ? ((float)cg_sw_sum[0][i] / (float)cg_sw_cnt[0][i])
			     : 0.0f;
		float r = hr ? ((float)cg_sw_sum[1][i] / (float)cg_sw_cnt[1][i])
			     : 0.0f;
		float v;

		if (hf && hr) {
			v = 0.5f * (f + r);
			fric_sum += 0.5f * (f - r); /* 摩擦随方向反号 */
			fric_n++;
			both++;
		} else if (hf) {
			v = f;
			single++;
		} else if (hr) {
			v = r;
			single++;
		} else {
			continue;
		}
		Config.cogging_map[i] = cg_to_i16((int32_t)v);
		cg_mark((int)i);
	}
	printf("cogging: sweep bins %u both, %u single\n", (unsigned)both,
	       (unsigned)single);
	if (fric_n)
		printf("cogging: sweep friction=%.1f mA (fwd-rev 差值/2)\n",
		       (double)(fric_sum / (float)fric_n * 1000.0f / COGGING_SCALE));
	cg_finish_map(); /* 补漏点 + 去均值 + 可选圆周低通 */
}

/* 恒速扫描: 加速 -> 累加 -> 换向 -> 建表 */
static bool cg_sweep_loop(void)
{
	float mech = cg_pos();
	float d = mech - cg_sw_prev;

	while (d > M_PI)
		d -= M_2PI;
	while (d < -M_PI)
		d += M_2PI;
	cg_sw_prev = mech;

	if ((cg_total_ticks % CAL_LOOP_HZ) == 0U) {
		printf("cogging: sweep dir=%+d trav=%.2f rev vel=%.3f "
		       "iq=%.3f\n",
		       (cg_sw_dir > 0.0f) ? 1 : -1,
		       (double)(cg_sw_travel / M_2PI), (double)encoder.vel,
		       (double)FOC.Iq_filt);
	}

	if (cg_sw_phase == 0U) { /* 加减速到目标转速 */
		float step = CG_SW_RAMP / (float)CAL_LOOP_HZ;
		float tgt = cg_sw_dir * CG_SW_RPS;

		if (cg_sw_target < tgt) {
			cg_sw_target += step;
			if (cg_sw_target > tgt)
				cg_sw_target = tgt;
		} else if (cg_sw_target > tgt) {
			cg_sw_target -= step;
			if (cg_sw_target < tgt)
				cg_sw_target = tgt;
		}
		Controller.input_velocity = cg_sw_target;
		if (fabsf(cg_sw_target - tgt) < 0.02f) {
			cg_sw_travel = 0.0f;
			cg_sw_phase = 1U;
		}
		return false;
	}

	if (cg_sw_phase == 1U) { /* 累加 */
		float skip = CG_SW_SKIP_REV * M_2PI;

		cg_sw_travel += fabsf(d);

		/* 停滞检测: 5s 内几乎没走就判失败, 别等 300s 超时
		 * (速度环增益太低/拖不动时就是这种表现) */
		if (fabsf(cg_sw_travel - cg_sw_last_travel) > 0.02f * M_2PI) {
			cg_sw_last_travel = cg_sw_travel;
			cg_sw_nomove_ms = 0U;
		} else if (++cg_sw_nomove_ms > 5000U) {
			printf("cogging: sweep 转子 5s 没动 (vel=%.3f) -> "
			       "检查速度环增益/摩擦\n",
			       (double)encoder.vel);
			cg_valid = false;
			cg_running = false;
			return true;
		}

		if (cg_sw_travel >= skip) {
			int idx = cg_index(mech);
			int k = (cg_sw_dir > 0.0f) ? 0 : 1;

			cg_sw_sum[k][idx] +=
			    (int32_t)(FOC.Iq_filt * COGGING_SCALE);
			cg_sw_cnt[k][idx]++;
		}
		if (cg_sw_travel >= skip + CG_SW_REVS * M_2PI) {
			if (cg_sw_dir > 0.0f) {
				cg_sw_dir = -1.0f;
				cg_sw_phase = 0U; /* 先减速到 0 再反向 */
			} else {
				cg_sw_phase = 2U;
			}
		}
		return false;
	}

	/* 收尾 */
	Controller.input_velocity = 0.0f;
	cg_sweep_build();
	cg_valid = true;
	cg_running = false;
	return true;
}

int anticogging_step(void)
{
	if (!cg_running)
		return -1;
	return (int)cg_phase;
}

bool anticogging_loop(void)
{
	if (!cg_running)
		return true;

	if (++cg_total_ticks > (uint32_t)CG_TIMEOUT_S * CAL_LOOP_HZ) {
		printf("cogging: timeout at step %d\n", cg_number);
		cg_valid = false;
		cg_running = false;
		return true;
	}
	if (cg_method == 1U)
		return cg_sweep_loop();

	if ((cg_total_ticks % CAL_LOOP_HZ) == 0U) {
		printf("cogging: step=%d ph=%d st=%d inpos=%.3f act=%.3f "
		       "iq=%.2f\n",
		       cg_number, (int)cg_phase, (int)cg_state,
		       (double)Controller.input_position, (double)cg_pos(),
		       (double)FOC.Iq_filt);
	}

	switch (cg_phase) {
	case CG_PH_GAP_FWD:
		cg_gap_step(+1.0f);
		break;
	case CG_PH_SCAN_FWD:
		cg_scan_step(+1.0f);
		break;
	case CG_PH_GAP_REV:
		cg_gap_step(-1.0f);
		break;
	case CG_PH_SCAN_REV:
		cg_scan_step(-1.0f);
		break;
	case CG_PH_DONE:
		cg_finish_map();
		cg_valid = true;
		cg_running = false;
		return true;
	}
	return false;
}

void anticogging_end(void)
{
	cg_running = false;

	foc_disarm();
	pwm_output_low_sides();

	/* 恢复位置环增益, 免得 50.0 被存进 Flash */
	Config.pos_p_gain = cg_pos_p_save;
	/* 恢复扫描期间关掉的摩擦前馈 */
	CONTROLLER_set_friction_ff(cg_ff_save);

	/* 停稳超时 >20% 判为不可信 */
	if (cg_settle_timeouts > (2U * COGGING_MAP_NUM / 5U)) {
		printf("cogging: too many settle timeouts (%u), mark invalid\n",
		       (unsigned)cg_settle_timeouts);
		cg_valid = false;
	}

	if (cg_valid) {
		Config.cogging_map_valid = 1;
		Config.anticogging_enable = 1;
		uint8_t sv = CONFIG_save_to_flash();
		int16_t mn = 32767, mx = -32768;
		for (uint32_t i = 0; i < COGGING_MAP_NUM; i++) {
			if (Config.cogging_map[i] < mn)
				mn = Config.cogging_map[i];
			if (Config.cogging_map[i] > mx)
				mx = Config.cogging_map[i];
		}
		printf("cogging: done, %u pts, %d steps, map %d~%d pkpk %d "
		       "(%.3f A)\n",
		       (unsigned)COGGING_MAP_NUM, cg_number, mn, mx,
		       (int)(mx - mn), (float)(mx - mn) / COGGING_SCALE);
		printf("cogging: save %s, timeout %u/%u\n",
		       (sv == 0U) ? "ok" : "FAIL",
		       (unsigned)cg_settle_timeouts,
		       (unsigned)(2U * COGGING_MAP_NUM));
		cg_report_harmonics();
		/* 自动 dump 整张表, 便于离线分析 */
		cg_dump_idx = 0U;
	} else {
		Config.cogging_map_valid = 0;
		printf("cogging: aborted, invalid (step=%d)\n", cg_number);
	}

	Controller.input_velocity = 0.0f;
	Controller.input_velocity_buffer = 0.0f;
}

/* 齿槽表 dump(0x2C): 每拍打 8 点, 不阻塞中断 */
void cogging_dump_request(void)
{
	cg_dump_idx = 0;
}

void cogging_dump_loop(void)
{
	if (cg_dump_idx >= COGGING_MAP_NUM)
		return;
	if (cg_dump_idx == 0U)
		printf("cogging_dump: begin valid=%ld\n",
		       (long)Config.cogging_map_valid);
	for (int k = 0; k < 8 && cg_dump_idx < COGGING_MAP_NUM; k++) {
		printf("%u,%d\n", (unsigned)cg_dump_idx,
		       (int)Config.cogging_map[cg_dump_idx]);
		cg_dump_idx++;
	}
	if (cg_dump_idx >= COGGING_MAP_NUM)
		printf("cogging_dump: end\n");
}
