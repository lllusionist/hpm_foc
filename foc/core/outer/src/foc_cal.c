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
#define FLUX_SPEED1 (2.0f * M_PI * 10.0f) /* 低速点: 80Hz 电时 4.5A 拉不住, 丢步 */
/* 高速点: 30Hz 时 0.7A 会电压饱和 */
#define FLUX_SPEED2 (2.0f * M_PI * 14.0f) /* 高速点: 回到不丢步范围 */
#define FLUX_SETTLE_TIME (1.5f) /* 每个转速稳定时间 (s) */
/* 每点采样点数 (1.5s, 低速点也够转一圈) */
#define FLUX_MEAS_SAMPLES (1500U)
/* 拖动电流系数 (云台 0.7A 以上易饱和) */
#define FLUX_IQ_GAIN (0.8f)
#define FLUX_RTEST_RATIO (0.5f) /* 同转速校验的降流比例 */

/* 电阻: 两点伏安法(相减抵消死区); 电感: 1kHz 正弦注入, Z²=R²+(ωL)² */
#define L_CAL_CYCLES (CAL_LOOP_HZ / 2U) /* 电感: 每轮正弦注入时长 0.5s */
/* 步长按量程 10% 派生, 不写死绝对值 */
/* R 标定步长: 按"每步约 calib_current/3 的电流增量"折算, 上限取量程的 10%.
 * 固定 10% 量程 (=0.1V) 对低阻电机太大: 5210 (R≈0.027Ω) 0.1V 就已经 3.7A,
 * 超过 calib_current=3A, 于是第一步就停 -> steps=0, 差分法退化成单点,
 * R 和 Voff 全废 (Voff 会变成恒等式 0.000).
 *   5210: 0.027V -> 1A/2A/3A 三点;  4310/2312 仍由 10% 上限决定 (行为不变) */
static float r_step; /* 本次 R 标定实际使用的步长 (V) */

static float r_calib_step(void)
{
	return fminf(Config.calib_voltage * 0.10f,
		     fmaxf(Config.calib_current * Config.phase_resistance /
			       3.0f,
			   Config.calib_voltage * 0.01f));
}
#define R_CAL_WINDOW (CAL_LOOP_HZ / 5U)		     /* 每个工作点停留 200ms */
#define CALIB_L_MIN_V (Config.calib_voltage * 0.15f) /* 电感注入幅值下限 (V)
						      * 原为 0.20: 5210 补偿修对后
						      * Z 只有 0.047Ω, 0.2V 就是
						      * 4.2A, 加上 3A 直流偏置峰值
						      * 到 7.2A, 离 9A 软限流太近 */
#define CALIB_L_MAX_PROBE_GAIN (1.5f)
#define CALIB_L_RAMP_TICKS (CAL_LOOP_HZ / 40U)
#define CALIB_L_SETTLE_TICKS (CAL_LOOP_HZ / 10U)
#define CALIB_L_MIN_RESPONSE_A (0.005f)
/* L 注入频率: 固定值, 不再跟 current_ctrl_bw 走.
 * 原来 vinj_omega = 2π·bw, 于是"降带宽保稳定"会连注入频率一起降,
 * Z/R 掉到 1 附近, L 就测不准了. 现在解耦: 环带宽可以按电机阻抗随便定,
 * 注入始终在 1kHz (5210: ωL=0.063Ω vs R=0.042Ω, Z/R≈1.8, 够用). */
#define CALIB_L_INJ_HZ (1000.0f)

static int encoder_error_arr[MAX_MOTOR_POLE_PAIRS * SAMPLES_PER_PPAIR];

static tCalibStep mCalibStep = CS_NULL;
static uint32_t loop_count;	 /* 1kHz 时间基准 */
static float phase_set;		 /* 电角度设定值 */
static int16_t sample_count;	 /* 采样点下标 */
static float next_sample_time;	 /* 下一个采样时刻 (秒) */
static int64_t start_count_long; /* 极对数标定起点计数 */
static bool dir_pp_runin;	 /* 极对数标定: 预跑阶段是否已完成 */
static uint32_t calib_oc_cnt;	 /* 标定软限流的连续越限拍数 */

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

/* J 标定: 加速->自由滑行, 摩擦相减: J = Kt·I/(2π·(a+ - a-)) */
#define J_CAL_IQ (Config.calib_current * 0.15f) /* 加速测试电流 (A) */
#define J_CAL_SAMPLES (150U)		       /* 每段采样点数 (1kHz -> 0.3s) */
#define J_SPINUP_RPS (2.0f) /* 起转目标 (rps): 太高会让 ACC 段冲到过流 */
#define J_SPINUP_MAX_RPS 20.0f /* 加速段转速上限: 超过就放弃, 防过流 */
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
static float l_v_amp_start;
static uint8_t l_pass;	      /* 0 = 探测轮, 1 = 测量轮 */

/* 标定期间临时改动的运行参数, calibration_end 无论成功与否都要恢复,
 * 否则中途失败会把临时值 (如 velocity_limit=10000) 留在 Config 里被保存 */
static float cal_vel_limit_save;
static int cal_ctrl_mode_save;
static bool cal_saved;

void calibration_start(void)
{
	if (!FOC.adc_ready) {
		FOC.fault_code = FOC_FAULT_ADC;
		foc_request_stage(IDLE);
		mCalibStep = CS_NULL;
		return;
	}

	r_step = r_calib_step();

	cal_vel_limit_save = Config.velocity_limit;
	cal_ctrl_mode_save = Controller.ctrl_mode;
	cal_saved = true;

	Config.encoder_dir = +1;
	Config.calib_valid = false;
	FOC.calib_mode = false;
	FOC.calib_vinj_mode = false;

	loop_count = 0;
	phase_set = 0;
	sample_count = 0;
	next_sample_time = 0;
	calib_oc_cnt = 0;
	memset(encoder_error_arr, 0, sizeof(encoder_error_arr));

	/* 跨标定残留会混入 Vq1(实测 0.2985→0.6011), 每次必须清零 */
	flux_v_sum = flux_i_sum = flux_vq_sum = flux_vd_sum = 0.0f;
	flux_vel_sum = 0.0f;
	flux_cnt = 0;

	/* 回显实际生效参数, 便于确认配置已下发 */
	printf("calib: cur=%.2fA vol=%.2fV step=%.3fV R0=%.4fOhm "
	       "drag_min=%.2fV flux_iq=%.2fA j_iq=%.2fA "
	       "dtcomp_in_voltage=%d\n",
	       Config.calib_current, Config.calib_voltage, (double)r_step,
	       Config.phase_resistance, (double)Config.calib_drag_v_min,
	       (double)(Config.calib_current * Config.calib_flux_iq_gain),
	       (double)(Config.calib_current * Config.calib_j_iq_gain),
	       (int)DT_COMP_IN_VOLTAGE);

	mCalibStep = CS_MOTOR_R_START;
}

void calibration_end(void)
{
	FOC.calib_mode = false;
	FOC.calib_vinj_mode = false;
	FOC.calib_adv_angle = false;
	foc_disarm();

	if (cal_saved) {
		Config.velocity_limit = cal_vel_limit_save;
		Controller.ctrl_mode = cal_ctrl_mode_save;
		Controller.input_current = 0.0f;
		Controller.cur_setpoint = 0.0f;
		cal_saved = false;
	}

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

/* 电流闭环 I/f 拖动: 电流环在中断里锁住 Iq, 电角度由标定循环推进.
 * 开环电压拖动在低 R/L、低阻尼电机上会电流塌陷 -> 转速 hunting,
 * 这里改为锁电流: 转矩恒定, 拖动平稳, 不污染极对数/零位. */
static void calib_drag_if(float iq, float elec_angle, float elec_vel)
{
	FOC.calib_vinj_mode = false;
	FOC.calib_mode = true;
	FOC.calib_adv_angle = false;
	FOC.calib_id = 0.0f;
	FOC.calib_iq = iq;
	FOC.calib_angle = elec_angle;
	FOC.calib_phase_vel = elec_vel;
}

/* ====== 各标定阶段 (原 calibration_loop 内的 switch case, 提取为函数) ====== */

/* R/L 标定 (CS_MOTOR_R_* ~ CS_MOTOR_L_END) */
static void cal_rl_step(float time, float ia_meas)
{
	(void)time; /* 本阶段不按时间切换, 仅 ia_meas 用到 */
	switch (mCalibStep) {
	case CS_MOTOR_R_START:
		loop_count = 0;
		r_volt = r_step;
		r_i1 = 0.0f;
		r_isum = 0.0f;
		r_icnt = 0;
		r_s1 = 0;
		r_di_sum = 0.0f;
		r_di_cnt = 0U;
		FOC.calib_mode = false;
		FOC.calib_vinj_mode = false;
		foc_set_resistance_voltage(r_volt);
		foc_arm();
		mCalibStep = CS_MOTOR_R_LOOP;
		break;

	case CS_MOTOR_R_LOOP: {
		r_isum += ia_meas;
		r_icnt++;
		if (loop_count >= R_CAL_WINDOW) {
			float Is = fabsf(r_isum / (float)r_icnt);
			r_isum = 0.0f;
			r_icnt = 0;
			loop_count = 0;
			/* 0.9A 以上才计入差分: 死区补偿在 DT_COMP_ITH(0.8A)
			 * 之后已饱和, 偏置恒定才能相减消掉 */
			if ((r_s1 > 0U) && (Is > 0.9f) && (r_i1 > 0.9f)) {
				r_di_sum += (Is - r_i1);
				r_di_cnt++;
			}
			if ((Is < Config.calib_current) &&
			    ((r_volt + r_step) <=
			     (Config.calib_voltage * 0.9f))) {
				r_i1 = Is;
				r_volt += r_step;
				foc_set_resistance_voltage(r_volt);
				r_s1++;
			} else {
				float di_avg;
				if (r_di_cnt > 0U) {
					di_avg = r_di_sum / (float)r_di_cnt;
					Config.phase_resistance =
					    r_step / di_avg;
				} else {
					/* 只有一个工作点: 差分无意义. 补偿修对后
					 * Voff 接近 0, V/I 才近似等于 R, 仅作兜底 */
					di_avg = Is - r_i1;
					if (Is > 0.5f)
						Config.phase_resistance =
						    r_volt / Is;
					printf("res: WARN single point only, "
					       "R is a rough V/I guess\n");
				}
				printf("res: %.4f Ohm (ures=%.3f I1=%.4f "
				       "I2=%.4f steps=%u di_avg=%.5f "
				       "Voff=%.3fV dtcomp=%d)\n",
				       Config.phase_resistance, r_volt, r_i1,
				       Is, (unsigned)r_s1, (double)di_avg,
				       (double)(r_volt - Is *
							      Config.phase_resistance),
				       (int)DT_COMP_IN_VOLTAGE);
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
		float voltage_limit = Config.calib_voltage * 0.9f;
		float bias = Config.calib_current * Config.phase_resistance;

		l_v_dc = fminf(bias, voltage_limit - CALIB_L_MIN_V);
		l_pass = 0;
		l_v_amp = fminf(fmaxf(bias, CALIB_L_MIN_V),
			       voltage_limit - l_v_dc);
		l_v_amp_start = l_v_amp;
		FOC.vinj_omega = 2.0f * M_PI * CALIB_L_INJ_HZ;
		FOC.vinj_amp = l_v_amp;
		FOC.vinj_dc = l_v_dc;
		FOC.inj_phase = 0;
		FOC.vinj_sum_sin = 0;
		FOC.vinj_sum_cos = 0;
		FOC.vinj_samples = 0;
		FOC.vinj_acc = false;
		loop_count = 0;
		FOC.calib_vinj_mode = true;
		foc_set_resistance_voltage(0.0f);
		foc_arm();
		mCalibStep = CS_MOTOR_L_LOOP;
	} break;

	case CS_MOTOR_L_LOOP:
		/* 先升幅再等稳态，解调只累加恒幅段。 */
		if (loop_count <= CALIB_L_RAMP_TICKS) {
			float fraction = (float)loop_count / CALIB_L_RAMP_TICKS;
			float delta = l_v_amp - l_v_amp_start;

			FOC.vinj_amp = l_v_amp_start + delta * fraction;
		}
		if (!FOC.vinj_acc && loop_count >= CALIB_L_SETTLE_TICKS)
			FOC.vinj_acc = true;
		if (loop_count >= L_CAL_CYCLES) {
			FOC.vinj_acc = false;
			mCalibStep = CS_MOTOR_L_END;
		}
		break;

	case CS_MOTOR_L_END: {
		float omega = 2.0f * M_PI * CALIB_L_INJ_HZ;
		int valid_samples = (int)FOC.vinj_samples;
		float I_amp =
		    (valid_samples > 0)
			? (2.0f / (float)valid_samples) *
			      sqrtf(SQ(FOC.vinj_sum_sin) + SQ(FOC.vinj_sum_cos))
			: 0.0f;
		float Z;

		if (!isfinite(I_amp) || I_amp <= CALIB_L_MIN_RESPONSE_A) {
			foc_disarm();
			printf("ind: no usable current response\n");
			foc_request_stage(IDLE);
			break;
		}
		Z = l_v_amp / I_amp;

		if (l_pass == 0) {
			float new_amp = Config.calib_current * Z;
			float voltage_headroom =
			    Config.calib_voltage * 0.9f - l_v_dc;
			float max_amp = fminf(voltage_headroom,
					      l_v_amp * CALIB_L_MAX_PROBE_GAIN);

			/* 小信号受死区影响，避免按探测阻抗一次放大数倍。 */
			new_amp = fminf(fmaxf(new_amp, CALIB_L_MIN_V), max_amp);
			printf(
			    "ind-probe: Iamp=%.3f Z=%.3f Vamp %.3f -> %.3f\n",
			    I_amp, Z, l_v_amp, new_amp);
			l_v_amp_start = l_v_amp;
			l_v_amp = new_amp;
			FOC.vinj_sum_sin = 0;
			FOC.vinj_sum_cos = 0;
			FOC.vinj_samples = 0;
			FOC.vinj_acc = false;
			l_pass = 1;
			loop_count = 0;
			mCalibStep = CS_MOTOR_L_LOOP;
			break;
		}

		float sys_R = Config.phase_resistance;
		float X_L = 0.0f;
		if (Z > sys_R)
			X_L = sqrtf(Z * Z - sys_R * sys_R);
		else
			X_L = 0.001f;
		float L_eq = X_L / omega;
		Config.phase_inductance = L_eq * 1.0f;
		FOC.calib_vinj_mode = false;
		FOC_update_current_ctrl_gain(Config.current_ctrl_bw);
		printf("ind: %.6f H (Iamp=%.3f Z=%.3f Vdc=%.3f Vamp=%.3f)\n",
		       Config.phase_inductance, I_amp, Z, l_v_dc, l_v_amp);
		phase_set = 0;
		loop_count = 0;
		mCalibStep = CS_DIR_PP_START;
	} break;

	default:
		break;
	}
}

/* 方向/极对数标定 (CS_DIR_PP_* ) */
static void cal_dir_pp_step(float time, float calib_phase_vel,
			    float total_calib_cycles)
{
	switch (mCalibStep) {
	case CS_DIR_PP_START:
		if (!FOC.is_armed)
			foc_arm();
		/* 2s 内把拖动电流从 0 拉满, 角度固定 0 先让转子对齐
		 * (与 5ed4143 已验证可用的版本一致, 不要改成别的轴向) */
		calib_drag_if(Config.calib_current * (time / 2.0f), 0.0f, 0.0f);
		if (time >= 2.0f) {
			taskENTER_CRITICAL();
			encoder.shadow_count = 0;
			taskEXIT_CRITICAL();
			start_count_long = 0;
			dir_pp_runin = false;
			loop_count = 0;
			phase_set = 0;
			mCalibStep = CS_DIR_PP_LOOP;
		}
		break;

	case CS_DIR_PP_LOOP:
		phase_set += calib_phase_vel / (float)CAL_LOOP_HZ;
		calib_drag_if(Config.calib_current, phase_set,
			      calib_phase_vel);
		if (!dir_pp_runin) {
			if (phase_set >= total_calib_cycles * M_2PI) {
				taskENTER_CRITICAL();
				encoder.shadow_count = 0;
				taskEXIT_CRITICAL();
				start_count_long = 0;
				dir_pp_runin = true;
			}
		} else if (phase_set >= 2.0f * total_calib_cycles * M_2PI) {
			FOC.calib_mode = false;
			mCalibStep = CS_DIR_PP_END;
		}
		break;

	case CS_DIR_PP_END: {
		taskENTER_CRITICAL();
		int64_t diff = encoder.shadow_count - start_count_long;
		taskEXIT_CRITICAL();
		if (abs((int)diff) < 500) {
			Config.calib_valid = false;
		} else {
			Config.encoder_dir = (diff > 0) ? 1 : -1;
			float mech_revs = fabsf((float)diff / ENCODER_CPR_F);
			float pp_raw = total_calib_cycles / mech_revs;
			Config.pole_pairs = (int)roundf(pp_raw);
			printf("dir_pp: diff=%lld mech_revs=%.5f pp_raw=%.4f "
			       "-> %d\n",
			       (long long)diff, mech_revs, pp_raw,
			       (int)Config.pole_pairs);
		}
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

	default:
		break;
	}
}

/* 编码器零位/偏移表标定 (CS_ENCODER_* ) */
static void cal_encoder_step(void)
{
	switch (mCalibStep) {
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
			float mech_phase = (phase_set + M_PI / 2.0f) /
					   (float)Config.pole_pairs;
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
		calib_drag_if(Config.calib_current, phase_set,
			      (M_2PI / (float)SAMPLES_PER_PPAIR) *
				  (float)CAL_LOOP_HZ);
		break;

	case CS_ENCODER_CCW_LOOP:
		if (sample_count >= 0) {
			float mech_phase = (phase_set + M_PI / 2.0f) /
					   (float)Config.pole_pairs;
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
			calib_drag_if(
			    Config.calib_current, phase_set,
			    -(M_2PI / (float)SAMPLES_PER_PPAIR) *
				(float)CAL_LOOP_HZ);
		} else {
			FOC.calib_mode = false;
			sample_count = 0;
			loop_count = 0;
			mCalibStep = CS_ENCODER_END;
		}
		break;

	case CS_ENCODER_END: {
		if ((Config.pole_pairs <= 0) ||
		    (Config.pole_pairs > (int32_t)MAX_MOTOR_POLE_PAIRS)) {
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

	default:
		break;
	}
}

/* 磁链标定 (CS_FLUX_* ) */
static void cal_flux_step(float time)
{
	switch (mCalibStep) {
	case CS_FLUX_START:
		Config.flux_linkage = 0;
		FOC.calib_adv_angle = true;
		FOC.calib_id = 0;
		FOC.calib_iq = Config.calib_current * Config.calib_flux_iq_gain;
		FOC.calib_phase_vel = 0;
		FOC.calib_angle = 0;
		FOC.calib_mode = true;
		loop_count = 0;
		mCalibStep = CS_FLUX_RAMP1;
		break;

	case CS_FLUX_RAMP1:
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
			float t1 = SQ(flux_v1) -
				   SQ(FLUX_SPEED1 * Config.phase_inductance * flux_i1);
			float t2 = SQ(flux_v2) -
				   SQ(FLUX_SPEED2 * Config.phase_inductance * flux_i2);
			float y1 = sqrtf((t1 > 0.0f) ? t1 : 0.0f);
			float y2 = sqrtf((t2 > 0.0f) ? t2 : 0.0f);
			float den = FLUX_SPEED2 * flux_i1 - FLUX_SPEED1 * flux_i2;
			float lambda = (y2 * flux_i1 - y1 * flux_i2) / den;
			float res = (y1 - FLUX_SPEED1 * lambda) / flux_i1;
			float lambda_simple =
			    (flux_v2 - flux_v1) / (FLUX_SPEED2 - FLUX_SPEED1);
			printf("flux pts: V1=%.4f I1=%.3f vel1=%.3f | V2=%.4f "
			       "I2=%.3f vel2=%.3f\n",
			       flux_v1, flux_i1, flux_vel1, flux_v2, flux_i2,
			       flux_vel2);
			printf("flux cmd: we1=%.1f we2=%.1f rad/s (预期 "
			       "vel1=%.3f vel2=%.3f rps)\n",
			       FLUX_SPEED1, FLUX_SPEED2,
			       FLUX_SPEED1 / (M_2PI * (float)Config.pole_pairs),
			       FLUX_SPEED2 / (M_2PI * (float)Config.pole_pairs));
			printf("flux: R=%.3f Ohm lambda=%.6f Wb (simple %.6f)\n",
			       res, lambda, lambda_simple);
			{
				float rr = (flux_r_est > 0.001f)
					       ? flux_r_est
					       : Config.phase_resistance;
				float lv1 = (flux_vq_a - rr * flux_i1) / FLUX_SPEED1;
				float lv2 = (flux_vq_b - rr * flux_i2) / FLUX_SPEED2;
				float lvp = ((flux_vq_b - flux_vq_a) -
					     rr * (flux_i2 - flux_i1)) /
					    (FLUX_SPEED2 - FLUX_SPEED1);
				float dvd = flux_vd_b - flux_vd_a;
				float dvq = flux_vq_b - flux_vq_a;
				float dvm = sqrtf(SQ(dvd) + SQ(dvq));
				float lam_vec = dvm / (FLUX_SPEED2 - FLUX_SPEED1);
				printf("flux Vq: Vq1=%.4f Vq2=%.4f Vd1=%.4f "
				       "Vd2=%.4f (R=%.4f)\n",
				       flux_vq_a, flux_vq_b, flux_vd_a,
				       flux_vd_b, rr);
				printf("flux Vq: lam1=%.6f lam2=%.6f (单点, "
				       "差%.0f%%)  两点法=%.6f\n",
				       lv1, lv2,
				       100.0f * (lv1 - lv2) /
					   ((lv1 + lv2) * 0.5f + 1e-9f), lvp);
				printf("flux vec: |dV|=%.4f (模差仅%.4f) -> "
				       "lambda_vec=%.6f Wb\n",
				       dvm, flux_v2 - flux_v1, lam_vec);
				if ((lam_vec > 0.00005f) && (lam_vec < 1.0f))
					Config.flux_linkage = lam_vec;
				else if ((lambda > 0.00005f) && (lambda < 1.0f))
					Config.flux_linkage = lambda;
			}
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

	default:
		break;
	}
}

/* 转动惯量 J 标定 (CS_MOTOR_J_* ) */
static void cal_inertia_step(float time)
{
	switch (mCalibStep) {
	case CS_MOTOR_J_START:
		Controller.ctrl_mode = CONTROL_MODE_CURRENT_RAMP;
		Controller.input_current = 0.0f;
		Controller.cur_setpoint = 0.0f;
		FOC.calib_mode = false;
		FOC.calib_vinj_mode = false;
		FOC.calib_id = 0.0f;
		FOC.calib_iq = 0.0f;
		foc_arm();
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
		float j_iq = Config.calib_current * Config.calib_j_iq_gain;
		float iq = (mCalibStep == CS_MOTOR_J_DEC) ? 0.0f : j_iq;

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
		FOC.calib_mode = false;
		float da = j_slope_acc - j_slope_dec;
		float kt = 1.5f * (float)Config.pole_pairs * Config.flux_linkage;
		float J = 0.0f;
		if (fabsf(da) > 1e-3f)
			J = kt * (Config.calib_current * Config.calib_j_iq_gain) /
			    (2.0f * 3.14159265f * da);
		printf("J: a+=%.3f a-=%.3f rps/s  Kt=%.5f -> J=%.3e kg.m2\n",
		       j_slope_acc, j_slope_dec, kt, J);
		if ((J > 1e-7f) && (J < 1e-1f)) {
			const float wbw = 2.0f * 3.14159265f * 10.0f;
			printf(
			    "J: 建议速度环 Kp=%.2f Ki=%.1f (zeta=1, bw=10Hz)\n",
			    4.0f * 3.14159265f * wbw * J / kt,
			    2.0f * 3.14159265f * J * wbw * wbw / kt);
		}
		mCalibStep = CS_REPORT_OFFSET_LUT;
	} break;

	default:
		break;
	}
}

/* 收尾: 报告结果 + 写 Flash (CS_REPORT_OFFSET_LUT) */
static void cal_report_step(float time)
{
	if (sample_count < (int32_t)OFFSET_LUT_NUM) {
		if (time > next_sample_time) {
			next_sample_time += 0.001f;
			sample_count++;
		}
		return;
	}

	/* ===== 标定结果汇总 ===== */
	float R = Config.phase_resistance;
	float L = Config.phase_inductance;
	float lm = Config.flux_linkage;
	float wc = M_2PI * Config.current_ctrl_bw;

	int32_t lut_min = 0, lut_max = 0;
	int32_t cg_min = 0, cg_max = 0;
	for (uint32_t i = 0; i < OFFSET_LUT_NUM; i++) {
		int32_t v = Config.offset_lut[i];
		if (i == 0U) { lut_min = v; lut_max = v; }
		if (v < lut_min) lut_min = v;
		if (v > lut_max) lut_max = v;
	}
	for (uint32_t i = 0; i < COGGING_MAP_NUM; i++) {
		int32_t v = (int32_t)Config.cogging_map[i];
		if (i == 0U) { cg_min = v; cg_max = v; }
		if (v < cg_min) cg_min = v;
		if (v > cg_max) cg_max = v;
	}

	bool calib_ok =
	    (Config.pole_pairs > 0) &&
	    (Config.pole_pairs <= (int32_t)MAX_MOTOR_POLE_PAIRS) &&
	    (Config.flux_linkage > 0.00001f) &&
	    (Config.phase_resistance > 0.0001f) &&
	    (Config.phase_inductance > 1e-9f);

	if (!calib_ok) {
		printf("calib: REJECTED (pp=%ld lam=%.6f R=%.4f L=%.6f)\n",
		       (long)Config.pole_pairs,
		       (double)Config.flux_linkage,
		       (double)Config.phase_resistance,
		       (double)Config.phase_inductance);
	}
	Config.calib_valid = calib_ok;

	printf("\n===== CALIB RESULT =====\n");
	printf(" R      = %.5f Ohm\n", R);
	printf(" L      = %.6f H\n", L);
	printf(" lambda = %.6f Wb\n", lm);
	printf(" Kt     = %.5f N.m/A\n", 1.5f * (float)Config.pole_pairs * lm);
	printf(" curKp  = %.4f (L*wc)  curKi = %.1f (R*wc)\n", L * wc, R * wc);
	printf(" poles  = %d pairs\n", (int)Config.pole_pairs);
	printf(" enc    = dir %d offset %d cnt (%.3f deg)\n",
	       (int)Config.encoder_dir, (int)Config.encoder_offset,
	       (float)Config.encoder_offset * 360.0f / ENCODER_CPR_F);
	printf(" offset_lut : min %d max %d pkpk %d\n",
	       (int)lut_min, (int)lut_max, (int)(lut_max - lut_min));
	printf(" cogging_map: min %d max %d pkpk %d\n",
	       (int)cg_min, (int)cg_max, (int)(cg_max - cg_min));
	printf(" valid  = %d\n", (int)Config.calib_valid);
	printf("========================\n");

	loop_count = 0;
	sample_count = 0;
	next_sample_time = 0;
	mCalibStep = CS_NULL;
	FOC.calib_mode = false;
	FOC.calib_adv_angle = false;
	foc_disarm();
	if (cal_saved) {
		Config.velocity_limit = cal_vel_limit_save;
		Controller.ctrl_mode = cal_ctrl_mode_save;
		cal_saved = false;
	}
	CONFIG_save_to_flash();
	foc_request_stage(IDLE);
}

/* ====== 标定主调度器 ====== */
void calibration_loop(void)
{
	float time = (float)loop_count / (float)CAL_LOOP_HZ;
	const float ia_meas = (FOC.Ia - FOC.Ia_midpoint) * FOC_CURRENT_SCALE;
	float calib_phase_vel = 12.0f * M_PI;
	float total_calib_cycles = 14.0f;

	/* 标定专用软限流: 12A 硬限幅是给功率级兜底的, 标定本身不该走到那一步.
	 * 超过 3×calib_current 就收尾, 留下一句可读原因, 而不是撞硬限幅后
	 * 只剩 FAULT 6 + 一堆 abc 电流. 连续 5 拍才算, 躲开纹波/噪声. */
	{
		float ib_meas =
		    (FOC.Ib - FOC.Ib_midpoint) * FOC_CURRENT_SCALE;
		float ic_meas =
		    (FOC.Ic - FOC.Ic_midpoint) * FOC_CURRENT_SCALE;
		float ilim = 3.0f * Config.calib_current;
		float imax = fabsf(ia_meas);

		if (fabsf(ib_meas) > imax)
			imax = fabsf(ib_meas);
		if (fabsf(ic_meas) > imax)
			imax = fabsf(ic_meas);
		if (FOC.is_armed && (imax > ilim)) {
			if (++calib_oc_cnt >= 5U) {
				printf("calib: OVER-CURRENT %.2fA > %.2fA "
				       "at step %d (id=%.2f iq=%.2f "
				       "cmd=%.2f vq=%.2f ang=%.2f rad)\n",
				       (double)imax, (double)ilim,
				       (int)mCalibStep, (double)FOC.Id,
				       (double)FOC.Iq, (double)FOC.calib_iq,
				       (double)FOC.Vq_out,
				       (double)FOC.calib_angle);
				calib_oc_cnt = 0;
				FOC.calib_mode = false;
				FOC.calib_vinj_mode = false;
				if (mCalibStep >= CS_MOTOR_J_START) {
					/* J 标定只打印、不参与 calib_valid;
					 * R/L/Pn/零位/LUT/λ 都在它之前测完了,
					 * 不能为一个可选的测量作废整次标定. */
					printf("calib: skip J (optional), "
					       "go to report\n");
					foc_disarm();
					Controller.input_current = 0.0f;
					Controller.cur_setpoint = 0.0f;
					loop_count = 0;
					mCalibStep = CS_REPORT_OFFSET_LUT;
					return;
				}
				Config.calib_valid = false;
				foc_disarm();
				foc_request_stage(IDLE);
				return;
			}
		} else {
			calib_oc_cnt = 0;
		}
	}

	/* 拖动阶段每 300ms 打一次 */
	if ((mCalibStep == CS_DIR_PP_LOOP) ||
	    (mCalibStep == CS_ENCODER_CW_LOOP) ||
	    (mCalibStep == CS_ENCODER_CCW_LOOP)) {
		static uint32_t drag_dbg_cnt;
		if (++drag_dbg_cnt >= 300U) {
			drag_dbg_cnt = 0;
			printf("drag(if): Iq=%.2fA I=%.2fA vel=%.3f rps "
			       "elec=%.2f rad\n",
			       (double)FOC.calib_iq, (double)ia_meas,
			       (double)encoder.vel, (double)phase_set);
		}
	}

	/* 丢步检测 */
	if ((mCalibStep >= CS_FLUX_RAMP1) && (mCalibStep <= CS_FLUX_MEAS2) &&
	    (Config.pole_pairs > 0) && (FOC.calib_phase_vel > 1.0f)) {
		static uint32_t slip_dbg;
		float exp_rps = FOC.calib_phase_vel /
				(M_2PI * (float)Config.pole_pairs);
		float dev = fabsf(encoder.vel - exp_rps) / exp_rps;
		if (dev > 0.25f) {
			if ((slip_dbg++ % 200U) == 0U)
				printf("flux: SLIP vel=%.3f exp=%.3f rps "
				       "(dev %.0f%%)\n",
				       (double)encoder.vel, (double)exp_rps,
				       (double)(dev * 100.0f));
		}
	}

	/* 分派到各阶段 */
	switch (mCalibStep) {
	case CS_MOTOR_R_START:
	case CS_MOTOR_R_LOOP:
	case CS_MOTOR_R_END:
	case CS_MOTOR_L_START:
	case CS_MOTOR_L_LOOP:
	case CS_MOTOR_L_END:
		cal_rl_step(time, ia_meas);
		break;

	case CS_DIR_PP_START:
	case CS_DIR_PP_LOOP:
	case CS_DIR_PP_END:
		cal_dir_pp_step(time, calib_phase_vel, total_calib_cycles);
		break;

	case CS_ENCODER_START:
	case CS_ENCODER_CW_LOOP:
	case CS_ENCODER_CCW_LOOP:
	case CS_ENCODER_END:
		cal_encoder_step();
		break;

	case CS_FLUX_START:
	case CS_FLUX_RAMP1:
	case CS_FLUX_MEAS1:
	case CS_FLUX_MEAS2:
	case CS_FLUX_RTEST:
	case CS_FLUX_RAMP2:
	case CS_FLUX_END:
		cal_flux_step(time);
		break;

	case CS_MOTOR_J_START:
	case CS_MOTOR_J_SPINUP:
	case CS_MOTOR_J_ACC:
	case CS_MOTOR_J_DEC:
	case CS_MOTOR_J_END:
		cal_inertia_step(time);
		break;

	case CS_REPORT_OFFSET_LUT:
		cal_report_step(time);
		break;

	default:
		break;
	}

	loop_count++;
}
