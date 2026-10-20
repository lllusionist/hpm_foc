#include <string.h>
#include "foc_cal.h"
#include "encoder.h"
#include "foc_task.h"
#include "foc_ctrl.h"

#define CAL_LOOP_HZ (1000U)
/* 1kHz 下 6.25Hz 电, 每拍一点 => 2240 点/机械圈 */
#define SAMPLES_PER_PPAIR (160U)
#define LUT_WINDOW (128U)	   /* offset LUT 滑动平均窗口 */
#define MAX_MOTOR_POLE_PAIRS (20U) /* 最大支持的极对数 */

/* 磁链: 开环拖动, 两转速点用矢量差分消掉 R 项 */
#define FLUX_SPEED1 (2.0f * M_PI * 10.0f) /* 低速点 (电角速度 rad/s) */
/* 高速点: 原 30Hz, 云台 30Hz+0.7A 电压饱和产生毛刺 */
#define FLUX_SPEED2 (2.0f * M_PI * 14.0f)
#define FLUX_SETTLE_TIME (1.5f) /* 每个转速稳定时间 (s) */
/* 每点采样点数 (1kHz -> 1.5s, 低速点也够转 1 圈) */
#define FLUX_MEAS_SAMPLES (1500U)
/* 拖动电流 = calib_current × 该系数 (原 1.4, 云台 0.7A 易饱和) */
#define FLUX_IQ_GAIN (0.8f)
#define FLUX_RTEST_RATIO (0.5f) /* 同转速校验的降流比例 */

/* ============ 电阻 / 电感 标定方案 (CN103560736B) ============
 * 电阻: 自适应两点伏安法 —— 电压自 Δu2 起逐步升压, 直到稳态电流达
 *       calib_current; 取末两点 Rs = Δu2/(I2-I1)。只依赖电压增量,
 *       且两点靠近, 死区/管压降等非线性相减抵消。
 * 电感: 自适应幅值两轮注入 —— 首轮取 u = I_target×R (因 |Z|>=R, 保证
 *       不过流) 并测出 |Z|; 次轮按 I_target×|Z| 修正后再测一轮。
 *       1000Hz 正弦的注入与同步解调在 40kHz 中断完成 (1kHz 循环里会
 *       退化成直流), 末轮由 Z² = R² + (ωL)² 解出 L。
 * =========================================================== */
#define L_CAL_CYCLES (CAL_LOOP_HZ / 2U) /* 电感: 每轮正弦注入时长 0.5s */
/* 步长按"电压量程"派生, 不写死绝对值 —— 这样同一套代码能同时适配:
 *   低阻航模 (0.03Ω): calib_voltage=1.0V -> 步长 0.05V, 每步约 1.7A
 *   高阻云台 (10Ω)  : calib_voltage=3.0V -> 步长 0.15V, 每步约 15mA
 * 两者都在约 20 步内扫完量程, 两点法的电流差也都够用。 */
/* Δu2: 电阻升压步长 (V) */
#define CALIB_R_STEP_V (Config.calib_voltage * 0.05f)
#define R_CAL_WINDOW (CAL_LOOP_HZ / 5U)		     /* 每个工作点停留 200ms */
#define CALIB_L_MIN_V (Config.calib_voltage * 0.05f) /* 电感注入幅值下限 (V) */

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

/* ===== 转动惯量 J 标定 =====
 * 交替给 +I 和 −I, 两次都在正转, 故摩擦方向相同:
 *   J·2π·a+ = Kt·I − Tf ;  J·2π·a− = −Kt·I − Tf
 *   => J = Kt·I / (π·(a+ − a−))   (Tf 自动消掉)              */
#define J_CAL_IQ (Config.calib_current * 0.5f) /* 测试电流: 标定电流的一半 */
#define J_CAL_SAMPLES (300U)		       /* 每段采样点数 (1kHz -> 0.3s) */
#define J_SPINUP_RPS (1.0f) /* 起转目标 (rps), 保证两段都在正转 */
static float j_sum_t, j_sum_v, j_sum_tv, j_sum_tt;
static uint32_t j_cnt;
static float j_slope_acc, j_slope_dec;
static float j_vel_limit_save; /* 标定期间需放开速度限幅 */

/* 最小二乘拟合 vel(t) 斜率, 单位 rps/s */
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

/* 电感: 注入幅值自适应 */
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

	/* 磁链累加器必须每次标定都清零: flux_vq_sum/flux_vd_sum 若跨标定残留,
     * 会与本次累加混在一起, 使 Vq1 偏大(实测 0.2985 -> 0.6011 = 加上了
     * 上次 MEAS2 的残留), 进而把 lambda 算成 2.4e-3、Kt 算成 0.0507。 */
	flux_v_sum = flux_i_sum = flux_vq_sum = flux_vd_sum = 0.0f;
	flux_vel_sum = 0.0f;
	flux_cnt = 0;

	/* 回显实际生效的参数, 便于确认配置是否已下发 */
	printf("calib: cur=%.2fA vol=%.2fV step=%.3fV R0=%.4fOhm\n",
	       Config.calib_current, Config.calib_voltage, CALIB_R_STEP_V,
	       Config.phase_resistance);

	mCalibStep = CS_MOTOR_R_START;
}

void calibration_end(void)
{
	FOC.calib_mode = false;
	FOC.calib_vinj_mode = false;
	foc_disarm();
	mCalibStep = CS_NULL;
}

void calibration_loop(void)
{
	float time = (float)loop_count / (float)CAL_LOOP_HZ;

	const float ia_meas = (FOC.Ia - FOC.Ia_midpoint) * FOC_CURRENT_SCALE;

	float voltage_calc = Config.phase_resistance * Config.calib_current;
	float drag_v = voltage_calc;
	if (drag_v < 0.2f)
		drag_v = 0.2f;
	if (drag_v > (Config.calib_voltage * 0.9f))
		drag_v = Config.calib_voltage * 0.9f;

	float calib_phase_vel = 12.0f * M_PI;
	float total_calib_cycles = 14.0f;

	switch (mCalibStep) {
	case CS_MOTOR_R_START:
		loop_count = 0;
		r_volt = CALIB_R_STEP_V; /* ures = Δu2 */
		r_i1 = 0.0f;		 /* Ires1 */
		r_isum = 0.0f;
		r_icnt = 0;
		r_s1 = 0; /* 升压次数 */
		FOC.calib_mode = false;
		FOC.calib_vinj_mode = false;
		foc_arm();
		mCalibStep = CS_MOTOR_R_LOOP;
		break;

	case CS_MOTOR_R_LOOP: {
		FOC_voltage(r_volt, 0, 0); /* d 轴注入恒定电压 (开环) */

		/* 200ms 窗口平均作为该工作点的稳态电流 */
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
				/* 未达阈值: 记录本点, 升压继续 */
				r_i1 = Is;
				r_volt += CALIB_R_STEP_V;
				r_s1++;
			} else {
				/* 达阈值: Rs = Δu2/(Ires2-Ires1), 只依赖电压增量, 非线性抵消 */
				float di = Is - r_i1;
				if (fabsf(di) > 1e-4f) {
					Config.phase_resistance =
					    CALIB_R_STEP_V / di;
				}
				printf("res: %.4f Ohm (ures=%.3f I1=%.4f "
				       "I2=%.4f steps=%u)\n",
				       Config.phase_resistance, r_volt, r_i1,
				       Is, (unsigned)r_s1);
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

		/* 自适应幅值: |Z|>=R, 故 u_amp = I_target×R 保证不过流 */
		l_v_amp = Config.calib_current * Config.phase_resistance;
		if (l_v_amp < CALIB_L_MIN_V)
			l_v_amp = CALIB_L_MIN_V;

		if ((l_v_dc + l_v_amp) > Config.calib_voltage * 0.9f) {
			l_v_amp = (Config.calib_voltage * 0.9f) - l_v_dc;
			if (l_v_amp < CALIB_L_MIN_V)
				l_v_amp = CALIB_L_MIN_V;
		}

		/* 1000Hz 正弦需 40kHz 中断产生, 1kHz 循环里会退化成直流 */
		FOC.vinj_omega =
		    2.0f * M_PI * Config.current_ctrl_bw; /* 1000 Hz */
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
		/* 丢弃起始瞬态后开始解调累加 */
		if (!FOC.vinj_acc && (loop_count >= 100U))
			FOC.vinj_acc = true;
		if (loop_count >= L_CAL_CYCLES) {
			FOC.vinj_acc = false;
			mCalibStep =
			    CS_MOTOR_L_END; /* 注入保持, 由 L_END 决定是否再测一轮 */
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

		/* ---- 探测轮: 用实测 |Z| 把幅值修正到刚好达到目标电流, 再测一轮 ---- */
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
			/* 预跑: 先空转一圈让转子牵入同步, 避免起动滑行期使圈数偏少 */
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
		FOC.calib_id = 0;
		FOC.calib_iq = Config.calib_current * FLUX_IQ_GAIN;
		FOC.calib_phase_vel = 0;
		FOC.calib_angle = 0;
		FOC.calib_mode = true;
		loop_count = 0;
		mCalibStep = CS_FLUX_RAMP1;
		break;

	case CS_FLUX_RAMP1:
		/* 从 0 缓缓升到 FLUX_SPEED1, 让转子平稳同步 */
		FOC.calib_phase_vel +=
		    FLUX_SPEED1 / (FLUX_SETTLE_TIME * (float)CAL_LOOP_HZ);
		if (FOC.calib_phase_vel > FLUX_SPEED1)
			FOC.calib_phase_vel = FLUX_SPEED1;
		FOC.calib_angle += FOC.calib_phase_vel / (float)CAL_LOOP_HZ;
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
		FOC.calib_angle += FOC.calib_phase_vel / (float)CAL_LOOP_HZ;
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
					       FLUX_IQ_GAIN * FLUX_RTEST_RATIO;
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
		FOC.calib_angle += FOC.calib_phase_vel / (float)CAL_LOOP_HZ;
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

			FOC.calib_iq = Config.calib_current * FLUX_IQ_GAIN;
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
		FOC.calib_angle += FOC.calib_phase_vel / (float)CAL_LOOP_HZ;
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
		pwm_output_low_sides();

		if (!flux_sat1 && !flux_sat2 && (flux_i1 > 0.05f) &&
		    (flux_i2 > 0.05f)) {
			/* y = R·i + ω·λ (把 ωLi 的平方项扣掉, 两个转速两个方程解 R 和 λ) */
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

			/* 两个转速点的实测电压/电流/转速 (vel 应 = w_e/(2π·p)) */
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
				/* 两点法(仅用 Vq): 同时消掉 R·Iq 与任何恒定电压偏置 */
				float lvp = ((flux_vq_b - flux_vq_a) -
					     rr * (flux_i2 - flux_i1)) /
					    (FLUX_SPEED2 - FLUX_SPEED1);
				/* 用 |ΔV| 而非 Δ|V|: 两者近乎垂直时模差会严重低估 */
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
			/* 防御: 极对数异常时不继续, 置无效并退出标定态 (否则会停在 CALIBRATION 不动) */
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
		Controller.input_current =
		    0.0f; /* CURRENT_RAMP 以 input_current 为目标 */
		Controller.cur_setpoint = 0.0f;
		FOC.calib_mode = false;
		FOC.calib_vinj_mode = false;
		FOC.calib_id = 0.0f; /* 清掉磁链标定残留的 7A 拖动电流 */
		FOC.calib_iq = 0.0f;
		foc_arm();
		/* 电流模式下 iq_set 会被 (velocity_limit - vel)*vel_p_gain 夹住,
         * 标定需要 2A 加速度电流, 故临时放开速度限幅 */
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
		/* 直接用电流环 + 编码器角度驱动, 绕开 Controller 的指令路径
         * (实测 CONTROLLER_loop 的 iq_set 送不到 FOC.Iq_set) */
		float iq =
		    ((mCalibStep == CS_MOTOR_J_DEC) ? -J_CAL_IQ : J_CAL_IQ);
		/* CALIBRATION 状态下 foc_adc_callback 只执行 calib_mode 分支
         * (stage 非 RUN/ANTICOGGING 时在普通电流环之前就 return 了),
         * 所以必须走 calib_mode; 但角度/角速度改用编码器实测值,
         * 电机才能自由加速 —— 这正是测转动惯量所需的。 */
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

		/* DEC 段接近零点就提前收工。
         * 因 |a-|/a+ = (KtI+Tf)/(KtI-Tf) 恒大于 1 (实测航模 2.35 倍),
         * 固定 0.3s 必然冲过零点反转: 摩擦方向随之翻转, 既造成异响/失步,
         * 也破坏 J = Kt·I/(π(a+ - a-)) "两段同向" 的前提。
         * 保留 30 点以上再拟合, 避免样本过少。 */
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
			J = kt * J_CAL_IQ / (3.14159265f * da);
		printf("J: a+=%.3f a-=%.3f rps/s  Kt=%.5f -> J=%.3e kg.m2\n",
		       j_slope_acc, j_slope_dec, kt, J);
		if ((J > 1e-7f) && (J < 1e-1f)) {
			/* 由 J 反算速度环初值: Kp = 4π·ζ·ωbw·J/Kt, Ki = 2π·J·ωbw²/Kt
             * 取 ζ=1, 带宽 10Hz (电流环 1000Hz 的 1/100) */
			const float wbw = 2.0f * 3.14159265f * 10.0f;
			printf(
			    "J: 建议速度环 Kp=%.2f Ki=%.1f (zeta=1, bw=10Hz)\n",
			    4.0f * 3.14159265f * wbw * J / kt,
			    2.0f * 3.14159265f * J * wbw * wbw / kt);
		}
		mCalibStep = CS_REPORT_OFFSET_LUT;
	} break;

	/* 收尾: 停留一段时间供上位机读回偏移表, 然后结束校准 */
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

/* 齿槽标定 (照搬 rare_foc): 位置环逐点步进测保持电流, 正反转平均, 表按机械角索引 */
#define CG_STEP_MS 50U	 /* 每步保持时间 (ms); 25ms 时 56/84 阶表噪声大 */
#define CG_GAP_STEPS 100 /* 起始/换向空走步数 */
#define CG_LOOP_DIV ((CG_STEP_MS * CAL_LOOP_HZ) / 1000U) /* 1kHz -> 50 */

static int cg_number;	     /* 已走步数 */
static uint32_t cg_loop_cnt; /* 25ms 分频计数 */
static uint32_t cg_total_ticks;
static bool cg_valid;
static bool cg_running;
static int16_t cg_tmp[COGGING_MAP_NUM];
static float cg_pos_p_save;

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

/* 单圈机械角 -> 补偿表下标 (0 ~ 255) */
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

/* 采样一次静止保持电流, 按倍率转成 int16 */
static int16_t cg_sample(void)
{
	int32_t raw = (int32_t)(FOC.Iq_filt * COGGING_SCALE);
	if (raw > 32767)
		raw = 32767;
	if (raw < -32768)
		raw = -32768;
	return (int16_t)raw;
}

void anticogging_start(void)
{
	/* 先存档, 任何路径退出都能在 anticogging_end 里恢复 */
	cg_pos_p_save = Config.pos_p_gain;

	if (!Config.calib_valid || (Config.pole_pairs <= 0)) {
		printf("cogging: calibration required first\n");
		cg_running = false;
		return;
	}

	foc_arm();
	FOC.calib_mode = false;
	FOC.calib_vinj_mode = false;

	/* 位置模式: 逐点步进定位 (照搬 rare_foc), 保持电流就是位置相关转矩 */
	Controller.ctrl_mode = CONTROL_MODE_POSITION_FILTER;
	Controller.target_reached = false;
	CONTROLLER_reset(); /* input_position 对齐当前实际机械位置 */

	/* 逐步定位需要更大的位置 P 才能克服静摩擦; 速度环保持 0.03/1.5 稳定.
     * 原值 5.0 时每步只给 0.12rps / 0.03A, 转子根本不动, 表全落在同一个 bin. */
	Config.pos_p_gain = 50.0f;

	/* 标定期间旧表失效, 避免运行时补偿污染采样 */
	Config.cogging_map_valid = 0;
	memset(Config.cogging_map, 0, sizeof(Config.cogging_map));

	cg_number = 0;
	cg_loop_cnt = 0;
	cg_total_ticks = 0;
	cg_valid = false;
	cg_running = true;
}

bool anticogging_loop(void)
{
	if (!cg_running)
		return true;

	/* 总超时: 正常约 712 步 × 25ms ≈ 18s, 60s 足够 */
	if (++cg_total_ticks > (uint32_t)(60.0f * (float)CAL_LOOP_HZ)) {
		cg_valid = false;
		cg_running = false;
		return true;
	}
	if ((cg_total_ticks % CAL_LOOP_HZ) == 0U) {
		printf("cogging: step=%d mode=%d stage=%d armed=%d inpos=%.3f "
		       "act=%.3f "
		       "pg=%.1f vg=%.3f iqset=%.3f Iq=%.2f\n",
		       cg_number, Controller.ctrl_mode, (int)foc_get_stage(),
		       (int)FOC.is_armed, (double)Controller.input_position,
		       (double)cg_pos(), (double)Config.pos_p_gain,
		       (double)Config.vel_p_gain, (double)FOC.Iq_set,
		       (double)FOC.Iq_filt);
	}

	/* 1kHz -> 25ms 一步 */
	if (++cg_loop_cnt < CG_LOOP_DIV)
		return false;
	cg_loop_cnt = 0;

	cg_number++;

	if (cg_number <= CG_GAP_STEPS) {
		/* 正转空走, 让位置环/速度环稳定 */
		cg_step(+1.0f);

	} else if (cg_number <= (CG_GAP_STEPS + (int)COGGING_MAP_NUM)) {
		/* 正转采样: 记录保持当前位置所需的电流 */
		int16_t index = (int16_t)cg_index(cg_pos());
		Config.cogging_map[index] = cg_sample();
		cg_step(+1.0f);

	} else if (cg_number <=
		   (CG_GAP_STEPS + (int)COGGING_MAP_NUM + CG_GAP_STEPS)) {
		/* 反转空走 */
		cg_step(-1.0f);

	} else if (cg_number <= (CG_GAP_STEPS + (int)COGGING_MAP_NUM +
				 CG_GAP_STEPS + (int)COGGING_MAP_NUM)) {
		/* 反转采样: 与正转结果取平均, 抵消静摩擦/滞回方向性 */
		int16_t index = (int16_t)cg_index(cg_pos());
		Config.cogging_map[index] =
		    (Config.cogging_map[index] + cg_sample()) / 2;
		cg_step(-1.0f);

	} else {
		/* 去掉直流分量 (平均摩擦/负载), 只保留位置相关转矩 */
		int32_t sum = 0;
		for (uint32_t i = 0; i < COGGING_MAP_NUM; i++)
			sum += Config.cogging_map[i];
		int32_t mean = sum / (int32_t)COGGING_MAP_NUM;
		for (uint32_t i = 0; i < COGGING_MAP_NUM; i++) {
			int32_t v = (int32_t)Config.cogging_map[i] - mean;
			if (v > 32767)
				v = 32767;
			if (v < -32768)
				v = -32768;
			Config.cogging_map[i] = (int16_t)v;
		}
		/* 5-bin 圆周滑动平均: 保留 14/28/42 阶, 压掉 56/84 阶的静摩擦/位置噪声.
         * 84 阶是齿槽, 但静态步进在 2rps 附近的高阶相位不可靠, 宁可少补. */
		for (uint32_t i = 0; i < COGGING_MAP_NUM; i++) {
			int32_t s = 0;
			for (int k = -2; k <= 2; k++) {
				int j = (int)i + k;
				while (j < 0)
					j += (int)COGGING_MAP_NUM;
				while (j >= (int)COGGING_MAP_NUM)
					j -= (int)COGGING_MAP_NUM;
				s += Config.cogging_map[j];
			}
			cg_tmp[i] = (int16_t)(s / 5);
		}
		for (uint32_t i = 0; i < COGGING_MAP_NUM; i++)
			Config.cogging_map[i] = cg_tmp[i];
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

	/* 必须在 CONFIG_save_to_flash() 之前恢复位置环增益, 否则 50.0 会写进 Flash */
	Config.pos_p_gain = cg_pos_p_save;

	if (cg_valid) {
		Config.cogging_map_valid = 1;
		Config.anticogging_enable = 1;
		CONFIG_save_to_flash();
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
	} else {
		Config.cogging_map_valid = 0;
		printf("cogging: aborted, invalid (step=%d)\n", cg_number);
	}

	Controller.input_velocity = 0.0f;
	Controller.input_velocity_buffer = 0.0f;
}
