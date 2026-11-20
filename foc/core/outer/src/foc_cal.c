#include <string.h>
#include <float.h>
#include "foc_cal.h"
#include "encoder.h"
#include "foc_task.h"

#define CAL_LOOP_HZ (1000U)
#define CAL_SOFT_OC_TICKS (5U)
/* 1kHz 标定循环 -> 每机械圈 2240 点 */
#define SAMPLES_PER_PPAIR (160U)
#define LUT_WINDOW (128U)	   /* offset LUT 滑动平均窗口 */
#define MAX_MOTOR_POLE_PAIRS (20U) /* 最大支持的极对数 */
#define DIR_PP_MIN_COUNTS (500)

/* 磁链: 开环拖动, 两转速点矢量差分消掉 R 项 */
#define FLUX_SPEED1 (2.0f * M_PI * 10.0f) /* 低速点: 80Hz 电时 4.5A 拉不住, 丢步 */
/* 高速点: 30Hz 时 0.7A 会电压饱和 */
#define FLUX_SPEED2 (2.0f * M_PI * 14.0f) /* 高速点: 回到不丢步范围 */
#define FLUX_SETTLE_TIME (1.5f) /* 每个转速稳定时间 (s) */
/* 每点采样点数 (1.5s, 低速点也够转一圈) */
#define FLUX_MEAS_SAMPLES (1500U)
#define FLUX_MAX_SPEED_ERROR (0.25f)
#define FLUX_MIN_WB (0.00005f)
#define FLUX_MAX_WB (1.0f)

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
#define R_CAL_MIN_DELTA_A (0.005f)
#define R_CAL_SINGLE_POINT_MIN_A (0.5f)
#define R_CAL_MIN_OHM (0.001f)
/* 5210 的 Z 约 0.047Ω。20% 量程注入叠加直流偏置会接近软限流。 */
#define CALIB_L_MIN_V_RATIO (0.15f)
#define CALIB_L_MAX_PROBE_GAIN (1.5f)
#define CALIB_L_RAMP_TICKS (CAL_LOOP_HZ / 40U)
#define CALIB_L_SETTLE_TICKS (CAL_LOOP_HZ / 10U)
#define CALIB_L_MIN_RESPONSE_A (0.005f)
#define L_CAL_MIN_H (1e-9f)
/* L 注入频率: 固定值, 不再跟 current_ctrl_bw 走.
 * 原来 vinj_omega = 2π·bw, 于是"降带宽保稳定"会连注入频率一起降,
 * Z/R 掉到 1 附近, L 就测不准了. 现在解耦: 环带宽可以按电机阻抗随便定,
 * 注入始终在 1kHz (5210: ωL=0.063Ω vs R=0.042Ω, Z/R≈1.8, 够用). */
#define CALIB_L_INJ_HZ (1000.0f)

static int encoder_error_arr[MAX_MOTOR_POLE_PAIRS * SAMPLES_PER_PPAIR];

enum {
	CAL_VALID_R = 1U << 0,
	CAL_VALID_L = 1U << 1,
	CAL_VALID_DIR_PP = 1U << 2,
	CAL_VALID_ENCODER = 1U << 3,
	CAL_VALID_FLUX = 1U << 4,
	CAL_REQUIRED = CAL_VALID_R | CAL_VALID_L | CAL_VALID_DIR_PP |
		       CAL_VALID_ENCODER | CAL_VALID_FLUX,
};

typedef struct {
	float v_mag;
	float i_mag;
	float velocity;
	float vd;
	float vq;
} flux_point_t;

typedef struct {
	flux_point_t sum;
	uint32_t count;
} flux_window_t;

typedef struct {
	flux_window_t window;
	flux_point_t low_speed;
	flux_point_t high_speed;
	float resistance_estimate;
	bool saturated;
} flux_calibration_t;

typedef struct {
	tCalibStep step;
	uint32_t ticks;
	uint32_t valid_stages;
	uint32_t overcurrent_ticks;
	float drive_angle;
	int16_t sample_index;
	bool dir_runin_done;
	bool exiting;
	flux_calibration_t flux;
} calibration_state_t;

static calibration_state_t cal;

/* J 标定: 加速->自由滑行, 摩擦相减: J = Kt·I/(2π·(a+ - a-)) */
#define J_CAL_SAMPLES (150U)		       /* 每段采样点数 (1kHz -> 0.15s) */
#define J_SPINUP_RPS (2.0f) /* 起转目标 (rps): 太高会让 ACC 段冲到过流 */
#define J_SPINUP_MAX_RPS 20.0f /* 加速段转速上限: 超过就放弃, 防过流 */
static float j_sum_t, j_sum_v, j_sum_tv, j_sum_tt;
static uint32_t j_cnt;
static float j_slope_acc, j_slope_dec;

/* 最小二乘拟合 vel(t) 斜率 (rps/s) */
static float j_slope(void)
{
	float n = (float)j_cnt;
	float den = n * j_sum_tt - j_sum_t * j_sum_t;
	if (fabsf(den) < 1e-6f)
		return 0.0f;
	return (n * j_sum_tv - j_sum_t * j_sum_v) / den * (float)CAL_LOOP_HZ;
}

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

static void cal_stop_drive(void)
{
	foc_disarm();
	FOC.calib_mode = false;
	FOC.calib_vinj_mode = false;
	FOC.calib_adv_angle = false;
	FOC.vinj_acc = false;
}

static void cal_request_idle(void)
{
	cal.exiting = true;
	foc_request_stage(IDLE);
}

static void cal_reject(const char *reason)
{
	cal_stop_drive();
	cal_request_idle();
	printf("calib: REJECTED step=%d (%s)\n", (int)cal.step, reason);
}

void calibration_start(void)
{
	cal = (calibration_state_t){0};
	if (!FOC.adc_ready) {
		FOC.fault_code |= FOC_FAULT_ADC;
		FOC.statusword.errors.adc_fault = 1;
		cal_reject("ADC not ready");
		return;
	}
	if (FOC.fault_code != FOC_FAULT_NONE) {
		cal_reject("fault latched");
		return;
	}
	if (!isfinite(Config.calib_current) || Config.calib_current <= 0.0f ||
	    !isfinite(Config.calib_voltage) || Config.calib_voltage <= 0.0f) {
		cal_reject("invalid test current or voltage");
		return;
	}

	r_step = r_calib_step();
	if (!(r_step > 0.0f)) {
		cal_reject("invalid resistance voltage step");
		return;
	}

	Config.cogging_map_valid = 0;
	Config.encoder_dir = +1;
	Config.calib_valid = false;
	FOC.calib_mode = false;
	FOC.calib_vinj_mode = false;
	FOC.calib_adv_angle = false;
	FOC.vinj_acc = false;

	memset(encoder_error_arr, 0, sizeof(encoder_error_arr));

	/* 回显实际生效参数, 便于确认配置已下发 */
	printf("calib: cur=%.2fA vol=%.2fV step=%.3fV R0=%.4fOhm "
	       "drag_min=%.2fV flux_iq=%.2fA j_iq=%.2fA "
	       "dtcomp_in_voltage=%d\n",
	       Config.calib_current, Config.calib_voltage, (double)r_step,
	       Config.phase_resistance, (double)Config.calib_drag_v_min,
	       (double)(Config.calib_current * Config.calib_flux_iq_gain),
	       (double)(Config.calib_current * Config.calib_j_iq_gain),
	       (int)DT_COMP_IN_VOLTAGE);

	cal.step = CS_MOTOR_R_START;
}

void calibration_end(void)
{
	cal_stop_drive();
	if (cal.step != CS_NULL && !cal.exiting)
		printf("calib: ABORTED at step %d\n", (int)cal.step);
	cal = (calibration_state_t){0};
}

int calibration_step(void)
{
	return (int)cal.step;
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

static void cal_finish_resistance(float current)
{
	float resistance = 0.0f;
	float di_avg = current - r_i1;

	if (r_di_cnt > 0U) {
		di_avg = r_di_sum / (float)r_di_cnt;
		if (!isfinite(di_avg) || di_avg <= R_CAL_MIN_DELTA_A) {
			cal_reject("R: insufficient differential current");
			return;
		}
		resistance = r_step / di_avg;
	} else if (current >= R_CAL_SINGLE_POINT_MIN_A) {
		resistance = r_volt / current;
	}
	if (!isfinite(resistance) || resistance < R_CAL_MIN_OHM) {
		cal_reject("R: no usable resistance measurement");
		return;
	}
	if (r_di_cnt == 0U)
		printf("res: WARN single point only, R is a rough V/I guess\n");

	Config.phase_resistance = resistance;
	cal.valid_stages |= CAL_VALID_R;
	printf("res: %.4f Ohm (ures=%.3f I1=%.4f I2=%.4f steps=%u "
	       "di_avg=%.5f Voff=%.3fV dtcomp=%d)\n",
	       resistance, r_volt, r_i1, current, (unsigned)r_s1,
	       di_avg, r_volt - current * resistance, (int)DT_COMP_IN_VOLTAGE);
	cal.step = CS_MOTOR_R_END;
}

/* R/L 标定 (CS_MOTOR_R_* ~ CS_MOTOR_L_END) */
static void cal_rl_step(float ia_meas)
{
	switch (cal.step) {
	case CS_MOTOR_R_START:
		cal.ticks = 0;
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
		if (!FOC.is_armed) {
			cal_reject("R: drive could not arm");
			break;
		}
		cal.step = CS_MOTOR_R_LOOP;
		break;

	case CS_MOTOR_R_LOOP: {
		r_isum += ia_meas;
		r_icnt++;
		if (cal.ticks >= R_CAL_WINDOW) {
			float Is = fabsf(r_isum / (float)r_icnt);

			if (!isfinite(Is)) {
				cal_reject("R: non-finite current response");
				break;
			}
			r_isum = 0.0f;
			r_icnt = 0;
			cal.ticks = 0;
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
				cal_finish_resistance(Is);
			}
		}
	} break;

	case CS_MOTOR_R_END:
		cal.ticks = 0;
		cal.step = CS_MOTOR_L_START;
		break;

	case CS_MOTOR_L_START: {
		float voltage_limit = Config.calib_voltage * 0.9f;
		float min_v = Config.calib_voltage * CALIB_L_MIN_V_RATIO;
		float bias = Config.calib_current * Config.phase_resistance;

		l_v_dc = fminf(bias, voltage_limit - min_v);
		l_pass = 0;
		l_v_amp = fminf(fmaxf(bias, min_v), voltage_limit - l_v_dc);
		l_v_amp_start = l_v_amp;
		FOC.vinj_omega = 2.0f * M_PI * CALIB_L_INJ_HZ;
		FOC.vinj_amp = l_v_amp;
		FOC.vinj_dc = l_v_dc;
		FOC.inj_phase = 0;
		FOC.vinj_sum_sin = 0;
		FOC.vinj_sum_cos = 0;
		FOC.vinj_samples = 0;
		FOC.vinj_acc = false;
		cal.ticks = 0;
		FOC.calib_vinj_mode = true;
		foc_set_resistance_voltage(0.0f);
		cal.step = CS_MOTOR_L_LOOP;
	} break;

	case CS_MOTOR_L_LOOP:
		/* 先升幅再等稳态，解调只累加恒幅段。 */
		if (cal.ticks <= CALIB_L_RAMP_TICKS) {
			float fraction = (float)cal.ticks / CALIB_L_RAMP_TICKS;
			float delta = l_v_amp - l_v_amp_start;

			FOC.vinj_amp = l_v_amp_start + delta * fraction;
		}
		if (!FOC.vinj_acc && cal.ticks >= CALIB_L_SETTLE_TICKS)
			FOC.vinj_acc = true;
		if (cal.ticks >= L_CAL_CYCLES) {
			FOC.vinj_acc = false;
			cal.step = CS_MOTOR_L_END;
		}
		break;

	case CS_MOTOR_L_END: {
		float omega = 2.0f * M_PI * CALIB_L_INJ_HZ;
		float min_v = Config.calib_voltage * CALIB_L_MIN_V_RATIO;
		int valid_samples = (int)FOC.vinj_samples;
		float I_amp =
		    (valid_samples > 0)
			? (2.0f / (float)valid_samples) *
			      sqrtf(SQ(FOC.vinj_sum_sin) + SQ(FOC.vinj_sum_cos))
			: 0.0f;
		float Z, resistance, inductance;

		if (!isfinite(I_amp) || I_amp <= CALIB_L_MIN_RESPONSE_A) {
			cal_reject("L: no usable current response");
			break;
		}
		Z = l_v_amp / I_amp;
		if (!isfinite(Z) || Z <= 0.0f) {
			cal_reject("L: invalid impedance");
			break;
		}

		if (l_pass == 0) {
			float new_amp = Config.calib_current * Z;
			float voltage_headroom =
			    Config.calib_voltage * 0.9f - l_v_dc;
			float max_amp = fminf(voltage_headroom,
					      l_v_amp * CALIB_L_MAX_PROBE_GAIN);

			/* 小信号受死区影响，避免按探测阻抗一次放大数倍。 */
			new_amp = fminf(fmaxf(new_amp, min_v), max_amp);
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
			cal.ticks = 0;
			cal.step = CS_MOTOR_L_LOOP;
			break;
		}

		resistance = Config.phase_resistance;
		if (Z <= resistance) {
			cal_reject("L: impedance must exceed phase resistance");
			printf("ind: Z=%.5f Ohm R=%.5f Ohm Iamp=%.3f A\n",
			       Z, resistance, I_amp);
			break;
		}
		inductance = sqrtf((Z - resistance) * (Z + resistance)) / omega;
		if (!isfinite(inductance) || inductance <= L_CAL_MIN_H) {
			cal_reject("L: invalid inductance");
			break;
		}
		Config.phase_inductance = inductance;
		cal.valid_stages |= CAL_VALID_L;
		FOC.calib_vinj_mode = false;
		FOC_update_current_ctrl_gain(Config.current_ctrl_bw);
		printf("ind: %.6f H (Iamp=%.3f Z=%.3f Vdc=%.3f Vamp=%.3f)\n",
		       Config.phase_inductance, I_amp, Z, l_v_dc, l_v_amp);
		cal.drive_angle = 0;
		cal.ticks = 0;
		cal.step = CS_DIR_PP_START;
	} break;

	default:
		break;
	}
}

/* 方向/极对数标定 (CS_DIR_PP_* ) */
static void cal_dir_pp_step(float time, float calib_phase_vel,
			    float total_calib_cycles)
{
	switch (cal.step) {
	case CS_DIR_PP_START:
		/* 2s 内把拖动电流从 0 拉满, 角度固定 0 先让转子对齐
		 * (与 5ed4143 已验证可用的版本一致, 不要改成别的轴向) */
		calib_drag_if(Config.calib_current * (time / 2.0f), 0.0f, 0.0f);
		if (time >= 2.0f) {
			taskENTER_CRITICAL();
			encoder.shadow_count = 0;
			taskEXIT_CRITICAL();
			cal.dir_runin_done = false;
			cal.ticks = 0;
			cal.drive_angle = 0;
			cal.step = CS_DIR_PP_LOOP;
		}
		break;

	case CS_DIR_PP_LOOP:
		cal.drive_angle += calib_phase_vel / (float)CAL_LOOP_HZ;
		calib_drag_if(Config.calib_current, cal.drive_angle,
			      calib_phase_vel);
		if (!cal.dir_runin_done) {
			if (cal.drive_angle >= total_calib_cycles * M_2PI) {
				taskENTER_CRITICAL();
				encoder.shadow_count = 0;
				taskEXIT_CRITICAL();
				cal.dir_runin_done = true;
			}
		} else if (cal.drive_angle >=
			   2.0f * total_calib_cycles * M_2PI) {
			FOC.calib_mode = false;
			cal.step = CS_DIR_PP_END;
		}
		break;

	case CS_DIR_PP_END: {
		int64_t diff;
		float mech_revs, pp_raw;
		int pole_pairs;

		taskENTER_CRITICAL();
		diff = encoder.shadow_count;
		taskEXIT_CRITICAL();
		if (diff > -DIR_PP_MIN_COUNTS && diff < DIR_PP_MIN_COUNTS) {
			cal_reject("dir_pp: no usable encoder motion");
			break;
		}
		mech_revs = fabsf((float)diff / ENCODER_CPR_F);
		pp_raw = total_calib_cycles / mech_revs;
		pole_pairs = (int)roundf(pp_raw);
		if (pole_pairs <= 0 || pole_pairs > (int)MAX_MOTOR_POLE_PAIRS) {
			cal_reject("dir_pp: pole-pair count outside range");
			printf("dir_pp: diff=%lld pp_raw=%.4f\n",
			       (long long)diff, pp_raw);
			break;
		}
		Config.encoder_dir = (diff > 0) ? 1 : -1;
		Config.pole_pairs = pole_pairs;
		cal.valid_stages |= CAL_VALID_DIR_PP;
		printf("dir_pp: diff=%lld mech_revs=%.5f pp_raw=%.4f -> %d\n",
		       (long long)diff, mech_revs, pp_raw, pole_pairs);
		cal.step = CS_ENCODER_START;
	} break;

	default:
		break;
	}
}

/* 编码器零位/偏移表标定 (CS_ENCODER_* ) */
static void cal_encoder_step(void)
{
	switch (cal.step) {
	case CS_ENCODER_START:
		cal.drive_angle = 0;
		cal.ticks = 0;
		cal.sample_index = 0;
		cal.step = CS_ENCODER_CW_LOOP;
		break;

	case CS_ENCODER_CW_LOOP:
		if (cal.sample_index <
		    (int32_t)(Config.pole_pairs * SAMPLES_PER_PPAIR)) {
			float mech_phase = (cal.drive_angle + M_PI / 2.0f) /
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
			encoder_error_arr[cal.sample_index] = error;
			cal.sample_index++;
			cal.drive_angle += M_2PI / (float)SAMPLES_PER_PPAIR;
		} else {
			cal.drive_angle -= M_2PI / (float)SAMPLES_PER_PPAIR;
			cal.ticks = 0;
			cal.sample_index--;
			cal.step = CS_ENCODER_CCW_LOOP;
		}
		calib_drag_if(Config.calib_current, cal.drive_angle,
			      (M_2PI / (float)SAMPLES_PER_PPAIR) *
				  (float)CAL_LOOP_HZ);
		break;

	case CS_ENCODER_CCW_LOOP:
		if (cal.sample_index >= 0) {
			float mech_phase = (cal.drive_angle + M_PI / 2.0f) /
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
			encoder_error_arr[cal.sample_index] =
			    (encoder_error_arr[cal.sample_index] + error) / 2;
			cal.sample_index--;
			cal.drive_angle -= M_2PI / (float)SAMPLES_PER_PPAIR;
			calib_drag_if(
			    Config.calib_current, cal.drive_angle,
			    -(M_2PI / (float)SAMPLES_PER_PPAIR) *
				(float)CAL_LOOP_HZ);
		} else {
			FOC.calib_mode = false;
			cal.sample_index = 0;
			cal.ticks = 0;
			cal.step = CS_ENCODER_END;
		}
		break;

	case CS_ENCODER_END: {
		if ((Config.pole_pairs <= 0) ||
		    (Config.pole_pairs > (int32_t)MAX_MOTOR_POLE_PAIRS)) {
			cal_reject("encoder: invalid pole-pair count");
			break;
		}
		uint32_t total_samples =
		    (uint32_t)Config.pole_pairs * SAMPLES_PER_PPAIR;
		int window = (int)LUT_WINDOW;
		if (cal.sample_index == 0) {
			int64_t sum = 0;
			for (uint32_t i = 0; i < total_samples; i++)
				sum += encoder_error_arr[i];
			Config.encoder_offset =
			    (int32_t)(sum / (int64_t)total_samples);
		}
		if (cal.sample_index < (int32_t)OFFSET_LUT_NUM) {
			int64_t window_sum = 0;
			for (int j = (-window) / 2; j < (window) / 2; j++) {
				int index = (int)((int32_t)cal.sample_index *
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
			int lut_index = (lut_offset + cal.sample_index) %
					(int)OFFSET_LUT_NUM;
			if (lut_index < 0)
				lut_index += (int)OFFSET_LUT_NUM;
			Config.offset_lut[lut_index] =
			    window_avg - Config.encoder_offset;
			cal.sample_index++;
		} else {
			cal.ticks = 0;
			cal.sample_index = 0;
			cal.valid_stages |= CAL_VALID_ENCODER;
			cal.step = CS_FLUX_START;
		}
	} break;

	default:
		break;
	}
}

static bool cal_flux_in_range(float flux)
{
	return flux > FLUX_MIN_WB && flux < FLUX_MAX_WB;
}

static void cal_finish_flux(void)
{
	const flux_calibration_t *flux_state = &cal.flux;
	const flux_point_t *low = &flux_state->low_speed;
	const flux_point_t *high = &flux_state->high_speed;
	float pole_pairs = (float)Config.pole_pairs;
	float exp1 = FLUX_SPEED1 / (M_2PI * pole_pairs);
	float exp2 = FLUX_SPEED2 / (M_2PI * pole_pairs);
	float speed_delta = FLUX_SPEED2 - FLUX_SPEED1;
	float phase_r = Config.phase_resistance;
	float phase_l = Config.phase_inductance;
	float t1, t2, y1, y2, v_i1, v_i2, den, rr, lv1, lv2, lvp;
	float dvd, dvq, dvm, lam_vec, flux;
	float lambda = NAN;
	float res = NAN;

	FOC.calib_mode = false;
	FOC.calib_adv_angle = false;
	if (!isfinite(low->velocity) || !isfinite(high->velocity) ||
	    fabsf(low->velocity - exp1) > FLUX_MAX_SPEED_ERROR * exp1 ||
	    fabsf(high->velocity - exp2) > FLUX_MAX_SPEED_ERROR * exp2) {
		cal_reject("flux: rotor speed did not track");
		printf("flux: vel %.3f/%.3f, %.3f/%.3f rps\n",
		       low->velocity, exp1, high->velocity, exp2);
		return;
	}

	if (flux_state->saturated ||
	    !(low->i_mag > 0.05f) || !(high->i_mag > 0.05f) ||
	    !isfinite(low->i_mag) || !isfinite(high->i_mag) ||
	    !isfinite(low->v_mag) || !isfinite(high->v_mag)) {
		cal_reject("flux: unusable current or voltage samples");
		return;
	}

	t1 = SQ(low->v_mag) - SQ(FLUX_SPEED1 * phase_l * low->i_mag);
	t2 = SQ(high->v_mag) - SQ(FLUX_SPEED2 * phase_l * high->i_mag);
	y1 = sqrtf(fmaxf(t1, 0.0f));
	y2 = sqrtf(fmaxf(t2, 0.0f));
	v_i1 = FLUX_SPEED2 * low->i_mag;
	v_i2 = FLUX_SPEED1 * high->i_mag;
	den = v_i1 - v_i2;
	if (fabsf(den) > FLT_EPSILON * fmaxf(v_i1, v_i2)) {
		lambda = (y2 * low->i_mag - y1 * high->i_mag) / den;
		res = (y1 - FLUX_SPEED1 * lambda) / low->i_mag;
	}

	dvd = high->vd - low->vd;
	dvq = high->vq - low->vq;
	dvm = sqrtf(SQ(dvd) + SQ(dvq));
	lam_vec = dvm / speed_delta;
	flux = cal_flux_in_range(lam_vec) ? lam_vec : lambda;
	if (!cal_flux_in_range(flux)) {
		cal_reject("flux: no usable flux linkage measurement");
		return;
	}

	printf("flux pts: V1=%.4f I1=%.3f vel1=%.3f | V2=%.4f "
	       "I2=%.3f vel2=%.3f\n",
	       low->v_mag, low->i_mag, low->velocity,
	       high->v_mag, high->i_mag, high->velocity);
	printf("flux cmd: we1=%.1f we2=%.1f rad/s (vel1=%.3f vel2=%.3f rps)\n",
	       FLUX_SPEED1, FLUX_SPEED2, exp1, exp2);
	printf("flux: R=%.3f Ohm lambda=%.6f Wb (simple %.6f)\n",
	       res, lambda, (high->v_mag - low->v_mag) / speed_delta);

	rr = phase_r;
	if (isfinite(flux_state->resistance_estimate) &&
	    flux_state->resistance_estimate > R_CAL_MIN_OHM)
		rr = flux_state->resistance_estimate;
	lv1 = (low->vq - rr * low->i_mag) / FLUX_SPEED1;
	lv2 = (high->vq - rr * high->i_mag) / FLUX_SPEED2;
	lvp = (dvq - rr * (high->i_mag - low->i_mag)) / speed_delta;
	printf("flux Vq: Vq1=%.4f Vq2=%.4f Vd1=%.4f Vd2=%.4f (R=%.4f)\n",
	       low->vq, high->vq, low->vd, high->vd, rr);
	printf("flux Vq: lam1=%.6f lam2=%.6f (单点, 差%.0f%%) 两点法=%.6f\n",
	       lv1, lv2, 100.0f * (lv1 - lv2) / ((lv1 + lv2) * 0.5f + 1e-9f),
	       lvp);
	printf("flux vec: |dV|=%.4f (模差仅%.4f) -> lambda_vec=%.6f Wb\n",
	       dvm, high->v_mag - low->v_mag, lam_vec);

	if ((res > 0.01f) && (res < 50.0f) &&
	    (fabsf(res - phase_r) < 0.2f * phase_r)) {
		Config.phase_resistance = res;
	} else {
		printf("flux: R rejected (%.4f vs %.4f)\n", res, phase_r);
	}

	Config.flux_linkage = flux;
	FOC_update_current_ctrl_gain(Config.current_ctrl_bw);
	cal.valid_stages |= CAL_VALID_FLUX;
	cal.ticks = 0;
	cal.step = CS_MOTOR_J_START;
}

static void cal_flux_begin_samples(tCalibStep step)
{
	cal.flux.window = (flux_window_t){0};
	cal.ticks = 0;
	cal.step = step;
}

static void cal_flux_accumulate(flux_window_t *window, float vq)
{
	float id = FOC.Id;
	float iq = FOC.Iq;

	window->sum.i_mag += sqrtf(SQ(id) + SQ(iq));
	window->sum.velocity += encoder.vel;
	window->sum.vq += vq;
	window->count++;
}

static flux_point_t cal_flux_average(const flux_window_t *window)
{
	float count = (float)window->count;

	return (flux_point_t){
		.v_mag = window->sum.v_mag / count,
		.i_mag = window->sum.i_mag / count,
		.velocity = window->sum.velocity / count,
		.vd = window->sum.vd / count,
		.vq = window->sum.vq / count,
	};
}

/* 磁链标定 (CS_FLUX_* ) */
static void cal_flux_step(float time)
{
	flux_calibration_t *flux = &cal.flux;
	flux_window_t *window = &flux->window;

	switch (cal.step) {
	case CS_FLUX_START:
		Config.flux_linkage = 0;
		FOC.calib_adv_angle = true;
		FOC.calib_id = 0;
		FOC.calib_iq = Config.calib_current * Config.calib_flux_iq_gain;
		FOC.calib_phase_vel = 0;
		FOC.calib_angle = 0;
		FOC.calib_mode = true;
		cal.ticks = 0;
		cal.step = CS_FLUX_RAMP1;
		break;

	case CS_FLUX_RAMP1:
		FOC.calib_phase_vel +=
		    FLUX_SPEED1 / (FLUX_SETTLE_TIME * (float)CAL_LOOP_HZ);
		if (FOC.calib_phase_vel > FLUX_SPEED1)
			FOC.calib_phase_vel = FLUX_SPEED1;
		if (FOC.calib_angle >= M_2PI)
			FOC.calib_angle -= M_2PI;
		if (time > FLUX_SETTLE_TIME)
			cal_flux_begin_samples(CS_FLUX_MEAS1);
		break;

	case CS_FLUX_MEAS1:
	case CS_FLUX_MEAS2: {
		float vd, vq;

		if (FOC.calib_angle >= M_2PI)
			FOC.calib_angle -= M_2PI;
		vd = FOC.Vd_out;
		vq = FOC.Vq_out;
		window->sum.v_mag += sqrtf(SQ(vd) + SQ(vq));
		window->sum.vd += vd;
		cal_flux_accumulate(window, vq);
		if (FOC.saturated)
			flux->saturated = true;
		if (window->count < FLUX_MEAS_SAMPLES)
			break;

		if (cal.step == CS_FLUX_MEAS1) {
			flux->low_speed = cal_flux_average(window);
			FOC.calib_iq = Config.calib_current *
				       Config.calib_flux_iq_gain *
				       Config.calib_flux_rtest_ratio;
			cal_flux_begin_samples(CS_FLUX_RTEST);
		} else {
			flux->high_speed = cal_flux_average(window);
			cal.step = CS_FLUX_END;
		}
	} break;

	case CS_FLUX_RTEST: {
		const flux_point_t *low = &flux->low_speed;
		flux_point_t reduced;
		float di;

		if (FOC.calib_angle >= M_2PI)
			FOC.calib_angle -= M_2PI;
		cal_flux_accumulate(window, FOC.Vq_out);
		if (window->count < FLUX_MEAS_SAMPLES)
			break;

		reduced = cal_flux_average(window);
		di = low->i_mag - reduced.i_mag;
		if (fabsf(di) > 0.1f)
			flux->resistance_estimate = (low->vq - reduced.vq) / di;
		printf("flux Rtest: I %.3f->%.3f A, Vq %.4f->%.4f V, "
		       "vel %.3f rps, R_est=%.4f Ohm\n",
		       low->i_mag, reduced.i_mag, low->vq, reduced.vq,
		       reduced.velocity, flux->resistance_estimate);
		FOC.calib_iq = Config.calib_current * Config.calib_flux_iq_gain;
		cal.ticks = 0;
		cal.step = CS_FLUX_RAMP2;
	} break;

	case CS_FLUX_RAMP2:
		FOC.calib_phase_vel += (FLUX_SPEED2 - FLUX_SPEED1) /
				       (FLUX_SETTLE_TIME * (float)CAL_LOOP_HZ);
		if (FOC.calib_phase_vel > FLUX_SPEED2)
			FOC.calib_phase_vel = FLUX_SPEED2;
		if (FOC.calib_angle >= M_2PI)
			FOC.calib_angle -= M_2PI;
		if (time > FLUX_SETTLE_TIME)
			cal_flux_begin_samples(CS_FLUX_MEAS2);
		break;

	case CS_FLUX_END:
		cal_finish_flux();
		break;

	default:
		break;
	}
}

/* 转动惯量 J 标定 (CS_MOTOR_J_* ) */
static void cal_inertia_step(float time)
{
	switch (cal.step) {
	case CS_MOTOR_J_START:
		FOC.calib_mode = false;
		FOC.calib_vinj_mode = false;
		FOC.calib_id = 0.0f;
		FOC.calib_iq = 0.0f;
		j_slope_acc = 0.0f;
		j_slope_dec = 0.0f;
		cal.ticks = 0;
		cal.step = CS_MOTOR_J_SPINUP;
		break;

	case CS_MOTOR_J_SPINUP:
	case CS_MOTOR_J_ACC:
	case CS_MOTOR_J_DEC: {
		float j_iq = Config.calib_current * Config.calib_j_iq_gain;
		float iq = (cal.step == CS_MOTOR_J_DEC) ? 0.0f : j_iq;

		if ((cal.step != CS_MOTOR_J_SPINUP) &&
		    (fabsf(encoder.vel) > J_SPINUP_MAX_RPS)) {
			FOC.calib_iq = 0.0f;
			printf("J: 转速 %.2f rps 超过 %.0f, 放弃本次 J\n",
			       (double)encoder.vel, (double)J_SPINUP_MAX_RPS);
			j_slope_acc = 0.0f;
			j_slope_dec = 0.0f;
			cal.step = CS_MOTOR_J_END;
			break;
		}
		FOC.calib_mode = true;
		FOC.calib_id = 0.0f;
		FOC.calib_iq = iq;
		FOC.calib_angle = encoder.phase;
		FOC.calib_phase_vel =
		    encoder.vel * M_2PI * (float)Config.pole_pairs;

		if (cal.step == CS_MOTOR_J_SPINUP) {
			if ((encoder.vel >= J_SPINUP_RPS) || (time > 3.0f)) {
				printf("J spinup: t=%.2f vel=%.3f Iq=%.3f "
				       "set=%.3f ciq=%.2f armed=%d\n",
				       (double)time, encoder.vel,
				       (double)FOC.Iq, (double)FOC.Iq_set,
				       (double)FOC.calib_iq, (int)FOC.is_armed);
				j_sum_t = j_sum_v = j_sum_tv = j_sum_tt = 0.0f;
				j_cnt = 0;
				cal.ticks = 0;
				cal.step = CS_MOTOR_J_ACC;
			}
			break;
		}

		j_sum_t += (float)j_cnt;
		j_sum_v += encoder.vel;
		j_sum_tv += (float)j_cnt * encoder.vel;
		j_sum_tt += (float)j_cnt * (float)j_cnt;
		j_cnt++;

		if ((cal.step == CS_MOTOR_J_DEC) && (j_cnt > 30U) &&
		    (encoder.vel < 0.3f)) {
			j_slope_dec = j_slope();
			cal.step = CS_MOTOR_J_END;
			break;
		}

		if (j_cnt >= J_CAL_SAMPLES) {
			if (cal.step == CS_MOTOR_J_ACC) {
				j_slope_acc = j_slope();
				j_sum_t = j_sum_v = j_sum_tv = j_sum_tt = 0.0f;
				j_cnt = 0;
				cal.step = CS_MOTOR_J_DEC;
			} else {
				j_slope_dec = j_slope();
				cal.step = CS_MOTOR_J_END;
			}
		}
	} break;

	case CS_MOTOR_J_END: {
		float da = j_slope_acc - j_slope_dec;
		float kt = 1.5f * (float)Config.pole_pairs * Config.flux_linkage;
		float J = 0.0f;

		cal_stop_drive();
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
		cal.step = CS_REPORT_OFFSET_LUT;
	} break;

	default:
		break;
	}
}

/* 收尾: 报告结果 + 写 Flash (CS_REPORT_OFFSET_LUT) */
static void cal_report_step(void)
{
	float R = Config.phase_resistance;
	float L = Config.phase_inductance;
	float lm = Config.flux_linkage;
	float wc = M_2PI * Config.current_ctrl_bw;
	int32_t lut_min = Config.offset_lut[0];
	int32_t lut_max = Config.offset_lut[0];
	int32_t cg_min = Config.cogging_map[0];
	int32_t cg_max = Config.cogging_map[0];
	uint8_t saved;

	if (cal.valid_stages != CAL_REQUIRED ||
	    Config.pole_pairs <= 0 ||
	    Config.pole_pairs > (int32_t)MAX_MOTOR_POLE_PAIRS ||
	    !isfinite(R) || R < R_CAL_MIN_OHM ||
	    !isfinite(L) || L <= L_CAL_MIN_H || !cal_flux_in_range(lm)) {
		cal_reject("required results incomplete or invalid");
		return;
	}
	cal_stop_drive();
	if (FOC.fault_code != FOC_FAULT_NONE) {
		cal_request_idle();
		printf("calib: REJECTED, fault before Flash save\n");
		return;
	}

	Config.cogging_map_valid = 0;
	Config.calib_valid = true;
	saved = CONFIG_save_to_flash();
	cal_request_idle();

	for (uint32_t i = 1U; i < OFFSET_LUT_NUM; i++) {
		int32_t v = Config.offset_lut[i];

		lut_min = MIN(lut_min, v);
		lut_max = MAX(lut_max, v);
	}
	for (uint32_t i = 1U; i < COGGING_MAP_NUM; i++) {
		int32_t v = (int32_t)Config.cogging_map[i];

		cg_min = MIN(cg_min, v);
		cg_max = MAX(cg_max, v);
	}

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
	printf(" cogging_map: min %d max %d pkpk %d (invalidated)\n",
	       (int)cg_min, (int)cg_max, (int)(cg_max - cg_min));
	printf(" valid  = %d flash=%s\n", (int)Config.calib_valid,
	       (saved == 0U) ? "ok" : "fail");
	printf("========================\n");
}

static bool cal_check_overcurrent(float ia)
{
	float ib = (FOC.Ib - FOC.Ib_midpoint) * FOC_CURRENT_SCALE;
	float ic = (FOC.Ic - FOC.Ic_midpoint) * FOC_CURRENT_SCALE;
	float limit = 3.0f * Config.calib_current;
	float peak = fmaxf(fabsf(ia), fmaxf(fabsf(ib), fabsf(ic)));
	tCalibStep step;
	float id, iq, iq_cmd, vq, angle;
	bool optional;

	if (!FOC.is_armed || !(peak > limit)) {
		cal.overcurrent_ticks = 0U;
		return false;
	}
	if (++cal.overcurrent_ticks < CAL_SOFT_OC_TICKS)
		return false;

	step = cal.step;
	id = FOC.Id;
	iq = FOC.Iq;
	iq_cmd = FOC.calib_iq;
	vq = FOC.Vq_out;
	angle = FOC.calib_angle;
	cal_stop_drive();
	cal.overcurrent_ticks = 0U;

	optional = step >= CS_MOTOR_J_START && step <= CS_MOTOR_J_END;
	if (optional) {
		cal.ticks = 0;
		cal.step = CS_REPORT_OFFSET_LUT;
	} else {
		cal_request_idle();
	}

	printf("calib: OVER-CURRENT %.2fA > %.2fA at step %d "
	       "(id=%.2f iq=%.2f cmd=%.2f vq=%.2f ang=%.2f rad)\n",
	       peak, limit, (int)step, id, iq, iq_cmd, vq, angle);
	if (optional)
		printf("calib: skip J (optional), go to report\n");
	return true;
}

/* ====== 标定主调度器 ====== */
void calibration_loop(void)
{
	float time = (float)cal.ticks / (float)CAL_LOOP_HZ;
	const float ia_meas = (FOC.Ia - FOC.Ia_midpoint) * FOC_CURRENT_SCALE;
	float calib_phase_vel = 12.0f * M_PI;
	float total_calib_cycles = 14.0f;

	if (cal.exiting) {
		foc_request_stage(IDLE);
		return;
	}
	if (cal.step == CS_NULL)
		return;
	if (FOC.fault_code != FOC_FAULT_NONE) {
		cal_reject("drive fault");
		return;
	}
	if (cal.step != CS_MOTOR_R_START &&
	    cal.step != CS_REPORT_OFFSET_LUT && !FOC.is_armed) {
		cal_reject("drive disarmed");
		return;
	}
	if (cal_check_overcurrent(ia_meas))
		return;

	if ((cal.step == CS_DIR_PP_LOOP) ||
	    (cal.step == CS_ENCODER_CW_LOOP) ||
	    (cal.step == CS_ENCODER_CCW_LOOP)) {
		static uint32_t drag_dbg_cnt;
		if (++drag_dbg_cnt >= 300U) {
			drag_dbg_cnt = 0;
			printf("drag(if): Iq=%.2fA I=%.2fA vel=%.3f rps "
			       "elec=%.2f rad\n",
			       (double)FOC.calib_iq, (double)ia_meas,
			       (double)encoder.vel, (double)cal.drive_angle);
		}
	}

	/* 丢步检测 */
	if ((cal.step >= CS_FLUX_RAMP1) && (cal.step <= CS_FLUX_MEAS2) &&
	    (Config.pole_pairs > 0) && (FOC.calib_phase_vel > 1.0f)) {
		static uint32_t slip_dbg;
		float exp_rps = FOC.calib_phase_vel /
				(M_2PI * (float)Config.pole_pairs);
		float dev = fabsf(encoder.vel - exp_rps) / exp_rps;
		if (dev > FLUX_MAX_SPEED_ERROR) {
			if ((slip_dbg++ % 200U) == 0U)
				printf("flux: SLIP vel=%.3f exp=%.3f rps "
				       "(dev %.0f%%)\n",
				       (double)encoder.vel, (double)exp_rps,
				       (double)(dev * 100.0f));
		}
	}

	/* 分派到各阶段 */
	switch (cal.step) {
	case CS_MOTOR_R_START:
	case CS_MOTOR_R_LOOP:
	case CS_MOTOR_R_END:
	case CS_MOTOR_L_START:
	case CS_MOTOR_L_LOOP:
	case CS_MOTOR_L_END:
		cal_rl_step(ia_meas);
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
		cal_report_step();
		break;

	default:
		break;
	}

	cal.ticks++;
}
