#include "foc.h"
#include "adc.h"
#include "encoder.h"
#include "foc_task.h"
#include "vofa.h"

volatile foc_data_t FOC;

/* 40 kHz 电流阶跃采样，完成后由 VOFA 整包上传。 */
#define FOC_STEP_N 800U
#define FOC_STEP_PRE 400U
static float step_buf[FOC_STEP_N][3];
static volatile uint16_t step_n;
static volatile float step_amp;
static volatile float step_base; /* 偏置电流, 小信号阶跃时先用 */
static volatile uint8_t step_state; /* 0 idle, 1 running, 2 done */

void FOC_step_set_base(float base)
{
	float lim = Config.current_limit;

	if (!isfinite(base))
		return;
	if (base > lim)
		base = lim;
	if (base < -lim)
		base = -lim;
	step_base = base;
}

void FOC_step_start(float amp)
{
	float lim = Config.current_limit;
	float b = step_base;

	if (step_state != 0U)
		return;
	/* 只在使能的 RUN 状态下接受, 否则电流环会在失能时驱动 PWM */
	if (!FOC.is_armed || (foc_get_stage() != RUN))
		return;
	if (!isfinite(amp) || !isfinite(b))
		return;
	if (b + amp > lim)
		amp = lim - b;
	if (b + amp < -lim)
		amp = -lim - b;
	step_amp = amp;
	step_n = 0;
	step_state = 1;
}

uint8_t FOC_step_state(void)
{
	return step_state;
}

uint16_t FOC_step_count(void)
{
	return FOC_STEP_N;
}

uint16_t FOC_step_pre(void)
{
	return FOC_STEP_PRE;
}

const float *FOC_step_data(void)
{
	return &step_buf[0][0];
}

void FOC_step_clear(void)
{
	step_state = 0;
}

/* ADC */
#define VBUS_DIVIDER_RATIO 16.0f
#define ADC_REF_VOLTAGE 3.3f	/* ADC参考电压3.3V */
#define ADC_RESOLUTION 65536.0f /* 16位ADC分辨率 */
#define VBUS_SCALE_FACTOR \
	(VBUS_DIVIDER_RATIO * ADC_REF_VOLTAGE / ADC_RESOLUTION)

#define VBUS_DEFAULT 12.0f

bool foc_adc_midpoint(void)
{
	float adc_a_sum = 0;
	float adc_b_sum = 0;
	float adc_c_sum = 0;
	float adc_ibus_sum = 0;
	float adc_va_sum = 0;
	float adc_vb_sum = 0;
	float adc_vc_sum = 0;
	uint32_t times = 0;
	uint32_t ibus_times = 0;

	for (uint8_t i = 0; i < 32; i++) {
		uint16_t va, vb, vc, vbus_raw;
		uint16_t ia, ib, ic;
		uint16_t ibus_raw;

		if (!adc1_read_all(&va, &vb, &vc, &vbus_raw) ||
		    !adc_current_snapshot(&ia, &ib, &ic)) {
			sys_delay_ms(1);
			continue;
		}

		adc_a_sum += ia;
		adc_b_sum += ib;
		adc_c_sum += ic;
		if (adc1_read_ibus(&ibus_raw)) {
			adc_ibus_sum += ibus_raw;
			ibus_times++;
		}
		adc_va_sum += (float)va;
		adc_vb_sum += (float)vb;
		adc_vc_sum += (float)vc;
		times += 1;
		sys_delay_ms(1);
	}
	if (times < 16U || ibus_times < 16U)
		return false;
	FOC.Ia_midpoint = adc_a_sum / times;
	FOC.Ib_midpoint = adc_b_sum / times;
	FOC.Ic_midpoint = adc_c_sum / times;
	FOC.Ibus_midpoint = adc_ibus_sum / ibus_times;
	FOC.Va_midpoint = adc_va_sum / times;
	FOC.Vb_midpoint = adc_vb_sum / times;
	FOC.Vc_midpoint = adc_vc_sum / times;
	FOC.adc_ready = true;
	return true;
}

/* 以 1 kHz 读取并滤波三相电压和母线电压。 */
void foc_update_voltages(void)
{
	uint16_t va, vb, vc, vbus_raw;
	uint16_t ibus_raw;
	const float k = (3.3f / 65536.0f) * FOC_VPHASE_DIVIDER;

	if (adc1_read_ibus(&ibus_raw)) {
		FOC.Ibus = ibus_raw;
		FOC.adc_ibus_missed = 0U;
		float ibus =
		    ((float)ibus_raw - FOC.Ibus_midpoint) * FOC_IBUS_SCALE;
		UTILS_LP_FAST(FOC.Ibus_filt, ibus, 0.1f);
	} else {
		FOC.adc_ibus_missed++;
	}

	if (!adc1_read_all(&va, &vb, &vc, &vbus_raw)) {
		FOC.adc_voltage_missed++;
		return;
	}
	FOC.adc_voltage_missed = 0;

	FOC.Va = ((float)va - FOC.Va_midpoint) * k;
	FOC.Vb = ((float)vb - FOC.Vb_midpoint) * k;
	FOC.Vc = ((float)vc - FOC.Vc_midpoint) * k;

	float vbus = (float)vbus_raw * (3.3f / 65536.0f) * FOC_VBUS_DIVIDER;
	FOC.Vbus = vbus;
	UTILS_LP_FAST(FOC.Vbus_filt, vbus, 0.1f);
}

/* 按相电流方向补偿死区及驱动器约 100 ns 延迟。 */
#define DT_DEAD_NS (PWM_DEAD_TIME_NS + 100U)
#define DT_COMP_ITH (0.8f)

static void foc_deadtime_comp(void)
{
	const float dt_comp =
	    (float)DT_DEAD_NS * 1e-9f / CURRENT_MEASURE_PERIOD;
	float ca =
	    ((FOC.Ia - FOC.Ia_midpoint) * FOC_CURRENT_SCALE) / DT_COMP_ITH;
	float cb =
	    ((FOC.Ib - FOC.Ib_midpoint) * FOC_CURRENT_SCALE) / DT_COMP_ITH;
	float cc =
	    ((FOC.Ic - FOC.Ic_midpoint) * FOC_CURRENT_SCALE) / DT_COMP_ITH;

	if (ca > 1.0f)
		ca = 1.0f;
	else if (ca < -1.0f)
		ca = -1.0f;
	if (cb > 1.0f)
		cb = 1.0f;
	else if (cb < -1.0f)
		cb = -1.0f;
	if (cc > 1.0f)
		cc = 1.0f;
	else if (cc < -1.0f)
		cc = -1.0f;

	FOC.dtc_a += dt_comp * ca;
	FOC.dtc_b += dt_comp * cb;
	FOC.dtc_c += dt_comp * cc;
}

void FOC_voltage(float Vd, float Vq, float phase)
{
	float v_to_mod = 1.5f / FOC.Vbus_filt;
	float mod_vd = Vd * v_to_mod;
	float mod_vq = Vq * v_to_mod;

	/* 电压矢量限幅到线性调制范围，避免过调制。 */
	float factor = 0.9f * SQRT3_BY_2 / sqrtf(SQ(mod_vd) + SQ(mod_vq));
	if (factor < 1.0f) {
		mod_vd *= factor;
		mod_vq *= factor;
	}

	/* 逆 Park 变换，将 dq 电压转换到 αβ 坐标系。 */
	float alpha, beta;
	float pwm_phase = phase;
	inverse_park(mod_vd, mod_vq, pwm_phase, &alpha, &beta);

	if (0 == SVPWM(alpha, beta, &FOC.dtc_a, &FOC.dtc_b, &FOC.dtc_c)) {
		foc_deadtime_comp();
		pwm_set_duty(FOC.dtc_a, FOC.dtc_b, FOC.dtc_c);
	}
}

/* 三相下桥导通，上桥关闭。 */
void pwm_output_low_sides(void)
{
	pwm_set_duty(0, 0, 0);
}

/* 解除软件强制输出，恢复正常 PWM。 */
void enable_bldc_pwm_output(void)
{
	for (pwm_channel_t ch = pwm_channel_0; ch <= pwm_channel_5; ch++)
		pwmv2_disable_software_force(PWM, ch);
}

/* 将 PWM 输出强制到无效电平，关闭功率输出。 */
void disable_bldc_pwm_output(void)
{
	for (pwm_channel_t ch = pwm_channel_0; ch <= pwm_channel_5; ch++) {
		pwmv2_set_force_update_time(PWM, ch, pwm_force_immediately);
		pwmv2_force_update_time_by_shadow(
		    PWM, ch, pwm_force_update_shadow_immediately);
		pwmv2_enable_force_by_software(PWM, ch);
		pwmv2_enable_software_force(PWM, ch);
	}

	pwmv2_shadow_register_unlock(PWM);
	for (pwm_channel_t ch = pwm_channel_0; ch <= pwm_channel_5; ch++)
		pwmv2_force_output(PWM, ch, pwm_force_output_1, false);
	pwmv2_shadow_register_lock(PWM);
}

void foc_disarm(void)
{
	if (!FOC.is_armed)
		return;

	taskENTER_CRITICAL();

	disable_bldc_pwm_output();

	FOC.Iq = 0;
	FOC.Iq_filt = 0;
	FOC.Id = 0;
	FOC.Id_filt = 0;
	FOC.Ibus = 0;
	FOC.Ibus_filt = 0;
	FOC.Power_filt = 0;

	FOC.current_ctrl_integral_d = 0;
	FOC.current_ctrl_integral_q = 0;

	FOC.Iq_set = 0;

	FOC.is_armed = false;

	taskEXIT_CRITICAL();
}

void foc_arm(void)
{
	if (!FOC.adc_ready) {
		FOC.fault_code = FOC_FAULT_ADC;
		return;
	}
	if (FOC.is_armed)
		return;

	taskENTER_CRITICAL();

	FOC.Iq = 0;
	FOC.Iq_filt = 0;
	FOC.Id = 0;
	FOC.Id_filt = 0;
	FOC.Ibus = 0;
	FOC.Ibus_filt = 0;
	FOC.Power_filt = 0;

	FOC.current_ctrl_integral_d = 0;
	FOC.current_ctrl_integral_q = 0;

	enable_bldc_pwm_output();

	/* 不在 arm 时清故障码: 故障锁存, 需上位机显式发 0x22 禁用后再 0x21 使能 */
	if (FOC.fault_code != FOC_FAULT_NONE) {
		taskEXIT_CRITICAL();
		/* 故障未清除, 立即重新失能 */
		disable_bldc_pwm_output();
		return;
	}
	FOC.is_armed = true;

	taskEXIT_CRITICAL();
}

void FOC_update_current_ctrl_gain(float bw)
{
	float bandwidth = bw * M_2PI;
	FOC.current_ctrl_p_gain = Config.phase_inductance * bandwidth;
	FOC.current_ctrl_i_gain = Config.phase_resistance * bandwidth;

	Config.current_kp = FOC.current_ctrl_p_gain;
	Config.current_ki = FOC.current_ctrl_i_gain;
}

void FOC_current(float Id_set, float Iq_set, float phase, float phase_vel)
{
	float ia = (FOC.Ia - FOC.Ia_midpoint) * FOC_CURRENT_SCALE;
	float ib = (FOC.Ib - FOC.Ib_midpoint) * FOC_CURRENT_SCALE;
	float ic = (FOC.Ic - FOC.Ic_midpoint) * FOC_CURRENT_SCALE;

	float i_alpha, i_beta;
	clarke_transform(ia, ib, ic, &i_alpha, &i_beta);

	float i_d, i_q;
	park_transform(i_alpha, i_beta, phase, &i_d, &i_q);

	float i_d_err = Id_set - i_d;
	float i_q_err = Iq_set - i_q;
	float v_d =
	    i_d_err * FOC.current_ctrl_p_gain + FOC.current_ctrl_integral_d;
	float v_q =
	    i_q_err * FOC.current_ctrl_p_gain + FOC.current_ctrl_integral_q;

	/* 反电势前馈与 d/q 解耦；标定期间关闭以保持模型不变。 */
	if (FOC.ff_enable && !FOC.calib_mode) {
		v_d += -phase_vel * Config.phase_inductance * i_q;
		v_q += phase_vel *
		       (Config.phase_inductance * i_d + Config.flux_linkage);
	}

	FOC.Vd_out = v_d;
	FOC.Vq_out = v_q;
	FOC.Vmag = sqrtf(SQ(v_d) + SQ(v_q));

	/* 将电压值归一化为 SVPWM 调制输入。 */
	float v_to_mod = 1.5f / FOC.Vbus_filt;
	float mod_vd = v_d * v_to_mod;
	float mod_vq = v_q * v_to_mod;

	/* 电压矢量限幅到线性调制范围，避免过调制。 */
	float factor = 0.9f * SQRT3_BY_2 / sqrtf(SQ(mod_vd) + SQ(mod_vq));
	if (factor < 1.0f) {
		mod_vd *= factor;
		mod_vq *= factor;
		FOC.current_ctrl_integral_d *= 0.99f;
		FOC.current_ctrl_integral_q *= 0.99f;
		FOC.saturated = true;
	} else {
		FOC.saturated = false;
		FOC.current_ctrl_integral_d +=
		    i_d_err *
		    (FOC.current_ctrl_i_gain * CURRENT_MEASURE_PERIOD);
		FOC.current_ctrl_integral_q +=
		    i_q_err *
		    (FOC.current_ctrl_i_gain * CURRENT_MEASURE_PERIOD);
	}

	/* 按电角速度外推一个电流环周期，补偿采样至 PWM 生效的延迟。 */
	float alpha, beta;
	float pwm_phase = phase + phase_vel * CURRENT_MEASURE_PERIOD;
	inverse_park(mod_vd, mod_vq, pwm_phase, &alpha, &beta);

	if (0 == SVPWM(alpha, beta, &FOC.dtc_a, &FOC.dtc_b, &FOC.dtc_c)) {
		foc_deadtime_comp();
		pwm_set_duty(FOC.dtc_a, FOC.dtc_b, FOC.dtc_c);
	}

	FOC.Iq = i_q;
	UTILS_LP_FAST(FOC.Iq_filt, FOC.Iq, 0.01f);
	FOC.Id = i_d;
	UTILS_LP_FAST(FOC.Id_filt, FOC.Id, 0.01f);

	/* 表贴式 PMSM 转矩估算：T = 1.5 × 极对数 × 磁链 × Iq。 */
	FOC.torque =
	    1.5f * (float)Config.pole_pairs * Config.flux_linkage * i_q;

	FOC.Power_filt = 1.5f * (v_d * i_d + v_q * i_q);
}

void foc_adc_callback(uint16_t ia_raw, uint16_t ib_raw, uint16_t ic_raw)
{
	/* adc.c 传来原始码值, 在这里存入 FOC 结构体 (保持分层清洁) */
	FOC.Ia = (float)ia_raw;
	FOC.Ib = (float)ib_raw;
	FOC.Ic = (float)ic_raw;

	/* 更新帧计数 (adc.c 内部也维护, 这里保持 FOC 结构体对外可见) */
	FOC.adc_current_frames = adc_current_frames();
	FOC.adc_current_missed = adc_current_missed();

	/* 由 Ia、Ib 重构 Ic；实测电流和仅用于诊断。 */
	FOC.isum =
	    ((FOC.Ia - FOC.Ia_midpoint) + (FOC.Ib - FOC.Ib_midpoint) +
	     (FOC.Ic - FOC.Ic_midpoint)) *
	    FOC_CURRENT_SCALE;
	if (FOC.adc_ready) {
		float ic_fix = FOC.Ic_midpoint -
			       (FOC.Ia - FOC.Ia_midpoint) -
			       (FOC.Ib - FOC.Ib_midpoint);
		if (ic_fix < 0.0f)
			ic_fix = 0.0f;
		else if (ic_fix > 65535.0f)
			ic_fix = 65535.0f;
		FOC.Ic = ic_fix;
	}

	/* 阶跃测试只在使能的 RUN 状态下执行; 失能时立即作废, 防止绕过使能输出 PWM */
	if ((step_state == 1U) &&
	    (!FOC.is_armed || (foc_get_stage() != RUN))) {
		step_state = 0U;
	}

	/* 锁定转子时记录 Iq 阶跃响应，用于分析电流环带宽。 */
	if (step_state == 1U) {
		uint16_t n = step_n;
		float ref = (n < FOC_STEP_PRE) ? step_base
					       : (step_base + step_amp);

		FOC_current(0.0f, ref, encoder.phase, 0.0f);
		if (n < FOC_STEP_N) {
			step_buf[n][0] = FOC.Iq;
			step_buf[n][1] = ref;
			step_buf[n][2] = FOC.Vq_out;
			step_n = (uint16_t)(n + 1U);
		}
		if (step_n >= FOC_STEP_N)
			step_state = 2U;
		return;
	}

	/* 波形采样由 40 kHz 降采样至 5 kHz；阶跃数据单独打包。 */
	static uint8_t vofa_div = 0;

	if (step_state == 0U) {
		if (++vofa_div >= VOFA_SAMPLE_DECIM) {
			vofa_div = 0;
			vofa_push_sample();
		}
	}

	if (!FOC.is_armed)
		return;

	/* ===== 中断级相电流硬限幅 (第 1 级保护) =====
	 * 本板无硬件过流引脚, 必须在 ISR 里补一级软件快速关断.
	 * 阈值取 protect_over_current (5kHz 任务的软保护值), 响应在下一拍
	 * (25µs) 内生效, 远快于 5kHz 任务的 200µs. */
	{
		float ia = (FOC.Ia - FOC.Ia_midpoint) * FOC_CURRENT_SCALE;
		float ib = (FOC.Ib - FOC.Ib_midpoint) * FOC_CURRENT_SCALE;
		float ic = (FOC.Ic - FOC.Ic_midpoint) * FOC_CURRENT_SCALE;
		float lim = Config.protect_over_current;

		if ((fabsf(ia) > lim) || (fabsf(ib) > lim) ||
		    (fabsf(ic) > lim)) {
			disable_bldc_pwm_output(); /* 立即关断, 不等任务 */
			FOC.fault_code = FOC_FAULT_PHASE_OC_ISR;
			FOC.is_armed = false;
			/* ISR 里不能调 foc_disarm(含临界区), 只置标志;
			 * FOC 任务在下一拍检测到 fault_code 后转 IDLE */
			return;
		}
	}

	/* 电感标定：在 α 轴注入正弦电压。 */
	if (FOC.calib_vinj_mode) {
		FOC.inj_phase += FOC.vinj_omega * CURRENT_MEASURE_PERIOD;
		if (FOC.inj_phase >= M_2PI)
			FOC.inj_phase -= M_2PI;

		float s = sin_f32(FOC.inj_phase);
		if (FOC.vinj_acc) {
			float ia =
			    (FOC.Ia - FOC.Ia_midpoint) * FOC_CURRENT_SCALE;
			FOC.vinj_sum_sin += ia * s;
			FOC.vinj_sum_cos += ia * cos_f32(FOC.inj_phase);
			FOC.vinj_samples++;
		}
		FOC_voltage(FOC.vinj_dc + FOC.vinj_amp * s, 0, 0);
		return;
	}

	/* 开环电流标定使用标定任务下发的角度。 */
	if (FOC.calib_mode) {
		if (FOC.calib_adv_angle) {
			FOC.calib_angle +=
			    FOC.calib_phase_vel * CURRENT_MEASURE_PERIOD;
			while (FOC.calib_angle >= M_2PI)
				FOC.calib_angle -= M_2PI;
			while (FOC.calib_angle < 0.0f)
				FOC.calib_angle += M_2PI;
		}
		FOC_current(FOC.calib_id, FOC.calib_iq, FOC.calib_angle,
			    FOC.calib_phase_vel);
		return;
	}

	if ((foc_get_stage() != RUN) && (foc_get_stage() != ANTICOGGING))
		return;

	/* 调试: 开环固定矢量 (旁路电流环) */
	if (FOC.openloop_voltage > 0.0f) {
		FOC_voltage(FOC.openloop_voltage, 0, FOC.openloop_angle);
		return;
	}

	/* 编码器以 5 kHz 更新，电流环为 40 kHz；按速度外推中间角度。 */
	static float phase_est;
	static uint32_t phase_seq_seen;

	if (encoder.phase_seq != phase_seq_seen) {
		/* 重同步时补偿编码器采样到发布的固定延迟。 */
		float sync_latency =
		    1.0f / (2.0f * (float)ENCODER_TASK_HZ) +
		    0.5f * CURRENT_MEASURE_PERIOD;

		phase_seq_seen = encoder.phase_seq;
		phase_est = encoder.phase + encoder.phase_vel * sync_latency;
	} else {
		phase_est += encoder.phase_vel * CURRENT_MEASURE_PERIOD;
	}
	while (phase_est >= M_2PI)
		phase_est -= M_2PI;
	while (phase_est < 0.0f)
		phase_est += M_2PI;

	float iq_set =
	    CLAMP(FOC.Iq_set, -Config.current_limit, Config.current_limit);
	float phase = phase_est + FOC.phase_offset;
	FOC_current(FOC.Id_set, iq_set, phase, encoder.phase_vel);
}

void foc_init(void)
{
	FOC.Vbus = VBUS_DEFAULT;
	FOC.Vbus_filt = VBUS_DEFAULT;
	FOC.Id_set = 0;
	FOC.Iq_set = 0;
	FOC.phase_offset = 0;
	FOC.openloop_voltage = 0;
	FOC.openloop_angle = 0;

	FOC.Vd_out = 0;
	FOC.Vq_out = 0;
	FOC.Vmag = 0;
	FOC.torque = 0;
	FOC.saturated = false;
	FOC.ff_enable = true;
	FOC.calib_mode = false;
	FOC.calib_angle = 0;
	FOC.calib_phase_vel = 0;
	FOC.calib_id = 0;
	FOC.calib_iq = 0;

	FOC.calib_vinj_mode = false;
	FOC.calib_adv_angle = false;
	FOC.vinj_omega = 0;
	FOC.vinj_amp = 0;
	FOC.vinj_dc = 0;
	FOC.inj_phase = 0;
	FOC.vinj_acc = false;
	FOC.vinj_sum_sin = 0;
	FOC.vinj_sum_cos = 0;
	FOC.vinj_samples = 0;
	FOC.current_ctrl_integral_d = 0;
	FOC.current_ctrl_integral_q = 0;
	FOC.Power_filt = 0;

	/* 从 Flash 加载的 kp/ki 若有效则直接用, 否则按带宽重算 */
	if (Config.current_kp > 0.0f && Config.current_kp < 1.0e5f &&
	    Config.current_ki >= 0.0f && Config.current_ki < 1.0e7f) {
		FOC.current_ctrl_p_gain = Config.current_kp;
		FOC.current_ctrl_i_gain = Config.current_ki;
	} else {
		FOC_update_current_ctrl_gain(Config.current_ctrl_bw);
	}
	FOC.adc_ready = false;
	FOC.adc_current_frames = 0U;
	FOC.adc_current_missed = 0U;
	FOC.adc_voltage_missed = 0U;
	FOC.adc_ibus_missed = 0U;
	adc1_init();
	if (!foc_adc_midpoint())
		FOC.fault_code = FOC_FAULT_ADC;
	adc_callback_set(foc_adc_callback);
}
