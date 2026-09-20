#include "foc.h"
#include "adc.h"
#include "encoder.h"
#include "foc_task.h"
#include "vofa.h"

volatile foc_data_t FOC;

/* ADC */
#define VBUS_DIVIDER_RATIO 21.0f // (10K+10K+1K)/1K = 21
#define ADC_REF_VOLTAGE 3.3f	 // ADC参考电压3.3V
#define ADC_RESOLUTION 65536.0f	 // 16位ADC分辨率
#define VBUS_SCALE_FACTOR                                                      \
	(VBUS_DIVIDER_RATIO * ADC_REF_VOLTAGE / ADC_RESOLUTION)

#define VBUS_DEFAULT 12.0f

void foc_adc_midpoint(void)
{
	float adc_a_sum = 0;
	float adc_b_sum = 0;
	float adc_c_sum = 0;
	float adc_bus_sum = 0;
	float adc_va_sum = 0;
	float adc_vb_sum = 0;
	float adc_vc_sum = 0;
	float times = 0;

	for (uint8_t i = 0; i < 32; i++) {
		uint16_t va, vb, vc, vbus_raw;

		adc1_read_all(&va, &vb, &vc, &vbus_raw);

		adc_a_sum += FOC.Ia;
		adc_b_sum += FOC.Ib;
		adc_c_sum += FOC.Ic;
		adc_bus_sum += FOC.Ibus;
		adc_va_sum += (float)va;
		adc_vb_sum += (float)vb;
		adc_vc_sum += (float)vc;
		times += 1;
		sys_delay_ms(1);
	}
	FOC.Ia_midpoint = adc_a_sum / times;
	FOC.Ib_midpoint = adc_b_sum / times;
	FOC.Ic_midpoint = adc_c_sum / times;
	FOC.Ibus_midpoint = adc_bus_sum / times;
	FOC.Va_midpoint = adc_va_sum / times;
	FOC.Vb_midpoint = adc_vb_sum / times;
	FOC.Vc_midpoint = adc_vc_sum / times;
}

/* 三相电压 + 母线电压: 1kHz 软件触发 ADC1 采一组, 换算 + 滤波 */
void foc_update_voltages(void)
{
	uint16_t va, vb, vc, vbus_raw;
	const float k = (3.3f / 65536.0f) * FOC_VPHASE_DIVIDER;

	adc1_read_all(&va, &vb, &vc, &vbus_raw);

	FOC.Va = ((float)va - FOC.Va_midpoint) * k;
	FOC.Vb = ((float)vb - FOC.Vb_midpoint) * k;
	FOC.Vc = ((float)vc - FOC.Vc_midpoint) * k;

	float vbus = (float)vbus_raw * (3.3f / 65536.0f) *
		     FOC_VBUS_DIVIDER;

	if ((vbus > 2.0f) && (vbus < 70.0f)) {
		FOC.Vbus = vbus;
		UTILS_LP_FAST(FOC.Vbus_filt, vbus, 0.1f);
	}
}

void FOC_voltage(float Vd, float Vq, float phase)
{
	float v_to_mod = 1.5f / FOC.Vbus_filt;
	float mod_vd = Vd * v_to_mod;
	float mod_vq = Vq * v_to_mod;

	// Vector modulation saturation, lock integrator if saturated
	float factor = 0.9f * SQRT3_BY_2 / sqrtf(SQ(mod_vd) + SQ(mod_vq));
	if (factor < 1.0f) {
		mod_vd *= factor;
		mod_vq *= factor;
	}

	// Inverse park transform
	float alpha, beta;
	float pwm_phase = phase;
	inverse_park(mod_vd, mod_vq, pwm_phase, &alpha, &beta);

	if (0 == SVPWM(alpha, beta, &FOC.dtc_a, &FOC.dtc_b, &FOC.dtc_c)) {
		pwm_set_duty(FOC.dtc_a, FOC.dtc_b, FOC.dtc_c);
	}
}

// 三相下桥全开, 上桥全关
void pwm_output_low_sides(void)
{
	pwm_set_duty(0, 0, 0);
}

// 恢复三相正常 PWM 输出
void enable_bldc_pwm_output(void)
{
	for (pwm_channel_t ch = pwm_channel_0; ch <= pwm_channel_5; ch++) {
		pwmv2_disable_software_force(PWM, ch);
	}
}

// 强制三相输出到无效电平, 完全关断
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
	for (pwm_channel_t ch = pwm_channel_0; ch <= pwm_channel_5; ch++) {
		pwmv2_force_output(PWM, ch, pwm_force_output_1, false);
	}
	pwmv2_shadow_register_lock(PWM);
}

void foc_disarm(void)
{
	if (!FOC.is_armed) {
		return;
	}

	taskENTER_CRITICAL();

	disable_bldc_pwm_output();

	FOC.Iq         = 0;
	FOC.Iq_filt    = 0;
	FOC.Id         = 0;
	FOC.Id_filt    = 0;
	FOC.Ibus       = 0;
	FOC.Ibus_filt  = 0;
	FOC.Power_filt = 0;

	FOC.current_ctrl_integral_d = 0;
	FOC.current_ctrl_integral_q = 0;

	FOC.Iq_set = 0;

	FOC.is_armed = false;

	taskEXIT_CRITICAL();
}

void foc_arm(void)
{
	if (FOC.is_armed) {
		return;
	}

	taskENTER_CRITICAL();

	FOC.Iq         = 0;
	FOC.Iq_filt    = 0;
	FOC.Id         = 0;
	FOC.Id_filt    = 0;
	FOC.Ibus       = 0;
	FOC.Ibus_filt  = 0;
	FOC.Power_filt = 0;

	FOC.current_ctrl_integral_d = 0;
	FOC.current_ctrl_integral_q = 0;

	enable_bldc_pwm_output();

	FOC.fault_code = FOC_FAULT_NONE;
	FOC.is_armed = true;

	taskEXIT_CRITICAL();
}

void FOC_update_current_ctrl_gain(float bw)
{
	float bandwidth = bw * M_2PI;
	FOC.current_ctrl_p_gain = Config.phase_inductance * bandwidth;
	FOC.current_ctrl_i_gain = Config.phase_resistance * bandwidth;
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
	float v_d = i_d_err * FOC.current_ctrl_p_gain + FOC.current_ctrl_integral_d;
	float v_q = i_q_err * FOC.current_ctrl_p_gain + FOC.current_ctrl_integral_q;

	// 反电势前馈 + d/q 交叉解耦 (校准模式不启用, 保证磁链标定模型不变)
	if (FOC.ff_enable && !FOC.calib_mode) {
		v_d += -phase_vel * Config.phase_inductance * i_q;
		v_q += phase_vel * (Config.phase_inductance * i_d + Config.flux_linkage);
	}

	FOC.Vd_out = v_d;
	FOC.Vq_out = v_q;
	FOC.Vmag   = sqrtf(SQ(v_d) + SQ(v_q));

	// voltage normalize = 1/(2/3*v_bus)
	float v_to_mod = 1.5f / FOC.Vbus_filt;
	float mod_vd = v_d * v_to_mod;
	float mod_vq = v_q * v_to_mod;

	// Vector modulation saturation, lock integrator if saturated
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
		    i_d_err * (FOC.current_ctrl_i_gain * CURRENT_MEASURE_PERIOD);
		FOC.current_ctrl_integral_q +=
		    i_q_err * (FOC.current_ctrl_i_gain * CURRENT_MEASURE_PERIOD);
	}

	// 角度外推一拍, 补偿计算到生效之间的延迟
	float alpha, beta;
	float pwm_phase = phase + phase_vel * CURRENT_MEASURE_PERIOD;
	inverse_park(mod_vd, mod_vq, pwm_phase, &alpha, &beta);

	if (0 == SVPWM(alpha, beta, &FOC.dtc_a, &FOC.dtc_b, &FOC.dtc_c)) {
		pwm_set_duty(FOC.dtc_a, FOC.dtc_b, FOC.dtc_c);
	}

	FOC.Iq = i_q;
	UTILS_LP_FAST(FOC.Iq_filt, FOC.Iq, 0.01f);
	FOC.Id = i_d;
	UTILS_LP_FAST(FOC.Id_filt, FOC.Id, 0.01f);

	// 转矩估算 (表贴式: T = 1.5·p·λ·iq)
	FOC.torque = 1.5f * (float)Config.pole_pairs * Config.flux_linkage * i_q;

	FOC.Power_filt = 1.5f * (v_d * i_d + v_q * i_q);
}

void foc_adc_callback(void)
{
	/* 波形采样: 40kHz 8 分频 -> 5kHz, 攒批由 vofa 任务发送 */
	static uint8_t vofa_div = 0;

	/*
	 * 母线电流: 采样点落在零矢量(电流在桥内续流), 只有运放输出自带 RC 滤波
	 * (或运放带宽远低于 40kHz) 时, 这里的读数才是有意义的平均值.
	 */
	float ibus = (FOC.Ibus - FOC.Ibus_midpoint) * FOC_IBUS_SCALE;
	UTILS_LP_FAST(FOC.Ibus_filt, ibus, 0.005f);

	if (++vofa_div >= VOFA_SAMPLE_DECIM) {
		vofa_div = 0;
		vofa_push_sample();
	}

	if (!FOC.is_armed) {
		return;
	}

	// 校准: α 轴正弦电压注入 (电感测量)
	if (FOC.calib_vinj_mode) {
		FOC.inj_phase += FOC.vinj_omega * CURRENT_MEASURE_PERIOD;
		if (FOC.inj_phase >= M_2PI) FOC.inj_phase -= M_2PI;

		float s = sin_f32(FOC.inj_phase);
		if (FOC.vinj_acc) {
			float ia = (FOC.Ia - FOC.Ia_midpoint) * FOC_CURRENT_SCALE;
			FOC.vinj_sum_sin += ia * s;
			FOC.vinj_sum_cos += ia * cos_f32(FOC.inj_phase);
			FOC.vinj_samples++;
		}
		FOC_voltage(FOC.vinj_dc + FOC.vinj_amp * s, 0, 0);
		return;
	}

	// 校准: 开环电流模式 (角度由校准状态机下发)
	if (FOC.calib_mode) {
		FOC_current(FOC.calib_id, FOC.calib_iq, FOC.calib_angle,
			    FOC.calib_phase_vel);
		return;
	}

	if (foc_get_stage() != RUN) {
		return;
	}

	// 调试: 开环固定矢量 (旁路电流环)
	if (FOC.openloop_voltage > 0.0f) {
		FOC_voltage(FOC.openloop_voltage, 0, FOC.openloop_angle);
		return;
	}

	float iq_set = CLAMP(FOC.Iq_set, -Config.current_limit,
			     Config.current_limit);
	float phase = encoder.phase + FOC.phase_offset;
	FOC_current(FOC.Id_set, iq_set, phase, encoder.phase_vel);
}

void foc_init(void)
{
	FOC.Vbus                    = VBUS_DEFAULT;
	FOC.Vbus_filt               = VBUS_DEFAULT;
	FOC.Id_set                  = 0;
	FOC.Iq_set                  = 0;
	FOC.phase_offset            = 0;
	FOC.openloop_voltage        = 0;
	FOC.openloop_angle          = 0;

	FOC.Vd_out      = 0;
	FOC.Vq_out      = 0;
	FOC.Vmag        = 0;
	FOC.torque      = 0;
	FOC.saturated   = false;
	FOC.ff_enable   = true;
	FOC.calib_mode  = false;
	FOC.calib_angle = 0;
	FOC.calib_phase_vel = 0;
	FOC.calib_id        = 0;
	FOC.calib_iq        = 0;

	FOC.calib_vinj_mode = false;
	FOC.vinj_omega      = 0;
	FOC.vinj_amp        = 0;
	FOC.vinj_dc         = 0;
	FOC.inj_phase       = 0;
	FOC.vinj_acc        = false;
	FOC.vinj_sum_sin    = 0;
	FOC.vinj_sum_cos    = 0;
	FOC.vinj_samples    = 0;
	FOC.current_ctrl_integral_d = 0;
	FOC.current_ctrl_integral_q = 0;
	FOC.Power_filt              = 0;

	FOC_update_current_ctrl_gain(Config.current_ctrl_bw);
	adc1_init();
	foc_adc_midpoint();
	adc_callback_set(foc_adc_callback);
}
