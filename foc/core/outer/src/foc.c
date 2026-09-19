#include "foc.h"
#include "adc.h"
#include "encoder.h"
#include "foc_task.h"

volatile foc_data_t FOC;

/* ADC */
#define VBUS_DIVIDER_RATIO 21.0f // (10K+10K+1K)/1K = 21
#define ADC_REF_VOLTAGE 3.3f	 // ADC参考电压3.3V
#define ADC_RESOLUTION 65536.0f	 // 16位ADC分辨率
#define VBUS_SCALE_FACTOR                                                      \
	(VBUS_DIVIDER_RATIO * ADC_REF_VOLTAGE / ADC_RESOLUTION)

/* 母线电压暂用常数; 等 VBUS 通道接进 ADC 后换成实测值 */
#define VBUS_DEFAULT 24.0f

void foc_adc_midpoint(void)
{
	float adc_a_sum = 0;
	float adc_b_sum = 0;
	float adc_c_sum = 0;
	float times = 0;

	for (uint8_t i = 0; i < 32; i++) {
		adc_a_sum += FOC.Ia;
		adc_b_sum += FOC.Ib;
		adc_c_sum += FOC.Ic;
		times += 1;
		sys_delay_ms(1);
	}
	FOC.Ia_midpoint = adc_a_sum / times;
	FOC.Ib_midpoint = adc_b_sum / times;
	FOC.Ic_midpoint = adc_c_sum / times;
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
	} else {
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

	FOC.Power_filt = 1.5f * (v_d * i_d + v_q * i_q);
}

void foc_adc_callback(void)
{
	if (!FOC.is_armed || (foc_get_stage() != RUN)) {
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
	Config.phase_resistance = 5.0f;	   // 5Ω
	Config.phase_inductance = 0.00164f; // 1.64mH

	FOC.Vbus                    = VBUS_DEFAULT;
	FOC.Vbus_filt               = VBUS_DEFAULT;
	FOC.Id_set                  = 0;
	FOC.Iq_set                  = 0;
	FOC.phase_offset            = 0;
	FOC.openloop_voltage        = 0;
	FOC.openloop_angle          = 0;
	FOC.current_ctrl_integral_d = 0;
	FOC.current_ctrl_integral_q = 0;
	FOC.Power_filt              = 0;

	FOC_update_current_ctrl_gain(Config.current_ctrl_bw);
	foc_adc_midpoint();
	adc_callback_set(foc_adc_callback);
}
