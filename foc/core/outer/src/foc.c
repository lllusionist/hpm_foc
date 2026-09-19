#include "foc.h"

volatile foc_data_t FOC;

/* ADC */
#define VBUS_DIVIDER_RATIO 21.0f // (10K+10K+1K)/1K = 21
#define ADC_REF_VOLTAGE 3.3f	 // ADC参考电压3.3V
#define ADC_RESOLUTION 65536.0f	 // 12位ADC分辨率
#define VBUS_SCALE_FACTOR                                                      \
	(VBUS_DIVIDER_RATIO * ADC_REF_VOLTAGE / ADC_RESOLUTION)

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
	float v_to_mod = 1.5f / 12.0f; // 1.5f / FOC.Vbus;
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

	FOC.Iq        = 0;
	FOC.Iq_filt   = 0;
	FOC.Ibus      = 0;
	FOC.Ibus_filt = 0;

	FOC.is_armed = false;

	taskEXIT_CRITICAL();
}

void foc_arm(void)
{
	if (FOC.is_armed) {
		return;
	}

	taskENTER_CRITICAL();

	FOC.Iq        = 0;
	FOC.Iq_filt   = 0;
	FOC.Ibus      = 0;
	FOC.Ibus_filt = 0;

	enable_bldc_pwm_output();

	FOC.is_armed = true;

	taskEXIT_CRITICAL();
}

void foc_init(void)
{
	Config.phase_resistance = 5; // 5Ω
	Config.phase_inductance = 0.00164; // 1.64mH
}