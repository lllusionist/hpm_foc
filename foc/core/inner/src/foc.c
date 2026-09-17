#include "foc.h"

volatile foc_data_t FOC;

/* ADC */
#define VBUS_DIVIDER_RATIO      21.0f                // (10K+10K+1K)/1K = 21
#define ADC_REF_VOLTAGE         3.3f                 // ADC参考电压3.3V
#define ADC_RESOLUTION          65536.0f              // 12位ADC分辨率
#define VBUS_SCALE_FACTOR       (VBUS_DIVIDER_RATIO * ADC_REF_VOLTAGE / ADC_RESOLUTION)
#define CURRENT_MEASURE_PERIOD (float) (1.0f / (float) PWM_FREQ_HZ)

void foc_adc_midpoint(void)
{
    float adc_a_sum = 0;
    float adc_b_sum = 0;
    float adc_c_sum = 0;
    float times = 0;

    for(uint8_t i = 0; i < 32; i++)
    {
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
    float v_to_mod = 1.5f / FOC.Vbus;
    float mod_vd   = Vd * v_to_mod;
    float mod_vq   = Vq * v_to_mod;

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
