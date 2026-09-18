#include "foc.h"

volatile foc_data_t FOC;

/* ADC */
#define VBUS_DIVIDER_RATIO 21.0f // (10K+10K+1K)/1K = 21
#define ADC_REF_VOLTAGE 3.3f	 // ADC参考电压3.3V
#define ADC_RESOLUTION 65536.0f	 // 12位ADC分辨率
#define VBUS_SCALE_FACTOR                                                      \
	(VBUS_DIVIDER_RATIO * ADC_REF_VOLTAGE / ADC_RESOLUTION)
#define CURRENT_MEASURE_PERIOD (float)(1.0f / (float)PWM_FREQ_HZ)

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
	float v_to_mod = 1.5f / FOC.Vbus;
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

// 状态机
static foc_stage_t s_stage = ST_BOOT;
static foc_stage_t s_stage_next = ST_BOOT;
static uint8_t s_next_ready = 1; // BOOT 不用退出动作, 直接可切

// 外部接口
static volatile foc_stage_t s_req = ST_BOOT;
static volatile uint8_t s_req_valid = 0;

void foc_request_stage(foc_stage_t stage)
{
	s_req = stage;
	s_req_valid = 1;
}

foc_stage_t foc_get_stage(void)
{
	return s_stage;
}

static void enter_stage(foc_stage_t stage)
{
	switch (stage) {
    case ST_BOOT:
        break;
	case ST_IDLE:
		pwm_set_duty(0, 0, 0);
		break; // 三相下桥全开 = 短路制动
	case ST_RUN:
		// TODO: 清 PI 积分
		break;
	case ST_CAL:
		// TODO: 开始标定
		break;
	case ST_FAULT:
		// TODO: 关断输出
		break;
	default:
		break;
	}
}

static void exit_stage(foc_stage_t stage)
{
	switch (stage) {
	case ST_RUN:
		pwm_set_duty(0, 0, 0);
		break;
	default:
		break;
	}
	s_next_ready = 1; // 想等自举电容充电, 就在这里倒计时
}

static void Task_FOC(void *argument)
{
	(void)argument;

	for (;;) {
		if (s_req_valid) {
			s_req_valid = 0;
			s_stage_next = s_req;
			s_next_ready = 0;
		}

		if (s_stage_next != s_stage) {
			exit_stage(s_stage);
			if (s_next_ready) {
				s_stage = s_stage_next;
				enter_stage(s_stage);
			}
		}

		switch (s_stage) {
		case ST_BOOT:
			s_stage_next = ST_IDLE;
			break; // 自检完进 IDLE
		case ST_IDLE:
			break;
		case ST_RUN:
			// TODO: 速度环 / 位置环
			break;
		case ST_CAL:
			// TODO: 标定流程
			break;
		case ST_FAULT:
			break;
		}

		vTaskDelay(pdMS_TO_TICKS(1));
	}
}

void foc_app_start(void)
{
	if (xTaskCreate(Task_FOC, "foc", 512, NULL, 10, NULL) != pdPASS) {
		printf("foc task create fail\n");
		while (1) {
		}
	}
}