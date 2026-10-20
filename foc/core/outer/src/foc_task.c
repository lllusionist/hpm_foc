#include "foc_task.h"
#include "foc_ctrl.h"
#include <stdio.h>

static tFSM sFSM;

/* 外部请求 */
static volatile tFSMState s_req = BOOT_UP;
static volatile uint8_t s_req_valid = 0;

void foc_request_stage(tFSMState state)
{
	s_req = state;
	s_req_valid = 1;
}

tFSMState foc_get_stage(void)
{
	return sFSM.state;
}

void foc_smf_init(void)
{
	sFSM.state = BOOT_UP;
	sFSM.state_next = BOOT_UP;
	sFSM.state_next_ready = 0;
}

int foc_smf_set_state(tFSMState now_state)
{
	int ret = 0;
	/* 当前状态 */
	switch (sFSM.state) {
	case BOOT_UP:
		if (now_state == IDLE)
			sFSM.state_next = IDLE;
		else
			ret = -1;
		break;
	case IDLE:
		switch (now_state) {
		case IDLE:
			foc_disarm();
			sFSM.state_next = IDLE;
			break;
		case RUN:
			break;
		case CALIBRATION:
			break;
		default:
			break;
		}
		break;
	case RUN:
		break;
	case CALIBRATION:
		break;
	default:
		break;
	}

	return ret;
}

static void enter_state(tFSMState state)
{
	switch (state) {
	case BOOT_UP:
		break;
	case IDLE:
		foc_disarm();
		pwm_output_low_sides();
		break;
	case RUN:
		CONTROLLER_reset();
		foc_arm();
		if (!FOC.is_armed)
			foc_request_stage(IDLE);
		break;
	case CALIBRATION:
		calibration_start();
		break;
	case ANTICOGGING:
		anticogging_start();
		break;
	default:
		break;
	}
}

static void exit_state(tFSMState state)
{
	switch (state) {
	case BOOT_UP:
		sFSM.state_next_ready = 1;
		break;
	case IDLE:
		sFSM.state_next_ready = 1;
		break;
	case RUN:
		sFSM.state_next_ready = 1;
		break;
	case CALIBRATION:
		calibration_end();
		sFSM.state_next_ready = 1;
		break;
	case ANTICOGGING:
		anticogging_end();
		sFSM.state_next_ready = 1;
		break;
	default:
		break;
	}
}

static bool foc_over_current(void)
{
	float limit = Config.protect_over_current;
	float ia = (FOC.Ia - FOC.Ia_midpoint) * FOC_CURRENT_SCALE;
	float ib = (FOC.Ib - FOC.Ib_midpoint) * FOC_CURRENT_SCALE;
	float ic = (FOC.Ic - FOC.Ic_midpoint) * FOC_CURRENT_SCALE;

	return (fabsf(ia) > limit) || (fabsf(ib) > limit) ||
	       (fabsf(ic) > limit);
}

/* 保护: 相过流(立即) / 母线过压 / 母线欠压; 返回故障码, 0 = 正常 */
static uint8_t foc_protection_check(void)
{
	static bool uv_warned = false;
	static uint32_t ov_cnt = 0;
	static uint32_t uv_cnt = 0;
	static uint32_t ibus_oc_cnt = 0;

	float vbus = FOC.Vbus;

	bool uv_ok = (Config.protect_under_voltage < (FOC.Vbus_filt * 0.9f));

	if (!uv_warned && (FOC.Vbus_filt > 1.0f) && !uv_ok) {
		uv_warned = true;
		printf("protect: UV threshold %dmV too high for bus %dmV, UV "
		       "disabled\n",
		       (int)(Config.protect_under_voltage * 1000.0f),
		       (int)(FOC.Vbus_filt * 1000.0f));
	}

	if (foc_over_current())
		return FOC_FAULT_PHASE_OC;

	/* 母线电流保护 (滤波值 + 计数去抖, 与过压一致) */
	if (FOC.Ibus_filt > Config.protect_i_bus_max) {
		if (++ibus_oc_cnt >= 3U)
			return FOC_FAULT_IBUS_OC;
	} else {
		ibus_oc_cnt = 0;
	}

	if (vbus > Config.protect_over_voltage) {
		if (++ov_cnt >= 3U)
			return FOC_FAULT_OVER_VOLTAGE;
	} else {
		ov_cnt = 0;
	}

	if (uv_ok && (vbus < Config.protect_under_voltage)) {
		if (++uv_cnt >= 20U)
			return FOC_FAULT_UNDER_VOLTAGE;
	} else {
		uv_cnt = 0;
	}

	return FOC_FAULT_NONE;
}

static void foc_check_and_trip(void)
{
	static uint32_t last_frames;
	static TickType_t last_change_tick;
	uint8_t fault = foc_protection_check();
	float vbus = FOC.Vbus;
	uint32_t frames;

	if ((vbus <= 2.0f) || (vbus >= 70.0f))
		fault = FOC_FAULT_ADC;

	if (FOC.adc_current_missed >= 3U)
		fault = FOC_FAULT_ADC;

	/*
	 * 停更判据按真实时间而非调用次数: 任务补跑时间隔仅几微秒,
	 * "连续 N 次相等" 会误报 (阻塞 printf 就会触发)。
	 */
	frames = FOC.adc_current_frames;
	if (frames != last_frames) {
		last_frames = frames;
		last_change_tick = xTaskGetTickCount();
	} else if ((xTaskGetTickCount() - last_change_tick) >
		   pdMS_TO_TICKS(10)) {
		fault = FOC_FAULT_ADC;
	}

	if (FOC.adc_voltage_missed >= 3U)
		fault = FOC_FAULT_ADC;
	if (FOC.adc_ibus_missed >= 3U)
		fault = FOC_FAULT_ADC;

	if (fault != FOC_FAULT_NONE) {
		FOC.fault_code = fault;
		printf("FAULT %u: Vbus %dmV, I %.2fA (uv %dmV ov %dmV)\n",
		       (unsigned)fault, (int)(FOC.Vbus * 1000.0f),
		       (double)FOC.Iq,
		       (int)(Config.protect_under_voltage * 1000.0f),
		       (int)(Config.protect_over_voltage * 1000.0f));
		foc_disarm();
		foc_request_stage(IDLE);
	}
}

static void Task_FOC(void *argument)
{
	TickType_t last = xTaskGetTickCount();
	uint32_t tick = 0;

	(void)argument;

	for (;;) {
		/* 5kHz 任务, 校准每 5 拍调一次 = 1kHz */
		tick++;
		bool slow_tick = ((tick % 5U) == 0U);

		if (s_req_valid) {
			s_req_valid = 0;
			sFSM.state_next = s_req;
			sFSM.state_next_ready = 0;
		}

		if (sFSM.state_next != sFSM.state) {
			exit_state(sFSM.state);
			if (sFSM.state_next_ready) {
				sFSM.state = sFSM.state_next;
				enter_state(sFSM.state);
			}
		}

		if (slow_tick)
			foc_update_voltages(); /* 三相/母线电压 1kHz */

		switch (sFSM.state) {
		case BOOT_UP: {
			static bool midpoint_done;
			if (!midpoint_done) {
				midpoint_done = true;
				if (!FOC.adc_ready && !foc_adc_midpoint()) {
					FOC.fault_code = FOC_FAULT_ADC;
					printf("ADC midpoint calibration "
					       "failed\n");
					foc_request_stage(IDLE);
				}
			}
		} break;
		case IDLE:
			break;
		case RUN:
			foc_check_and_trip();
			break;
		case CALIBRATION:
			if (slow_tick)
				calibration_loop(); /* 校准 1kHz */
			foc_check_and_trip();
			break;
		case ANTICOGGING:
			if (slow_tick) {
				if (anticogging_loop()) /* 齿槽补偿标定 1kHz */
					foc_request_stage(IDLE);
			}
			foc_check_and_trip();
			break;
		default:
			break;
		}

		CONTROLLER_loop(); /* 外环 5kHz */

		vTaskDelayUntil(&last, 2); /* 2 tick @10kHz = 0.2ms */
	}
}

void foc_app_start(void)
{
	if (xTaskCreate(Task_FOC, "foc", 512, NULL, 15, NULL) != pdPASS) {
		printf("foc task create fail\n");
		while (1) {
		}
	}
}
