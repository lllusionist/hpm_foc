#include "foc_task.h"
#include "foc_ctrl.h"
#include "encoder.h"
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

/* 保护: 相过流(立即) / 母线过压 / 母线欠压; 返回故障码, 0 = 正常
 *
 * L4 修正: 欠压阈值改为固定比较, 不再与滤波值挂钩, 否则缓慢下跌时
 *         保护会把自己关掉; 阈值 0 表示禁用.
 * L5 修正: 过压/欠压去抖改为"连续 N 次 1kHz 新采样"而不是任务调用次数.
 *         电压 1kHz 更新, 任务 5kHz 执行, 原"3次"实为同一采样重复比较. */
static uint8_t foc_protection_check(void)
{
	/* 电压去抖: 只在 adc_voltage_missed 没有增加时才认为是新采样 */
	static uint32_t last_volt_frames = 0;
	static uint32_t ov_new_cnt = 0;
	static uint32_t uv_new_cnt = 0;
	static uint32_t ibus_new_cnt = 0;

	float vbus = FOC.Vbus;
	bool new_volt_sample = (FOC.adc_voltage_missed == 0U) &&
			       (FOC.adc_current_frames != last_volt_frames);

	if (new_volt_sample)
		last_volt_frames = FOC.adc_current_frames;

	if (foc_over_current())
		return FOC_FAULT_PHASE_OC;

	/* 母线电流保护: 滤波值 + 新采样去抖 */
	if (FOC.Ibus_filt > Config.protect_i_bus_max) {
		if (new_volt_sample && (++ibus_new_cnt >= 3U))
			return FOC_FAULT_IBUS_OC;
	} else {
		ibus_new_cnt = 0;
	}

	/* 过压: 需连续 3 次真实新采样 */
	if (vbus > Config.protect_over_voltage) {
		if (new_volt_sample && (++ov_new_cnt >= 3U))
			return FOC_FAULT_OVER_VOLTAGE;
	} else {
		ov_new_cnt = 0;
	}

	/* 欠压: 阈值 0 = 禁用; 需连续 5 次真实新采样 (~5ms)
	 * 不再与 Vbus_filt 比较, 防止缓慢下跌时阈值跟着降导致保护自闭 */
	if ((Config.protect_under_voltage > 0.0f) &&
	    (vbus < Config.protect_under_voltage)) {
		if (new_volt_sample && (++uv_new_cnt >= 5U))
			return FOC_FAULT_UNDER_VOLTAGE;
	} else {
		uv_new_cnt = 0;
	}

	return FOC_FAULT_NONE;
}

static void foc_check_and_trip(void)
{
	static uint32_t last_frames;
	static TickType_t last_change_tick;
	static uint32_t last_phase_seq;
	static TickType_t last_seq_tick;
	uint8_t fault = foc_protection_check();
	float vbus = FOC.Vbus;
	uint32_t frames;

	if ((vbus <= 2.0f) || (vbus >= 70.0f))
		fault = FOC_FAULT_ADC;

	if (FOC.adc_current_missed >= 3U)
		fault = FOC_FAULT_ADC;

	/* 按真实时间判停更: 任务补跑间隔只有几微秒, 按次数会误报 */
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

	/* 编码器停更检测: phase_seq 应每 200µs (5kHz) 递增,
	 * 超过 5ms 没变说明 SPI/DMA 链路断了 */
	if (encoder.phase_seq != last_phase_seq) {
		last_phase_seq = encoder.phase_seq;
		last_seq_tick = xTaskGetTickCount();
	} else if ((xTaskGetTickCount() - last_seq_tick) >
		   pdMS_TO_TICKS(ENCODER_SEQ_TIMEOUT_MS)) {
		fault = FOC_FAULT_ENCODER;
	}

	/* ISR 级硬限幅已关断并置码, 任务只需补做状态切换 */
	if (FOC.fault_code == FOC_FAULT_PHASE_OC_ISR)
		fault = FOC_FAULT_PHASE_OC_ISR;

	if (fault != FOC_FAULT_NONE) {
		/* 先关断再打印: printf 走阻塞 UART, 一行约 5ms.
		 * 现场值在关断前取出 (foc_disarm 会清零 Iq) */
		float iq_at_fault = FOC.Iq;
		float vbus_at_fault = FOC.Vbus;

		foc_disarm();
		FOC.fault_code = fault;
		foc_request_stage(IDLE);
		printf("FAULT %u: Vbus %dmV, I %.2fA (uv %dmV ov %dmV)",
		       (unsigned)fault, (int)(vbus_at_fault * 1000.0f),
		       (double)iq_at_fault,
		       (int)(Config.protect_under_voltage * 1000.0f),
		       (int)(Config.protect_over_voltage * 1000.0f));
		if (foc_get_stage() == CALIBRATION)
			printf(" calib_step=%d", calibration_step());
		if (foc_get_stage() == ANTICOGGING)
			printf(" anticog_step=%d", anticogging_step());
		printf("\n");
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
			if (slow_tick)
				cogging_dump_loop();
			break;
		case RUN:
			foc_check_and_trip();
			/* 指令超时: 上位机断连后主动失能, 防止电机按
			 * 最后命令无限运行 */
			if (CONTROLLER_cmd_timed_out()) {
				printf("cmd timeout: auto disable\n");
				foc_disarm();
				foc_request_stage(IDLE);
			}
			break;
		case CALIBRATION:
			if (slow_tick)
				calibration_loop(); /* 校准 1kHz */
			foc_check_and_trip();
			break;
		case ANTICOGGING:
			if (slow_tick) {
				cogging_dump_loop();
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
