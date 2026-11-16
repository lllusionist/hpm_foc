#include "foc_task.h"
#include "foc_ctrl.h"
#include "encoder.h"
#include "sys.h"
#include "adc.h"
#include <stdio.h>

/* ADC 中断心跳计数 (定义在 foc.c) */
extern volatile uint32_t g_wdg_adc_cnt;

/* 时序一致性断言: 改 configTICK_RATE_HZ / ENCODER_TASK_HZ / PWM_FREQ_HZ
 * 时这些宏必须同步更新, 否则 PLL 增益、校准频率、vTaskDelayUntil 周期
 * 会静默出错. 加编译期检查让改错时直接报错. */
_Static_assert(configTICK_RATE_HZ == 10000,
	       "tick rate changed: check ENCODER_HALF_TICKS / Task_FOC delay");
_Static_assert(ENCODER_HALF_TICKS == 1,
	       "encoder half tick != 1: ENCODER_TASK_HZ or tick rate changed");

static tFSM sFSM;

#define HEARTBEAT_TIMEOUT_MS (500U)
static volatile TickType_t s_last_heartbeat_tick;

void foc_heartbeat_from_isr(void)
{
	s_last_heartbeat_tick = xTaskGetTickCountFromISR();
}

static bool heartbeat_timed_out(void)
{
	TickType_t last = s_last_heartbeat_tick;

	return (xTaskGetTickCount() - last) >=
	       pdMS_TO_TICKS(HEARTBEAT_TIMEOUT_MS);
}

/* 外部请求 */
static volatile tFSMState s_req = BOOT_UP;
static volatile uint8_t s_req_valid = 0;
static volatile bool s_clear_fault_pending;
/* 故障请求优先: 置位后非故障请求被拒绝, 直到 foc_clear_fault_req() */
static volatile uint8_t s_fault_req = 0;

void foc_request_stage(tFSMState state)
{
	/* 故障触发的请求 (IDLE) 优先于上位机的 RUN/CALIBRATION 请求.
	 * s_fault_req 置位后, 只有 IDLE 请求能通过; 其它请求被丢弃. */
	if (s_fault_req && state != IDLE)
		return;
	s_req = state;
	s_req_valid = 1;
}

/* 由 foc_check_and_trip 调用: 故障关断后锁定请求槽 */
void foc_fault_request(tFSMState state)
{
	s_fault_req = 1;
	s_req = state;
	s_req_valid = 1;
}

/* 由上位机 CLEAR_FAULT 或 DISABLE 后调用: 解锁请求槽 */
void foc_clear_fault_req(void)
{
	s_fault_req = 0;
}

void foc_request_fault_clear(void)
{
	s_clear_fault_pending = true;
}

tFSMState foc_get_stage(void)
{
	return sFSM.state;
}

/* 合法状态转移表:
 * - BOOT_UP  -> IDLE (ADC 零点完成后自动, 或零点失败后 IDLE 等待)
 * - IDLE     -> RUN / CALIBRATION / ANTICOGGING
 * - RUN      -> IDLE (故障/禁用)
 * - CALIBRATION -> IDLE (完成/故障)
 * - ANTICOGGING -> IDLE (完成/故障)
 * 任何不在表里的转移都被拒绝 (打印日志, 不改状态) */
static bool transition_allowed(tFSMState from, tFSMState to)
{
	switch (from) {
	case BOOT_UP:
		return (to == IDLE);
	case IDLE:
		return (to == RUN) || (to == CALIBRATION) ||
		       (to == ANTICOGGING) || (to == IDLE);
	case RUN:
		return (to == IDLE);
	case CALIBRATION:
		return (to == IDLE);
	case ANTICOGGING:
		return (to == IDLE);
	default:
		return false;
	}
}

int foc_smf_set_state(tFSMState now_state)
{
	/* 保留此函数防止链接警告; 实际转移逻辑已移入 Task_FOC */
	(void)now_state;
	return -1;
}

static void enter_state(tFSMState state)
{
	switch (state) {
	case BOOT_UP:
		break;
	case IDLE:
		foc_disarm();
		break;
	case RUN:
		CONTROLLER_reset();
		/* 每次进入 RUN 都开始计时，包括从未收到心跳的情况。 */
		taskENTER_CRITICAL();
		s_last_heartbeat_tick = xTaskGetTickCount();
		taskEXIT_CRITICAL();
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

/* 任务级母线保护，返回故障码，0 = 正常。
 *
 * L4 修正: 欠压阈值改为固定比较, 不再与滤波值挂钩, 否则缓慢下跌时
 *         保护会把自己关掉; 阈值 0 表示禁用.
 * L5 修正: 过压/欠压去抖改为"连续 N 次 1kHz 新采样"而不是任务调用次数.
 *         电压 1kHz 更新, 任务 5kHz 执行, 原"3次"实为同一采样重复比较. */
static uint8_t foc_protection_check(void)
{
	/* 电压去抖: 只在有新的 1kHz 电压采样 (adc_voltage_frames 自增) 时才计数 */
	static uint32_t last_volt_frames = 0;
	static uint32_t ov_new_cnt = 0;
	static uint32_t uv_new_cnt = 0;
	static uint32_t ibus_new_cnt = 0;

	float vbus = FOC.Vbus;
	bool new_volt_sample = (FOC.adc_voltage_frames != last_volt_frames);

	if (new_volt_sample)
		last_volt_frames = FOC.adc_voltage_frames;

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

	/* 连续丢帧: adc.c 的计数在有效帧清零、无效帧自增, 必须任务直接读,
	 * 不能依赖回调镜像 (回调只在有效帧被调用, 镜像恒为 0) */
	FOC.adc_current_missed = adc_current_missed();
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
		foc_fault_request(IDLE);
		printf("FAULT %u: Vbus %dmV, I %.2fA (uv %dmV ov %dmV)",
		       (unsigned)fault, (int)(vbus_at_fault * 1000.0f),
		       (double)iq_at_fault,
		       (int)(Config.protect_under_voltage * 1000.0f),
		       (int)(Config.protect_over_voltage * 1000.0f));
		if (foc_get_stage() == CALIBRATION)
			printf(" calib_step=%d", calibration_step());
		if (foc_get_stage() == ANTICOGGING)
			printf(" anticog_step=%d", anticogging_step());
		if (fault == FOC_FAULT_PHASE_OC_ISR) {
			foc_phase_trip_t trip;

			if (foc_read_phase_trip(&trip)) {
				printf(" abc=%.2f/%.2f/%.2fA limit=%.2fA"
				       " profile=%ld frame=%lu Rreq=%.3fV"
				       " Lac=%.3fV",
				       trip.ia, trip.ib, trip.ic, trip.limit,
				       (long)trip.motor_profile,
				       (unsigned long)trip.frames_since_arm,
				       trip.resistance_voltage,
				       trip.inductance_amplitude);
				printf("\n loop: id=%.2f iq=%.2f iq_cmd=%.2f"
				       " vd=%.3f vq=%.3f intd=%.3f intq=%.3f"
				       " ang=%.3f kp=%.4f ki=%.1f",
				       (double)trip.id, (double)trip.iq,
				       (double)trip.iq_cmd, (double)trip.vd,
				       (double)trip.vq, (double)trip.int_d,
				       (double)trip.int_q, (double)trip.angle,
				       (double)trip.kp, (double)trip.ki);
			}
		}
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

		/* 跨层看门狗: 每 100ms (500 拍 @5kHz) 检查 ADC 中断是否还活着.
		 * ADC 计数器在增 -> 喂狗; 否则不喂, 500ms 后硬件复位. */
		if ((tick % 500U) == 0U) {
			static uint32_t last_adc_cnt;
			uint32_t now = g_wdg_adc_cnt;
			if (now != last_adc_cnt) {
				last_adc_cnt = now;
				sys_wdg_feed();
			}
		}

		if (s_req_valid) {
			s_req_valid = 0;
			if (transition_allowed(sFSM.state, s_req)) {
				sFSM.state_next = s_req;
				sFSM.state_next_ready = 0;
			} else {
				printf("FSM: rejected %d->%d\n",
				       (int)sFSM.state, (int)s_req);
			}
		}

		if (sFSM.state_next != sFSM.state) {
			exit_state(sFSM.state);
			if (sFSM.state_next_ready) {
				sFSM.state = sFSM.state_next;
				enter_state(sFSM.state);
			}
		}

		if (s_clear_fault_pending) {
			s_clear_fault_pending = false;
			if (sFSM.state == IDLE && !FOC.is_armed)
				FOC.fault_code = FOC_FAULT_NONE;
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
					foc_fault_request(IDLE);
				}
			}
			/* ADC 零点已就绪: 清除上电时留下的 ADC 故障码, 自动进 IDLE.
			 * foc_init() 在调度器启动前运行, 彼时全局中断关闭,
			 * 第一次标定必然失败并留下 FOC_FAULT_ADC; 任务里重试后应清除. */
			if (FOC.adc_ready && (foc_get_stage() == BOOT_UP)) {
				if (FOC.fault_code == FOC_FAULT_ADC)
					FOC.fault_code = FOC_FAULT_NONE;
				foc_clear_fault_req();
				foc_request_stage(IDLE);
			}
		} break;
		case IDLE:
			if (slow_tick)
				cogging_dump_loop();
			break;
		case RUN:
			foc_check_and_trip();
			if (FOC.is_armed && heartbeat_timed_out()) {
				foc_disarm();
				foc_fault_request(IDLE);
				printf("heartbeat timeout: auto disable\n");
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
