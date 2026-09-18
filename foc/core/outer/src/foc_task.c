#include "foc_task.h"

// 状态机
static foc_stage_t s_stage = ST_BOOT;
static foc_stage_t s_stage_next = ST_BOOT;
static uint8_t s_next_ready = 1;

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