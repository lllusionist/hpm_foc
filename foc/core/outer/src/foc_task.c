#include "foc_task.h"
#include <stdio.h>

static tFSM sFSM;

// 外部请求
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
    sFSM.state            = BOOT_UP;
    sFSM.state_next       = BOOT_UP;
    sFSM.state_next_ready = 0;
}

int foc_smf_set_state(tFSMState now_state)
{
	int ret = 0;
	switch (sFSM.state) // 当前状态
	{
	case BOOT_UP:
		if(now_state == IDLE) {
			sFSM.state_next = IDLE;
		}
		else {
			ret = -1;
		}
		break;
	case IDLE:
		switch (now_state)
		{
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
        foc_arm();
        break;
    case CALIBRATION:
        calibration_start();
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
    default:
        break;
    }
}

// 用换算成安培的三相电流做过流判断 (FOC.Ia 是原始码值)
static bool foc_over_current(void)
{
    float limit = Config.protect_over_current;
    float ia = (FOC.Ia - FOC.Ia_midpoint) * FOC_CURRENT_SCALE;
    float ib = (FOC.Ib - FOC.Ib_midpoint) * FOC_CURRENT_SCALE;
    float ic = (FOC.Ic - FOC.Ic_midpoint) * FOC_CURRENT_SCALE;

    return (fabsf(ia) > limit) || (fabsf(ib) > limit) || (fabsf(ic) > limit);
}

static void Task_FOC(void *argument)
{
    (void)argument;

    for (;;) {
        if (s_req_valid) {
            s_req_valid           = 0;
            sFSM.state_next       = s_req;
            sFSM.state_next_ready = 0;
        }

        if (sFSM.state_next != sFSM.state) {
            exit_state(sFSM.state);
            if (sFSM.state_next_ready) {
                sFSM.state = sFSM.state_next;
                enter_state(sFSM.state);
            }
        }

        switch (sFSM.state) {
        case BOOT_UP: {
            static bool midpoint_done;
            if (!midpoint_done) {
                midpoint_done = true;
                foc_adc_midpoint();
            }
        } break;
        case IDLE:
            break;
        case RUN:
            if (foc_over_current()) {
                foc_disarm();
                foc_request_stage(IDLE);
            }
            break;
        case CALIBRATION:
            calibration_loop();
            if (foc_over_current()) {
                foc_disarm();
                foc_request_stage(IDLE);
            }
            break;
        default:
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
