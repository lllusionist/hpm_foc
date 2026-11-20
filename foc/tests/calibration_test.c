#include <assert.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "foc_cal.h"

#define CAL_TEST_TICKS (40000U)
#define CAL_TEST_CPR (ENCODER_CPR_F / 1000.0f)
#define CAL_TEST_POLE_PAIRS (14)
#define CAL_TEST_FLUX_WB (0.000846f)

typedef enum {
	TEST_SUCCESS_5210,
	TEST_SUCCESS_4310,
	TEST_NO_RESISTANCE_RESPONSE,
	TEST_BAD_INDUCTANCE,
	TEST_NO_ROTATION,
	TEST_BAD_FLUX,
	TEST_SOFT_CURRENT_TRIP,
	TEST_OPTIONAL_J_TRIP,
	TEST_FLASH_FAILURE,
	TEST_HARD_CURRENT_TRIP,
	TEST_RTEST_SATURATION,
} test_scenario_t;

volatile foc_data_t FOC;
volatile tEncoder encoder;
tConfig Config;
tController Controller;
static tConfig saved;
static bool idle_requested;
static bool soft_trip_logged;
static bool save_failed_logged;
static bool flash_failure;
static unsigned int saves;
static float resistance_voltage;
static float actual_resistance;
static float actual_inductance;

static int calibration_log(const char *format, ...)
{
	char line[256];
	va_list arguments;

	va_start(arguments, format);
	vsnprintf(line, sizeof(line), format, arguments);
	va_end(arguments);
	if (strstr(line, "calib: OVER-CURRENT") != NULL) {
		assert(!FOC.is_armed);
		soft_trip_logged = true;
	}
	if (strstr(line, "flash=fail") != NULL)
		save_failed_logged = true;
	return 0;
}

#define printf calibration_log
#include "foc_cal.c"
#undef printf

void foc_arm(void)
{
	FOC.is_armed = FOC.adc_ready && (FOC.fault_code == FOC_FAULT_NONE);
}

void foc_disarm(void)
{
	FOC.is_armed = false;
	FOC.Iq = 0.0f;
	FOC.Id = 0.0f;
	resistance_voltage = 0.0f;
}

void foc_request_stage(tFSMState state)
{
	assert(state == IDLE);
	idle_requested = true;
}

void foc_set_resistance_voltage(float voltage)
{
	resistance_voltage = voltage;
}

void FOC_update_current_ctrl_gain(float bandwidth)
{
	Config.current_kp = Config.phase_inductance * bandwidth * M_2PI;
	Config.current_ki = Config.phase_resistance * bandwidth * M_2PI;
}

uint8_t CONFIG_save_to_flash(void)
{
	assert(!FOC.is_armed);
	saves++;
	if (flash_failure)
		return 1U;
	saved = Config;
	return 0U;
}

static void set_up(test_scenario_t scenario)
{
	bool low_current = scenario == TEST_SUCCESS_4310;

	actual_resistance = low_current ? 8.29f : 0.0421f;
	actual_inductance = low_current ? 0.0034f : 0.000012f;
	Config = (tConfig){
		.calib_valid = 1,
		.pole_pairs = CAL_TEST_POLE_PAIRS,
		.encoder_dir = 1,
		.phase_resistance = low_current ? 8.29f : 0.030f,
		.phase_inductance = actual_inductance,
		.flux_linkage = CAL_TEST_FLUX_WB,
		.calib_current = low_current ? 0.5f : 3.0f,
		.calib_voltage = low_current ? 10.0f : 1.0f,
		.calib_flux_iq_gain = low_current ? 0.8f : 1.5f,
		.calib_flux_rtest_ratio = 0.7f,
		.calib_j_iq_gain = 0.6f,
		.current_ctrl_bw = 300.0f,
		.velocity_limit = 40.0f,
		.anticogging_enable = 1,
		.cogging_map_valid = 1,
		.cogging_map = {100, 200, 300},
	};
	FOC = (foc_data_t){.adc_ready = true};
	encoder = (tEncoder){0};
	saved = Config;
	flash_failure = (scenario == TEST_FLASH_FAILURE);
	Controller.ctrl_mode = CONTROL_MODE_POSITION_PROFILE;
	idle_requested = false;
	soft_trip_logged = false;
	save_failed_logged = false;
	saves = 0U;
}

static void simulate_feedback(test_scenario_t scenario)
{
	int step = calibration_step();
	float phase_current = 0.0f;
	float electrical_velocity = FOC.calib_phase_vel;
	float mechanical_velocity = electrical_velocity /
				    (M_2PI * CAL_TEST_POLE_PAIRS);

	switch (step) {
	case CS_MOTOR_R_LOOP:
		if (scenario != TEST_NO_RESISTANCE_RESPONSE)
			phase_current = resistance_voltage / actual_resistance;
		if (scenario == TEST_SOFT_CURRENT_TRIP)
			phase_current = 10.0f;
		break;

	case CS_MOTOR_L_LOOP:
		if (scenario == TEST_HARD_CURRENT_TRIP) {
			FOC.is_armed = false;
			FOC.fault_code |= FOC_FAULT_PHASE_OC_ISR;
		}
		break;

	case CS_MOTOR_L_END: {
		float impedance = hypotf(actual_resistance,
					 FOC.vinj_omega * actual_inductance);

		if (scenario == TEST_BAD_INDUCTANCE)
			impedance = actual_resistance * 0.8f;
		FOC.vinj_samples = 100U;
		FOC.vinj_sum_sin = 50.0f * FOC.vinj_amp / impedance;
		FOC.vinj_sum_cos = 0.0f;
		break;
	}

	case CS_DIR_PP_LOOP:
		if (scenario != TEST_NO_ROTATION)
			encoder.shadow_count +=
			    (int64_t)(CAL_TEST_CPR * mechanical_velocity);
		break;

	case CS_ENCODER_CW_LOOP:
	case CS_ENCODER_CCW_LOOP: {
		float turns = (FOC.calib_angle + M_PI / 2.0f) /
			      (M_2PI * CAL_TEST_POLE_PAIRS);
		float angle_counts = (turns - floorf(turns)) * ENCODER_CPR_F;

		encoder.raw = (uint32_t)angle_counts;
		break;
	}

	case CS_FLUX_MEAS1:
	case CS_FLUX_RTEST:
	case CS_FLUX_MEAS2:
		FOC.Iq = FOC.calib_iq;
		FOC.Vd_out = -electrical_velocity * actual_inductance * FOC.Iq;
		FOC.Vq_out = actual_resistance * FOC.Iq +
			     electrical_velocity * CAL_TEST_FLUX_WB;
		encoder.vel = mechanical_velocity;
		FOC.saturated =
		    (scenario == TEST_BAD_FLUX && step == CS_FLUX_MEAS1) ||
		    (scenario == TEST_RTEST_SATURATION &&
		     step == CS_FLUX_RTEST);
		break;

	case CS_MOTOR_J_SPINUP:
	case CS_MOTOR_J_ACC:
		encoder.vel = 2.1f;
		if (scenario == TEST_OPTIONAL_J_TRIP)
			phase_current = 10.0f;
		break;

	default:
		break;
	}

	FOC.Ia = phase_current / FOC_CURRENT_SCALE;
	FOC.Ib = -FOC.Ia * 0.5f;
	FOC.Ic = FOC.Ib;
}

static void run_calibration(test_scenario_t scenario)
{
	int final_step;
	float inductance_ratio;
	float flux_current, expected_flux;

	set_up(scenario);
	calibration_start();
	for (unsigned int i = 0U; i < CAL_TEST_TICKS && !idle_requested; i++) {
		simulate_feedback(scenario);
		calibration_loop();
	}
	assert(idle_requested);
	final_step = calibration_step();
	calibration_loop();
	calibration_end();
	assert(!FOC.is_armed);
	assert(Controller.ctrl_mode == CONTROL_MODE_POSITION_PROFILE);
	assert(Config.velocity_limit == 40.0f);

	switch (scenario) {
	case TEST_SUCCESS_5210:
	case TEST_SUCCESS_4310:
	case TEST_OPTIONAL_J_TRIP:
	case TEST_RTEST_SATURATION:
		assert(Config.calib_valid == 1 && saves == 1U);
		assert(saved.calib_valid == 1 && saved.cogging_map_valid == 0);
		assert(Config.cogging_map_valid == saved.cogging_map_valid);
		assert(saved.pole_pairs == CAL_TEST_POLE_PAIRS);
		inductance_ratio = saved.phase_inductance / actual_inductance;
		assert(fabsf(inductance_ratio - 1.0f) < 0.02f);
		flux_current = Config.calib_current * Config.calib_flux_iq_gain;
		expected_flux = hypotf(CAL_TEST_FLUX_WB,
				       actual_inductance * flux_current);
		assert(fabsf(saved.flux_linkage / expected_flux - 1.0f) <
		       0.002f);
		break;
	case TEST_FLASH_FAILURE:
		assert(Config.calib_valid == 1 && saves == 1U);
		assert(save_failed_logged);
		assert(saved.cogging_map_valid == 1);
		break;
	default:
		assert(Config.calib_valid == 0 && saves == 0U);
		assert(saved.calib_valid == 1 && saved.cogging_map_valid == 1);
		break;
	}

	if (scenario == TEST_SOFT_CURRENT_TRIP ||
	    scenario == TEST_OPTIONAL_J_TRIP)
		assert(soft_trip_logged);
	if (scenario == TEST_HARD_CURRENT_TRIP) {
		assert(final_step == CS_MOTOR_L_LOOP);
		assert(FOC.fault_code & FOC_FAULT_PHASE_OC_ISR);
	}
	if (scenario == TEST_NO_ROTATION)
		assert(final_step == CS_DIR_PP_END);
}

int main(int argc, char **argv)
{
	test_scenario_t scenario;

	assert(argc == 2);
	scenario = (test_scenario_t)atoi(argv[1]);
	run_calibration(scenario);
	if (scenario == TEST_BAD_INDUCTANCE)
		run_calibration(TEST_SUCCESS_5210);
	return 0;
}
