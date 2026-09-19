#include <string.h>
#include "foc_cal.h"
#include "encoder.h"

#define CAL_LOOP_HZ (1000U)
#define SAMPLES_PER_PPAIR (128U)
#define MAX_MOTOR_POLE_PAIRS (20U) // 最大支持的极对数

static int encoder_error_arr[MAX_MOTOR_POLE_PAIRS * SAMPLES_PER_PPAIR];

static tCalibStep mCalibStep = CS_NULL;
static uint32_t loop_count;    // 1kHz 时间基准
static float phase_set;        // 电角度设定值
static int16_t sample_count;   // 采样点下标
static float next_sample_time; // 下一个采样时刻 (秒)

void calibration_start(void)
{
    Config.encoder_dir = +1;
    Config.calib_valid = false;

    loop_count = 0;
    phase_set = 0;
    sample_count = 0;
    next_sample_time = 0;
    memset(encoder_error_arr, 0, sizeof(encoder_error_arr));

    mCalibStep = CS_DIR_PP_START;
}

void calibration_end(void)
{
    foc_disarm();
    mCalibStep = CS_NULL;
}

void calibration_loop(void)
{
    float time = (float)loop_count / (float)CAL_LOOP_HZ;
    float voltage_calc = Config.phase_resistance * Config.calib_current;

    // encoder
    float calib_phase_vel = 12.0f * M_PI; // 角速度
    float total_calib_cycles = 14.0f;

    switch (mCalibStep) {
    case CS_DIR_PP_START:
        if (loop_count == 0) {
            foc_arm();
        }
        FOC_voltage(voltage_calc * time, 0, 0.0f);
        if (time >= 2.0f) {
            loop_count = 0;
            encoder.shadow_count = 0;
            phase_set = 0;
            mCalibStep = CS_DIR_PP_LOOP;
        }
        break;
    case CS_DIR_PP_LOOP:
        phase_set += calib_phase_vel / (float)CAL_LOOP_HZ;
        FOC_voltage(voltage_calc, 0, phase_set);
        if (phase_set >= total_calib_cycles * M_2PI) {
            pwm_output_low_sides();
            mCalibStep = CS_DIR_PP_END;
        }
        break;
    case CS_DIR_PP_END:
        if (abs((int)encoder.shadow_count) < 500) {
            Config.calib_valid = false;
        } else {
            Config.encoder_dir = (encoder.shadow_count > 0) ? 1 : -1;
            float mech_revs = fabsf((float)encoder.shadow_count / ENCODER_CPR_F);
            Config.pole_pairs = (int)roundf(total_calib_cycles / mech_revs);
            mCalibStep = CS_ENCODER_START;
        }
        break;

    case CS_ENCODER_START:
        phase_set = 0;
        loop_count = 0;
        sample_count = 0;
        next_sample_time = 0;
        mCalibStep = CS_ENCODER_CW_LOOP;
        break;
    case CS_ENCODER_CW_LOOP:
        if (sample_count < (int32_t)(Config.pole_pairs * SAMPLES_PER_PPAIR)) {
            if (time > next_sample_time) {
                next_sample_time += M_2PI / ((float)SAMPLES_PER_PPAIR * calib_phase_vel);
                float mech_phase = phase_set / (float)Config.pole_pairs;
                float mech_phase_wrapped = fmodf(mech_phase, M_2PI);
                if (mech_phase_wrapped < 0) mech_phase_wrapped += M_2PI;
                int count_ref = (int)(mech_phase_wrapped * ENCODER_CPR_F / M_2PI);
                int actual_count = (Config.encoder_dir == 1) ? encoder.raw : (ENCODER_CPR - 1 - encoder.raw);
                int error = actual_count - count_ref;
                while (error >  ENCODER_CPR / 2) error -= ENCODER_CPR;
                while (error < -ENCODER_CPR / 2) error += ENCODER_CPR;
                encoder_error_arr[sample_count] = error;
                sample_count++;
            }
            phase_set += calib_phase_vel / (float)CAL_LOOP_HZ;
        } else {
            phase_set -= calib_phase_vel / (float)CAL_LOOP_HZ;
            loop_count = 0;
            sample_count--;
            next_sample_time = 0;
            mCalibStep = CS_ENCODER_CCW_LOOP;
        }
        FOC_voltage(voltage_calc, 0, phase_set);
        break;
    case CS_ENCODER_CCW_LOOP:
        if (sample_count >= 0) {
            if (time > next_sample_time) {
                next_sample_time += M_2PI / ((float)SAMPLES_PER_PPAIR * calib_phase_vel);
                float mech_phase = phase_set / (float)Config.pole_pairs;
                float mech_phase_wrapped = fmodf(mech_phase, M_2PI);
                if (mech_phase_wrapped < 0) mech_phase_wrapped += M_2PI;
                int count_ref = (int)(mech_phase_wrapped * ENCODER_CPR_F / M_2PI);
                int actual_count = (Config.encoder_dir == 1) ? encoder.raw : (ENCODER_CPR - 1 - encoder.raw);
                int error = actual_count - count_ref;
                while (error >  ENCODER_CPR / 2) error -= ENCODER_CPR;
                while (error < -ENCODER_CPR / 2) error += ENCODER_CPR;
                encoder_error_arr[sample_count] = (encoder_error_arr[sample_count] + error) / 2;
                sample_count--;
            }
            phase_set -= calib_phase_vel / (float)CAL_LOOP_HZ;
            FOC_voltage(voltage_calc, 0, phase_set);
        } else {
            pwm_output_low_sides();
            sample_count = 0;
            loop_count = 0;
            mCalibStep = CS_ENCODER_END;
        }
        break;
    case CS_ENCODER_END: {
        if ((Config.pole_pairs <= 0) || (Config.pole_pairs > (int32_t)MAX_MOTOR_POLE_PAIRS)) {
            Config.calib_valid = false;
            mCalibStep = CS_NULL;
            break;
        }

        uint32_t total_samples = (uint32_t)Config.pole_pairs * SAMPLES_PER_PPAIR;
        int window = (int)SAMPLES_PER_PPAIR;

        if (sample_count == 0) {
            int64_t sum = 0;
            for (uint32_t i = 0; i < total_samples; i++) {
                sum += encoder_error_arr[i];
            }
            Config.encoder_offset = (int32_t)(sum / (int64_t)total_samples);
        }

        if (sample_count < (int32_t)OFFSET_LUT_NUM) {
            int64_t window_sum = 0;
            for (int j = (-window) / 2; j < (window) / 2; j++) {
                int index = (int)((int32_t)sample_count * (int32_t)total_samples / (int32_t)OFFSET_LUT_NUM) + j;
                while (index < 0) index += (int)total_samples;
                while (index >= (int)total_samples) index -= (int)total_samples;
                window_sum += encoder_error_arr[index];
            }
            int32_t window_avg = (int32_t)(window_sum / window);
            int lut_offset = (int)((int64_t)encoder_error_arr[0] * OFFSET_LUT_NUM / ENCODER_CPR);
            int lut_index = (lut_offset + sample_count) % (int)OFFSET_LUT_NUM;
            if (lut_index < 0) lut_index += (int)OFFSET_LUT_NUM;
            Config.offset_lut[lut_index] = window_avg - Config.encoder_offset;
            sample_count++;
        } else {
            mCalibStep = CS_NULL;
            Config.calib_valid = true;
            foc_request_stage(IDLE);
        }
    } break;

    default:
        break;
    }
    loop_count++;
}
