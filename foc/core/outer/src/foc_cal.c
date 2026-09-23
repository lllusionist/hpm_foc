#include <string.h>
#include "foc_cal.h"
#include "encoder.h"
#include "foc_task.h"

#define CAL_LOOP_HZ (1000U)
#define SAMPLES_PER_PPAIR (128U)
#define MAX_MOTOR_POLE_PAIRS (20U) // 最大支持的极对数

/* 磁链测量: 开环电流拖动, 两个转速各采一段, 用两点法消掉 R 项 */
#define FLUX_SPEED1 (2.0f * M_PI * 20.0f) // 低速点 (电角速度 rad/s)
#define FLUX_SPEED2 (2.0f * M_PI * 40.0f) // 高速点
#define FLUX_SETTLE_TIME (0.5f)		  // 每个转速稳定时间 (s)
#define FLUX_MEAS_SAMPLES (300U)	  // 每个转速采样点数 (1kHz)

/* 电阻/电感测量 (参考 dgm / rare_foc) */
#define R_CAL_CYCLES (CAL_LOOP_HZ * 2U) // 电阻: 两点法总时长 2s
#define L_CAL_CYCLES (CAL_LOOP_HZ / 2U) // 电感: 正弦注入时长 0.5s
#define L_INJECT_HZ (1000.0f)		// 电感注入频率 (Hz)

static int encoder_error_arr[MAX_MOTOR_POLE_PAIRS * SAMPLES_PER_PPAIR];

static tCalibStep mCalibStep = CS_NULL;
static uint32_t loop_count;    // 1kHz 时间基准
static float phase_set;        // 电角度设定值
static int16_t sample_count;   // 采样点下标
static float next_sample_time; // 下一个采样时刻 (秒)

static float flux_v_sum, flux_i_sum; // 电压/电流模长累加
static float flux_v1, flux_i1, flux_v2, flux_i2;
static uint32_t flux_cnt;
static bool flux_sat, flux_sat1, flux_sat2;

static float r_v1, r_i1, r_v2, r_i2; // 电阻两点法累加
static uint32_t r_s1, r_s2;
static float l_v_dc, l_v_amp; // 电感注入的直流偏置/正弦幅值

void calibration_start(void)
{
	if (!FOC.adc_ready) {
		FOC.fault_code = FOC_FAULT_ADC;
		foc_request_stage(IDLE);
		mCalibStep = CS_NULL;
		return;
	}

    Config.encoder_dir = +1;
    Config.calib_valid = false;
    FOC.calib_mode     = false;
    FOC.calib_vinj_mode = false;

    loop_count = 0;
    phase_set = 0;
    sample_count = 0;
    next_sample_time = 0;
    memset(encoder_error_arr, 0, sizeof(encoder_error_arr));

    mCalibStep = CS_MOTOR_R_START;
}

void calibration_end(void)
{
    FOC.calib_mode      = false;
    FOC.calib_vinj_mode = false;
    foc_disarm();
    mCalibStep = CS_NULL;
}

void calibration_loop(void)
{
    float time = (float)loop_count / (float)CAL_LOOP_HZ;
    float voltage_calc = Config.phase_resistance * Config.calib_current;
    float drag_v       = voltage_calc;
    if (drag_v < 0.2f) drag_v = 0.2f;
    if (drag_v > (Config.calib_voltage * 0.9f)) drag_v = Config.calib_voltage * 0.9f;

    // encoder
    float calib_phase_vel = 12.0f * M_PI; // 角速度
    float total_calib_cycles = 14.0f;

    switch (mCalibStep) {
    case CS_MOTOR_R_START:
        foc_arm();
        FOC.calib_mode      = true;
        FOC.calib_id        = 0;
        FOC.calib_iq        = 0;
        FOC.calib_angle     = 0;
        FOC.calib_phase_vel = 0;
        r_v1 = r_i1 = r_v2 = r_i2 = 0;
        r_s1 = r_s2 = 0;
        loop_count = 0;
        mCalibStep = CS_MOTOR_R_LOOP;
        break;

    case CS_MOTOR_R_LOOP:
        // 静止, d 轴给电流; 两点法: 前半段 0.25 倍, 后半段满电流
        if (loop_count < (R_CAL_CYCLES / 2U)) {
            FOC.calib_id = Config.calib_current * 0.25f;
            if (loop_count > (uint32_t)(R_CAL_CYCLES * 0.4f)) {
                r_v1 += FOC.Vd_out;
                r_i1 += FOC.Id;
                r_s1++;
            }
        } else {
            FOC.calib_id = Config.calib_current;
            if (loop_count > (uint32_t)(R_CAL_CYCLES * 0.9f)) {
                r_v2 += FOC.Vd_out;
                r_i2 += FOC.Id;
                r_s2++;
            }
        }
        if (loop_count >= R_CAL_CYCLES) {
            FOC.calib_mode = false;
            pwm_output_low_sides();
            mCalibStep = CS_MOTOR_R_END;
        }
        break;

    case CS_MOTOR_R_END: {
        if ((r_s1 > 0U) && (r_s2 > 0U) && !FOC.saturated) {
            float v1 = r_v1 / (float)r_s1;
            float i1 = r_i1 / (float)r_s1;
            float v2 = r_v2 / (float)r_s2;
            float i2 = r_i2 / (float)r_s2;

            if (fabsf(i2 - i1) > 0.05f) {
                Config.phase_resistance = (v2 - v1) / (i2 - i1);
            } else if (i2 > 0.01f) {
                Config.phase_resistance = v2 / i2;
            }
            if (Config.phase_resistance < 0.001f) Config.phase_resistance = 0.001f;

            printf("res: %.4f Ohm (V1=%.3f I1=%.3f V2=%.3f I2=%.3f)\n",
                   Config.phase_resistance, v1, i1, v2, i2);
        } else {
            printf("res measure invalid\n");
        }
        loop_count = 0;
        mCalibStep = CS_MOTOR_L_START;
    } break;

    case CS_MOTOR_L_START:
        l_v_dc  = Config.calib_current * Config.phase_resistance;
        FOC.calib_l_freq = Config.current_ctrl_bw;
        if (FOC.calib_l_freq < 100.0f) FOC.calib_l_freq = 1000.0f;
        l_v_amp = (Config.phase_resistance > 1.0f) ? 3.0f : 1.5f;
        l_v_amp = l_v_amp * (FOC.calib_l_freq / 1000.0f);
        if ((l_v_dc + l_v_amp) > (Config.calib_voltage * 0.9f)) {
            l_v_amp = (Config.calib_voltage * 0.9f) - l_v_dc;
            if (l_v_amp < 0.5f) l_v_amp = 0.5f;
        }
        loop_count          = 0;
        FOC.vinj_omega      = 2.0f * M_PI * FOC.calib_l_freq;
        FOC.vinj_amp        = l_v_amp;
        FOC.vinj_dc         = l_v_dc;
        FOC.inj_phase       = 0;
        FOC.vinj_sum_sin    = 0;
        FOC.vinj_sum_cos    = 0;
        FOC.vinj_samples    = 0;
        FOC.vinj_acc        = false;
        FOC.calib_vinj_mode = true;
        mCalibStep          = CS_MOTOR_L_LOOP;
        break;

    case CS_MOTOR_L_LOOP:
        if (!FOC.vinj_acc && (loop_count >= 100U)) {
            FOC.vinj_acc = true; // 跳过起始瞬态后开始解调累加
        }
        if (loop_count >= L_CAL_CYCLES) {
            FOC.vinj_acc        = false;
            FOC.calib_vinj_mode = false;
            pwm_output_low_sides();
            mCalibStep = CS_MOTOR_L_END;
        }
        break;

    case CS_MOTOR_L_END: {
        float i_amp = 0.0f;
        if (FOC.vinj_samples > 0U) {
            i_amp = 2.0f *
                    sqrtf(SQ(FOC.vinj_sum_sin) + SQ(FOC.vinj_sum_cos)) /
                    (float)FOC.vinj_samples;
        }
        float z  = (i_amp > 0.005f) ? (l_v_amp / i_amp) : 9999.0f;
        float xl = (z > Config.phase_resistance)
                       ? sqrtf(SQ(z) - SQ(Config.phase_resistance))
                       : 0.001f;
        Config.phase_inductance = xl / (2.0f * M_PI * FOC.calib_l_freq);
        FOC_update_current_ctrl_gain(Config.current_ctrl_bw);

        printf("ind: %.6f H (Iamp=%.3f Z=%.3f Vdc=%.3f Vamp=%.3f)\n",
               Config.phase_inductance, i_amp, z, l_v_dc, l_v_amp);

        phase_set  = 0;
        loop_count = 0;
        mCalibStep = CS_DIR_PP_START;
    } break;

    case CS_DIR_PP_START:
        if (loop_count == 0) {
            foc_arm();
        }
        FOC_voltage(drag_v * time, 0, 0.0f);
        if (time >= 2.0f) {
            loop_count = 0;
            encoder.shadow_count = 0;
            phase_set = 0;
            mCalibStep = CS_DIR_PP_LOOP;
        }
        break;
    case CS_DIR_PP_LOOP:
        phase_set += calib_phase_vel / (float)CAL_LOOP_HZ;
        FOC_voltage(drag_v, 0, phase_set);
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

    case CS_FLUX_START:
        Config.flux_linkage = 0;
        FOC.calib_id        = 0;
        FOC.calib_iq        = Config.calib_current;
        FOC.calib_phase_vel = 0;
        FOC.calib_angle     = 0;
        FOC.calib_mode      = true;
        loop_count          = 0;
        mCalibStep          = CS_FLUX_RAMP1;
        break;

    case CS_FLUX_RAMP1:
        // 从 0 缓缓升到 FLUX_SPEED1, 让转子平稳同步
        FOC.calib_phase_vel += FLUX_SPEED1 / (FLUX_SETTLE_TIME * (float)CAL_LOOP_HZ);
        if (FOC.calib_phase_vel > FLUX_SPEED1) FOC.calib_phase_vel = FLUX_SPEED1;
        FOC.calib_angle += FOC.calib_phase_vel / (float)CAL_LOOP_HZ;
        if (FOC.calib_angle >= M_2PI) FOC.calib_angle -= M_2PI;
        if (time > FLUX_SETTLE_TIME) {
            flux_v_sum = 0;
            flux_i_sum = 0;
            flux_cnt   = 0;
            flux_sat   = false;
            loop_count = 0;
            mCalibStep = CS_FLUX_MEAS1;
        }
        break;

    case CS_FLUX_MEAS1:
    case CS_FLUX_MEAS2: {
        FOC.calib_angle += FOC.calib_phase_vel / (float)CAL_LOOP_HZ;
        if (FOC.calib_angle >= M_2PI) FOC.calib_angle -= M_2PI;

        flux_v_sum += sqrtf(SQ(FOC.Vd_out) + SQ(FOC.Vq_out));
        flux_i_sum += sqrtf(SQ(FOC.Id) + SQ(FOC.Iq));
        if (FOC.saturated) flux_sat = true;
        flux_cnt++;

        if (flux_cnt >= FLUX_MEAS_SAMPLES) {
            float v_avg = flux_v_sum / (float)flux_cnt;
            float i_avg = flux_i_sum / (float)flux_cnt;

            if (mCalibStep == CS_FLUX_MEAS1) {
                flux_v1    = v_avg;
                flux_i1    = i_avg;
                flux_sat1  = flux_sat;
                loop_count = 0;
                mCalibStep = CS_FLUX_RAMP2;
            } else {
                flux_v2   = v_avg;
                flux_i2   = i_avg;
                flux_sat2 = flux_sat;
                mCalibStep = CS_FLUX_END;
            }
        }
    } break;

    case CS_FLUX_RAMP2:
        // 从 FLUX_SPEED1 升到 FLUX_SPEED2
        FOC.calib_phase_vel +=
            (FLUX_SPEED2 - FLUX_SPEED1) / (FLUX_SETTLE_TIME * (float)CAL_LOOP_HZ);
        if (FOC.calib_phase_vel > FLUX_SPEED2) FOC.calib_phase_vel = FLUX_SPEED2;
        FOC.calib_angle += FOC.calib_phase_vel / (float)CAL_LOOP_HZ;
        if (FOC.calib_angle >= M_2PI) FOC.calib_angle -= M_2PI;
        if (time > FLUX_SETTLE_TIME) {
            flux_v_sum = 0;
            flux_i_sum = 0;
            flux_cnt   = 0;
            flux_sat   = false;
            loop_count = 0;
            mCalibStep = CS_FLUX_MEAS2;
        }
        break;

    case CS_FLUX_END: {
        FOC.calib_mode = false;
        pwm_output_low_sides();

        if (!flux_sat1 && !flux_sat2 && (flux_i1 > 0.05f) && (flux_i2 > 0.05f)) {
            /* y = R·i + ω·λ (把 ωLi 的平方项扣掉, 两个转速两个方程解 R 和 λ) */
            float t1 = SQ(flux_v1) - SQ(FLUX_SPEED1 * Config.phase_inductance * flux_i1);
            float t2 = SQ(flux_v2) - SQ(FLUX_SPEED2 * Config.phase_inductance * flux_i2);
            float y1 = sqrtf((t1 > 0.0f) ? t1 : 0.0f);
            float y2 = sqrtf((t2 > 0.0f) ? t2 : 0.0f);

            float den = FLUX_SPEED2 * flux_i1 - FLUX_SPEED1 * flux_i2;
            float lambda = (y2 * flux_i1 - y1 * flux_i2) / den;
            float res    = (y1 - FLUX_SPEED1 * lambda) / flux_i1;
            float lambda_simple =
                (flux_v2 - flux_v1) / (FLUX_SPEED2 - FLUX_SPEED1);

            printf("flux: R=%.3f Ohm lambda=%.6f Wb (simple %.6f)\n",
                   res, lambda, lambda_simple);

            if ((res > 0.01f) && (res < 50.0f)) {
                Config.phase_resistance = res;
            }
            if ((lambda > 0.00005f) && (lambda < 1.0f)) {
                Config.flux_linkage = lambda;
            }
            FOC_update_current_ctrl_gain(Config.current_ctrl_bw);
        } else {
            printf("flux measure invalid (saturated or current too small)\n");
        }

        phase_set  = 0;
        loop_count = 0;
        mCalibStep = CS_NULL;
        Config.calib_valid = true;
        CONFIG_save_to_flash();
        foc_request_stage(IDLE);
    } break;

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
        FOC_voltage(drag_v, 0, phase_set);
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
            FOC_voltage(drag_v, 0, phase_set);
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
            loop_count = 0;
            mCalibStep = CS_FLUX_START;
        }
    } break;

    default:
        break;
    }
    loop_count++;
}
