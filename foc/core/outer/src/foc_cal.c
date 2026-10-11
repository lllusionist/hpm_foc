#include <string.h>
#include "foc_cal.h"
#include "encoder.h"
#include "foc_task.h"
#include "foc_ctrl.h"

#define CAL_LOOP_HZ (1000U)
#define SAMPLES_PER_PPAIR (512U)
#define LUT_WINDOW        (128U) /* offset LUT 滑动平均窗口: 与原始平滑强度相当(512密度下≈12.9°) */
#define MAX_MOTOR_POLE_PAIRS (20U) // 最大支持的极对数

/* 磁链测量: 开环电流拖动, 两个转速各采一段, 用两点法消掉 R 项 */
#define FLUX_SPEED1 (2.0f * M_PI * 10.0f) // 低速点 (电角速度 rad/s)
#define FLUX_SPEED2 (2.0f * M_PI * 30.0f) // 高速点
#define FLUX_SETTLE_TIME (1.5f)		  // 每个转速稳定时间 (s)
#define FLUX_MEAS_SAMPLES (800U)	  // 每个转速采样点数 (1kHz -> 0.5s)
#define FLUX_IQ_GAIN (1.4f)   /* 磁链标定拖动电流 = calib_current × 该系数
                               * (拖动要克服齿槽/摩擦, 5A 会失步产生异响) */

/* 电阻/电感测量 (参考 dgm / rare_foc) */
#define R_CAL_CYCLES (CAL_LOOP_HZ * 2U) // 电阻: 两点法总时长 2s
#define L_CAL_CYCLES (CAL_LOOP_HZ / 2U) // 电感: 正弦注入时长 0.5s
#define L_INJECT_HZ (1000.0f)		// 电感注入频率 (Hz)

/* ===== 自适应注入参数 (CN103560736B「永磁同步电机静止型离线参数辨识方法」) =====
 * 该专利的核心是: 注入电压【从小到大逐步增加】, 直到实测电流达到阈值,
 * 而不是用固定电压。这样对小电阻电机天然不会过流。
 */
#define CALIB_R_STEP_V (0.05f)		    // Δu2: 电阻自适应升压步长 (V); 太小会被死区/PWM量化淹没
#define R_CAL_WINDOW   (CAL_LOOP_HZ / 5U)  // 每个工作点停留 200ms 后测稳态电流
#define CALIB_L_MIN_V  (0.05f)		    // 电感注入幅值下限 (V), 保底信噪比

static int encoder_error_arr[MAX_MOTOR_POLE_PAIRS * SAMPLES_PER_PPAIR];

static tCalibStep mCalibStep = CS_NULL;
static uint32_t loop_count;    // 1kHz 时间基准
static float phase_set;        // 电角度设定值
static int16_t sample_count;   // 采样点下标
static float next_sample_time; // 下一个采样时刻 (秒)
static int64_t start_count_long; // 极对数标定起点计数 (对应 rare 的 start_count_long)
static bool dir_pp_runin;        // 极对数标定: 预跑阶段是否已完成

static float flux_v_sum, flux_i_sum; // 电压/电流模长累加
static float flux_v1, flux_i1, flux_v2, flux_i2;
static float flux_vel1, flux_vel2;   // 两个转速点的实测机械转速 (rps)
static float flux_vel_sum;           // 转速窗口累加 (取平均, 避免撞上噪声尖峰)
static uint32_t flux_cnt;
static bool flux_sat, flux_sat1, flux_sat2;

/* 电阻: 自适应两点伏安法 (CN103560736B 步骤3) */
static float    r_volt;   /* 当前注入电压 ures (V) */
static float    r_i1;     /* 上一个工作点的稳态电流 Ires1 (A) */
static float    r_isum;   /* 窗口内电流累加 */
static uint32_t r_icnt;   /* 窗口内累加计数 */
static uint32_t r_s1;     /* 升压次数 */

/* 电感: 注入幅值自适应 (CN103560736B 步骤1 的 d 轴脉动版本) */
static float   l_v_dc, l_v_amp;  /* 注入的直流偏置 / 正弦幅值 */
static uint8_t l_pass;           /* 0 = 探测轮, 1 = 测量轮 */

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

    /* 每次标定先回显关键参数: 便于确认"恢复默认参数"是否已让新配置生效。
     * 若 cur 不是 5.00A / vol 不是 1.00V, 说明 Flash 里还是旧配置。 */
    printf("calib: cur=%.2fA vol=%.2fV step=%.3fV R0=%.4fOhm\n",
           Config.calib_current, Config.calib_voltage,
           CALIB_R_STEP_V, Config.phase_resistance);

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

    /* ===== 平台适配 =====
     * 1) rare 的 FOC.Ia 在 get_iphase_a() 里已换算成安培;
     *    本工程 FOC.Ia 是原始 ADC 值, 需减中点再乘比例。
     * 2) 电感注入的 1000Hz 正弦无法在 1kHz 循环里产生 (见 CS_MOTOR_L_START),
     *    注入与解调放到 40kHz 的 foc_adc_callback 里, 算法与 rare 一致。
     * 注: 电阻/电感已按 CN103560736B 改成【自适应注入】, 不再逐行等同 rare。 */
    const float ia_meas = (FOC.Ia - FOC.Ia_midpoint) * FOC_CURRENT_SCALE;

    float voltage_calc = Config.phase_resistance * Config.calib_current;
    float drag_v       = voltage_calc;
    if (drag_v < 0.2f) drag_v = 0.2f;
    if (drag_v > (Config.calib_voltage * 0.9f)) drag_v = Config.calib_voltage * 0.9f;

    // encoder
    float calib_phase_vel = 12.0f * M_PI; // 角速度
    float total_calib_cycles = 14.0f;

    switch (mCalibStep) {
    case CS_MOTOR_R_START:
        loop_count = 0;
        r_volt  = CALIB_R_STEP_V;   /* 步骤3-1: ures = Δu2 */
        r_i1    = 0.0f;             /* Ires1 */
        r_isum  = 0.0f;
        r_icnt  = 0;
        r_s1    = 0;                /* 升压次数 */
        FOC.calib_mode      = false;  /* 平台适配: 本工程电流环校准模式必须关闭 */
        FOC.calib_vinj_mode = false;  /* 平台适配: 本工程电压注入模式必须关闭 */
        foc_arm();                    /* = rare 的 FOC_PWM_START() */
        mCalibStep = CS_MOTOR_R_LOOP;
        break;

    case CS_MOTOR_R_LOOP: {
        /* 在 d 轴注入恒定电压 ures (开环, 不经过电流环) */
        FOC_voltage(r_volt, 0, 0);

        /* 步骤3-2: 停留 200ms 后取平均作为该工作点的稳态电流 */
        r_isum += ia_meas;
        r_icnt++;

        if (loop_count >= R_CAL_WINDOW) {
            float Is = fabsf(r_isum / (float)r_icnt);
            r_isum = 0.0f;
            r_icnt = 0;
            loop_count = 0;

            if ((Is < Config.calib_current) &&
                ((r_volt + CALIB_R_STEP_V) <= (Config.calib_voltage * 0.9f))) {
                /* 步骤3-3: 电流未达阈值 -> 记录本点, 升压继续 */
                r_i1 = Is;
                r_volt += CALIB_R_STEP_V;
                r_s1++;
            } else {
                /* 步骤3-4: 电流达到阈值 -> Rs = Δu2 / (Ires2 - Ires1)
                 * 只需电压【增量】准确; 两工作点靠得近, 逆变器死区/压降等
                 * 非线性因素相减抵消 */
                float di = Is - r_i1;
                if (fabsf(di) > 1e-4f) {
                    Config.phase_resistance = CALIB_R_STEP_V / di;
                }
                printf("res: %.4f Ohm (ures=%.3f I1=%.4f I2=%.4f steps=%u)\n",
                       Config.phase_resistance, r_volt, r_i1, Is, (unsigned)r_s1);
                mCalibStep = CS_MOTOR_R_END;
            }
        }
    } break;

    case CS_MOTOR_R_END: {
        if (Config.phase_resistance < 0.001f) Config.phase_resistance = 0.001f;
        loop_count = 0;
        mCalibStep = CS_MOTOR_L_START;
    } break;

    case CS_MOTOR_L_START: {
        l_v_dc = Config.calib_current * Config.phase_resistance;
        l_pass = 0;   /* 第 0 轮 = 探测轮 */

        /* ===== 自适应注入幅值 (CN103560736B 步骤1 的 d 轴脉动版本) =====
         * rare 原版用固定 1.5V/3V: 对低阻电机(如 0.09Ω)会产生几十安电流,
         * 把电流采样打到饱和、把母线拉垮, 电感测量随之全废。
         * 专利思路是"注入电压从小开始, 逐步加大直到电流达到目标"。
         * 这里取它的等价起点 —— 因 |Z| = √(R²+(ωL)²) ≥ R 恒成立, 故
         *     u_amp = I_target × R
         * 可保证注入电流 ≤ I_target (物理上不会过流); 第二轮再按实测 |Z|
         * 把幅值修正到刚好达到目标电流。 */
        l_v_amp = Config.calib_current * Config.phase_resistance;
        if (l_v_amp < CALIB_L_MIN_V) l_v_amp = CALIB_L_MIN_V;

        if ((l_v_dc + l_v_amp) > Config.calib_voltage * 0.9f) {
            l_v_amp = (Config.calib_voltage * 0.9f) - l_v_dc;
            if (l_v_amp < CALIB_L_MIN_V) l_v_amp = CALIB_L_MIN_V;
        }

        /* ===== 平台适配 (第 3 处) =====
         * rare 的 CALIBRATION_loop 跑 20kHz, 所以能在该循环里直接产生
         * omega = 2π·current_ctrl_bw = 1000Hz 的正弦。
         * 本工程 calibration_loop 跑 1kHz —— 在 1kHz 循环里算 sin(omega·time)
         * 会退化成 sin(2π·k) ≡ 0, 注入变成纯直流、解调失效。
         * 故把【注入 + 同步解调】放到 40kHz 的 PWM 中断 (foc_adc_callback)
         * 里执行, 算法与 rare 的 CS_MOTOR_L_LOOP 完全一致。 */
        FOC.vinj_omega   = 2.0f * M_PI * Config.current_ctrl_bw; // 1000 Hz
        FOC.vinj_amp     = l_v_amp;
        FOC.vinj_dc      = l_v_dc;
        FOC.inj_phase    = 0;
        FOC.vinj_sum_sin = 0;
        FOC.vinj_sum_cos = 0;
        FOC.vinj_samples = 0;
        FOC.vinj_acc     = false;

        loop_count = 0;
        FOC.calib_vinj_mode = true;
        foc_arm();                        /* = rare 的 FOC_PWM_START() */
        mCalibStep = CS_MOTOR_L_LOOP;
    } break;

    case CS_MOTOR_L_LOOP:
        // 丢弃起始瞬态后开始解调累加 (对应 rare 的 loop_count >= 100)
        if (!FOC.vinj_acc && (loop_count >= 100U)) {
            FOC.vinj_acc = true;
        }
        if (loop_count >= L_CAL_CYCLES) {
            FOC.vinj_acc = false;
            mCalibStep = CS_MOTOR_L_END;   /* 注入保持, 由 L_END 决定是否再测一轮 */
        }
        break;

    case CS_MOTOR_L_END: {
        float omega = 2.0f * M_PI * Config.current_ctrl_bw;
        int valid_samples = (int)FOC.vinj_samples;
        float I_amp = (valid_samples > 0)
                          ? (2.0f / (float)valid_samples) *
                                sqrtf(SQ(FOC.vinj_sum_sin) + SQ(FOC.vinj_sum_cos))
                          : 0.0f;
        float Z = (I_amp > 0.005f) ? (l_v_amp / I_amp) : 9999.0f;

        /* ---- 探测轮: 用实测 |Z| 把幅值修正到刚好达到目标电流, 再测一轮 ---- */
        if (l_pass == 0) {
            float new_amp = Config.calib_current * Z;
            if (new_amp < CALIB_L_MIN_V) new_amp = CALIB_L_MIN_V;
            if ((l_v_dc + new_amp) > Config.calib_voltage * 0.9f) {
                new_amp = (Config.calib_voltage * 0.9f) - l_v_dc;
                if (new_amp < CALIB_L_MIN_V) new_amp = CALIB_L_MIN_V;
            }

            printf("ind-probe: Iamp=%.3f Z=%.3f Vamp %.3f -> %.3f\n",
                   I_amp, Z, l_v_amp, new_amp);

            l_v_amp          = new_amp;
            FOC.vinj_amp     = new_amp;
            FOC.inj_phase    = 0;
            FOC.vinj_sum_sin = 0;
            FOC.vinj_sum_cos = 0;
            FOC.vinj_samples = 0;
            FOC.vinj_acc     = false;
            l_pass           = 1;
            loop_count       = 0;
            mCalibStep       = CS_MOTOR_L_LOOP;
            break;
        }

        /* ---- 测量轮: 计算电感 ---- */
        float sys_R = Config.phase_resistance;

        float X_L = 0.0f;
        if (Z > sys_R) {
            X_L = sqrtf(Z * Z - sys_R * sys_R);
        } else {
            X_L = 0.001f; // 测不出感抗时给极小值
        }

        float L_eq = X_L / omega;
        Config.phase_inductance = L_eq * 1.0f;

        FOC.calib_vinj_mode = false;
        pwm_output_low_sides();
        FOC_update_current_ctrl_gain(Config.current_ctrl_bw);

        printf("ind: %.6f H (Iamp=%.3f Z=%.3f Vdc=%.3f Vamp=%.3f)\n",
               Config.phase_inductance, I_amp, Z, l_v_dc, l_v_amp);   /* 本工程调试输出 */

        phase_set  = 0;
        loop_count = 0;
        mCalibStep = CS_DIR_PP_START;
    } break;

    case CS_DIR_PP_START:
        if (loop_count == 0) {
            foc_arm();
        }
        FOC_voltage((drag_v * time / 2.0f), 0, 0.0f);

        if (time >= 2.0f) {
            encoder.shadow_count = 0;
            start_count_long = 0;
            dir_pp_runin = false;
            loop_count = 0;
            phase_set = 0;
            mCalibStep = CS_DIR_PP_LOOP;
        }
        break;
    case CS_DIR_PP_LOOP:
        phase_set += calib_phase_vel / (float)CAL_LOOP_HZ;
        FOC_voltage(drag_v, 0, phase_set);

        if (!dir_pp_runin) {
            /* 预跑: 先空转 total_calib_cycles 个电周期, 让转子充分"牵入"同步。
             * 同步电机从静止起转会有一段低于同步转速的滑行期(pole slipping),
             * 直接测量会让圈数偏少, 导致 pole_pairs 被 round 到相邻整数
             * (实测偏差 6.7% -> 14 对极被算成 15)。 */
            if (phase_set >= total_calib_cycles * M_2PI) {
                encoder.shadow_count = 0;
                start_count_long = 0;
                dir_pp_runin = true;
            }
        } else if (phase_set >= 2.0f * total_calib_cycles * M_2PI) {
            pwm_output_low_sides();
            mCalibStep = CS_DIR_PP_END;
        }
        break;
    case CS_DIR_PP_END: {
        int64_t diff = encoder.shadow_count - start_count_long;

        if (abs((int)diff) < 500) {
            Config.calib_valid = false;
        } else {
            Config.encoder_dir = (diff > 0) ? 1 : -1;
            float mech_revs = fabsf((float)diff / ENCODER_CPR_F);
            float pp_raw    = total_calib_cycles / mech_revs;
            Config.pole_pairs = (int)roundf(pp_raw);
            /* pp_raw 应非常接近整数; 若像 14.93 说明仍有瞬态残差 */
            printf("dir_pp: diff=%lld mech_revs=%.5f pp_raw=%.4f -> %d\n",
                   (long long)diff, mech_revs, pp_raw, (int)Config.pole_pairs);
        }

        /* 保护: 本项目 encoder_error_arr 按 MAX_MOTOR_POLE_PAIRS*SAMPLES_PER_PPAIR 分配,
         * 极对数无效时不再进入编码器采样, 直接结束并标记无效, 避免数组越界。
         * (rare 在此处会无条件继续, 但它没有这个数组尺寸保护) */
        if ((Config.pole_pairs <= 0) || (Config.pole_pairs > (int)MAX_MOTOR_POLE_PAIRS)) {
            Config.calib_valid = false;
            loop_count = 0;
            mCalibStep = CS_NULL;
            foc_request_stage(IDLE);
            break;
        }

        mCalibStep = CS_ENCODER_START;
    } break;

    case CS_FLUX_START:
        Config.flux_linkage = 0;
        FOC.calib_id        = 0;
        FOC.calib_iq        = Config.calib_current * FLUX_IQ_GAIN;
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
            flux_vel_sum = 0.0f;
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
        flux_vel_sum += encoder.vel;
        if (FOC.saturated) flux_sat = true;
        flux_cnt++;

        if (flux_cnt >= FLUX_MEAS_SAMPLES) {
            float v_avg = flux_v_sum / (float)flux_cnt;
            float i_avg = flux_i_sum / (float)flux_cnt;

            if (mCalibStep == CS_FLUX_MEAS1) {
                flux_v1    = v_avg;
                flux_i1    = i_avg;
                flux_sat1  = flux_sat;
                flux_vel1  = flux_vel_sum / (float)flux_cnt;   /* 窗口平均转速 */
                loop_count = 0;
                flux_vel_sum = 0.0f;
                mCalibStep = CS_FLUX_RAMP2;
            } else {
                flux_v2   = v_avg;
                flux_i2   = i_avg;
                flux_sat2 = flux_sat;
                flux_vel2 = flux_vel_sum / (float)flux_cnt;
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
            flux_vel_sum = 0.0f;
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

            /* 诊断: 两个转速点的实测电压/电流/机械转速
             * 期望 vel 与给定电角速度匹配: vel = w_e/(2*pi*pole_pairs) */
            printf("flux pts: V1=%.4f I1=%.3f vel1=%.3f | V2=%.4f I2=%.3f vel2=%.3f\n",
                   flux_v1, flux_i1, flux_vel1, flux_v2, flux_i2, flux_vel2);
            printf("flux cmd: we1=%.1f we2=%.1f rad/s (预期 vel1=%.3f vel2=%.3f rps)\n",
                   FLUX_SPEED1, FLUX_SPEED2,
                   FLUX_SPEED1 / (M_2PI * (float)Config.pole_pairs),
                   FLUX_SPEED2 / (M_2PI * (float)Config.pole_pairs));
            printf("flux: R=%.3f Ohm lambda=%.6f Wb (simple %.6f)\n",
                   res, lambda, lambda_simple);

            /* 只在和电阻标定值接近时才采纳: 磁链标定若失步(异响/滑差),
             * 算出的 R 是垃圾, 不应覆盖专门用两点伏安法测得的电阻 */
            if ((res > 0.01f) && (res < 50.0f) &&
                (fabsf(res - Config.phase_resistance) < 0.5f * Config.phase_resistance)) {
                Config.phase_resistance = res;
            } else {
                printf("flux: R rejected (%.4f vs %.4f), keep %.4f\n",
                       res, Config.phase_resistance, Config.phase_resistance);
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
        mCalibStep = CS_REPORT_OFFSET_LUT;
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
            /* 防御: 极对数异常时不继续, 置无效并退出标定态 (否则会停在 CALIBRATION 不动) */
            Config.calib_valid = false;
            mCalibStep = CS_NULL;
            foc_request_stage(IDLE);
            break;
        }

        uint32_t total_samples = (uint32_t)Config.pole_pairs * SAMPLES_PER_PPAIR;
        int window = (int)LUT_WINDOW;

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
            sample_count = 0;
            next_sample_time = 0;
            mCalibStep = CS_FLUX_START;
        }
    } break;

    /* 收尾 (照搬 rare 的 CS_REPORT_OFFSET_LUT):
     * 停留 OFFSET_LUT_NUM × 1ms, 供上位机读回偏移表, 然后结束校准 */
    case CS_REPORT_OFFSET_LUT:
        if (sample_count < (int32_t)OFFSET_LUT_NUM) {
            if (time > next_sample_time) {
                next_sample_time += 0.001f;
                sample_count++;
            }
        } else {
            /* ===== 标定结果汇总 ===== */
            float R  = Config.phase_resistance;
            float L  = Config.phase_inductance;
            float lm = Config.flux_linkage;
            float wc = M_2PI * Config.current_ctrl_bw;   /* 电流环设计带宽 rad/s */

            int32_t lut_min = 0, lut_max = 0;
            int32_t cg_min  = 0, cg_max  = 0;
            for (uint32_t i = 0; i < OFFSET_LUT_NUM; i++) {
                int32_t v = Config.offset_lut[i];
                if (i == 0U) { lut_min = v; lut_max = v; }
                if (v < lut_min) lut_min = v;
                if (v > lut_max) lut_max = v;
            }
            for (uint32_t i = 0; i < COGGING_MAP_NUM; i++) {
                int32_t v = (int32_t)Config.cogging_map[i];
                if (i == 0U) { cg_min = v; cg_max = v; }
                if (v < cg_min) cg_min = v;
                if (v > cg_max) cg_max = v;
            }

            Config.calib_valid = true;

            printf("\n===== CALIB RESULT =====\n");
            printf(" R      = %.5f Ohm\n", R);
            printf(" L      = %.6f H\n", L);
            printf(" lambda = %.6f Wb\n", lm);
            printf(" Kt     = %.5f N.m/A\n", 1.5f * (float)Config.pole_pairs * lm);
            printf(" curKp  = %.4f (L*wc)  curKi = %.1f (R*wc)\n", L * wc, R * wc);
            printf(" poles  = %d pairs\n", (int)Config.pole_pairs);
            printf(" enc    = dir %d offset %d cnt (%.3f deg)\n",
                   (int)Config.encoder_dir, (int)Config.encoder_offset,
                   (float)Config.encoder_offset * 360.0f / ENCODER_CPR_F);
            printf(" offset_lut : min %d max %d pkpk %d\n",
                   (int)lut_min, (int)lut_max, (int)(lut_max - lut_min));
            printf(" cogging_map: min %d max %d pkpk %d\n",
                   (int)cg_min, (int)cg_max, (int)(cg_max - cg_min));
            printf(" valid  = %d\n", (int)Config.calib_valid);
            printf("========================\n");

            loop_count = 0;
            sample_count = 0;
            next_sample_time = 0;
            mCalibStep = CS_NULL;
            CONFIG_save_to_flash();
            foc_request_stage(IDLE);
        }
        break;

    default:
        break;
    }
    loop_count++;
}

/* ===================== 齿槽转矩补偿标定 =====================
 * 理论 (照搬 rare_foc):
 *   ① 位置环【逐点步进】: 每 25ms 把位置指令前进一步(1/256 圈机械角),
 *      停稳后记录"保持该位置所需的 q 轴电流" FOC.Iq_filt。
 *   ② 静止测量 => 不含动摩擦, 所以【不减均值】。
 *   ③ 正转测一遍、反转再测一遍, 两者【取平均】, 抵消静摩擦/滞回的方向性。
 *   ④ 表按【单圈机械角】索引: 256 点覆盖一整圈机械角, 而不是一个电周期。
 * 运行时由 CONTROLLER_loop 按机械角查表, 前馈叠加到 iq_set。
 */
static int      cg_number;   /* 已走步数 */
static uint32_t cg_loop_cnt; /* 步进分频计数 */
static bool     cg_valid;    /* 本次标定是否成功 */
static bool     cg_running;

/* 位置指令前进一步 (单圈机械角, 循环卷绕) */
static void cg_step(float dir)
{
    const float delta   = dir * (M_2PI / (float)COGGING_MAP_NUM);
    float       pos_ref = Controller.input_position + delta;
    while (pos_ref >= M_2PI) pos_ref -= M_2PI;
    while (pos_ref < 0.0f)   pos_ref += M_2PI;
    Controller.input_position = pos_ref;
}

/* 当前单圈机械位置 -> 补偿表下标 (0 ~ 255) */
static int cg_index(void)
{
    int idx = (int)nearbyintf((float)COGGING_MAP_NUM *
                              Controller.input_position / M_2PI);
    if (idx >= (int)COGGING_MAP_NUM) idx = 0;
    if (idx < 0) idx = 0;
    return idx;
}

/* 采样一次: 实际 q 轴电流(滤波) 按倍率转成 int16 */
static int16_t cg_sample(void)
{
    int32_t raw = (int32_t)(FOC.Iq_filt * COGGING_SCALE);
    if (raw > 32767)  raw = 32767;
    if (raw < -32768) raw = -32768;
    return (int16_t)raw;
}

void anticogging_start(void)
{
    if (!Config.calib_valid || (Config.pole_pairs <= 0)) {
        printf("cogging: calibration required first\n");
        cg_running = false;
        return;
    }

    foc_arm();
    FOC.calib_mode      = false;
    FOC.calib_vinj_mode = false;

    /* 位置模式: 逐点步进定位 (照搬 rare_foc) */
    Controller.ctrl_mode = CONTROL_MODE_POSITION_FILTER;
    CONTROLLER_reset();              /* input_position 对齐当前实际机械位置 */
    Controller.target_reached = false;

    /* 标定期间旧表失效, 避免运行时补偿污染采样 */
    Config.cogging_map_valid = 0;
    memset(Config.cogging_map, 0, sizeof(Config.cogging_map));

    cg_number   = 0;
    cg_loop_cnt = 0;
    cg_valid    = false;
    cg_running  = true;
}

bool anticogging_loop(void)
{
    static const int gap_number = 100;

    if (!cg_running) {
        return true;
    }

    // loop contrl 0.025s
    if (++cg_loop_cnt < 25) {   /* rare 为 20kHz 下 500 拍, 本工程 1kHz 故 25 拍, 同为 25ms */
        return false;
    }
    cg_loop_cnt = 0;

    cg_number++;

    if (cg_number <= gap_number) {
        /* ① 正转空走 */
        cg_step(+1.0f);

    } else if (cg_number <= (gap_number + (int)COGGING_MAP_NUM)) {
        /* ② 正转采样: 记录保持当前位置所需的电流 */
        int16_t index = (int16_t)cg_index();
        Config.cogging_map[index] = cg_sample();
        cg_step(+1.0f);

    } else if (cg_number <= (gap_number + (int)COGGING_MAP_NUM + gap_number)) {
        /* ③ 反转空走 */
        cg_step(-1.0f);

    } else if (cg_number <= (gap_number + (int)COGGING_MAP_NUM +
                             gap_number + (int)COGGING_MAP_NUM)) {
        /* ④ 反转采样: 与正转结果取平均, 抵消静摩擦/滞回方向性 */
        int16_t index = (int16_t)cg_index();
        Config.cogging_map[index] = (Config.cogging_map[index] + cg_sample()) / 2;
        cg_step(-1.0f);

    } else {
        /* ⑤ 结束 */
        cg_valid   = true;
        cg_running = false;
        return true;
    }

    return false;
}

void anticogging_end(void)
{
    cg_running = false;

    foc_disarm();
    pwm_output_low_sides();

    if (cg_valid) {
        Config.cogging_map_valid  = 1;
        Config.anticogging_enable = 1;
        CONFIG_save_to_flash();
        printf("cogging: done, %u pts, %d steps\n",
               (unsigned)COGGING_MAP_NUM, cg_number);
    } else {
        Config.cogging_map_valid = 0;
        printf("cogging: aborted, invalid\n");
    }

    Controller.input_velocity        = 0.0f;
    Controller.input_velocity_buffer = 0.0f;
}
