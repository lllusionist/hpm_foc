#ifndef __FOC_H__
#define __FOC_H__

#include "hpm_common.h"
#include "sys.h"
#include "util.h"
#include "tim.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdbool.h>
#include "config.h"

#define FOC_CURRENT_SHUNT_OHM (0.005f)
#define FOC_CURRENT_AMP_GAIN (20.0f)
#define FOC_CURRENT_SCALE \
	((3.3f / 65536.0f) / (FOC_CURRENT_SHUNT_OHM * FOC_CURRENT_AMP_GAIN))

#define FOC_IBUS_SHUNT_OHM (0.005f)
#define FOC_IBUS_AMP_GAIN (20.0f)
#define FOC_IBUS_SCALE \
	((3.3f / 65536.0f) / (FOC_IBUS_SHUNT_OHM * FOC_IBUS_AMP_GAIN))

#define FOC_VBUS_DIVIDER (16.0f)
#define FOC_VPHASE_DIVIDER (16.0f)

/* 故障码 */
#define FOC_FAULT_NONE 0
#define FOC_FAULT_PHASE_OC 1	  /* 相过流 */
#define FOC_FAULT_OVER_VOLTAGE 2  /* 母线过压 */
#define FOC_FAULT_UNDER_VOLTAGE 3 /* 母线欠压 */
#define FOC_FAULT_ADC 4
#define FOC_FAULT_IBUS_OC 5 /* 母线过流 */

typedef struct {

	volatile bool is_armed;
	volatile uint32_t fault_code; /* 0=正常, 见 FOC_FAULT_* */
	volatile bool adc_ready;
	volatile uint32_t adc_current_frames;
	volatile uint32_t adc_current_missed;
	volatile uint32_t adc_voltage_missed;
	volatile uint32_t adc_ibus_missed;

	volatile float Ia;
	volatile float Ib;
	volatile float Ic;
	volatile float Ibus;
	volatile float Ibus_filt;

	volatile float Ia_midpoint;
	volatile float Ib_midpoint;
	volatile float Ic_midpoint;
	volatile float Ibus_midpoint;

	volatile float Va_midpoint;
	volatile float Vb_midpoint;
	volatile float Vc_midpoint;

	volatile float Va;
	volatile float Vb;
	volatile float Vc;
	volatile float Vbus;
	volatile float Vbus_filt;

	volatile float Iq;
	volatile float Iq_filt;
	volatile float Id;
	volatile float Id_filt;

	volatile float Id_set; /* d 轴电流目标 (A) */
	volatile float Iq_set; /* q 轴电流目标 (A) */

	volatile float phase_offset; /* 调试用: 电角度附加偏移 (rad) */

	/* 调试用: 开环固定电压矢量 (>0 时旁路电流环, 直接输出该矢量) */
	volatile float openloop_voltage; /* 幅值 (V) */
	volatile float openloop_angle;	 /* 电角度 (rad) */

	volatile float current_ctrl_integral_d; /* 电流环 d 轴积分项 */
	volatile float current_ctrl_integral_q; /* 电流环 q 轴积分项 */

	volatile float dtc_a;
	volatile float dtc_b;
	volatile float dtc_c;

	volatile float
	    current_ctrl_p_gain; /* 电流环 P (上位机 USB 命令可在线改) */
	volatile float current_ctrl_i_gain; /* 电流环 I */

	volatile float Power_filt; /* 功率 (W) */
	volatile float torque;	   /* 转矩估算 (N·m) */

	volatile float Vd_out;	 /* 电流环输出电压 d 分量 (V) */
	volatile float Vq_out;	 /* 电流环输出电压 q 分量 (V) */
	volatile float Vmag;	 /* 指令电压幅值 |V| (V) */
	volatile bool saturated; /* 电压饱和标志 */
	volatile bool ff_enable; /* 反电势前馈/解耦使能 (调试用) */

	volatile float isum;

	/* 校准用: 开环电流模式 (校准任务下发角度, 中断里跑电流环) */
	volatile bool calib_mode;
	volatile float calib_angle;	/* 电角度 (rad) */
	volatile float calib_phase_vel;
	/* 1 = 由 20kHz 中断按 calib_phase_vel 推进 calib_angle (磁链标定用):
	 * 1kHz 推进在 500rad/s 时会有 29° 阶梯误差, d 轴电压被拉偏 */
	volatile bool calib_adv_angle; /* 电角速度 (rad/s) */
	volatile float calib_id;	/* d 轴目标 (A) */
	volatile float calib_iq;	/* q 轴目标 (A) */

	/* 校准用: α 轴正弦电压注入 (电感测量, 解调累加在中断里做) */
	volatile bool calib_vinj_mode;
	volatile float vinj_omega; /* 注入角频率 (rad/s) */
	volatile float vinj_amp;   /* 正弦幅值 (V) */
	volatile float vinj_dc;	   /* 直流偏置 (V) */
	volatile float inj_phase;  /* 注入相位, 中断里推进 */
	volatile bool vinj_acc;	   /* 使能解调累加 */
	volatile float vinj_sum_sin;
	volatile float vinj_sum_cos;
	volatile uint32_t vinj_samples;

} foc_data_t;

extern volatile foc_data_t FOC;

bool foc_adc_midpoint(void);
void foc_update_voltages(void);
void FOC_voltage(float Vd, float Vq, float phase);
void FOC_current(float Id_set, float Iq_set, float phase, float phase_vel);
void FOC_update_current_ctrl_gain(float bw);
void enable_bldc_pwm_output(void);
void disable_bldc_pwm_output(void);
void pwm_output_low_sides(void);
void foc_arm(void);
void foc_disarm(void);
void foc_init(void);

#endif /* __FOC_H__ */
