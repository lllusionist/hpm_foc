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

/* 电流采样换算: 低边 5mΩ + 20 倍放大 = 0.1V/A, ADC 3.3V/16bit */
#define FOC_CURRENT_SHUNT_OHM (0.005f)
#define FOC_CURRENT_AMP_GAIN (20.0f)
#define FOC_CURRENT_SCALE                                                      \
	((3.3f / 65536.0f) / (FOC_CURRENT_SHUNT_OHM * FOC_CURRENT_AMP_GAIN))

typedef struct {

	volatile bool is_armed;
	
	volatile float Ia;
	volatile float Ib;
	volatile float Ic;
	volatile float Ibus;
	volatile float Ibus_filt;

	volatile float Ia_midpoint;
	volatile float Ib_midpoint;
	volatile float Ic_midpoint;

	volatile float Va;
	volatile float Vb;
	volatile float Vc;
	volatile float Vbus;
	volatile float Vbus_filt;

	volatile float Iq;
	volatile float Iq_filt;
	volatile float Id;
	volatile float Id_filt;

	volatile float Id_set; // d 轴电流目标 (A)
	volatile float Iq_set; // q 轴电流目标 (A)

	volatile float phase_offset; // 调试用: 电角度附加偏移 (rad)

	/* 调试用: 开环固定电压矢量 (>0 时旁路电流环, 直接输出该矢量) */
	volatile float openloop_voltage; // 幅值 (V)
	volatile float openloop_angle;	 // 电角度 (rad)

	volatile float current_ctrl_integral_d; // 电流环 d 轴积分项
	volatile float current_ctrl_integral_q; // 电流环 q 轴积分项

	volatile float dtc_a;
	volatile float dtc_b;
	volatile float dtc_c;

	volatile float
	    current_ctrl_p_gain; // 电流环 P (上位机 USB 命令可在线改)
	volatile float current_ctrl_i_gain; // 电流环 I

	volatile float Power_filt; // 功率 (W)

} foc_data_t;

extern volatile foc_data_t FOC;

void foc_adc_midpoint(void);
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
