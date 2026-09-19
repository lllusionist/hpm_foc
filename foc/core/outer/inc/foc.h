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

	volatile float Iq;
	volatile float Iq_filt;
	volatile float Id;
	volatile float Id_filt;

	volatile float dtc_a;
	volatile float dtc_b;
	volatile float dtc_c;

	volatile float
	    current_ctrl_p_gain; // 电流环 P (上位机 USB 命令可在线改)
	volatile float current_ctrl_i_gain; // 电流环 I

} foc_data_t;

extern volatile foc_data_t FOC;

void foc_adc_midpoint(void);
void FOC_voltage(float Vd, float Vq, float phase);
void enable_bldc_pwm_output(void);
void disable_bldc_pwm_output(void);
void pwm_output_low_sides(void);
void foc_arm(void);
void foc_disarm(void);
void foc_init(void);

#endif /* __FOC_H__ */
