#ifndef __FOC_H__
#define __FOC_H__

#include "hpm_common.h"
#include "sys.h"
#include "util.h"
#include "tim.h"
#include "FreeRTOS.h"
#include "task.h"

typedef struct {
	volatile float Ia;
	volatile float Ib;
	volatile float Ic;
	volatile float Ibus;

	volatile float Ia_midpoint;
	volatile float Ib_midpoint;
	volatile float Ic_midpoint;

	volatile float Va;
	volatile float Vb;
	volatile float Vc;
	volatile float Vbus;

	volatile float Iq;
	volatile float Id;

	volatile float dtc_a;
	volatile float dtc_b;
	volatile float dtc_c;

	volatile float
	    current_ctrl_p_gain; // 电流环 P (上位机 USB 命令可在线改)
	volatile float current_ctrl_i_gain; // 电流环 I

} foc_data_t;

typedef enum led_mode {
	ST_BOOT,
	ST_IDLE,  // 空闲
	ST_RUN,	  // 运行
	ST_CAL,	  // 校准
	ST_FAULT, // 故障
} foc_stage_t;

extern volatile foc_data_t FOC;

void foc_adc_midpoint(void);
void FOC_voltage(float Vd, float Vq, float phase);

#endif /* __FOC_H__ */
