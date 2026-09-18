#ifndef __TIM_H__
#define __TIM_H__

#include "hpm_pwmv2_drv.h"
#include "hpm_trgm_drv.h"
#include "hpm_clock_drv.h"
#include "hpm_iomux.h"
#include "SEGGER_RTT.h"
#include <stdio.h>

#define PWM HPM_PWM0
#define PWM_FREQ_HZ (40000U)	// 载波频率
#define PWM_DEAD_TIME_NS (500U) // 死区(ns)
#define PWM_MIN_ON_NS (1000U)	// 最小导通脉宽(ns)

void pwm_init(void);
void pwm_set_duty(float du, float dv, float dw);

#endif /* __TIM_H__ */
