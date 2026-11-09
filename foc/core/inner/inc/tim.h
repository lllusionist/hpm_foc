#ifndef __TIM_H__
#define __TIM_H__

#include "hpm_pwmv2_drv.h"
#include "hpm_trgm_drv.h"
#include "hpm_clock_drv.h"
#include "hpm_iomux.h"
#include "SEGGER_RTT.h"
#include <stdio.h>

#define PWM HPM_PWM0
/* 载波频率: 20k->40k 是为了压 9µH 电机的电流纹波 (ΔI ∝ 1/f).
 * 代价: 死区电压误差翻倍 (100ns/25µs -> 96mV), 开关损耗翻倍.
 * CURRENT_MEASURE_PERIOD / 死区补偿 / 积分 / 角度外推 都由它派生, 改一处即可 */
#define PWM_FREQ_HZ (40000U) /* 载波频率 */
#define CURRENT_MEASURE_PERIOD (float)(1.0f / (float)PWM_FREQ_HZ)
#define PWM_DEAD_TIME_NS (0U) /* 死区(ns): 0=完全依靠 FD6288 自带的 100ns */
#define PWM_MIN_ON_NS (1000U) /* 最小导通脉宽(ns) */

void pwm_init(void);
void pwm_set_duty(float du, float dv, float dw);

#endif /* __TIM_H__ */
