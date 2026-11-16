#ifndef __TIM_H__
#define __TIM_H__

#include "hpm_pwmv2_drv.h"
#include "hpm_trgm_drv.h"
#include "hpm_clock_drv.h"
#include "hpm_iomux.h"
#include <stdio.h>

#define PWM HPM_PWM0
/* 载波频率: 20k->40k 是为了压 9µH 电机(5210)的电流纹波 (ΔI ∝ 1/f).
 * 代价: 死区电压误差翻倍(100ns/25µs ≈ 96mV, 对 0.03Ω 约 3.2A)、
 * 开关损耗和 ADC 中断负载翻倍. CURRENT_MEASURE_PERIOD / 死区补偿 /
 * 电流环积分 / 角度外推都由这个宏派生. */
#define PWM_FREQ_HZ (40000U) /* 载波频率 */
#define CURRENT_MEASURE_PERIOD (float)(1.0f / (float)PWM_FREQ_HZ)
/* 软件死区保持 0: FD6288 自带 100ns 互锁, MOSFET td(off)+tf ≈ 27ns,
 * 100ns 足够防直通 (再加只会增大死区误差). 死区电压误差由
 * foc_deadtime_comp() 按有效死区 100ns 补偿, 频率变更时自动缩放. */
#define PWM_DEAD_TIME_NS (0U) /* 死区(ns): 0=完全依靠 FD6288 自带的 100ns */
#define PWM_MIN_ON_NS (1000U) /* 最小导通脉宽(ns) */

void pwm_init(void);
void pwm_set_duty(float du, float dv, float dw);

#endif /* __TIM_H__ */
