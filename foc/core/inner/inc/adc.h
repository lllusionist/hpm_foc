#ifndef __ADC_H__
#define __ADC_H__

#include "hpm_common.h"
#include "hpm_adc16_drv.h"
#include "hpm_trgm_drv.h"
#include "hpm_clock_drv.h"
#include "hpm_interrupt.h"
#include "hpm_soc.h"
#include <stdint.h>

/* ADC 中断完成回调: 传入三相电流原始码值 (uint16, 未减零偏).
 * adc.c (inner 层) 不再直接依赖 foc.h (outer 层), 保持分层清洁. */
typedef void (*adc_callback_t)(uint16_t ia, uint16_t ib, uint16_t ic);

void adc_init(void);
void adc1_init(void);
bool adc1_read_all(uint16_t *va, uint16_t *vb, uint16_t *vc, uint16_t *vbus);
bool adc1_read_ibus(uint16_t *ibus);
bool adc_current_snapshot(uint16_t *ia, uint16_t *ib, uint16_t *ic);
void adc_callback_set(adc_callback_t sink);
/* 中断帧计数器 (供 foc_task 检测 ADC 停更) */
uint32_t adc_current_frames(void);
uint32_t adc_current_missed(void);

#endif /* __ADC_H__ */
