#ifndef __ADC_H__
#define __ADC_H__

#include "hpm_common.h"
#include "hpm_adc16_drv.h"
#include "hpm_trgm_drv.h"
#include "hpm_clock_drv.h"
#include "hpm_interrupt.h"
#include "hpm_soc.h"

typedef void (*adc_callback_t)(void);

void adc_init(void);
void adc_callback_set(adc_callback_t sink);

#endif /* __ADC_H__ */
