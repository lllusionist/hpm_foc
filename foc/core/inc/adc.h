#ifndef __ADC_H__
#define __ADC_H__

#include "hpm_common.h"
#include "hpm_adc16_drv.h"
#include "hpm_trgm_drv.h"
#include "hpm_clock_drv.h"
#include "hpm_interrupt.h"
#include "hpm_soc.h"

typedef struct {
    float Ia;
    float Ib;
    float Ic;

    float Va;
    float Vb;
    float Vc;
} foc_data_t;

extern volatile foc_data_t FOC;
extern volatile uint32_t g_adc_cnt; 

void adc_init(void);

#endif /* __ADC_H__ */
