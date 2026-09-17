#ifndef __FOC_H__
#define __FOC_H__

#include "hpm_common.h"
#include "sys.h"
#include "util.h"
#include "tim.h"

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
    
} foc_data_t;

extern volatile foc_data_t FOC;

void foc_adc_midpoint(void);

#endif /* __FOC_H__ */
