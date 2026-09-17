#ifndef __FOC_H__
#define __FOC_H__

#include "hpm_common.h"
#include "sys.h"

typedef struct {
    float Ia;
    float Ib;
    float Ic;
    float Ibus;

    float Ia_midpoint;
    float Ib_midpoint;
    float Ic_midpoint;

    float Va;
    float Vb;
    float Vc;
    float Vbus;

    float Iq;
    float Id;
    
} foc_data_t;

extern volatile foc_data_t FOC;

void foc_adc_midpoint(void);

#endif /* __FOC_H__ */
