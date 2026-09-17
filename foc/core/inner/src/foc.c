#include "foc.h"

volatile foc_data_t FOC;

void foc_adc_midpoint(void)
{
    float adc_a_sum = 0;
    float adc_b_sum = 0;
    float adc_c_sum = 0;
    float times = 0;

    for(uint8_t i = 0; i < 32; i++)
    {
        adc_a_sum += FOC.Ia;
        adc_b_sum += FOC.Ib;
        adc_c_sum += FOC.Ic;
        times += 1;
        sys_delay_ms(1);
    }
    FOC.Ia_midpoint = adc_a_sum / times;
    FOC.Ib_midpoint = adc_b_sum / times;
    FOC.Ic_midpoint = adc_c_sum / times;
}