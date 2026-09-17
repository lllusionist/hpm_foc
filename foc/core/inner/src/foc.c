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
        FOC.Ia += adc_a_sum;
        FOC.Ib += adc_b_sum;
        FOC.Ic += adc_c_sum;
        sys_delay_ms(1);
    }
    FOC.Ia_midpoint = FOC.Ia / 5;
    FOC.Ib_midpoint = FOC.Ib / 5;
    FOC.Ic_midpoint = FOC.Ic / 5;
}