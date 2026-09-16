#include <stdio.h>
#include "main.h"
#include "sys.h"
#include "tim.h"
#include "adc.h"

int main(void)
{
    sys_board_init();

    printf("main start\n");

    pwm_init();
    adc_init();

    while (1) {
        printf("cnt=%u ia=%.0f ib=%.0f ic=%.0f\n", g_adc_cnt, FOC.Ia, FOC.Ib, FOC.Ic);
        sys_delay_ms(100);
    }

    return 0;
}
