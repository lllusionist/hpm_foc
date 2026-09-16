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
        sys_delay_ms(100);
    }

    return 0;
}
