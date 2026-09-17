#include <stdio.h>
#include "main.h"
#include "sys.h"
#include "tim.h"
#include "adc.h"
#include "mt6835.h"
#include "dma.h"
#include "FreeRTOS.h"
#include "task.h"

int main(void)
{
    sys_board_init();

    printf("main start\n");

    pwm_init();
    adc_init();
    dma_mgr_init();
    mt6835_init();


    vTaskStartScheduler();

    while (1) {
    }

    return 0;
}
