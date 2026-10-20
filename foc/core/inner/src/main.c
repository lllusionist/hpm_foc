#include <stdio.h>
#include "main.h"
#include "sys.h"
#include "tim.h"
#include "adc.h"
#include "encoder.h"
#include "dma.h"
#include "cdc_acm.h"
#include "vofa.h"
#include "config.h"
#include "flash.h"
#include "foc_ctrl.h"
#include "foc_task.h"
#include "FreeRTOS.h"
#include "task.h"

int main(void)
{
	config_init();
	sys_board_init();

	flash_init();
	if (CONFIG_load_from_flash() != 0)
		printf("no valid config, use defaults\n");

	sys_usb_init();
	pwm_init();
	adc_init();
	dma_mgr_init();
	ENCODER_init();
	encoder_app_start();
	vofa_start();
	foc_init();
	CONTROLLER_init();
	foc_app_start();

	vTaskStartScheduler();

	while (1) {
	}

	return 0;
}
