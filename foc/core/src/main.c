#include <stdio.h>
#include "main.h"
#include "sys.h"
#include "tim.h"

int main(void)
{
    sys_board_init();

    printf("main start\n");

    pwm_init();

    while (1) {
    }

    return 0;
}
