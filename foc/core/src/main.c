#include <stdio.h>
#include "board.h"
#include "main.h"
#include "sys.h"

int main(void)
{

    sys_pmp_init();
    sys_init();
    sys_console_init();
    printf("foc start\n");

    while (1) {
    }

    return 0;
}
