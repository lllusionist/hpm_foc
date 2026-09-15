/*
 * Copyright (c) 2021 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include <stdio.h>

#include "board.h"

int main(void)
{
    board_init();

    printf("foc start\n");

    while (1) {
    }

    return 0;
}
