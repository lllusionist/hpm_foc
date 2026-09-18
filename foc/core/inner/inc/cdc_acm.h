/*
 * Copyright (c) 2022-2023 HPMicro
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#ifndef CDC_ACM_H
#define CDC_ACM_H

#include <stdint.h>
#include <stdbool.h>

void cdc_acm_init(uint8_t busid, uint32_t reg_base);
void cdc_acm_data_send_with_dtr_test(uint8_t busid);
void cdc_acm_transmit(uint8_t *buf, uint16_t len);

#endif
