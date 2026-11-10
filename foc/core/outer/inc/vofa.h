#ifndef __VOFA_H__
#define __VOFA_H__

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "foc.h"
#include "encoder.h"

#define byte0(dw_temp) (*(char *)(&dw_temp))
#define byte1(dw_temp) (*((char *)(&dw_temp) + 1))
#define byte2(dw_temp) (*((char *)(&dw_temp) + 2))
#define byte3(dw_temp) (*((char *)(&dw_temp) + 3))

#define MAX_BUFFER_SIZE 1024

/* 波形采样率固定 5kHz, 分频由载波频率自动派生 (40k/8, 20k/4).
 * 手写 decim 时改 PWM 频率很容易忘记同步, 会变成 10kHz 把 USB CDC 打爆 */
#define VOFA_SAMPLE_RATE_HZ (5000U)
#define VOFA_SAMPLE_DECIM (PWM_FREQ_HZ / VOFA_SAMPLE_RATE_HZ)

void vofa_start(void);
void vofa_push_sample(void);
void vofa_send(void);
void vofa_request_config(void);
void vofa_transmit(uint8_t *buf, uint16_t len);
#endif /* __VOFA_H__ */
