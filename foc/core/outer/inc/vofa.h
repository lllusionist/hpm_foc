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

/* 采样率 = ADC 中断频率 / VOFA_SAMPLE_DECIM (20k/4=5k), 攒批发出 */
#define VOFA_SAMPLE_DECIM 4

void vofa_start(void);
void vofa_push_sample(void);
void vofa_send(void);
void vofa_request_config(void);
void vofa_transmit(uint8_t *buf, uint16_t len);
#endif /* __VOFA_H__ */
