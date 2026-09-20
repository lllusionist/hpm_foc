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

/* 波形采样率 = ADC 中断频率 / VOFA_SAMPLE_DECIM (40kHz/8 = 5kHz), 攒批后由 vofa 任务整包发出 */
#define VOFA_SAMPLE_DECIM 8

void vofa_start(void);
void vofa_push_sample(void);
void vofa_send_data(uint8_t num, float data);
void vofa_sendframetail(void);
void vofa_Receive(uint8_t *Buf, uint16_t *Len);
float vofa_cmd_parse(uint8_t *cmdBuf, char *arg);
void vofa_send(void);
#endif /* __VOFA_H__ */
