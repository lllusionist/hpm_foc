#include "vofa.h"
#include "cdc_acm.h"
#include "math.h"

uint8_t send_buf[MAX_BUFFER_SIZE];
uint16_t cnt = 0;

/*================================= Justfloat =====================================*/
/*
1. vofa_demo()					循环调用
2. vofa_transmit				通信接口
3. vofa_send_data				发送接口
4. vofa_sendframetail			发送帧尾
*/
/**
***********************************************************************
* @brief:      vofa_transmit(uint8_t* buf, uint16_t len)
* @param:		   void
* @retval:     void
* @details:    修改通信工具，USART或者USB
***********************************************************************
**/
void vofa_transmit(uint8_t *buf, uint16_t len)
{
	cdc_acm_transmit(buf, len);
}
/**
***********************************************************************
* @brief:      vofa_send_data(float data)
* @param[in]:  num: 数据编号 data: 数据 
* @retval:     void
* @details:    将浮点数据拆分成单字节
***********************************************************************
**/
void vofa_send_data(uint8_t num, float data)
{
	(void)num; /* 通道号只用来标顺序, 数据本身就是按调用顺序排的 */

	send_buf[cnt++] = byte0(data);
	send_buf[cnt++] = byte1(data);
	send_buf[cnt++] = byte2(data);
	send_buf[cnt++] = byte3(data);
}
/**
***********************************************************************
* @brief      vofa_sendframetail(void)
* @param      NULL 
* @retval     void
* @details:   给数据包发送帧尾
***********************************************************************
**/
void vofa_sendframetail(void)
{
	send_buf[cnt++] = 0x00;
	send_buf[cnt++] = 0x00;
	send_buf[cnt++] = 0x80;
	send_buf[cnt++] = 0x7f;

	/* 将数据和帧尾打包发送 */
	vofa_transmit((uint8_t *)send_buf, cnt);
	cnt = 0; // 每次发送完帧尾都需要清零
}

void vofa_send(void)
{
	/* 通道顺序要和 VOFA+ 里配的一致; 这里是 8 通道 */
	vofa_send_data(0, (FOC.Ia - FOC.Ia_midpoint) * FOC_CURRENT_SCALE); // A
	vofa_send_data(1, (FOC.Ib - FOC.Ib_midpoint) * FOC_CURRENT_SCALE);
	vofa_send_data(2, (FOC.Ic - FOC.Ic_midpoint) * FOC_CURRENT_SCALE);
	vofa_send_data(3, FOC.Id);      // 电流环反馈
	vofa_send_data(4, FOC.Iq);
	vofa_send_data(5, FOC.Iq_set);  // 电流环目标
	vofa_send_data(6, encoder.phase);
	vofa_send_data(7, encoder.phase_vel);

	vofa_sendframetail();
}

#define VOFA_PERIOD_MS 1 // 波形周期 (ms): 1 = 1kHz

static void Vofa_task(void *argument)
{
	TickType_t last = xTaskGetTickCount();

	(void)argument;

	for (;;) {
		vTaskDelayUntil(&last, pdMS_TO_TICKS(VOFA_PERIOD_MS));
		vofa_send();
	}
}

void vofa_start(void)
{
	if (xTaskCreate(Vofa_task, "vofa", 512, NULL, 5, NULL) != pdPASS) {
		printf("vofa task create fail\n");
		while (1) {
		}
	}
}
