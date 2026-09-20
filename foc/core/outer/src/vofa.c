#include "vofa.h"
#include "cdc_acm.h"
#include "foc_ctrl.h"
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

/*================================ 批量发送 ====================================*/
/*
 * USB CDC 全速每秒只能搬约 1000 个包, 单样本发送时 1kHz 已是上限,
 * 提速只会被 cdc_acm_transmit 丢包(抽帧).
 * 因此: ADC 中断按 VOFA_SAMPLE_DECIM 分频采样进环形缓冲(默认 5kHz),
 * vofa 任务按 1ms 把攒下的多个样本打包进同一包 USB 发出,
 * 上位机/VOFA+ 仍按帧尾逐个解析 -> 有效采样率 = 采样分频, 与 USB 包率解耦.
 */
#define VOFA_NCH        16
#define VOFA_RING       64                                       /* 环形缓冲样本数 */
#define VOFA_BATCH_MAX  (MAX_BUFFER_SIZE / (VOFA_NCH * 4 + 4))    /* 单包最多样本数 */

static float vofa_ring[VOFA_RING][VOFA_NCH];
static volatile uint16_t vofa_head = 0;
static volatile uint16_t vofa_tail = 0;

static void vofa_snapshot(float *d)
{
	d[0] = (FOC.Ia - FOC.Ia_midpoint) * FOC_CURRENT_SCALE; // A
	d[1] = (FOC.Ib - FOC.Ib_midpoint) * FOC_CURRENT_SCALE;
	d[2] = (FOC.Ic - FOC.Ic_midpoint) * FOC_CURRENT_SCALE;
	d[3] = FOC.Id;      // 电流环反馈
	d[4] = FOC.Iq;
	d[5] = FOC.Iq_set;  // 电流环目标
	d[6] = Controller.vel_setpoint; // 速度环目标 (rps)
	d[7] = encoder.vel;             // 速度反馈 (rps)
	d[8] = FOC.torque;              // 转矩估算 (N·m)
	d[9] = FOC.Vbus_filt;           // 母线电压 (V)
	d[10] = FOC.Vmag;               // 指令电压幅值 |V| (V)
	d[11] = FOC.saturated ? 1.0f : 0.0f; // 电压饱和标志
	d[12] = FOC.Vd_out;             // d 轴指令电压 (V)
	d[13] = FOC.Vq_out;             // q 轴指令电压 (V)
	d[14] = encoder.phase;          // 电角度 (rad)
	d[15] = encoder.phase_vel;      // 电角速度 (rad/s)
}

/* 由 ADC 中断按分频调用 */
void vofa_push_sample(void)
{
	uint16_t next = (uint16_t)((vofa_head + 1U) % VOFA_RING);

	if (next == vofa_tail) { /* 满: 丢最旧 */
		vofa_tail = (uint16_t)((vofa_tail + 1U) % VOFA_RING);
	}

	vofa_snapshot(vofa_ring[vofa_head]);
	vofa_head = next;
}

static uint8_t *vofa_pack(uint8_t *p, const float *d)
{
	uint8_t i;

	for (i = 0; i < VOFA_NCH; i++) {
		memcpy(p, &d[i], 4);
		p += 4;
	}

	*p++ = 0x00;
	*p++ = 0x00;
	*p++ = 0x80;
	*p++ = 0x7f;

	return p;
}

void vofa_send(void)
{
	uint8_t *p = send_buf;
	uint16_t n = 0;

	while ((vofa_tail != vofa_head) && (n < VOFA_BATCH_MAX)) {
		p = vofa_pack(p, vofa_ring[vofa_tail]);
		vofa_tail = (uint16_t)((vofa_tail + 1U) % VOFA_RING);
		n++;
	}

	if (n > 0U) {
		vofa_transmit(send_buf, (uint16_t)(p - send_buf));
	}
}

#define VOFA_PERIOD_MS 1 // USB 发包周期 (ms): 1 = 1kHz

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
