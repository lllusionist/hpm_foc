#include "vofa.h"
#include "cdc_acm.h"
#include "foc_ctrl.h"
#include "config.h"
#include "math.h"
#include "stdio.h"
#include "string.h"

uint8_t send_buf[MAX_BUFFER_SIZE];

/* 读取设备参数的请求标志 (由 USB 命令置位, 在 vofa 任务包边界发出, 避免拆包) */
static volatile uint8_t cfg_dump_req = 0;

void vofa_request_config(void)
{
	cfg_dump_req = 1;
}

/* 以一行 ASCII 回传当前参数: $FOCFG,KEY=VAL,...\n */
static void vofa_send_config(void)
{
	static char line[320];
	int n = snprintf(
		line, sizeof(line),
		"$FOCFG,CLIM=%.5g,VLIM=%.5g,POSKP=%.5g,VELKP=%.5g,VELKI=%.5g,"
		"CURBW=%.5g,CURKP=%.5g,CURKI=%.5g,UV=%.5g,OV=%.5g,"
		"OP=%ld,INV=%ld,PP=%ld,TW=%.5g,PW=%.5g,VRR=%.5g,CRR=%.5g,"
		"PFBW=%.5g,CAL=%ld\n",
		Config.current_limit, Config.velocity_limit, Config.pos_p_gain,
		Config.vel_p_gain, Config.vel_i_gain, Config.current_ctrl_bw,
		Config.current_kp, Config.current_ki, Config.protect_under_voltage,
		Config.protect_over_voltage, (long)Config.default_op_mode,
		(long)Config.invert_motor_dir, (long)Config.pole_pairs,
		Config.target_velcity_window, Config.target_position_window,
		Config.velocity_ramp_rate, Config.current_ramp_rate,
		Config.position_filter_bw, (long)Config.calib_valid);
	if (n > 0) {
		if (n > (int)sizeof(line) - 1) {
			n = (int)sizeof(line) - 1;
		}
		vofa_transmit((uint8_t *)line, (uint16_t)n);
	}
}

/*================================ 批量发送 ====================================*/
/*
 * USB CDC 全速每秒只能搬约 1000 个包, 单样本发送 1kHz 已是上限.
 * 因此: ADC 中断按分频采样进环形缓冲(默认 5kHz), vofa 任务按 1ms 把攒下的
 * 多个样本打包进同一包 USB 发出; 上位机仍按帧尾逐个解析,
 * 有效采样率 = 采样分频, 与 USB 包率解耦.
 */
#define VOFA_NCH        22
#define VOFA_RING       64 /* 环形缓冲样本数 */
#define VOFA_BATCH_MAX  (MAX_BUFFER_SIZE / (VOFA_NCH * 4 + 4))

static float vofa_ring[VOFA_RING][VOFA_NCH];
static volatile uint16_t vofa_head = 0;
static volatile uint16_t vofa_tail = 0;

void vofa_transmit(uint8_t *buf, uint16_t len)
{
	cdc_acm_transmit(buf, len);
}

static void vofa_snapshot(float *d)
{
	d[0] = (FOC.Ia - FOC.Ia_midpoint) * FOC_CURRENT_SCALE; // A 相电流 (A)
	d[1] = (FOC.Ib - FOC.Ib_midpoint) * FOC_CURRENT_SCALE; // B 相电流 (A)
	d[2] = (FOC.Ic - FOC.Ic_midpoint) * FOC_CURRENT_SCALE; // C 相电流 (A)
	d[3] = FOC.Id;					       // d 轴电流反馈
	d[4] = FOC.Iq;					       // q 轴电流反馈
	d[5] = FOC.Iq_set;				       // q 轴电流目标
	d[6] = Controller.vel_setpoint;			       // 速度环目标 (rps)
	d[7] = encoder.vel;				       // 速度反馈 (rps)
	d[8] = FOC.torque;				       // 转矩估算 (N·m)
	d[9] = FOC.Vbus_filt;				       // 母线电压 (V)
	d[10] = FOC.Vmag;				       // 指令电压幅值 (V)
	d[11] = FOC.saturated ? 1.0f : 0.0f;		       // 电压饱和标志
	d[12] = FOC.Vd_out;				       // d 轴指令电压 (V)
	d[13] = FOC.Vq_out;				       // q 轴指令电压 (V)
	d[14] = encoder.phase;				       // 电角度 (rad)
	d[15] = encoder.phase_vel;			       // 电角速度 (rad/s)
	d[16] = FOC.isum;				       // 三相电流和 (A), 恒应≈0
	d[17] = FOC.Va;					       // A 相电压 (V)
	d[18] = FOC.Vb;					       // B 相电压 (V)
	d[19] = FOC.Vc;					       // C 相电压 (V)
	d[20] = (float)FOC.fault_code;			       // 故障码
	d[21] = FOC.Ibus_filt;			       // 母线电流 (A)
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
		if (cfg_dump_req) {
			cfg_dump_req = 0;
			vofa_send_config();
		}
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
