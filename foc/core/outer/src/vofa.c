#include "vofa.h"
#include "cdc_acm.h"
#include "foc_ctrl.h"
#include "foc_task.h"
#include "config.h"
#include "position_profile.h"
#include "math.h"
#include "stdio.h"
#include "string.h"

uint8_t send_buf[MAX_BUFFER_SIZE];

/* 读取设备参数的请求标志 (由 USB 命令置位, 在 vofa 任务包边界发出, 避免拆包) */
static volatile uint8_t cfg_dump_req = 0;
/* 读取标准状态字的请求标志 (0x62) */
static volatile uint8_t sw_dump_req = 0;

void vofa_request_config(void)
{
	cfg_dump_req = 1;
}

void vofa_request_statusword(void)
{
	sw_dump_req = 1;
}

/* 以一行 ASCII 回传标准状态字: $FOCSW,STATUS=0xXX,ERRORS=0xXX,ARM=%d,ON=%d,REACH=%d\n */
static bool vofa_send_statusword(void)
{
	static char line[64];

	/* target_reached 属 Controller 语义, 发送时动态同步 (不跨层维护) */
	FOC.statusword.status.target_reached = Controller.target_reached ? 1 : 0;

	int n = snprintf(
	    line, sizeof(line),
	    "$FOCSW,STATUS=0x%04X,ERRORS=0x%04X,ARM=%d,ON=%d,REACH=%d\n",
	    FOC.statusword.status.status_code, FOC.statusword.errors.errors_code,
	    (int)FOC.statusword.status.armed, (int)FOC.statusword.status.switched_on,
	    (int)FOC.statusword.status.target_reached);
	if (n <= 0 || n >= (int)sizeof(line) - 1)
		return false;
	return cdc_acm_transmit((const uint8_t *)line, (uint16_t)n);
}

/* 以一行 ASCII 回传当前参数: $FOCFG,KEY=VAL,...\n */
static bool vofa_send_config(void)
{
	static char line[MAX_BUFFER_SIZE];
	foc_phase_trip_t trip;
	int appended;
	int n = snprintf(
	    line, sizeof(line),
	    "$FOCFG,CLIM=%.5g,VLIM=%.5g,POSKP=%.5g,VELKP=%.5g,VELKI=%.5g,"
	    "CURBW=%.5g,CURKP=%.5g,CURKI=%.5g,UV=%.5g,OV=%.5g,"
	    "OP=%ld,INV=%ld,PP=%ld,TW=%.5g,PW=%.5g,VRR=%.5g,CRR=%.5g,"
	    "PFBW=%.5g,CAL=%ld,R=%.5g,L=%.5g,LAM=%.5g,COG=%ld,"
	    "PROF=%ld",
	    Config.current_limit, Config.velocity_limit, Config.pos_p_gain,
	    Config.vel_p_gain, Config.vel_i_gain, Config.current_ctrl_bw,
	    Config.current_kp, Config.current_ki, Config.protect_under_voltage,
	    Config.protect_over_voltage, (long)Config.default_op_mode,
	    (long)Config.invert_motor_dir, (long)Config.pole_pairs,
	    Config.target_velcity_window, Config.target_position_window,
	    Config.velocity_ramp_rate, Config.current_ramp_rate,
	    Config.position_filter_bw, (long)Config.calib_valid,
	    Config.phase_resistance, Config.phase_inductance,
	    Config.flux_linkage, (long)Config.cogging_map_valid,
	    (long)Config.motor_profile);
	if (n <= 0 || n >= (int)sizeof(line) - 1)
		return false;
	if (foc_read_phase_trip(&trip)) {
		appended = snprintf(
		    line + n, sizeof(line) - (size_t)n,
		    ",OCIA=%.5g,OCIB=%.5g,OCIC=%.5g,OCLIM=%.5g,"
		    "OCPROF=%ld,OCFR=%lu,OCVR=%.5g,OCVL=%.5g",
		    trip.ia, trip.ib, trip.ic, trip.limit,
		    (long)trip.motor_profile,
		    (unsigned long)trip.frames_since_arm,
		    trip.resistance_voltage, trip.inductance_amplitude);
		if (appended > 0 && appended < (int)sizeof(line) - n - 1)
			n += appended;
	}
	line[n++] = '\n';
	return cdc_acm_transmit((const uint8_t *)line, (uint16_t)n);
}

/* 批量发送 */
/* USB CDC 约 1000 包/s: ADC 分频采样入环形缓冲, 任务每 1ms 打包,
 * 有效采样率与 USB 包率解耦 */
#define VOFA_NCH 25
#define VOFA_RING 64 /* 环形缓冲样本数 */
#define VOFA_BATCH_MAX (MAX_BUFFER_SIZE / (VOFA_NCH * 4 + 4))

static float vofa_ring[VOFA_RING][VOFA_NCH];
static volatile uint16_t vofa_head = 0;
static volatile uint16_t vofa_tail = 0;

void vofa_transmit(uint8_t *buf, uint16_t len)
{
	cdc_acm_transmit(buf, len);
}

static void vofa_snapshot(float *d)
{
	float pos_actual = (float)encoder.count_aligned * (M_2PI / ENCODER_CPR_F);
	float pos_target = NAN;
	float pos_error = NAN;

	if (FOC.is_armed && foc_get_stage() == RUN) {
		if (Controller.ctrl_mode == CONTROL_MODE_POSITION_FILTER)
			pos_target = Controller.input_position;
		else if (Controller.ctrl_mode == CONTROL_MODE_POSITION_PROFILE)
			pos_target = Controller.pos_setpoint;
	}
	if (isfinite(pos_target)) {
		/* 用位置环的角度差，保留跨圈时的最短路径方向。 */
		pos_error = position_profile_wrap_angle(pos_target - pos_actual);
		pos_target = position_profile_wrap_angle(pos_target);
	}
	if (pos_actual > M_PI)
		pos_actual -= M_2PI;

	/* A 相电流 (A) */
	d[0] = (FOC.Ia - FOC.Ia_midpoint) * FOC_CURRENT_SCALE;
	/* B 相电流 (A) */
	d[1] = (FOC.Ib - FOC.Ib_midpoint) * FOC_CURRENT_SCALE;
	/* C 相电流 (A) */
	d[2] = (FOC.Ic - FOC.Ic_midpoint) * FOC_CURRENT_SCALE;
	d[3] = FOC.Id;			     /* d 轴电流反馈 */
	d[4] = FOC.Iq;			     /* q 轴电流反馈 */
	d[5] = FOC.Iq_set;		     /* q 轴电流目标 */
	d[6] = Controller.vel_setpoint;	     /* 速度环目标 (rps) */
	d[7] = encoder.vel;		     /* 速度反馈 (rps) */
	d[8] = FOC.torque;		     /* 转矩估算 (N·m) */
	d[9] = FOC.Vbus_filt;		     /* 母线电压 (V) */
	d[10] = FOC.Vmag;		     /* 指令电压幅值 (V) */
	d[11] = FOC.saturated ? 1.0f : 0.0f; /* 电压饱和标志 */
	d[12] = FOC.Vd_out;		     /* d 轴指令电压 (V) */
	d[13] = FOC.Vq_out;		     /* q 轴指令电压 (V) */
	d[14] = encoder.phase;		     /* 电角度 (rad) */
	d[15] = encoder.phase_vel;	     /* 电角速度 (rad/s) */
	d[16] = FOC.isum;		     /* 三相电流和 (A), 恒应≈0 */
	d[17] = FOC.Va;			     /* A 相电压 (V) */
	d[18] = FOC.Vb;			     /* B 相电压 (V) */
	d[19] = FOC.Vc;			     /* C 相电压 (V) */
	d[20] = (float)FOC.statusword.errors.errors_code; /* 故障位掩码 */
	d[21] = FOC.Ibus_filt;		     /* 母线电流 (A) */
	d[22] = pos_target;		     /* 机械目标角 (rad) */
	d[23] = pos_actual;		     /* 机械实际角 (rad) */
	d[24] = pos_error;		     /* 跟随误差 (rad) */
}

/* 由 ADC 中断按分频调用 */
void vofa_push_sample(void)
{
	uint16_t next = (uint16_t)((vofa_head + 1U) % VOFA_RING);

	if (next == vofa_tail) {
		/* 缓冲满: 丢弃本次最新样本 (不动 tail), 保留 ISR 只操作 head.
		 * 原来是丢最旧 (修改 tail), ISR 和任务两个写者会竞争 tail */
		return;
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

	if (n > 0U)
		vofa_transmit(send_buf, (uint16_t)(p - send_buf));
}

#define VOFA_PERIOD_MS 1 /* USB 发包周期 (ms): 1 = 1kHz */

/* 电流阶跃数据整包上传: "$STEP,FS=..,N=..,CH=3,PRE=..\n" + n*3 个 float.
 * 分块发送并留间隔, 避免 CDC 上一次没发完就丢包. */
static void vofa_dump_step(void)
{
	char hdr[64];
	uint16_t n = FOC_step_count();
	const uint8_t *p = (const uint8_t *)FOC_step_data();
	uint32_t remaining = (uint32_t)n * 3U * 4U;
	int len = snprintf(hdr, sizeof(hdr),
			   "$STEP,FS=%lu,N=%u,CH=3,PRE=%u\n",
			   (unsigned long)PWM_FREQ_HZ, (unsigned)n,
			   (unsigned)FOC_step_pre());

	if (len > 0)
		vofa_transmit((uint8_t *)hdr, (uint16_t)len);
	vTaskDelay(pdMS_TO_TICKS(5));
	while (remaining > 0U) {
		uint32_t chunk = (remaining > 1024U) ? 1024U : remaining;

		vofa_transmit((uint8_t *)p, (uint16_t)chunk);
		p += chunk;
		remaining -= chunk;
		vTaskDelay(pdMS_TO_TICKS(5));
	}
	FOC_step_clear();
}

static void Vofa_task(void *argument)
{
	TickType_t last = xTaskGetTickCount();

	(void)argument;

	for (;;) {
		vTaskDelayUntil(&last, pdMS_TO_TICKS(VOFA_PERIOD_MS));
		uint8_t st = FOC_step_state();

		if (st == 2U) { /* 阶跃采完: 优先上传, 期间不发波形 */
			vofa_dump_step();
			last = xTaskGetTickCount(); /* 上传耗时不补跑 */
			continue;
		}
		/* 波形和参数回读共用 IN 端点，回包成功前保留请求。 */
		if (cfg_dump_req) {
			cfg_dump_req = 0;
			if (!vofa_send_config())
				cfg_dump_req = 1;
			continue;
		}
		if (sw_dump_req) {
			sw_dump_req = 0;
			if (!vofa_send_statusword())
				sw_dump_req = 1;
			continue;
		}
		if (st == 0U)
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
