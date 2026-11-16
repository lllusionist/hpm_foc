#include <string.h>
#include "anticog.h"
#include "config.h"
#include "encoder.h"
#include "foc.h"
#include "foc_ctrl.h"
#include "foc_task.h"
#include "util.h"
#include <math.h>

#define CAL_LOOP_HZ (1000U)
#define COGGING_MAP_NUM_ COGGING_MAP_NUM

/* 以下代码从 foc_cal.c 原样搬出, 仅改 include */
/* 齿槽标定(位置保持法): 每点停稳后多点平均, 正反两遍消摩擦 */
#define CG_GAP_STEPS 100      /* 起始/换向空走步数 */
#define CG_GAP_MS 50U	      /* 空走每步时间 */
#define CG_SETTLE_POS 0.0025f /* 到位窗口 (rad, 约 0.14° 机械 = 1/5 bin);
                               * 原来 0.006 (半个 bin) 会让相邻点索引乱跳 */
#define CG_SETTLE_VEL 0.05f   /* 停稳速度窗口 (rps) */
#define CG_SETTLE_TICKS 15U   /* 连续 15ms 满足才算停稳 (5ms 还在余振里) */
#define CG_SETTLE_MAX_MS 300U /* 单点最长等待 */
#define CG_AVG_SAMPLES 64U    /* 每点平均样本数 (1kHz -> 64ms):
                               * 单点电流噪声 ~9mA, 取 8 点时均方误差 3.2mA,
                               * 已占齿槽表幅值(14.5mA)的 22%; 取 64 点降到 1.1mA */
#define CG_TIMEOUT_S 300U     /* 总超时: 512点x2方向, 每点最多 300ms 停稳 */
#define CG_LP_BINS 1U	      /* 1 = 不做圆周低通 (避免 84 阶相位滞后) */

/* 恒速扫描法(0x2E): 匀速转 N 圈, 按机械角分箱平均;
 * 齿槽位置锁定(同向叠加), 摩擦随方向反号(相消) */
/* 扫描转速要让齿槽频率落在速度环带宽内: 0.1rps 时 84 阶=8.4Hz 可信;
 * 0.3rps 时 25Hz 相位滞后, 实测该表对 84 阶无效 */
#define CG_SW_RPS_DEFAULT 0.10f	    /* 扫描转速 (rps). 试过 0.05: 84 阶只从 -29% 改善到 -40%,
				 * 但会粘滑(vel 频繁掉到 0)、14 阶反而从 -84% 退到 -64%,
				 * 总 std 仅 -19.2% -> -20.7% (在测量噪声内). 回到 0.1 更稳也更快 */
#define CG_SW_REVS 4.0f	    /* 每方向累加圈数 (0.1rps: 4.5圈x2 = 90s, 每箱 ~78 点) */
#define CG_SW_RAMP 1.5f	    /* 速度指令斜坡 (rps/s) */
#define CG_SW_SKIP_REV 0.5f /* 每方向先空转半圈再开始累加 */

static uint8_t cg_method = 0U; /* 0 = 位置保持(0x28), 1 = 恒速扫描(0x2E) */
static uint8_t cg_ff_save = 1U; /* 扫描前保存摩擦前馈开关, 结束后恢复 */
static float cg_sw_prev;  /* 上一拍机械角 */
static float cg_sw_travel; /* 本方向累计转角 (rad) */
static float cg_sw_dir;	   /* +1 / -1 */
static float cg_sw_target; /* 当前速度指令 */
static float cg_sw_last_travel; /* 上次记录的位置 (判停滞) */
static uint16_t cg_sw_nomove_ms; /* 连续不动计数 (ms) */
static uint8_t cg_sw_phase; /* 0=加减速 1=累加 2=收尾 */
static int32_t cg_sw_sum[2][COGGING_MAP_NUM];
static uint16_t cg_sw_cnt[2][COGGING_MAP_NUM];


typedef enum {
	CG_PH_GAP_FWD,
	CG_PH_SCAN_FWD,
	CG_PH_GAP_REV,
	CG_PH_SCAN_REV,
	CG_PH_DONE
} tCgPhase;

typedef enum { CG_ST_MOVE, CG_ST_SETTLE, CG_ST_AVG } tCgState;

static int cg_number; /* 已走步数 (诊断) */
static uint32_t cg_total_ticks;
static bool cg_valid;
static bool cg_running;
static int16_t cg_tmp[COGGING_MAP_NUM];
static float cg_pos_p_save;
static tCgPhase cg_phase;
static tCgState cg_state;
static int cg_step_idx;	 /* 当前阶段内步数 */
static uint32_t cg_tick; /* 当前步内 1kHz 计数 */
static uint32_t cg_settle_cnt;
static uint32_t cg_settle_timeouts; /* 停稳超时点数 (整次标定的质量指标) */
static int32_t cg_iq_sum;
static uint32_t cg_iq_n;
static uint8_t cg_written[(COGGING_MAP_NUM + 7U) / 8U];
/* 齿槽表 dump 游标: >= COGGING_MAP_NUM 表示空闲 */
static uint32_t cg_dump_idx = COGGING_MAP_NUM;

/* 位置指令前进/后退一步 (单圈机械角, 循环卷绕) */
static void cg_step(float dir)
{
	const float delta = dir * (M_2PI / (float)COGGING_MAP_NUM);
	float pos_ref = Controller.input_position + delta;
	while (pos_ref >= M_2PI)
		pos_ref -= M_2PI;
	while (pos_ref < 0.0f)
		pos_ref += M_2PI;
	Controller.input_position = pos_ref;
}

/* 单圈机械角 -> 补偿表下标 (0 ~ COGGING_MAP_NUM-1) */
static int cg_index(float pos)
{
	int idx = (int)nearbyintf((float)COGGING_MAP_NUM * pos / M_2PI);
	if (idx >= (int)COGGING_MAP_NUM)
		idx = 0;
	if (idx < 0)
		idx = 0;
	return idx;
}

/* 当前机械角 (rad, 单圈), 用对齐计数保证与校准零位一致 */
static float cg_pos(void)
{
	return (float)encoder.count_aligned * (M_2PI / ENCODER_CPR_F);
}

/* 位置误差卷绕到 [-π, π] */
static float cg_pos_err(void)
{
	float perr = Controller.input_position - cg_pos();
	while (perr > M_PI)
		perr -= M_2PI;
	while (perr < -M_PI)
		perr += M_2PI;
	return perr;
}

/* 静止判据: 位置误差和速度都进入窗口 */
static bool cg_settled(void)
{
	return (fabsf(cg_pos_err()) < CG_SETTLE_POS) &&
	       (fabsf(encoder.vel) < CG_SETTLE_VEL);
}

static int16_t cg_to_i16(int32_t raw)
{
	if (raw > 32767)
		raw = 32767;
	if (raw < -32768)
		raw = -32768;
	return (int16_t)raw;
}

static int cg_wrap(int idx)
{
	while (idx < 0)
		idx += (int)COGGING_MAP_NUM;
	while (idx >= (int)COGGING_MAP_NUM)
		idx -= (int)COGGING_MAP_NUM;
	return idx;
}

static void cg_mark(int idx)
{
	cg_written[idx >> 3] |= (uint8_t)(1U << (idx & 7));
}

static bool cg_is_written(int idx)
{
	return (cg_written[idx >> 3] & (uint8_t)(1U << (idx & 7))) != 0U;
}

void anticogging_start(void)
{
	cg_pos_p_save = Config.pos_p_gain;

	/* 硬要求只有极对数: 齿槽扫描靠速度环拖动, 需要正确的电角度换算.
	 * calib_valid 只是"标定走完"的印章, 标定反复中断时不该把扫描一起锁死,
	 * 降级为警告 (编码器零位可能不准 -> 换相有偏差, 但仍然会转). */
	if (Config.pole_pairs <= 0) {
		printf("cogging: refused, pole_pairs=%ld -> run 0x27 "
		       "to measure pole pairs first\n",
		       (long)Config.pole_pairs);
		cg_running = false;
		return;
	}
	if (!Config.calib_valid)
		printf("cogging: WARN calib_valid=0 (标定未走完), "
		       "编码器零位可能不准, 仍尝试扫描\n");

	foc_arm();
	FOC.calib_mode = false;
	FOC.calib_vinj_mode = false;

	Config.cogging_map_valid = 0;
	memset(Config.cogging_map, 0, sizeof(Config.cogging_map));
	memset(cg_written, 0, sizeof(cg_written));

	cg_number = 0;
	cg_total_ticks = 0;
	cg_valid = false;
	cg_running = true;
	cg_iq_sum = 0;
	cg_iq_n = 0;
	cg_settle_timeouts = 0;

	if (cg_method == 1U) {
		Controller.ctrl_mode = CONTROL_MODE_VELOCITY_RAMP;
		Controller.target_reached = false;
		CONTROLLER_reset();
		cg_ff_save = CONTROLLER_get_friction_ff();
		CONTROLLER_set_friction_ff(0U); /* 前馈换挡会注入台阶, 扫描期间关 */
		memset(cg_sw_sum, 0, sizeof(cg_sw_sum));
		memset(cg_sw_cnt, 0, sizeof(cg_sw_cnt));
		cg_sw_prev = cg_pos();
		cg_sw_travel = 0.0f;
		cg_sw_dir = 1.0f;
		cg_sw_target = 0.0f;
		cg_sw_phase = 0U;
		cg_sw_last_travel = 0.0f;
		cg_sw_nomove_ms = 0U;
		printf("cogging: sweep start, %.2f rps x %.0f revs x2 dirs "
		       "(skip %.1f rev), timeout=%us\n",
		       (double)Config.cog_sweep_rps, (double)CG_SW_REVS,
		       (double)CG_SW_SKIP_REV, (unsigned)CG_TIMEOUT_S);
		return;
	}

	Controller.ctrl_mode = CONTROL_MODE_POSITION_FILTER;
	Controller.target_reached = false;
	CONTROLLER_reset();
	Config.pos_p_gain = 50.0f;
	cg_phase = CG_PH_GAP_FWD;
	cg_state = CG_ST_MOVE;
	cg_step_idx = 0;
	cg_tick = 0;
	cg_settle_cnt = 0;
	printf("cogging: start, %u pts x2 dirs, pos_p=%.1f, timeout=%us\n",
	       (unsigned)COGGING_MAP_NUM, (double)Config.pos_p_gain,
	       (unsigned)CG_TIMEOUT_S);
}

/* 扫完一遍后处理: 去均值 + 圆周低通 */
static void cg_finish_map(void)
{
	int32_t sum = 0;
	int32_t mean;

	/* 漏采的 bin 用两侧最近有效点插值 */
	uint32_t missing = 0;
	for (uint32_t i = 0; i < COGGING_MAP_NUM; i++) {
		if (cg_is_written((int)i))
			continue;
		missing++;
		int lo_d = 1;
		while ((lo_d < (int)COGGING_MAP_NUM) &&
		       !cg_is_written(cg_wrap((int)i - lo_d)))
			lo_d++;
		int hi_d = 1;
		while ((hi_d < (int)COGGING_MAP_NUM) &&
		       !cg_is_written(cg_wrap((int)i + hi_d)))
			hi_d++;
		if ((lo_d >= (int)COGGING_MAP_NUM) ||
		    (hi_d >= (int)COGGING_MAP_NUM))
			continue;
		int32_t vl = Config.cogging_map[cg_wrap((int)i - lo_d)];
		int32_t vh = Config.cogging_map[cg_wrap((int)i + hi_d)];
		Config.cogging_map[i] =
		    (int16_t)(vl + (vh - vl) * lo_d / (lo_d + hi_d));
		cg_mark((int)i);
	}
	if (missing)
		printf("cogging: filled %u missing bins\n", (unsigned)missing);

	for (uint32_t i = 0; i < COGGING_MAP_NUM; i++)
		sum += Config.cogging_map[i];
	mean = sum / (int32_t)COGGING_MAP_NUM;
	for (uint32_t i = 0; i < COGGING_MAP_NUM; i++)
		Config.cogging_map[i] =
		    cg_to_i16((int32_t)Config.cogging_map[i] - mean);

	/* 圆周低通: 1 bin = 不滤波(避免高阶相位滞后) */
	for (uint32_t i = 0; i < COGGING_MAP_NUM; i++) {
		int32_t s = 0;
		for (int k = -(int)(CG_LP_BINS / 2U);
		     k <= (int)(CG_LP_BINS / 2U); k++) {
			int j = (int)i + k;
			while (j < 0)
				j += (int)COGGING_MAP_NUM;
			while (j >= (int)COGGING_MAP_NUM)
				j -= (int)COGGING_MAP_NUM;
			s += Config.cogging_map[j];
		}
		cg_tmp[i] = (int16_t)(s / (int32_t)CG_LP_BINS);
	}
	for (uint32_t i = 0; i < COGGING_MAP_NUM; i++)
		Config.cogging_map[i] = cg_tmp[i];
}

/* 谐波自检: 判断表里是齿槽(84/168 阶)还是摩擦/编码器误差 */
static void cg_report_harmonics(void)
{
	int best_h[3] = { 0, 0, 0 };
	float best_a[3] = { 0.0f, 0.0f, 0.0f };

	for (int h = 1; h <= 200; h++) {
		float wr = cosf(6.2831853f * (float)h / (float)COGGING_MAP_NUM);
		float wi = -sinf(6.2831853f * (float)h / (float)COGGING_MAP_NUM);
		float cr = 1.0f, ci = 0.0f;
		float sr = 0.0f, si = 0.0f;

		for (uint32_t i = 0; i < COGGING_MAP_NUM; i++) {
			float v = (float)Config.cogging_map[i];

			sr += v * cr;
			si += v * ci;
			float t = cr * wr - ci * wi;

			ci = cr * wi + ci * wr;
			cr = t;
		}
		float a = sqrtf(sr * sr + si * si) * 2.0f /
			  (float)COGGING_MAP_NUM;

		for (int k = 0; k < 3; k++) {
			if (a > best_a[k]) {
				for (int j = 2; j > k; j--) {
					best_a[j] = best_a[j - 1];
					best_h[j] = best_h[j - 1];
				}
				best_a[k] = a;
				best_h[k] = h;
				break;
			}
		}
	}
	printf("cogging: top harm h=%d(%.1fmA) h=%d(%.1fmA) h=%d(%.1fmA)\n",
	       best_h[0], (double)(best_a[0] * 1000.0f / COGGING_SCALE),
	       best_h[1], (double)(best_a[1] * 1000.0f / COGGING_SCALE),
	       best_h[2], (double)(best_a[2] * 1000.0f / COGGING_SCALE));
	/* 物理意义阶次: 14=极对数, 28=极数, 24=槽数, 84/168=齿槽周期 */
	{
		static const int probe[] = { 6, 12, 14, 24, 28, 42, 84, 168 };
		printf("cogging: harm of interest ");
		for (uint32_t k = 0; k < sizeof(probe) / sizeof(probe[0]); k++) {
			int h = probe[k];
			float wr = cosf(6.2831853f * (float)h /
					(float)COGGING_MAP_NUM);
			float wi = -sinf(6.2831853f * (float)h /
					 (float)COGGING_MAP_NUM);
			float cr = 1.0f, ci = 0.0f, sr = 0.0f, si = 0.0f;
			for (uint32_t i = 0; i < COGGING_MAP_NUM; i++) {
				float v = (float)Config.cogging_map[i];
				sr += v * cr;
				si += v * ci;
				float t = cr * wr - ci * wi;
				ci = cr * wi + ci * wr;
				cr = t;
			}
			float a = sqrtf(sr * sr + si * si) * 2.0f /
				  (float)COGGING_MAP_NUM;
			printf("%d:%.1fmA ", h,
			       (double)(a * 1000.0f / COGGING_SCALE));
		}
		printf("\n");
	}
}

/* 空走: 每 CG_GAP_MS 一步 */
static void cg_gap_step(float dir)
{
	if (++cg_tick < CG_GAP_MS)
		return;
	cg_tick = 0;
	cg_step(dir);
	cg_number++;
	if (++cg_step_idx < CG_GAP_STEPS)
		return;

	cg_step_idx = 0;
	if (cg_phase == CG_PH_GAP_FWD)
		cg_phase = CG_PH_SCAN_FWD;
	else
		cg_phase = CG_PH_SCAN_REV;
	cg_state = CG_ST_MOVE;
}

/* 采样: 移动 -> 等停稳 -> 多点平均 */
static void cg_scan_step(float dir)
{
	switch (cg_state) {
	case CG_ST_MOVE:
		cg_step(dir);
		cg_number++;
		cg_state = CG_ST_SETTLE;
		cg_settle_cnt = 0;
		cg_tick = 0;
		break;

	case CG_ST_SETTLE:
		cg_tick++;
		if (cg_settled()) {
			if (++cg_settle_cnt >= CG_SETTLE_TICKS) {
				cg_iq_sum = 0;
				cg_iq_n = 0;
				cg_state = CG_ST_AVG;
			}
		} else {
			cg_settle_cnt = 0;
		}
		if (cg_tick >= CG_SETTLE_MAX_MS) {
			/* 个别点被齿槽夹住就就地采样并计数, 超时超 20% 才判失败 */
			cg_settle_timeouts++;
			printf("cogging: settle timeout #%u at step %d\n",
			       (unsigned)cg_settle_timeouts, cg_number);
			cg_iq_sum = 0;
			cg_iq_n = 0;
			cg_state = CG_ST_AVG;
		}
		break;

	case CG_ST_AVG: {
		cg_iq_sum += (int32_t)(FOC.Iq_filt * COGGING_SCALE);
		if (++cg_iq_n < CG_AVG_SAMPLES)
			break;

		int16_t v = cg_to_i16(cg_iq_sum / (int32_t)cg_iq_n);
		/* 索引用指令位置: 每步正好一个 bin, 实测位置会落进相邻 bin */
		int idx = cg_index(Controller.input_position);
		if ((cg_phase == CG_PH_SCAN_FWD) || !cg_is_written(idx)) {
			Config.cogging_map[idx] = v;
			cg_mark(idx);
		} else {
			Config.cogging_map[idx] =
			    (Config.cogging_map[idx] + v) / 2;
		}

		cg_state = CG_ST_MOVE;
		if (++cg_step_idx < (int)COGGING_MAP_NUM)
			break;

		cg_step_idx = 0;
		if (cg_phase == CG_PH_SCAN_FWD)
			cg_phase = CG_PH_GAP_REV;
		else
			cg_phase = CG_PH_DONE;
		cg_state = CG_ST_MOVE;
		cg_tick = 0;
	} break;
	}
}

/* 0 = 位置保持(0x28), 1 = 恒速扫描(0x2E) */
void anticogging_set_method(uint8_t m)
{
	cg_method = (m != 0U) ? 1U : 0U;
}

/* 建表: 正反两方向平均 */
static void cg_sweep_build(void)
{
	uint32_t both = 0, single = 0;
	float fric_sum = 0.0f;
	uint32_t fric_n = 0;

	for (uint32_t i = 0; i < COGGING_MAP_NUM; i++) {
		bool hf = cg_sw_cnt[0][i] > 0U;
		bool hr = cg_sw_cnt[1][i] > 0U;
		float f = hf ? ((float)cg_sw_sum[0][i] / (float)cg_sw_cnt[0][i])
			     : 0.0f;
		float r = hr ? ((float)cg_sw_sum[1][i] / (float)cg_sw_cnt[1][i])
			     : 0.0f;
		float v;

		if (hf && hr) {
			v = 0.5f * (f + r);
			fric_sum += 0.5f * (f - r); /* 摩擦随方向反号 */
			fric_n++;
			both++;
		} else if (hf) {
			v = f;
			single++;
		} else if (hr) {
			v = r;
			single++;
		} else {
			continue;
		}
		Config.cogging_map[i] = cg_to_i16((int32_t)v);
		cg_mark((int)i);
	}
	printf("cogging: sweep bins %u both, %u single\n", (unsigned)both,
	       (unsigned)single);
	if (fric_n)
		printf("cogging: sweep friction=%.1f mA (fwd-rev 差值/2)\n",
		       (double)(fric_sum / (float)fric_n * 1000.0f / COGGING_SCALE));
	cg_finish_map(); /* 补漏点 + 去均值 + 可选圆周低通 */
}

/* 恒速扫描: 加速 -> 累加 -> 换向 -> 建表 */
static bool cg_sweep_loop(void)
{
	float mech = cg_pos();
	float d = mech - cg_sw_prev;

	while (d > M_PI)
		d -= M_2PI;
	while (d < -M_PI)
		d += M_2PI;
	cg_sw_prev = mech;

	if ((cg_total_ticks % CAL_LOOP_HZ) == 0U) {
		printf("cogging: sweep dir=%+d trav=%.2f rev vel=%.3f "
		       "iq=%.3f enc_bad=%lu\n",
		       (cg_sw_dir > 0.0f) ? 1 : -1,
		       (double)(cg_sw_travel / M_2PI), (double)encoder.vel,
		       (double)FOC.Iq_filt,
		       (unsigned long)encoder.bad_frame_cnt);
	}

	/* 粘滑检测: 0.1rps 时静/动摩擦差会让转子"跳一下停一下",
	 * 采到的电流是历史相关而不是位置相关 -> 扫出来的表没用 */
	if (cg_sw_phase == 1U) {
		static uint16_t slip_ms;
	
		if (fabsf(encoder.vel - cg_sw_target) >
		    0.5f * fabsf(cg_sw_target)) {
			if (++slip_ms > 3000U) {
				printf("cogging: 速度粘滑 (vel=%.3f tgt=%.3f), "
				       "\u8868\u4e0d\u53ef\u4fe1 -> \u505c\u6b62; \u8c03\u5927 cog_sweep_rps\n",
				       (double)encoder.vel, (double)cg_sw_target);
				cg_valid = false;
				cg_running = false;
				return true;
			}
		} else {
			slip_ms = 0U;
		}
	}
	
	if (cg_sw_phase == 0U) { /* 加减速到目标转速 */
		float step = CG_SW_RAMP / (float)CAL_LOOP_HZ;
		float tgt = cg_sw_dir * Config.cog_sweep_rps;

		if (cg_sw_target < tgt) {
			cg_sw_target += step;
			if (cg_sw_target > tgt)
				cg_sw_target = tgt;
		} else if (cg_sw_target > tgt) {
			cg_sw_target -= step;
			if (cg_sw_target < tgt)
				cg_sw_target = tgt;
		}
		Controller.input_velocity = cg_sw_target;
		if (fabsf(cg_sw_target - tgt) < 0.02f) {
			cg_sw_travel = 0.0f;
			cg_sw_phase = 1U;
		}
		return false;
	}

	if (cg_sw_phase == 1U) { /* 累加 */
		float skip = CG_SW_SKIP_REV * M_2PI;

		cg_sw_travel += fabsf(d);

		/* 停滞检测: 5s 内几乎没走就判失败, 别等 300s 超时
		 * (速度环增益太低/拖不动时就是这种表现) */
		if (fabsf(cg_sw_travel - cg_sw_last_travel) > 0.02f * M_2PI) {
			cg_sw_last_travel = cg_sw_travel;
			cg_sw_nomove_ms = 0U;
		} else if (++cg_sw_nomove_ms > 5000U) {
			printf("cogging: sweep 转子 5s 没动 (vel=%.3f) -> "
			       "检查速度环增益/摩擦\n",
			       (double)encoder.vel);
			cg_valid = false;
			cg_running = false;
			return true;
		}

		if (cg_sw_travel >= skip) {
			int idx = cg_index(mech);
			int k = (cg_sw_dir > 0.0f) ? 0 : 1;

			cg_sw_sum[k][idx] +=
			    (int32_t)(FOC.Iq_filt * COGGING_SCALE);
			cg_sw_cnt[k][idx]++;
		}
		if (cg_sw_travel >= skip + CG_SW_REVS * M_2PI) {
			if (cg_sw_dir > 0.0f) {
				cg_sw_dir = -1.0f;
				cg_sw_phase = 0U; /* 先减速到 0 再反向 */
			} else {
				cg_sw_phase = 2U;
			}
		}
		return false;
	}

	/* 收尾 */
	Controller.input_velocity = 0.0f;
	cg_sweep_build();
	cg_valid = true;
	cg_running = false;
	return true;
}

int anticogging_step(void)
{
	if (!cg_running)
		return -1;
	return (int)cg_phase;
}

bool anticogging_loop(void)
{
	if (!cg_running)
		return true;

	if (++cg_total_ticks > (uint32_t)CG_TIMEOUT_S * CAL_LOOP_HZ) {
		printf("cogging: timeout at step %d\n", cg_number);
		cg_valid = false;
		cg_running = false;
		return true;
	}
	if (cg_method == 1U)
		return cg_sweep_loop();

	if ((cg_total_ticks % CAL_LOOP_HZ) == 0U) {
		printf("cogging: step=%d ph=%d st=%d inpos=%.3f act=%.3f "
		       "iq=%.2f\n",
		       cg_number, (int)cg_phase, (int)cg_state,
		       (double)Controller.input_position, (double)cg_pos(),
		       (double)FOC.Iq_filt);
	}

	switch (cg_phase) {
	case CG_PH_GAP_FWD:
		cg_gap_step(+1.0f);
		break;
	case CG_PH_SCAN_FWD:
		cg_scan_step(+1.0f);
		break;
	case CG_PH_GAP_REV:
		cg_gap_step(-1.0f);
		break;
	case CG_PH_SCAN_REV:
		cg_scan_step(-1.0f);
		break;
	case CG_PH_DONE:
		cg_finish_map();
		cg_valid = true;
		cg_running = false;
		return true;
	}
	return false;
}

void anticogging_end(void)
{
	cg_running = false;

	foc_disarm();

	/* 恢复位置环增益, 免得 50.0 被存进 Flash */
	Config.pos_p_gain = cg_pos_p_save;
	/* 恢复扫描期间关掉的摩擦前馈 */
	CONTROLLER_set_friction_ff(cg_ff_save);

	/* 停稳超时 >20% 判为不可信 */
	if (cg_settle_timeouts > (2U * COGGING_MAP_NUM / 5U)) {
		printf("cogging: too many settle timeouts (%u), mark invalid\n",
		       (unsigned)cg_settle_timeouts);
		cg_valid = false;
	}

	if (cg_valid) {
		Config.cogging_map_valid = 1;
		Config.anticogging_enable = 1;
		uint8_t sv = CONFIG_save_to_flash();
		int16_t mn = 32767, mx = -32768;
		for (uint32_t i = 0; i < COGGING_MAP_NUM; i++) {
			if (Config.cogging_map[i] < mn)
				mn = Config.cogging_map[i];
			if (Config.cogging_map[i] > mx)
				mx = Config.cogging_map[i];
		}
		printf("cogging: done, %u pts, %d steps, map %d~%d pkpk %d "
		       "(%.3f A)\n",
		       (unsigned)COGGING_MAP_NUM, cg_number, mn, mx,
		       (int)(mx - mn), (float)(mx - mn) / COGGING_SCALE);
		printf("cogging: save %s, timeout %u/%u\n",
		       (sv == 0U) ? "ok" : "FAIL",
		       (unsigned)cg_settle_timeouts,
		       (unsigned)(2U * COGGING_MAP_NUM));
		cg_report_harmonics();
		/* 自动 dump 整张表, 便于离线分析 */
		cg_dump_idx = 0U;
	} else {
		Config.cogging_map_valid = 0;
		printf("cogging: aborted, invalid (step=%d)\n", cg_number);
	}

	Controller.input_velocity = 0.0f;
	Controller.input_velocity_buffer = 0.0f;
}

/* 齿槽表 dump(0x2C): 每拍打 8 点, 不阻塞中断 */
void cogging_dump_request(void)
{
	cg_dump_idx = 0;
}

void cogging_dump_loop(void)
{
	if (cg_dump_idx >= COGGING_MAP_NUM)
		return;
	if (cg_dump_idx == 0U)
		printf("cogging_dump: begin valid=%ld\n",
		       (long)Config.cogging_map_valid);
	for (int k = 0; k < 8 && cg_dump_idx < COGGING_MAP_NUM; k++) {
		printf("%u,%d\n", (unsigned)cg_dump_idx,
		       (int)Config.cogging_map[cg_dump_idx]);
		cg_dump_idx++;
	}
	if (cg_dump_idx >= COGGING_MAP_NUM)
		printf("cogging_dump: end\n");
}
