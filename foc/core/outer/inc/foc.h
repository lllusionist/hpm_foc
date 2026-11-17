#ifndef __FOC_H__
#define __FOC_H__

#include "hpm_common.h"
#include "sys.h"
#include "util.h"
#include "tim.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdbool.h>
#include "config.h"

#define FOC_CURRENT_SHUNT_OHM (0.005f)
#define FOC_CURRENT_AMP_GAIN (20.0f)
#define FOC_CURRENT_SCALE \
	((3.3f / 65536.0f) / (FOC_CURRENT_SHUNT_OHM * FOC_CURRENT_AMP_GAIN))

#define FOC_IBUS_SHUNT_OHM (0.005f)
#define FOC_IBUS_AMP_GAIN (20.0f)
#define FOC_IBUS_SCALE \
	((3.3f / 65536.0f) / (FOC_IBUS_SHUNT_OHM * FOC_IBUS_AMP_GAIN))

#define FOC_VBUS_DIVIDER (16.0f)
#define FOC_VPHASE_DIVIDER (16.0f)

/* 开环电压通道 (FOC_voltage: R/L 标定 + 电感注入) 里的死区补偿开关.
 *
 * 实验结论 (2026-11-17, 5210, 已定论): 补偿的方向原来是**反的**, 它在叠加畸变
 * 而不是抵消. 用 0x27 的 R 段打印的 Voff = ures - I*R 做 A/B:
 *   补偿开: Voff = 0.244V,  L 标定 24.8µH, Kp = 0.047
 *   补偿关: Voff = 0.133V,  L 标定 11.9µH, Kp = 0.0225
 * 差值 0.111V ≈ 补偿量 0.096V -> 符号反了. 现在两处补偿 (FOC_voltage 与
 * FOC_current 的给定前馈) 都已改成减号, 且 DT_DEAD_NS 按实测 0.133V 调到 +140ns.
 * 预期下一次 0x27: Voff ≈ 0, L ≈ 12µH, Kp ≈ 0.0225.
 * 这个开关留着做回归: 改成 0 就能再测一遍 Voff, 平时必须是 1. */
#define DT_COMP_IN_VOLTAGE 1

/* 故障码：位掩码，可多故障并发（借鉴 dgm 的位域状态字设计）。
 * 每个故障占一个 bit，用 |= 置位、& 判断，避免多故障同时发生时被覆盖。 */
#define FOC_FAULT_NONE (0x0000u)
#define FOC_FAULT_PHASE_OC (1u << 0)     /* 相过流 (5kHz 任务检测) */
#define FOC_FAULT_OVER_VOLTAGE (1u << 1) /* 母线过压 */
#define FOC_FAULT_UNDER_VOLTAGE (1u << 2) /* 母线欠压 */
#define FOC_FAULT_ADC (1u << 3)          /* ADC / 采样异常 */
#define FOC_FAULT_IBUS_OC (1u << 4)      /* 母线过流 */
#define FOC_FAULT_PHASE_OC_ISR (1u << 5) /* 相过流硬限幅 (40kHz ISR 检测) */
#define FOC_FAULT_ENCODER (1u << 6)      /* 编码器停更或错误率过高 */

typedef struct {
	float ia;
	float ib;
	float ic;
	float limit;
	float resistance_voltage;
	float inductance_amplitude;
	int32_t motor_profile;
	uint32_t frames_since_arm;
	/* 电流环内部状态快照: 区分"环输出打满(自激)"还是"反馈丢了" */
	float id;     /* 上一拍实测 d 轴电流 (A) */
	float iq;     /* 上一拍实测 q 轴电流 (A) */
	float iq_cmd; /* 上一拍 iq 给定 (A) */
	float vd;     /* 上一拍输出电压 d (V) */
	float vq;     /* 上一拍输出电压 q (V) */
	float int_d;  /* 上一拍 d 轴积分项 (V) */
	float int_q;  /* 上一拍 q 轴积分项 (V) */
	float angle;  /* 上一拍电角度 (rad) */
	float kp;
	float ki;
} foc_phase_trip_t;

/* 标准状态字（借鉴 dgm 的 tMCStatusword，CiA 402 极简子集）：
 * 状态位与错误位分离，各自可用整体字段按字节收发，也可按 bit 读写。 */
typedef struct {
	union {
		uint16_t status_code;
		struct {
			uint16_t switched_on : 1;    /* 电机已使能 */
			uint16_t target_reached : 1; /* 控制目标到位 */
			uint16_t armed : 1;          /* PWM 已使能 (foc_arm) */
			uint16_t reserved : 13;
		};
	} status;

	union {
		uint16_t errors_code;
		struct {
			uint16_t phase_oc : 1;      /* 相过流 */
			uint16_t over_voltage : 1;  /* 母线过压 */
			uint16_t under_voltage : 1; /* 母线欠压 */
			uint16_t adc_fault : 1;     /* ADC 采样异常 */
			uint16_t ibus_oc : 1;       /* 母线过流 */
			uint16_t encoder_fault : 1; /* 编码器停更 */
			uint16_t reserved2 : 10;
		};
	} errors;
} foc_statusword_t;

typedef struct {

	volatile bool is_armed;
	volatile uint32_t fault_code; /* 位掩码, 0=正常, 见 FOC_FAULT_* */
	volatile foc_statusword_t statusword; /* 标准状态字 (状态+错误分离) */
	volatile bool adc_ready;
	volatile uint32_t adc_current_frames;
	volatile uint32_t adc_current_missed;
	volatile uint32_t adc_voltage_frames;
	volatile uint32_t adc_voltage_missed;
	volatile uint32_t adc_ibus_missed;

	volatile float Ia;
	volatile float Ib;
	volatile float Ic;
	volatile float Ibus;
	volatile float Ibus_filt;

	volatile float Ia_midpoint;
	volatile float Ib_midpoint;
	volatile float Ic_midpoint;
	volatile float Ibus_midpoint;

	volatile float Va_midpoint;
	volatile float Vb_midpoint;
	volatile float Vc_midpoint;

	volatile float Va;
	volatile float Vb;
	volatile float Vc;
	volatile float Vbus;
	volatile float Vbus_filt;

	volatile float Iq;
	volatile float Iq_filt;
	volatile float Id;
	volatile float Id_filt;

	volatile float Id_set; /* d 轴电流目标 (A) */
	volatile float Iq_set; /* q 轴电流目标 (A) */

	volatile float phase_offset; /* 调试用: 电角度附加偏移 (rad) */

	/* 调试用: 开环固定电压矢量 (>0 时旁路电流环, 直接输出该矢量) */
	volatile float openloop_voltage; /* 幅值 (V) */
	volatile float openloop_angle;	 /* 电角度 (rad) */

	volatile float current_ctrl_integral_d; /* 电流环 d 轴积分项 */
	volatile float current_ctrl_integral_q; /* 电流环 q 轴积分项 */

	volatile float
	    current_ctrl_p_gain; /* 电流环 P (上位机 USB 命令可在线改) */
	volatile float current_ctrl_i_gain; /* 电流环 I */

	volatile float Vd_out;	 /* 电流环输出电压 d 分量 (V) */
	volatile float Vq_out;	 /* 电流环输出电压 q 分量 (V) */
	volatile float Vmag;	 /* 指令电压幅值 |V| (V) */
	volatile bool saturated; /* 电压饱和标志 */
	volatile bool ff_enable; /* 反电势前馈/解耦使能 (调试用) */

	volatile float Power_filt; /* 功率 (W) */
	volatile float torque;	   /* 转矩估算 (N·m) */

	volatile float isum;

	/* 校准用: 开环电流模式 (校准任务下发角度, 中断里跑电流环) */
	volatile bool calib_mode;
	volatile float calib_angle;	/* 电角度 (rad) */
	volatile float calib_phase_vel;
	/* 1 = 由电流环中断按 calib_phase_vel 推进 calib_angle (磁链标定用):
	 * 1kHz 推进在 500rad/s 时会有 29° 阶梯误差, d 轴电压被拉偏 */
	volatile bool calib_adv_angle; /* 电角速度 (rad/s) */
	volatile float calib_id;	/* d 轴目标 (A) */
	volatile float calib_iq;	/* q 轴目标 (A) */

	/* 校准用: α 轴正弦电压注入 (电感测量, 解调累加在中断里做) */
	volatile bool calib_vinj_mode;
	volatile float vinj_omega; /* 注入角频率 (rad/s) */
	volatile float vinj_amp;   /* 正弦幅值 (V) */
	volatile float vinj_dc;	   /* 直流偏置 (V) */
	volatile float inj_phase;  /* 注入相位, 中断里推进 */
	volatile bool vinj_acc;	   /* 使能解调累加 */
	volatile float vinj_sum_sin;
	volatile float vinj_sum_cos;
	volatile uint32_t vinj_samples;

} foc_data_t;

extern volatile foc_data_t FOC;

bool foc_adc_midpoint(void);
bool foc_read_phase_trip(foc_phase_trip_t *trip);
void foc_update_voltages(void);
void FOC_update_current_ctrl_gain(float bw);
void foc_set_resistance_voltage(float voltage);
void foc_arm(void);
void foc_disarm(void);
void foc_init(void);
/* ADC 中断回调: 由 adc.c 以原始码值调用, 不再直接写 FOC 结构体 */
void foc_adc_callback(uint16_t ia_raw, uint16_t ib_raw, uint16_t ic_raw);

/* 电流阶跃测试 (临时调试): 40kHz 注入 Iq 阶跃并记录响应 */
void FOC_step_set_base(float base); /* 偏置电流 (小信号阶跃用) */
void FOC_step_start(float amp);
uint8_t FOC_step_state(void);	 /* 0=空闲 1=采集中 2=采完待发 */
uint16_t FOC_step_count(void);	 /* 每通道采样点数 */
uint16_t FOC_step_pre(void);	 /* 阶跃前预置点数 */
const float *FOC_step_data(void); /* [Iq, ref, Vq] * count */
void FOC_step_clear(void);

#endif /* __FOC_H__ */
