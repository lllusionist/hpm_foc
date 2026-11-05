#ifndef __CONFIG_H__
#define __CONFIG_H__

#include "foc_ctrl.h"
#include "hpm_common.h"
#include "hpm_crc32.h"
#include <stdio.h>

/* 布局变更 (齿槽表 256 -> 512 点): 旧 Flash 配置自动失效 */
#define CONFIG_MAGIC 0x5A5A1235

#define OFFSET_LUT_NUM 512U
#define COGGING_MAP_NUM 512U /* 512点 × 2字节 = 1024字节 */
/* 补偿表缩放: Iq(A) × 5000 → int16 (量程 ±6.55A) */
#define COGGING_SCALE 5000.0f

typedef struct {
	uint32_t magic; /* 配置区魔数（标识有效配置） */

	int32_t invert_motor_dir; /* 电机方向反转（0/1） */
	int32_t pole_pairs;	  /* 极对数 */
	float phase_resistance;	  /* 相电阻（Ω） */
	float phase_inductance;	  /* 相电感（H） */
	float flux_linkage;	  /* 永磁磁链 λ（Wb，峰值相磁链，校准得到） */
	float calib_current;	  /* 校准电流（A） */
	float calib_voltage;	  /* 校准电压（V） */
	float current_limit;	  /* 电流限制（A） */
	float velocity_limit;	  /* 速度限制（rps） */
	int32_t calib_valid;	  /* 校准数据有效标志 */

	int32_t encoder_offset; /* 编码器零位偏移 */
	int32_t offset_lut
	    [OFFSET_LUT_NUM]; /* 编码器偏移查找表（128个int32 = 512字节） */
	int32_t encoder_dir;  /* 编码器方向 */
	float protect_under_voltage; /* 欠压保护阈值（V） */
	float protect_over_voltage;  /* 过压保护阈值（V） */
	float protect_over_current;  /* 过流保护阈值（A） */
	float protect_i_bus_max;     /* 母线电流最大限制（A） */

	int32_t default_op_mode;      /* 默认工作模式 */
	float pos_p_gain;	      /* 位置环比例增益 */
	float vel_p_gain;	      /* 速度环比例增益 */
	float vel_i_gain;	      /* 速度环积分增益 */
	float current_ff_gain;	      /* 电流前馈增益 */
	float current_ctrl_bw;	      /* 电流环带宽（Hz） */
	int32_t anticogging_enable;   /* 齿槽转矩补偿使能 */
	int32_t sync_target_enable;   /* 目标值同步使能 */
	float target_velcity_window;  /* 目标速度到位窗口 */
	float target_position_window; /* 目标位置到位窗口 */
	float current_ramp_rate;      /* 电流斜坡速率 */
	float velocity_ramp_rate;     /* 速度斜坡速率 */
	float position_filter_bw;     /* 位置滤波带宽（Hz） */
	float profile_velocity;	      /* 轮廓模式最大速度 */
	float profile_accel;	      /* 轮廓模式加速度 */
	float profile_decel;	      /* 轮廓模式减速度 */

	int32_t node_id;	       /* CAN节点ID */
	int32_t can_baudrate;	       /* CAN波特率 */
	int32_t heartbeat_consumer_ms; /* 心跳消费超时时间（ms） */
	int32_t heartbeat_producer_ms; /* 心跳生产者周期（ms） */

	int16_t
	    cogging_map[COGGING_MAP_NUM]; /* 齿槽补偿表(int16, 按机械角512点) */
	uint32_t cogging_map_crc;	  /* 齿槽补偿表CRC校验 */
	int32_t cogging_map_valid;	  /* 齿槽补偿表有效标志 */

	uint32_t crc32; /* 结构体整体CRC32校验 */

	/* 以下字段在 crc32 之后, 不参与上面的 CRC: 旧 Flash 记录仍能通过校验 */
	uint32_t current_gain_magic; /* current_kp/ki 有效标志 */
	float current_kp;	     /* 电流环 Kp（V/A） */
	float current_ki;	     /* 电流环 Ki（V/(A·s)） */

	/* I/F 强拖参数 (同样在 crc32 之后; 由 if_cfg_magic 校验有效性) */
	uint32_t if_cfg_magic;
	float if_current;	/* 强拖电流 (A) */
	float if_accel_rps2;	/* 频率斜坡加速度 (机械 rps/s) */
	float if_switch_rps;	/* 交接转速 (机械 rps) */
	float if_blend_ms;	/* 角度交接平滑时间 (ms) */
	float if_stall_deg;	/* 失步/负载角阈值 (电角度) */
	float if_hold_ms;	/* 交接前稳态保持时间 (ms), 追加在末尾 */

	/* 标定得到的机械参数 (同样在 crc32 之后, 由 mech_cfg_magic 校验) */
	uint32_t mech_cfg_magic;
	float inertia;	       /* 转子惯量 J (kg·m²), CS_MOTOR_J_* 实测 */
	float friction_current; /* 稳速时的摩擦电流 (A), 闭环稳态 Iq 自动记录 */
} tConfig;

extern tConfig Config;

void config_init(void);
uint32_t CONFIG_calculate_crc(const tConfig *cfg);
uint8_t CONFIG_save_to_flash(void);
uint8_t CONFIG_load_from_flash(void);

#endif /* __CONFIG_H__ */