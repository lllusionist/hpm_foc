#ifndef __CONFIG_H__
#define __CONFIG_H__

#include "foc_ctrl.h"
#include "hpm_common.h"
#include "hpm_crc32.h"
#include <stdio.h>

// 布局版本: magic+version 变更时旧 Flash 配置自动失效
// v1 = 原始布局 (crc32 只覆盖部分字段)
// v2 = crc32 移到最后, 覆盖全部字段 (本次变更)
#define CONFIG_MAGIC   0x5A5A1236   /* magic 也变更, 确保与 v1 记录不兼容 */
#define CONFIG_VERSION 2U

#define OFFSET_LUT_NUM 512U
#define COGGING_MAP_NUM 512U // 512点 × 2字节 = 1024字节
// 补偿表缩放: Iq(A) × 5000 → int16 (量程 ±6.55A)
#define COGGING_SCALE 5000.0f

typedef struct {
	uint32_t magic;   // 配置区魔数
	uint32_t version; // 布局版本号 (CONFIG_VERSION)

	int32_t invert_motor_dir; // 电机方向反转(0/1)
	int32_t pole_pairs;       // 极对数
	float phase_resistance;   // 相电阻(Ω)
	float phase_inductance;   // 相电感(H)
	float flux_linkage;       // 永磁磁链 λ(Wb)
	float calib_current;      // 校准电流(A)
	float calib_voltage;      // 校准电压(V)
	float current_limit;      // 电流限制(A)
	float velocity_limit;     // 速度限制(rps)
	int32_t calib_valid;      // 校准数据有效标志

	int32_t encoder_offset;      // 编码器零位偏移
	int32_t offset_lut[OFFSET_LUT_NUM]; // 编码器偏移查找表(512点)
	int32_t encoder_dir;         // 编码器方向
	float protect_under_voltage; // 欠压保护阈值(V), 0=禁用
	float protect_over_voltage;  // 过压保护阈值(V)
	float protect_over_current;  // 过流保护阈值(A)
	float protect_i_bus_max;     // 母线电流最大限制(A)

	int32_t default_op_mode;      // 默认工作模式
	float pos_p_gain;             // 位置环比例增益
	float vel_p_gain;             // 速度环比例增益
	float vel_i_gain;             // 速度环积分增益
	float current_ff_gain;        // 电流前馈增益
	float current_ctrl_bw;        // 电流环带宽(Hz)
	int32_t anticogging_enable;   // 齿槽转矩补偿使能
	int32_t sync_target_enable;   // 目标值同步使能
	float target_velcity_window;  // 目标速度到位窗口
	float target_position_window; // 目标位置到位窗口
	float current_ramp_rate;      // 电流斜坡速率
	float velocity_ramp_rate;     // 速度斜坡速率
	float position_filter_bw;     // 位置滤波带宽(Hz)
	float profile_velocity;	      /* 轨迹速度上限 (rps) */
	float profile_accel;	      /* 轨迹加速度上限 (rps/s) */
	float profile_decel;	      /* 轨迹减速度上限 (rps/s) */

	int32_t node_id;               // CAN节点ID
	int32_t can_baudrate;          // CAN波特率
	int32_t heartbeat_consumer_ms; // 心跳消费超时时间(ms)
	int32_t heartbeat_producer_ms; // 心跳生产者周期(ms)

	int16_t cogging_map[COGGING_MAP_NUM]; // 齿槽补偿表(int16, 按机械角512点)
	uint32_t cogging_map_crc;             // 齿槽补偿表CRC校验
	int32_t cogging_map_valid;            // 齿槽补偿表有效标志

	float current_kp;            // 电流环 Kp(V/A)
	float current_ki;            // 电流环 Ki(V/(A·s))
	int32_t motor_profile;       // MOTOR_PROFILE_xxx

	float calib_drag_v_min;       // 开环拖动电压下限 (V)
	float calib_flux_iq_gain;     // 磁链标定拖动电流 = calib_current × 该系数
	float calib_j_iq_gain;        // J 标定加速电流 = calib_current × 该系数
	float calib_flux_rtest_ratio; // 磁链同速校验的降流比例
	float cog_sweep_rps;          // 齿槽扫描速度 (rps)

	float vel_integrator_limit; // Flash v2 旧积分限幅，保留用于回退固件
	float friction_move_a;      // 动摩擦前馈电流 (A)
	float friction_break_a;     // 静摩擦前馈电流 (A)

	// crc32 必须是最后一个字段: CONFIG_calculate_crc 覆盖其之前的所有内容
	uint32_t crc32; // 结构体整体CRC32校验 (覆盖除自身外的全部字段)
} tConfig;

// 电机配置档编号 (USB 0x5F 下发时直接传编号, $FOCFG 的 PROF 也是它)
#define MOTOR_PROFILE_4310 0 // 云台 4310: 8.29Ω/3.4mH/14 对极/0.9A
#define MOTOR_PROFILE_5210 1 // 航模 5210: 0.03Ω/9µH/KV300/44A
#define MOTOR_PROFILE_2312 2 // 航模 2312: 0.144Ω/28.5µH/7 对极/KV960
#define MOTOR_PROFILE_MAX MOTOR_PROFILE_2312

extern tConfig Config;

void config_init(void);
// 套用电机配置档 (只改参数, 调用方负责重算电流环增益/存 Flash)
void CONFIG_apply_motor_profile(int32_t profile);
uint32_t CONFIG_calculate_crc(const tConfig *cfg);
uint8_t CONFIG_save_to_flash(void);
uint8_t CONFIG_load_from_flash(void);

#endif // __CONFIG_H__
