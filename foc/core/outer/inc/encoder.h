#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "hpm_common.h"
#include "hpm_soc.h"
#include "hpm_spi_drv.h"
#include "hpm_clock_drv.h"
#include "hpm_gpio_drv.h"
#include <stdio.h>
#include "spi.h"
#include "FreeRTOS.h"
#include "task.h"

#define ENCODER_SPI HPM_SPI1
#define ENCODER_SPI_SCLK_FREQ (10000000UL) // 芯片上限 16MHz
#define ENCODER_CS_GPIO HPM_GPIO0
#define ENCODER_CS_PORT GPIO_DO_GPIOC
#define ENCODER_CS_PIN (11U)
#define ENCODER_FRAME_LEN (6U)
#define ENCODER_UPDATE_FREQ (5000U) // 改这个要重算 PLL 增益
#define ENCODER_TASK_HZ ENCODER_UPDATE_FREQ // 采集任务频率
// 半个周期要几个 tick; 先除再算, 不能走 pdMS_TO_TICKS(毫秒) 那条路
#define ENCODER_HALF_TICKS (configTICK_RATE_HZ / ENCODER_TASK_HZ / 2)

// 角度统一按 24 位: 芯片 21 位左移 3 位
#define ENCODER_CPR 16777216
#define ENCODER_CPR_F 16777216.0f
#define ENCODER_CPR_HALF 8388608
#define ENCODER_MASK 0x00FFFFFFU

#define ENCODER_PLL_BW_HZ (100.0f) // 环路带宽
#define ENCODER_PLL_SNAP_VEL (ENCODER_CPR_F / 60.0f) // 约 1 转/分

// 方向/零位/极对数由校准结果 Config.encoder_dir/encoder_offset/pole_pairs 提供

typedef struct sEncoder
{
	uint8_t need_init; // 前N次采样只对齐, 不跑 PLL

	uint32_t raw;		 // 传感器原始值
	uint32_t count_in_cpr;	 // 方向修正后的单圈值
	uint32_t count_in_cpr_prev; // 上一拍单圈值

	int64_t shadow_count;	 // 过零累计计数

	float pos;		 // 机械位置, 圈
	float vel;		 // 机械速度, 圈/秒
	float phase;		 // 电角度 rad
	float phase_vel;	 // 电角速度 rad/s

	float pll_pos;		 // PLL 位置, 计数
	float pll_vel;		 // PLL 速度, 计数/秒
	float pll_kp;		 // PLL 比例增益
	float pll_ki;		 // PLL 积分增益
	float snap_threshold;	 // 静止阈值

	uint32_t error_cnt;	 // 传输启动失败次数
} tEncoder;

extern volatile tEncoder encoder;

void ENCODER_init(void);
void ENCODER_start_read(void);
void ENCODER_loop(void);
void ENCODER_reset(void);
void encoder_app_start(void);

#endif /* __ENCODER_H__ */
