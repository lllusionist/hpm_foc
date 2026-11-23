#ifndef __FOC_CTRL_H__
#define __FOC_CTRL_H__

#include "foc.h"
#include <stdbool.h>
#include "util.h"

typedef enum {
	CONTROL_MODE_CURRENT_RAMP = 0,	   /* 电流斜坡 */
	CONTROL_MODE_VELOCITY_RAMP = 1,	   /* 速度斜坡 */
	CONTROL_MODE_POSITION_FILTER = 2,  /* 位置滤波控制 */
	CONTROL_MODE_POSITION_PROFILE = 3, /* S 形轨迹规划 */
} tControlMode;

typedef struct sController {
	int ctrl_mode; /* 当前控制模式 */

	volatile float input_position; /* 位置输入 (锁存值) */
	float input_velocity; /* 速度输入 */
	float input_current;  /* 电流输入 */

	float input_position_buffer; /* 位置输入缓冲 (命令写入) */
	float input_velocity_buffer;
	float input_current_buffer;

	float pos_setpoint; /* 位置设定点 */
	float vel_setpoint; /* 速度设定点 */
	float cur_setpoint; /* 电流设定点 */

	bool input_updated;	   /* 输入更新标志位 */
	bool target_reached;	   /* 到位标志 */
	float input_pos_filter_kp; /* 位置滤波器比例系数 */
	float input_pos_filter_ki; /* 位置滤波器积分系数 */
	float vel_integrator;	   /* 速度积分器状态 */

	/* 中断里只记请求, 由 1kHz 控制任务执行 (中断里不能调任务级 API) */
	volatile bool op_mode_pending;
	volatile tControlMode op_mode_req;
	volatile bool home_pending;
	volatile uint8_t config_cmd_pending; /* 1=保存 2=恢复默认 */
	volatile int32_t motor_profile_pending; /* >=0: 待套用的配置档 */
	volatile bool enable_refused_pending; /* 使能被拒 (未标定), 任务里打印 */
} tController;

extern tController Controller;

void CONTROLLER_init(void);
void CONTROLLER_reset(void);
/* 摩擦前馈在线开关 (0x2B) */
void CONTROLLER_set_friction_ff(uint8_t en);
uint8_t CONTROLLER_get_friction_ff(void);
void CONTROLLER_sync_callback(void);
int CONTROLLER_set_op_mode(tControlMode mode);
int CONTROLLER_set_home(void);
void CONTROLLER_update_input_pos_filter_gain(float bw);
void CONTROLLER_loop(void);

#endif /* __FOC_CTRL_H__ */
