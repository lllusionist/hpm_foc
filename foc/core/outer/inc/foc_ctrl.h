#ifndef __FOC_CTRL_H__
#define __FOC_CTRL_H__

#include "foc.h"
#include <stdbool.h>
#include "util.h"

typedef enum {
	CONTROL_MODE_CURRENT_RAMP = 0,	   /* 电流斜坡 */
	CONTROL_MODE_VELOCITY_RAMP = 1,	   /* 速度斜坡 */
	CONTROL_MODE_POSITION_FILTER = 2,  /* 位置滤波控制 */
	CONTROL_MODE_POSITION_PROFILE = 3, /* 轨迹规划 (暂按位置滤波处理) */
	/* I/F 强拖: 电角度按频率斜坡强制, 电流由电流环锁定;
	 * 到切换转速后交接到速度环。属启动过程模式,
	 * 不能作为上电默认模式 (见 CONTROLLER_init) */
	CONTROL_MODE_IF = 4,
} tControlMode;

/* I/F 强拖阶段 (任务侧状态机, 与中断共享的命令在 FOC.if_*) */
typedef enum {
	IF_STAGE_OFF = 0, /* 未启动 */
	IF_STAGE_DRAG,	  /* 强制角斜坡 + 电流闭环 */
	IF_STAGE_CUT,	  /* 角度已交给编码器, 偏置收敛中 */
} tIfStage;

typedef struct sController {
	int ctrl_mode; /* 当前控制模式 */

	float input_position; /* 位置输入 (锁存值) */
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

	/* I/F 强拖状态 (5kHz 任务私有; 与中断共享的命令在 FOC.if_*) */
	tIfStage if_stage;	/* OFF / DRAG / CUT */
	uint16_t if_tick;	/* 启动后 tick: 超时与打印分频共用 */
	uint16_t if_hold_cnt;	/* 交接前稳态保持计数 */
	uint16_t if_loss_cnt;	/* 失步去抖计数 */
	uint16_t if_blend_left; /* 角度交接剩余 tick */
	float if_drag_i;	/* 锁存的强拖电流 (A) */
	float if_omega_sw;	/* 锁存的交接电角速度 (rad/s) */
	float if_stall_rad;	/* 锁存的失步角 (rad) */
	float if_blend_step;	/* 角度交接每拍步长 (rad) */
} tController;

extern tController Controller;

void CONTROLLER_init(void);
void CONTROLLER_reset(void);
void CONTROLLER_sync_callback(void);
int CONTROLLER_set_op_mode(tControlMode mode);
int CONTROLLER_set_home(void);
void CONTROLLER_update_input_pos_filter_gain(float bw);
void CONTROLLER_loop(void);

#endif /* __FOC_CTRL_H__ */
