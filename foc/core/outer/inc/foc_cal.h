#ifndef __FOC_CAL_H__
#define __FOC_CAL_H__

#include "config.h"
#include "foc.h"
#include "tim.h"
#include "encoder.h"
#include "foc_task.h"

typedef enum eCalibStep {
	CS_NULL = 0,

	/* 电阻 */
	CS_MOTOR_R_START,
	CS_MOTOR_R_LOOP,
	CS_MOTOR_R_END,

	/* 电感 */
	CS_MOTOR_L_START,
	CS_MOTOR_L_LOOP,
	CS_MOTOR_L_END,

	/* 极对数 */
	CS_DIR_PP_START,
	CS_DIR_PP_LOOP,
	CS_DIR_PP_END,

	/* 磁链测量 (开环电流 + 两点法) */
	CS_FLUX_START,
	CS_FLUX_RAMP1,
	CS_FLUX_MEAS1,
	CS_FLUX_RTEST, /* 降流复测: ΔVq=Δ(R·Iq), 消掉反电势 */
	CS_FLUX_RAMP2,
	CS_FLUX_MEAS2,
	CS_FLUX_END,

	/* 编码器校准 */
	CS_ENCODER_START,
	CS_ENCODER_CW_LOOP,
	CS_ENCODER_CCW_LOOP,
	CS_ENCODER_END,

	/* 转动惯量 J: 交替给 ±Iq, 用加速度斜率差消掉摩擦 */
	/*   J·2π·a+ = Kt·I − Tf ;  J·2π·a− = −Kt·I − Tf */
	/*   => J = Kt·I / (π·(a+ − a−)) */
	CS_MOTOR_J_START,
	CS_MOTOR_J_SPINUP,
	CS_MOTOR_J_ACC,
	CS_MOTOR_J_DEC,
	CS_MOTOR_J_END,

	/* 收尾: 停留一会让上位机读回偏移表 */
	CS_REPORT_OFFSET_LUT,

} tCalibStep;

void calibration_start(void);
void calibration_end(void);
void calibration_loop(void);

/* 齿槽转矩补偿标定: 速度环低速匀速拖动, 按电角度分桶采样 Iq_set 生成补偿表 */
void anticogging_start(void);
bool anticogging_loop(void); /* 返回 true 表示标定结束 */
void anticogging_end(void);

#endif /* __FOC_CAL_H__ */