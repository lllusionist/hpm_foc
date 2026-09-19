#ifndef __FOC_CAL_H__
#define __FOC_CAL_H__

#include "config.h"
#include "foc.h"
#include "tim.h"
#include "encoder.h"
#include "foc_task.h"

typedef enum {
    CONTROL_MODE_CURRENT_RAMP     = 0, // 电流斜坡
    CONTROL_MODE_VELOCITY_RAMP    = 1, // 速度斜坡
    CONTROL_MODE_POSITION_FILTER  = 2, // 位置滤波控制
    CONTROL_MODE_POSITION_PROFILE = 3, // 轨迹规划
} tControlMode;

typedef enum eCalibStep {
    CS_NULL = 0,

    // 电阻
    CS_MOTOR_R_START,
    CS_MOTOR_R_LOOP,
    CS_MOTOR_R_END,

    // 电感
    CS_MOTOR_L_START,
    CS_MOTOR_L_LOOP,
    CS_MOTOR_L_END,

    // 极对数
    CS_DIR_PP_START,
    CS_DIR_PP_LOOP,
    CS_DIR_PP_END,

    // 磁链测量 (开环电流 + 两点法)
    CS_FLUX_START,
    CS_FLUX_RAMP1,
    CS_FLUX_MEAS1,
    CS_FLUX_RAMP2,
    CS_FLUX_MEAS2,
    CS_FLUX_END,

    // 编码器校准
    CS_ENCODER_START,
    CS_ENCODER_CW_LOOP,
    CS_ENCODER_CCW_LOOP,
    CS_ENCODER_END,

} tCalibStep;

void calibration_start(void);
void calibration_end(void);
void calibration_loop(void);

#endif /* __FOC_CAL_H__ */