#ifndef __FOC_CAL_H__
#define __FOC_CAL_H__

#include "config.h"
#include "foc.h"
#include "tim.h"
#include "encoder.h"
#include "foc_task.h"

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

    // 收尾 (rare 的 CS_REPORT_OFFSET_LUT): 停留一段时间供上位机读回偏移表, 然后结束
    CS_REPORT_OFFSET_LUT,

} tCalibStep;

void calibration_start(void);
void calibration_end(void);
void calibration_loop(void);

/* 齿槽转矩补偿标定: 速度环低速匀速拖动, 按电角度分桶采样 Iq_set 生成补偿表 */
void anticogging_start(void);
bool anticogging_loop(void);   /* 返回 true 表示标定结束 */
void anticogging_end(void);

#endif /* __FOC_CAL_H__ */