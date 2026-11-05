#ifndef __FOC_MATLAB_H__
#define __FOC_MATLAB_H__

#include <stdbool.h>

/*
 * MATLAB/Simulink 生成的 FOC 模型移植层 (core/outer/matlab/FOC_Model.c)
 *
 * 注意: 文件名不能叫 foc_model.* —— Windows 文件名不区分大小写,
 * 会和生成代码的 FOC_Model.c/h 撞名, 导致 include 解析到错误的头文件。
 *
 * 生成代码自己包含: Clarke/Park、电流环、速度环、SVPWM、SMO 观测器、
 * I/F 强拖状态机 (Chart1: IDLE -> Align -> Open -> ThetaAlign -> Run)。
 * 本文件只做三件事:
 *   1. 用标定值覆盖生成代码里的电机参数 (Rs/L/Pn)
 *   2. 每 2 个 20kHz 中断调一次 FOC_Model_step() (模型 TS = 100us)
 *   3. 输入 ia/ib/ic/Vbus/Speed_ref/MotorOnOff, 输出 Tcmp -> pwm_set_duty()
 *
 * 模型单位约定 (与生成代码一致, 不要改):
 *   ia/ib/ic: A      Vbus: V      Speed_ref: 机械 rpm
 *   Tcmp1~3 : 计数, 满量程 8000 (= 原工程 ARR+1)
 */

void FOC_MATLAB_init(void);

/* 20kHz ADC 中断里调用; 内部 2 分频到模型的 10kHz */
void FOC_MATLAB_tick(void);

void FOC_MATLAB_set_enable(bool on);
void FOC_MATLAB_set_speed_rps(float rps);

/* 观测值 (VOFA / 调试用) */
float FOC_MATLAB_speed_rpm(void); /* 模型输出的机械转速 */
float FOC_MATLAB_speed_ref_rpm(void); /* 模型速度环目标 (rpm) */
float FOC_MATLAB_theta(void);	  /* 观测器电角度 (rad) */
float FOC_MATLAB_theta_error(void); /* 观测角 - 编码器角, 卷绕到 ±π (rad) */
float FOC_MATLAB_state(void);	  /* Chart1 当前状态 */

#endif /* __FOC_MATLAB_H__ */
