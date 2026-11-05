/*
 * File: FOC_Model.c
 *
 * Code generated for Simulink model 'FOC_Model'.
 *
 * Model version                  : 2.173
 * Simulink Coder version         : 9.6 (R2021b) 14-May-2021
 * C/C++ source code generated on : Thu Aug 24 20:12:38 2023
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "FOC_Model.h"

/* ============================================================
 * HPM 移植补丁: 4310 云台电机整定 (手改, 重新生成模型会丢失!)
 *
 * 生成代码原本按另一台电机配: 开环 iq = 1.0 A / 频率斜坡 10/3 rps/s /
 * OpenStage 3 s, 内部电压饱和 +-12.47 V = 0.9*24/sqrt(3) -> 原设计是 24 V 母线.
 * 我们 2026-11 起改用 24 V 电源, 交接转速可以回到 6 rps (原设计的一半),
 * 此时反电势 ~7 V, SMO 角度误差显著小于 12 V/3 rps 时.
 * 当前取值:
 *   - 对位 iq 0.4 A, 0.3 s
 *   - 开环 iq 0.35 A, 斜坡 5 rps/s, 1.2 s (终速 6.0 rps)
 *   - ThetaAlign 保持 6.0 rps, iq 收到 0.30 A
 *   - 速度环 q 电流限幅 +-1.0 A (额定 0.9 A 的 1.1 倍)
 * 重新生成模型时, 请把这些值直接填进 Simulink 的 Constant 模块.
 * ============================================================ */
#define HPM_IF_IQ_ALIGN 0.20F  /* 对位电流 (A): 从 0.4 降到 0.2, 对位"弹跳"减半 */
#define HPM_IF_IQ_OPEN 0.35F   /* 开环强拖电流 (A) */
#define HPM_IF_IQ_TAIL 0.00F   /* ThetaAlign 末尾电流: 最后 10% 卸到 0, 交接零转矩冲击 */
#define HPM_IF_IQ_HOLD 0.04F   /* ThetaAlign 中段维持电流 = 实测 3.5rps 摩擦电流 */
#define HPM_IF_SPD_PRELOAD 0.04F /* 速度环积分器预置值 (= 上面的维持电流, 无扰切换) */
#define HPM_IF_RPS_S 2.0F      /* 开环频率斜坡 (机械 rps/s), 缺省值; 运行时由
                                * HPM_if_ramp_rps_s 覆盖 */
#define HPM_IF_ALIGN_CNT 4000U /* 对位段 0.4 s @10 kHz */
#define HPM_IF_OPEN_CNT 17500U /* 开环段 1.75 s -> 3.5 rps, 缺省值 */
#define HPM_IF_THETA_CNT 5000U /* ThetaAlign 段 0.5 s */
#define HPM_IF_HOLD_RPS 3.5F   /* ThetaAlign 段保持转速 (rps) = 开环终速, 缺省值;
                                * 运行时由 HPM_if_hold_rps 覆盖 */
#define HPM_IQ_LIMIT 1.0F      /* 速度环 q 电流限幅 (A) */

/* Named constants for Chart: '<S2>/Chart1' */
#define IN_AlignStage                  ((uint8_T)1U)
#define IN_IDLE                        ((uint8_T)2U)
#define IN_OpenStage                   ((uint8_T)3U)
#define IN_RunStage                    ((uint8_T)4U)
#define IN_ThetaAlign                  ((uint8_T)5U)
#ifndef UCHAR_MAX
#include <limits.h>
#endif

#if ( UCHAR_MAX != (0xFFU) ) || ( SCHAR_MAX != (0x7F) )
#error Code was generated for compiler with different sized uchar/char. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

#if ( USHRT_MAX != (0xFFFFU) ) || ( SHRT_MAX != (0x7FFF) )
#error Code was generated for compiler with different sized ushort/short. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

#if ( UINT_MAX != (0xFFFFFFFFU) ) || ( INT_MAX != (0x7FFFFFFF) )
#error Code was generated for compiler with different sized uint/int. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

#if ( ULONG_MAX != (0xFFFFFFFFU) ) || ( LONG_MAX != (0x7FFFFFFF) )
#error Code was generated for compiler with different sized ulong/long. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

/* Skipping ulong_long/long_long check: insufficient preprocessor integer range. */

/* Exported block signals */
real32_T SMOWm;                        /* '<S13>/Gain1' */
real32_T SMOTheta;                     /* '<S24>/Unit Delay' */
real32_T ialpha;                       /* '<S2>/Clark' */
real32_T ibeta;                        /* '<S2>/Clark' */
real32_T ualpha;                       /* '<S2>/In_park' */
real32_T ubeta;                        /* '<S2>/In_park' */
real32_T state;                        /* '<S2>/Chart1' */

/* Exported data definition */

/* Definition for custom storage class: Struct */
curr_kpki_type curr_kpki = {
  /* curr_d_ki */
  35.0F,

  /* curr_d_kp */
  0.017F
};

motor_type motor = {
  /* L */
  0.003F,

  /* Pn */
  14.0F,

  /* Rs */
  6.8F
};

spd_kpki_type spd_kpki = {
  /* spd_ki */
  0.000144F,

  /* spd_kp */
  0.003389F
};

/* Block signals and states (default storage) */
DW rtDW;

/* External inputs (root inport signals with default storage) */
ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;

/* [HPM 补丁] I/F 参数运行时可设: 由移植层按上位机目标转速算出来, 这样
 * 开环终速永远等于目标转速 -> 交接零误差 (目标低于开环终速时, 速度环
 * 要从 3.5 刹到 0.8, 低转速 SMO 直接失锁, 实测跑到反向 -10 rps) */
real32_T HPM_if_ramp_rps_s = HPM_IF_RPS_S;
real32_T HPM_if_hold_rps = HPM_IF_HOLD_RPS;
uint16_T HPM_if_open_cnt = HPM_IF_OPEN_CNT;
/* I/F 三段电流同样由移植层按额定电流/实测摩擦电流算出来 */
real32_T HPM_if_iq_align = HPM_IF_IQ_ALIGN;
real32_T HPM_if_iq_open = HPM_IF_IQ_OPEN;
real32_T HPM_if_iq_hold = HPM_IF_IQ_HOLD;
real32_T HPM_if_iq_preload = HPM_IF_SPD_PRELOAD;

/* Real-time model */
static RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;
/* [HPM 补丁] 进 RunStage 后的强制摩擦电流拍数 (见 case IN_RunStage) */
static uint16_T hpm_run_entry_cnt;
extern real32_T rt_modf(real32_T u0, real32_T u1);
static void Clark(real32_T rtu_ia, real32_T rtu_ib, real32_T rtu_ic, real32_T
                  *rty_ialpha, real32_T *rty_ibeta);
static void In_park(real32_T rtu_ud, real32_T rtu_uq, real32_T rtu_theta_sin,
                    real32_T rtu_theta_cos, real32_T *rty_ualpha, real32_T
                    *rty_ubeta);
static void Park(real32_T rtu_ialpha, real32_T rtu_ibeta, real32_T rtu_theta_sin,
                 real32_T rtu_theta_cos, real32_T *rty_id, real32_T *rty_iq);
static void SVPWM1(real32_T rtu_Valpha, real32_T rtu_Vbeta, real32_T rtu_v_bus,
                   real32_T rty_tABC[3]);
static void Curr_loop_Init(real_T *rty_RestsSingal, DW_Curr_loop *localDW);
static void Curr_loop_Enable(DW_Curr_loop *localDW);
static void Curr_loop(RT_MODEL * const rtM, real32_T rtu_ia, real32_T rtu_ib,
                      real32_T rtu_ic, real32_T rtu_MotorOnOff, real32_T
                      rtu_iq_ref, real32_T rtu_VDC, real32_T rty_Tcmp1[3],
                      real32_T *rty_Tcmp2, real32_T *rty_Tcmp3, real32_T
                      *rty_SMOWm, real_T *rty_RestsSingal, DW_Curr_loop *localDW);
static void rate_scheduler(void);

/*
 *         This function updates active task flag for each subrate.
 *         The function is called at model base rate, hence the
 *         generated code self-manages all its subrates.
 */
static void rate_scheduler(void)
{
  /* Compute which subrates run during the next base time step.  Subrates
   * are an integer multiple of the base rate counter.  Therefore, the subtask
   * counter is reset when it reaches its limit (zero means run).
   */
  (rtM->Timing.TaskCounters.TID[1])++;
  if ((rtM->Timing.TaskCounters.TID[1]) > 9) {/* Sample time: [0.001s, 0.0s] */
    rtM->Timing.TaskCounters.TID[1] = 0;
  }
}

/* Output and update for atomic system: '<S2>/Clark' */
static void Clark(real32_T rtu_ia, real32_T rtu_ib, real32_T rtu_ic, real32_T
                  *rty_ialpha, real32_T *rty_ibeta)
{
  /* Sum: '<S5>/Add1' incorporates:
   *  Gain: '<S5>/Gain'
   *  Gain: '<S5>/Gain1'
   *  Sum: '<S5>/Add'
   */
  *rty_ialpha = 0.666666687F * rtu_ia - (rtu_ib + rtu_ic) * 0.333333343F;

  /* Gain: '<S5>/Gain2' incorporates:
   *  Sum: '<S5>/Add2'
   */
  *rty_ibeta = (rtu_ib - rtu_ic) * 0.577350259F;
}

/* Output and update for atomic system: '<S2>/In_park' */
static void In_park(real32_T rtu_ud, real32_T rtu_uq, real32_T rtu_theta_sin,
                    real32_T rtu_theta_cos, real32_T *rty_ualpha, real32_T
                    *rty_ubeta)
{
  /* Sum: '<S11>/Add' incorporates:
   *  Product: '<S11>/Product'
   *  Product: '<S11>/Product1'
   */
  *rty_ualpha = rtu_ud * rtu_theta_cos - rtu_uq * rtu_theta_sin;

  /* Sum: '<S11>/Add1' incorporates:
   *  Product: '<S11>/Product2'
   *  Product: '<S11>/Product3'
   */
  *rty_ubeta = rtu_ud * rtu_theta_sin + rtu_uq * rtu_theta_cos;
}

/* Output and update for atomic system: '<S2>/Park' */
static void Park(real32_T rtu_ialpha, real32_T rtu_ibeta, real32_T rtu_theta_sin,
                 real32_T rtu_theta_cos, real32_T *rty_id, real32_T *rty_iq)
{
  /* Sum: '<S12>/Add' incorporates:
   *  Product: '<S12>/Product'
   *  Product: '<S12>/Product1'
   */
  *rty_id = rtu_ialpha * rtu_theta_cos + rtu_ibeta * rtu_theta_sin;

  /* Sum: '<S12>/Add1' incorporates:
   *  Product: '<S12>/Product2'
   *  Product: '<S12>/Product3'
   */
  *rty_iq = rtu_ibeta * rtu_theta_cos - rtu_ialpha * rtu_theta_sin;
}

/* Output and update for atomic system: '<S2>/SVPWM1' */
static void SVPWM1(real32_T rtu_Valpha, real32_T rtu_Vbeta, real32_T rtu_v_bus,
                   real32_T rty_tABC[3])
{
  real32_T rtb_Min;
  real32_T rtb_Sum1_j;
  real32_T rtb_Sum_b;

  /* Gain: '<S79>/Gain' */
  rtb_Min = -0.5F * rtu_Valpha;

  /* Gain: '<S79>/Gain1' */
  rtb_Sum1_j = 0.866025388F * rtu_Vbeta;

  /* Sum: '<S79>/Sum' */
  rtb_Sum_b = rtb_Min + rtb_Sum1_j;

  /* Sum: '<S79>/Sum1' */
  rtb_Sum1_j = rtb_Min - rtb_Sum1_j;

  /* Gain: '<S80>/Gain' incorporates:
   *  MinMax: '<S80>/Min'
   *  MinMax: '<S80>/Min1'
   *  Sum: '<S80>/Sum'
   */
  rtb_Min = (fminf(fminf(rtu_Valpha, rtb_Sum_b), rtb_Sum1_j) + fmaxf(fmaxf
              (rtu_Valpha, rtb_Sum_b), rtb_Sum1_j)) * -0.5F;

  /* Gain: '<S14>/PWM_HalfPeriod' incorporates:
   *  Constant: '<S14>/Constant'
   *  Gain: '<S14>/Gain'
   *  Product: '<S14>/Divide'
   *  Sum: '<S14>/Sum'
   *  Sum: '<S14>/Sum1'
   */
  rty_tABC[0] = (-(rtb_Min + rtu_Valpha) / rtu_v_bus + 0.5F) * 8000.0F;
  rty_tABC[1] = (-(rtb_Min + rtb_Sum_b) / rtu_v_bus + 0.5F) * 8000.0F;
  rty_tABC[2] = (-(rtb_Min + rtb_Sum1_j) / rtu_v_bus + 0.5F) * 8000.0F;
}

real32_T rt_modf(real32_T u0, real32_T u1)
{
  real32_T y;
  y = u0;
  if (u1 == 0.0F) {
    if (u0 == 0.0F) {
      y = u1;
    }
  } else if (u0 == 0.0F) {
    y = 0.0F / u1;
  } else {
    boolean_T yEq;
    y = fmodf(u0, u1);
    yEq = (y == 0.0F);
    if ((!yEq) && (u1 > floorf(u1))) {
      real32_T q;
      q = fabsf(u0 / u1);
      yEq = (fabsf(q - floorf(q + 0.5F)) <= FLT_EPSILON * q);
    }

    if (yEq) {
      y = 0.0F;
    } else if ((u0 < 0.0F) != (u1 < 0.0F)) {
      y += u1;
    }
  }

  return y;
}

/* System initialize for function-call system: '<S1>/Curr_loop' */
static void Curr_loop_Init(real_T *rty_RestsSingal, DW_Curr_loop *localDW)
{
  /* Start for SwitchCase: '<S2>/Switch Case1' */
  localDW->SwitchCase1_ActiveSubsystem = -1;

  /* SystemInitialize for Chart: '<S2>/Chart1' */
  *rty_RestsSingal = 0.0;

  /* SystemInitialize for IfAction SubSystem: '<S2>/If Action Subsystem2' */
  /* InitializeConditions for DiscreteIntegrator: '<S8>/Discrete-Time Integrator' */
  localDW->DiscreteTimeIntegrator_PrevRese = 2;

  /* InitializeConditions for DiscreteIntegrator: '<S8>/Discrete-Time Integrator1' */
  localDW->DiscreteTimeIntegrator1_PrevRes = 2;

  /* End of SystemInitialize for SubSystem: '<S2>/If Action Subsystem2' */
}

/* Enable for function-call system: '<S1>/Curr_loop' */
static void Curr_loop_Enable(DW_Curr_loop *localDW)
{
  localDW->Curr_loop_RESET_ELAPS_T = true;

  /* Enable for DiscreteIntegrator: '<S116>/Integrator' */
  localDW->Integrator_SYSTEM_ENABLE = 1U;

  /* Enable for DiscreteIntegrator: '<S166>/Integrator' */
  localDW->Integrator_SYSTEM_ENABLE_e = 1U;

  /* Enable for DiscreteIntegrator: '<S58>/Integrator' */
  localDW->Integrator_SYSTEM_ENABLE_l = 1U;
}

/* Output and update for function-call system: '<S1>/Curr_loop' */
static void Curr_loop(RT_MODEL * const rtM, real32_T rtu_ia, real32_T rtu_ib,
                      real32_T rtu_ic, real32_T rtu_MotorOnOff, real32_T
                      rtu_iq_ref, real32_T rtu_VDC, real32_T rty_Tcmp1[3],
                      real32_T *rty_Tcmp2, real32_T *rty_Tcmp3, real32_T
                      *rty_SMOWm, real_T *rty_RestsSingal, DW_Curr_loop *localDW)
{
  real32_T DiscreteTimeIntegrator;
  real32_T DiscreteTimeIntegrator1_n;
  real32_T DiscreteTimeIntegrator1_n_0;
  real32_T Integrator;
  real32_T Integrator_j;
  real32_T rtb_Add1;
  real32_T rtb_Cos;
  real32_T rtb_IProdOut;
  real32_T rtb_IntegralGain_b;
  real32_T rtb_Sign;
  real32_T rtb_Sign1;
  real32_T rtb_SignPreIntegrator_bb;
  real32_T rtb_SignPreSat_g;
  real32_T rtb_Sin;
  real32_T rtb_Sum1_j;
  uint32_T Curr_loop_ELAPS_T;
  uint32_T Curr_loop_PREV_T_tmp;
  int16_T Integrator_g;
  int16_T rtb_IProdOut_p;
  int8_T rtAction;
  int8_T rtPrevAction;
  if (localDW->Curr_loop_RESET_ELAPS_T) {
    Curr_loop_ELAPS_T = 0U;
  } else {
    Curr_loop_ELAPS_T = rtM->Timing.clockTick0 - localDW->Curr_loop_PREV_T;
  }

  /* Update for IfAction SubSystem: '<S2>/If Action Subsystem4' incorporates:
   *  ActionPort: '<S10>/Action Port'
   */
  /* Update for IfAction SubSystem: '<S2>/If Action Subsystem2' incorporates:
   *  ActionPort: '<S8>/Action Port'
   */
  /* Outputs for IfAction SubSystem: '<S2>/If Action Subsystem4' incorporates:
   *  ActionPort: '<S10>/Action Port'
   */
  /* Outputs for IfAction SubSystem: '<S2>/If Action Subsystem2' incorporates:
   *  ActionPort: '<S8>/Action Port'
   */
  /* SwitchCase: '<S2>/Switch Case1' */
  Curr_loop_PREV_T_tmp = rtM->Timing.clockTick0;

  /* End of Outputs for SubSystem: '<S2>/If Action Subsystem2' */
  /* End of Outputs for SubSystem: '<S2>/If Action Subsystem4' */
  /* End of Update for SubSystem: '<S2>/If Action Subsystem2' */
  /* End of Update for SubSystem: '<S2>/If Action Subsystem4' */
  localDW->Curr_loop_PREV_T = Curr_loop_PREV_T_tmp;
  localDW->Curr_loop_RESET_ELAPS_T = false;

  /* Chart: '<S2>/Chart1' */
  if (localDW->temporalCounter_i1 < 32767U) {
    localDW->temporalCounter_i1++;
  }

  if (localDW->is_active_c7_FOC_Model == 0U) {
    localDW->is_active_c7_FOC_Model = 1U;
    localDW->is_c7_FOC_Model = IN_IDLE;
  } else {
    switch (localDW->is_c7_FOC_Model) {
     case IN_AlignStage:
      if (localDW->temporalCounter_i1 >= HPM_IF_ALIGN_CNT) {
        localDW->is_c7_FOC_Model = IN_OpenStage;
        localDW->temporalCounter_i1 = 0U;
        localDW->ZReset = 0.0F;
        localDW->cnt = 0.0F;
      } else if (rtu_MotorOnOff == 0.0F) {
        localDW->is_c7_FOC_Model = IN_IDLE;
      } else {
        state = 2.0F;
        *rty_RestsSingal = 0.0;
      }
      break;

     case IN_IDLE:
      if (rtu_MotorOnOff == 1.0F) {
        localDW->is_c7_FOC_Model = IN_AlignStage;
        localDW->temporalCounter_i1 = 0U;
      } else {
        state = 1.0F;
        *rty_RestsSingal = 0.0;
      }
      break;

     case IN_OpenStage:
      if (rtu_MotorOnOff == 0.0F) {
        localDW->is_c7_FOC_Model = IN_IDLE;
      } else if (localDW->temporalCounter_i1 >= HPM_if_open_cnt) {
        localDW->is_c7_FOC_Model = IN_ThetaAlign;
        localDW->temporalCounter_i1 = 0U;
      } else {
        if (localDW->cnt == 1.0F) {
          localDW->ZReset = 1.0F;
        }

        localDW->cnt = 1.0F;
        state = 3.0F;
        *rty_RestsSingal = 0.0;
      }
      break;

     case IN_RunStage:
      if (rtu_MotorOnOff == 0.0F) {
        localDW->is_c7_FOC_Model = IN_IDLE;
      } else {
        /* [HPM 补丁] 进 RunStage 的前 3 ms 用摩擦电流充当电流指令:
         * 速度环在开环段一直在算, 积分饱和到 +1 A, 其输出经 Rate Transition
         * 滞后送进来, 切换瞬间会命令 1 A -> 电流尖峰把转子弹飞.
         * 注意: 本 case 每个 tick 都执行, 只有"刚进入"(state 还不是 5)时才设置,
         * 否则计数器被反复置 30, 速度环会被永久旁路 (实测跑飞 620 rpm). */
        if (state != 5.0F)
          hpm_run_entry_cnt = 30U;
        state = 5.0F;
        *rty_RestsSingal = 1.0;
      }
      break;

     default:
      /* case IN_ThetaAlign: */
      if (localDW->temporalCounter_i1 >= HPM_IF_THETA_CNT) {
        localDW->is_c7_FOC_Model = IN_RunStage;
      } else {
        state = 4.0F;
        *rty_RestsSingal = 0.0;
      }
      break;
    }
  }

  /* End of Chart: '<S2>/Chart1' */

  /* Outputs for Atomic SubSystem: '<S2>/Clark' */
  /* SignalConversion generated from: '<S2>/Clark' */
  Clark(rtu_ia, rtu_ib, rtu_ic, &ialpha, &ibeta);

  /* End of Outputs for SubSystem: '<S2>/Clark' */

  /* UnitDelay: '<S24>/Unit Delay' */
  SMOTheta = localDW->UnitDelay_DSTATE;

  /* SwitchCase: '<S2>/Switch Case1' */
  rtPrevAction = localDW->SwitchCase1_ActiveSubsystem;
  rtAction = -1;
  switch ((int32_T)state) {
   case 1:
    rtAction = 0;
    break;

   case 2:
    rtAction = 1;
    break;

   case 3:
    rtAction = 2;
    break;

   case 4:
    rtAction = 3;
    break;

   case 5:
    rtAction = 4;
    break;
  }

  localDW->SwitchCase1_ActiveSubsystem = rtAction;
  switch (rtAction) {
   case 0:
    /* Outputs for IfAction SubSystem: '<S2>/If Action Subsystem' incorporates:
     *  ActionPort: '<S6>/Action Port'
     */
    /* Merge: '<S2>/Merge' incorporates:
     *  Constant: '<S6>/Constant'
     *  SignalConversion generated from: '<S6>/theta_fd'
     */
    localDW->Merge = 0.0F;

    /* Merge: '<S2>/Merge1' incorporates:
     *  Constant: '<S6>/Constant1'
     *  SignalConversion generated from: '<S6>/iq_ref'
     */
    localDW->Merge1 = 0.0F;

    /* End of Outputs for SubSystem: '<S2>/If Action Subsystem' */
    break;

   case 1:
    /* Outputs for IfAction SubSystem: '<S2>/If Action Subsystem1' incorporates:
     *  ActionPort: '<S7>/Action Port'
     */
    /* Merge: '<S2>/Merge' incorporates:
     *  Constant: '<S7>/Constant'
     *  SignalConversion generated from: '<S7>/theta_fd'
     */
    localDW->Merge = 0.0F;

    /* Merge: '<S2>/Merge1' incorporates:
     *  Constant: '<S7>/Constant1'
     *  SignalConversion generated from: '<S7>/iq_ref'
     */
    /* [HPM 补丁] 对位电流按时间线性爬升: 原来第一拍就是 0.4 A, 配合
     * 150 Hz 电流环变成转矩冲击, 实测转子被反冲到 -3.4 rps (起转"咯噔") */
    localDW->Merge1 = HPM_if_iq_align *
      (float)localDW->temporalCounter_i1 / (float)HPM_IF_ALIGN_CNT;

    /* End of Outputs for SubSystem: '<S2>/If Action Subsystem1' */
    break;

   case 2:
    {
      uint32_T IfActionSubsystem2_ELAPS_T;
      if (rtAction != rtPrevAction) {
        localDW->IfActionSubsystem2_RESET_ELAPS_ = true;

        /* Enable for IfAction SubSystem: '<S2>/If Action Subsystem2' incorporates:
         *  ActionPort: '<S8>/Action Port'
         */
        /* Enable for SwitchCase: '<S2>/Switch Case1' incorporates:
         *  DiscreteIntegrator: '<S8>/Discrete-Time Integrator'
         *  DiscreteIntegrator: '<S8>/Discrete-Time Integrator1'
         */
        localDW->DiscreteTimeIntegrator_SYSTEM_E = 1U;
        localDW->DiscreteTimeIntegrator1_SYSTE_f = 1U;

        /* End of Enable for SubSystem: '<S2>/If Action Subsystem2' */
      }

      /* Outputs for IfAction SubSystem: '<S2>/If Action Subsystem2' incorporates:
       *  ActionPort: '<S8>/Action Port'
       */
      if (localDW->IfActionSubsystem2_RESET_ELAPS_) {
        IfActionSubsystem2_ELAPS_T = 0U;
      } else {
        IfActionSubsystem2_ELAPS_T = Curr_loop_PREV_T_tmp -
          localDW->IfActionSubsystem2_PREV_T;
      }

      localDW->IfActionSubsystem2_PREV_T = Curr_loop_PREV_T_tmp;
      localDW->IfActionSubsystem2_RESET_ELAPS_ = false;

      /* DiscreteIntegrator: '<S8>/Discrete-Time Integrator' */
      if (localDW->DiscreteTimeIntegrator_SYSTEM_E != 0) {
        /* DiscreteIntegrator: '<S8>/Discrete-Time Integrator' */
        DiscreteTimeIntegrator = localDW->DiscreteTimeIntegrator_DSTATE;
      } else if ((localDW->ZReset > 0.0F) &&
                 (localDW->DiscreteTimeIntegrator_PrevRese <= 0)) {
        localDW->DiscreteTimeIntegrator_DSTATE = 0.0F;

        /* DiscreteIntegrator: '<S8>/Discrete-Time Integrator' */
        DiscreteTimeIntegrator = localDW->DiscreteTimeIntegrator_DSTATE;
      } else {
        /* DiscreteIntegrator: '<S8>/Discrete-Time Integrator' */
        DiscreteTimeIntegrator = 0.0001F * (real32_T)IfActionSubsystem2_ELAPS_T *
          localDW->DiscreteTimeIntegrator_PREV_U +
          localDW->DiscreteTimeIntegrator_DSTATE;
      }

      /* End of DiscreteIntegrator: '<S8>/Discrete-Time Integrator' */

      /* DiscreteIntegrator: '<S8>/Discrete-Time Integrator1' */
      if (localDW->DiscreteTimeIntegrator1_SYSTE_f != 0) {
        /* DiscreteIntegrator: '<S8>/Discrete-Time Integrator1' */
        DiscreteTimeIntegrator1_n = localDW->DiscreteTimeIntegrator1_DSTAT_j;
      } else if ((localDW->ZReset > 0.0F) &&
                 (localDW->DiscreteTimeIntegrator1_PrevRes <= 0)) {
        localDW->DiscreteTimeIntegrator1_DSTAT_j = 0.0F;

        /* DiscreteIntegrator: '<S8>/Discrete-Time Integrator1' */
        DiscreteTimeIntegrator1_n = localDW->DiscreteTimeIntegrator1_DSTAT_j;
      } else {
        /* DiscreteIntegrator: '<S8>/Discrete-Time Integrator1' */
        DiscreteTimeIntegrator1_n = 0.0001F * (real32_T)
          IfActionSubsystem2_ELAPS_T * localDW->DiscreteTimeIntegrator1_PREV__h
          + localDW->DiscreteTimeIntegrator1_DSTAT_j;
      }

      /* End of DiscreteIntegrator: '<S8>/Discrete-Time Integrator1' */

      /* Merge: '<S2>/Merge' incorporates:
       *  Constant: '<S8>/Constant'
       *  Math: '<S8>/Mod'
       */
      localDW->Merge = rt_modf(DiscreteTimeIntegrator1_n, 6.28318548F);

      /* Merge: '<S2>/Merge1' incorporates:
       *  Constant: '<S8>/Constant2'
       *  SignalConversion generated from: '<S8>/iq_ref'
       */
      localDW->Merge1 = HPM_if_iq_open;

      /* Update for DiscreteIntegrator: '<S8>/Discrete-Time Integrator' incorporates:
       *  Gain: '<S8>/Gain'
       *  Product: '<S8>/Product'
       */
      localDW->DiscreteTimeIntegrator_SYSTEM_E = 0U;
      localDW->DiscreteTimeIntegrator_DSTATE = DiscreteTimeIntegrator;
      if (localDW->ZReset > 0.0F) {
        localDW->DiscreteTimeIntegrator_PrevRese = 1;
      } else if (localDW->ZReset < 0.0F) {
        localDW->DiscreteTimeIntegrator_PrevRese = -1;
      } else if (localDW->ZReset == 0.0F) {
        localDW->DiscreteTimeIntegrator_PrevRese = 0;
      } else {
        localDW->DiscreteTimeIntegrator_PrevRese = 2;
      }

      localDW->DiscreteTimeIntegrator_PREV_U = motor.Pn * 6.28318548F *
        HPM_if_ramp_rps_s;

      /* End of Update for DiscreteIntegrator: '<S8>/Discrete-Time Integrator' */

      /* Update for DiscreteIntegrator: '<S8>/Discrete-Time Integrator1' */
      localDW->DiscreteTimeIntegrator1_SYSTE_f = 0U;
      localDW->DiscreteTimeIntegrator1_DSTAT_j = DiscreteTimeIntegrator1_n;
      if (localDW->ZReset > 0.0F) {
        localDW->DiscreteTimeIntegrator1_PrevRes = 1;
      } else if (localDW->ZReset < 0.0F) {
        localDW->DiscreteTimeIntegrator1_PrevRes = -1;
      } else if (localDW->ZReset == 0.0F) {
        localDW->DiscreteTimeIntegrator1_PrevRes = 0;
      } else {
        localDW->DiscreteTimeIntegrator1_PrevRes = 2;
      }

      localDW->DiscreteTimeIntegrator1_PREV__h = DiscreteTimeIntegrator;

      /* End of Update for DiscreteIntegrator: '<S8>/Discrete-Time Integrator1' */
      /* End of Outputs for SubSystem: '<S2>/If Action Subsystem2' */
    }
    break;

   case 3:
    {
      uint32_T IfActionSubsystem2_ELAPS_T;
      if (rtAction != rtPrevAction) {
        localDW->IfActionSubsystem4_RESET_ELAPS_ = true;

        /* Enable for IfAction SubSystem: '<S2>/If Action Subsystem4' incorporates:
         *  ActionPort: '<S10>/Action Port'
         */
        /* Enable for SwitchCase: '<S2>/Switch Case1' incorporates:
         *  DiscreteIntegrator: '<S10>/Discrete-Time Integrator1'
         */
        localDW->DiscreteTimeIntegrator1_SYSTEM_ = 1U;

        /* End of Enable for SubSystem: '<S2>/If Action Subsystem4' */
      }

      /* Outputs for IfAction SubSystem: '<S2>/If Action Subsystem4' incorporates:
       *  ActionPort: '<S10>/Action Port'
       */
      if (localDW->IfActionSubsystem4_RESET_ELAPS_) {
        IfActionSubsystem2_ELAPS_T = 0U;
      } else {
        IfActionSubsystem2_ELAPS_T = Curr_loop_PREV_T_tmp -
          localDW->IfActionSubsystem4_PREV_T;
      }

      localDW->IfActionSubsystem4_PREV_T = Curr_loop_PREV_T_tmp;
      localDW->IfActionSubsystem4_RESET_ELAPS_ = false;

      /* [HPM 补丁] 原生成代码在 ThetaAlign 段用的是另一组积分器状态
       * (_DSTATE/_PREV_U), 与 OpenStage 段(_DSTAT_j/_PREV__h)不连续:
       * 3->4 切换瞬间驱动角直接跳到 0, 实测 Vq 一步跳变 13.5 V, 电机被
       * 拉停, 随后观测器失锁空转. 这里改成沿用 OpenStage 的角度状态,
       * 保证角度连续 (速度仍按 HPM_IF_HOLD_RPS 恒定) */
      /* DiscreteIntegrator: '<S10>/Discrete-Time Integrator1' */
      if (localDW->DiscreteTimeIntegrator1_SYSTEM_ != 0) {
        /* DiscreteIntegrator: '<S10>/Discrete-Time Integrator1' */
        DiscreteTimeIntegrator = localDW->DiscreteTimeIntegrator1_DSTAT_j;
      } else {
        /* DiscreteIntegrator: '<S10>/Discrete-Time Integrator1' */
        DiscreteTimeIntegrator = 0.0001F * (real32_T)IfActionSubsystem2_ELAPS_T *
          localDW->DiscreteTimeIntegrator1_PREV__h +
          localDW->DiscreteTimeIntegrator1_DSTAT_j;
      }

      /* End of DiscreteIntegrator: '<S10>/Discrete-Time Integrator1' */

      /* Merge: '<S2>/Merge' incorporates:
       *  Constant: '<S10>/Constant'
       *  Math: '<S10>/Mod'
       *  SignalConversion generated from: '<S10>/Theta_fd'
       */
      /* [HPM 补丁] 这段原代码直接输出强制角, 4->5 再一步切到 SMOTheta.
       * 2026-11-05 试错结论:
       *   - 在 ThetaAlign 里混合角度: 电流环要追一个正在转的坐标系,
       *     实测把 ThetaAlign 段电流搞出 22 次/s 的尖峰, 且 Vd 乱跳 -> 已回退;
       *   - 只切角度不降电流: 切换瞬间转矩冲击与电流成正比 (Vq 一帧跳 11.5 V).
       * 最终做法: 本段继续用强制角(稳定), 电流分三段收到 0, 让 4->5 的
       * 冲击正比于很小的残余电流; 切换后另外钳住 3 ms 的 iq 指令(见 case 4). */
      {
        float k = (float)localDW->temporalCounter_i1 /
                  (float)HPM_IF_THETA_CNT;
        float iq;

        if (k > 1.0F)
          k = 1.0F;
        /* 电流: 前 30% 由开环值收到摩擦电流, 之后一直保持.
         * 末尾卸到 0 会让转速在交接前掉到 2.5 rps, 速度环再补过头 -> 起振.
         * 交接冲击的真正来源(速度环积分饱和后滞后的 1 A)已在 case 4 钳掉 */
        if (k < 0.3F) {
          iq = HPM_if_iq_open +
               (k / 0.3F) * (HPM_if_iq_hold - HPM_if_iq_open);
        } else {
          iq = HPM_if_iq_hold;
        }
        localDW->Merge1 = iq;
        localDW->Merge = rt_modf(DiscreteTimeIntegrator, 6.28318548F);
      }

      /* If: '<S10>/If' incorporates:
       *  Constant: '<S10>/Constant1'
       *  Constant: '<S16>/Constant'
       *  SignalConversion generated from: '<S16>/Out1'
       *  Sum: '<S10>/Add'
       *  UnitDelay: '<S10>/Unit Delay'
       */
      if (localDW->UnitDelay_DSTATE_b + 0.01F >= 1.0F) {
        /* Outputs for IfAction SubSystem: '<S10>/If Action Subsystem' incorporates:
         *  ActionPort: '<S16>/Action Port'
         */
        rtb_Sign = 1.0F;

        /* End of Outputs for SubSystem: '<S10>/If Action Subsystem' */
      } else {
        rtb_Sign = localDW->UnitDelay_DSTATE_b + 0.01F;
      }

      /* End of If: '<S10>/If' */

      /* Merge: '<S2>/Merge1' incorporates:
       *  Constant: '<S10>/Constant2'
       *  Constant: '<S10>/Constant4'
       *  Product: '<S10>/Product1'
       *  Sum: '<S10>/Add4'
       */
      /* [HPM 补丁] 电流参考已在上面的角度混合块里按 k 从 IQ_OPEN 收到 IQ_TAIL,
       * 这里不再覆盖 (原公式 1.0-sign*0.7 首拍 ~0.99A 会电流冲击) */

      /* Update for DiscreteIntegrator: '<S10>/Discrete-Time Integrator1' incorporates:
       *  Gain: '<S10>/Gain'
       */
      localDW->DiscreteTimeIntegrator1_SYSTEM_ = 0U;
      localDW->DiscreteTimeIntegrator1_DSTAT_j = DiscreteTimeIntegrator;
      localDW->DiscreteTimeIntegrator1_PREV__h = motor.Pn * 6.28318548F *
        HPM_if_hold_rps;

      /* Update for UnitDelay: '<S10>/Unit Delay' incorporates:
       *  Constant: '<S10>/Constant1'
       *  Sum: '<S10>/Add'
       */
      localDW->UnitDelay_DSTATE_b += 0.01F;

      /* End of Outputs for SubSystem: '<S2>/If Action Subsystem4' */
    }
    break;

   case 4:
    /* Outputs for IfAction SubSystem: '<S2>/If Action Subsystem3' incorporates:
     *  ActionPort: '<S9>/Action Port'
     */
    /* Merge: '<S2>/Merge' incorporates:
     *  Inport: '<S9>/theta_Close'
     */
    localDW->Merge = SMOTheta;

    /* Merge: '<S2>/Merge1' incorporates:
     *  Inport: '<S9>/iq_CloseRef'
     */
    /* [HPM 补丁] 进 RunStage 前 3 ms 用摩擦电流, 避开速度环滞后的 1 A 指令 */
    if (hpm_run_entry_cnt > 0U) {
      hpm_run_entry_cnt--;
      localDW->Merge1 = HPM_if_iq_preload;
    } else {
      localDW->Merge1 = rtu_iq_ref;
    }

    /* End of Outputs for SubSystem: '<S2>/If Action Subsystem3' */
    break;
  }

  /* Trigonometry: '<S2>/Cos' */
  rtb_Cos = cosf(localDW->Merge);

  /* Trigonometry: '<S2>/Sin' */
  rtb_Sin = sinf(localDW->Merge);

  /* Outputs for Atomic SubSystem: '<S2>/Park' */
  Park(ialpha, ibeta, rtb_Sin, rtb_Cos, &rtb_SignPreIntegrator_bb,
       &DiscreteTimeIntegrator);

  /* End of Outputs for SubSystem: '<S2>/Park' */

  /* Sum: '<S15>/Sum1' incorporates:
   *  Constant: '<S15>/Constant'
   */
  rtb_Sum1_j = 0.0F - rtb_SignPreIntegrator_bb;

  /* DiscreteIntegrator: '<S116>/Integrator' */
  if (localDW->Integrator_SYSTEM_ENABLE != 0) {
    /* DiscreteIntegrator: '<S116>/Integrator' */
    Integrator_g = localDW->Integrator_DSTATE_f;
  } else {
    /* DiscreteIntegrator: '<S116>/Integrator' */
    /* [HPM 补丁] 生成代码这里直接 (int16_T) 截断, 既会把 od 轴积分限死在
     * ±32767(下面 x1e-4 = ±3.28V, 远小于 q 轴的 ±12.47V), 又会回绕成反号.
     * 24V 下反电势 4~5V, 角度误差一漏到 d 轴就需要更大的 Vd, 积分器到顶
     * 或回绕 -> id 失控(实测冲 0.36A) -> 电流"毛刺". 改成带饱和的 int32,
     * 并把量化从 1e-4V 放大到 1e-3V (范围 ±32.8V, 分辨率 1mV 足够) */
    {
      int32_T hpm_acc = (int32_T)Curr_loop_ELAPS_T *
                        (int32_T)localDW->Integrator_PREV_U_l +
                        (int32_T)localDW->Integrator_DSTATE_f;

      if (hpm_acc > 32767)
        hpm_acc = 32767;
      else if (hpm_acc < -32768)
        hpm_acc = -32768;
      Integrator_g = (int16_T)hpm_acc;
    }
  }

  /* End of DiscreteIntegrator: '<S116>/Integrator' */

  /* Sum: '<S125>/Sum' incorporates:
   *  Constant: '<S15>/Constant'
   *  Constant: '<S15>/Constant3'
   *  DiscreteIntegrator: '<S116>/Integrator'
   *  Product: '<S121>/PProd Out'
   *  Sum: '<S15>/Sum1'
   */
  DiscreteTimeIntegrator1_n = (0.0F - rtb_SignPreIntegrator_bb) *
    curr_kpki.curr_d_kp + (real32_T)Integrator_g * 0.001F;

  /* Sum: '<S15>/Sum7' */
  rtb_IProdOut = localDW->Merge1 - DiscreteTimeIntegrator;

  /* DiscreteIntegrator: '<S166>/Integrator' */
  if (localDW->Integrator_SYSTEM_ENABLE_e != 0) {
    /* DiscreteIntegrator: '<S166>/Integrator' */
    Integrator = localDW->Integrator_DSTATE;
  } else {
    /* DiscreteIntegrator: '<S166>/Integrator' */
    Integrator = 0.0001F * (real32_T)Curr_loop_ELAPS_T
      * localDW->Integrator_PREV_U + localDW->Integrator_DSTATE;
  }

  /* End of DiscreteIntegrator: '<S166>/Integrator' */

  /* Sum: '<S175>/Sum' incorporates:
   *  Constant: '<S15>/Constant1'
   *  Product: '<S171>/PProd Out'
   */
  rtb_SignPreSat_g = rtb_IProdOut * curr_kpki.curr_d_kp + Integrator;

  /* Saturate: '<S123>/Saturation' */
  if (DiscreteTimeIntegrator1_n > 12.4707661F) {
    DiscreteTimeIntegrator1_n_0 = 12.4707661F;
  } else if (DiscreteTimeIntegrator1_n < -12.4707661F) {
    DiscreteTimeIntegrator1_n_0 = -12.4707661F;
  } else {
    DiscreteTimeIntegrator1_n_0 = DiscreteTimeIntegrator1_n;
  }

  /* End of Saturate: '<S123>/Saturation' */

  /* Saturate: '<S173>/Saturation' */
  if (rtb_SignPreSat_g > 12.4707661F) {
    rtb_SignPreIntegrator_bb = 12.4707661F;
  } else if (rtb_SignPreSat_g < -12.4707661F) {
    rtb_SignPreIntegrator_bb = -12.4707661F;
  } else {
    rtb_SignPreIntegrator_bb = rtb_SignPreSat_g;
  }

  /* End of Saturate: '<S173>/Saturation' */

  /* Outputs for Atomic SubSystem: '<S2>/In_park' */
  /* SignalConversion generated from: '<S2>/In_park' */
  In_park(DiscreteTimeIntegrator1_n_0, rtb_SignPreIntegrator_bb, rtb_Sin,
          rtb_Cos, &ualpha, &ubeta);

  /* End of Outputs for SubSystem: '<S2>/In_park' */

  /* Outputs for Atomic SubSystem: '<S2>/SVPWM1' */
  SVPWM1(ualpha, ubeta, rtu_VDC, rty_Tcmp1);

  /* End of Outputs for SubSystem: '<S2>/SVPWM1' */

  /* SignalConversion generated from: '<S2>/Tcmp2' */
  *rty_Tcmp2 = rty_Tcmp1[1];

  /* SignalConversion generated from: '<S2>/Tcmp3' */
  *rty_Tcmp3 = rty_Tcmp1[2];

  /* Sum: '<S22>/Add8' incorporates:
   *  UnitDelay: '<S22>/Unit Delay4'
   */
  rtb_Sign = localDW->UnitDelay4_DSTATE - ialpha;

  /* Signum: '<S22>/Sign' */
  if (rtb_Sign < 0.0F) {
    rtb_Sign = -1.0F;
  } else if (rtb_Sign > 0.0F) {
    rtb_Sign = 1.0F;
  }

  /* End of Signum: '<S22>/Sign' */

  /* Sum: '<S22>/Add11' incorporates:
   *  Delay: '<S13>/Delay'
   *  Gain: '<S22>/Gain20'
   *  Gain: '<S22>/Gain22'
   *  Product: '<S22>/Product2'
   *  UnitDelay: '<S22>/Unit Delay2'
   *  UnitDelay: '<S22>/Unit Delay3'
   */
  rtb_Cos = (localDW->UnitDelay2_DSTATE - 0.0001F * localDW->Delay_DSTATE *
             localDW->UnitDelay3_DSTATE) + 0.01F / motor.L * rtb_Sign;

  /* Sum: '<S22>/Add10' incorporates:
   *  UnitDelay: '<S22>/Unit Delay5'
   */
  rtb_Sign1 = localDW->UnitDelay5_DSTATE - ibeta;

  /* Signum: '<S22>/Sign1' */
  if (rtb_Sign1 < 0.0F) {
    rtb_Sign1 = -1.0F;
  } else if (rtb_Sign1 > 0.0F) {
    rtb_Sign1 = 1.0F;
  }

  /* End of Signum: '<S22>/Sign1' */

  /* Sum: '<S22>/Add12' incorporates:
   *  Delay: '<S13>/Delay'
   *  Gain: '<S22>/Gain24'
   *  Gain: '<S22>/Gain25'
   *  Product: '<S22>/Product3'
   *  UnitDelay: '<S22>/Unit Delay2'
   *  UnitDelay: '<S22>/Unit Delay3'
   */
  rtb_Sin = (0.0001F * localDW->Delay_DSTATE * localDW->UnitDelay2_DSTATE +
             localDW->UnitDelay3_DSTATE) + 0.01F / motor.L * rtb_Sign1;

  /* Sum: '<S21>/Add8' incorporates:
   *  Product: '<S21>/Product4'
   *  Product: '<S21>/Product5'
   *  Trigonometry: '<S21>/Cos'
   *  Trigonometry: '<S21>/Sin'
   */
  rtb_IntegralGain_b = (0.0F - rtb_Cos * cosf(SMOTheta)) - rtb_Sin * sinf
    (SMOTheta);

  /* DiscreteIntegrator: '<S58>/Integrator' */
  if (localDW->Integrator_SYSTEM_ENABLE_l != 0) {
    /* DiscreteIntegrator: '<S58>/Integrator' */
    Integrator_j = localDW->Integrator_DSTATE_m;
  } else {
    /* DiscreteIntegrator: '<S58>/Integrator' */
    Integrator_j = 0.0001F * (real32_T)Curr_loop_ELAPS_T
      * localDW->Integrator_PREV_U_e + localDW->Integrator_DSTATE_m;
  }

  /* End of DiscreteIntegrator: '<S58>/Integrator' */

  /* Sum: '<S67>/Sum' incorporates:
   *  Gain: '<S63>/Proportional Gain'
   */
  DiscreteTimeIntegrator = 212.1F * rtb_IntegralGain_b + Integrator_j;

  /* Saturate: '<S65>/Saturation' incorporates:
   *  DeadZone: '<S51>/DeadZone'
   */
  if (DiscreteTimeIntegrator > 6283.18555F) {
    rtb_SignPreIntegrator_bb = 6283.18555F;
    DiscreteTimeIntegrator -= 6283.18555F;
  } else {
    if (DiscreteTimeIntegrator < -6283.18555F) {
      rtb_SignPreIntegrator_bb = -6283.18555F;
    } else {
      rtb_SignPreIntegrator_bb = DiscreteTimeIntegrator;
    }

    if (DiscreteTimeIntegrator >= -6283.18555F) {
      DiscreteTimeIntegrator = 0.0F;
    } else {
      DiscreteTimeIntegrator -= -6283.18555F;
    }
  }

  /* End of Saturate: '<S65>/Saturation' */

  /* Sum: '<S20>/Add1' incorporates:
   *  Constant: '<S20>/Constant'
   *  Delay: '<S20>/Delay'
   *  Product: '<S20>/Product'
   *  Sum: '<S20>/Add'
   */
  rtb_Add1 = (rtb_SignPreIntegrator_bb - localDW->Delay_DSTATE_k) * 0.003F +
    localDW->Delay_DSTATE_k;

  /* Gain: '<S13>/Gain1' incorporates:
   *  Gain: '<S13>/Gain'
   */
  *rty_SMOWm = 1.0F / motor.Pn * rtb_Add1 * 9.54929638F;

  /* Gain: '<S55>/Integral Gain' */
  rtb_IntegralGain_b *= 2500.0F;

  /* Sum: '<S24>/Sum1' incorporates:
   *  Gain: '<S24>/Ts'
   */
  rtb_SignPreIntegrator_bb = 0.0001F * rtb_SignPreIntegrator_bb + SMOTheta;

  /* Gain: '<S22>/Gain' incorporates:
   *  Gain: '<S22>/Gain17'
   */
  DiscreteTimeIntegrator1_n_0 = 1.0F - motor.Rs * 0.0001F / motor.L;

  /* Sum: '<S22>/Add5' incorporates:
   *  Gain: '<S22>/Gain'
   *  Gain: '<S22>/Gain15'
   *  Gain: '<S22>/Gain16'
   *  Gain: '<S22>/Gain21'
   *  UnitDelay: '<S22>/Unit Delay'
   *  UnitDelay: '<S22>/Unit Delay2'
   */
  rtb_Sign = ((DiscreteTimeIntegrator1_n_0 * localDW->UnitDelay_DSTATE_j -
               0.0001F / motor.L * localDW->UnitDelay2_DSTATE) + 0.0001F /
              motor.L * ualpha) - 0.0002F / motor.L * rtb_Sign;

  /* Sum: '<S22>/Add9' incorporates:
   *  Gain: '<S22>/Gain17'
   *  Gain: '<S22>/Gain18'
   *  Gain: '<S22>/Gain19'
   *  Gain: '<S22>/Gain23'
   *  UnitDelay: '<S22>/Unit Delay1'
   *  UnitDelay: '<S22>/Unit Delay3'
   */
  rtb_Sign1 = ((DiscreteTimeIntegrator1_n_0 * localDW->UnitDelay1_DSTATE -
                0.0001F / motor.L * localDW->UnitDelay3_DSTATE) + 0.0001F /
               motor.L * ubeta) - 0.0002F / motor.L * rtb_Sign1;

  /* DeadZone: '<S109>/DeadZone' */
  if (DiscreteTimeIntegrator1_n > 12.4707661F) {
    DiscreteTimeIntegrator1_n -= 12.4707661F;
  } else if (DiscreteTimeIntegrator1_n >= -12.4707661F) {
    DiscreteTimeIntegrator1_n = 0.0F;
  } else {
    DiscreteTimeIntegrator1_n -= -12.4707661F;
  }

  /* End of DeadZone: '<S109>/DeadZone' */

  /* Product: '<S113>/IProd Out' incorporates:
   *  Constant: '<S15>/Constant4'
   */
  rtb_IProdOut_p = (int16_T)floorf(rtb_Sum1_j * curr_kpki.curr_d_ki *
    0.1F);

  /* DeadZone: '<S159>/DeadZone' */
  if (rtb_SignPreSat_g > 12.4707661F) {
    rtb_SignPreSat_g -= 12.4707661F;
  } else if (rtb_SignPreSat_g >= -12.4707661F) {
    rtb_SignPreSat_g = 0.0F;
  } else {
    rtb_SignPreSat_g -= -12.4707661F;
  }

  /* End of DeadZone: '<S159>/DeadZone' */

  /* Product: '<S163>/IProd Out' incorporates:
   *  Constant: '<S15>/Constant2'
   */
  rtb_IProdOut *= curr_kpki.curr_d_ki;

  /* If: '<S75>/If1' */
  if (rtb_SignPreIntegrator_bb > 6.28318548F) {
    /* Outputs for IfAction SubSystem: '<S75>/If Action Subsystem2' incorporates:
     *  ActionPort: '<S77>/Action Port'
     */
    /* Update for UnitDelay: '<S24>/Unit Delay' incorporates:
     *  Constant: '<S77>/2pi'
     *  Sum: '<S77>/Subtract1'
     */
    localDW->UnitDelay_DSTATE = rtb_SignPreIntegrator_bb - 6.28318548F;

    /* End of Outputs for SubSystem: '<S75>/If Action Subsystem2' */
  } else if (rtb_SignPreIntegrator_bb < 0.0F) {
    /* Outputs for IfAction SubSystem: '<S75>/If Action Subsystem3' incorporates:
     *  ActionPort: '<S78>/Action Port'
     */
    /* Update for UnitDelay: '<S24>/Unit Delay' incorporates:
     *  Constant: '<S78>/Angle_offset'
     *  Sum: '<S78>/Subtract1'
     */
    localDW->UnitDelay_DSTATE = rtb_SignPreIntegrator_bb + 6.28318548F;

    /* End of Outputs for SubSystem: '<S75>/If Action Subsystem3' */
  } else {
    /* Outputs for IfAction SubSystem: '<S75>/If Action Subsystem1' incorporates:
     *  ActionPort: '<S76>/Action Port'
     */
    /* Update for UnitDelay: '<S24>/Unit Delay' incorporates:
     *  Inport: '<S76>/In1'
     */
    localDW->UnitDelay_DSTATE = rtb_SignPreIntegrator_bb;

    /* End of Outputs for SubSystem: '<S75>/If Action Subsystem1' */
  }

  /* End of If: '<S75>/If1' */

  /* Update for SwitchCase: '<S2>/Switch Case1' */
  switch (localDW->SwitchCase1_ActiveSubsystem) {
   case 0:
   case 1:
   case 4:
    break;

   case 2:
    /* Update for IfAction SubSystem: '<S2>/If Action Subsystem2' incorporates:
     *  ActionPort: '<S8>/Action Port'
     */
    localDW->IfActionSubsystem2_PREV_T = Curr_loop_PREV_T_tmp;
    localDW->IfActionSubsystem2_RESET_ELAPS_ = false;

    /* End of Update for SubSystem: '<S2>/If Action Subsystem2' */
    break;

   case 3:
    /* Update for IfAction SubSystem: '<S2>/If Action Subsystem4' incorporates:
     *  ActionPort: '<S10>/Action Port'
     */
    localDW->IfActionSubsystem4_PREV_T = Curr_loop_PREV_T_tmp;
    localDW->IfActionSubsystem4_RESET_ELAPS_ = false;

    /* End of Update for SubSystem: '<S2>/If Action Subsystem4' */
    break;
  }

  /* Update for DiscreteIntegrator: '<S116>/Integrator' */
  localDW->Integrator_SYSTEM_ENABLE = 0U;
  localDW->Integrator_DSTATE_f = Integrator_g;

  /* Signum: '<S107>/SignPreSat' */
  if (DiscreteTimeIntegrator1_n < 0.0F) {
    DiscreteTimeIntegrator1_n_0 = -1.0F;
  } else if (DiscreteTimeIntegrator1_n > 0.0F) {
    DiscreteTimeIntegrator1_n_0 = 1.0F;
  } else {
    DiscreteTimeIntegrator1_n_0 = DiscreteTimeIntegrator1_n;
  }

  /* End of Signum: '<S107>/SignPreSat' */

  /* Signum: '<S107>/SignPreIntegrator' */
  if (rtb_IProdOut_p < 0) {
    Integrator_g = -1;
  } else {
    Integrator_g = (int16_T)(rtb_IProdOut_p > 0);
  }

  /* End of Signum: '<S107>/SignPreIntegrator' */

  /* Switch: '<S107>/Switch' incorporates:
   *  DataTypeConversion: '<S107>/DataTypeConv1'
   *  DataTypeConversion: '<S107>/DataTypeConv2'
   *  Logic: '<S107>/AND3'
   *  RelationalOperator: '<S107>/Equal1'
   *  RelationalOperator: '<S107>/NotEqual'
   */
  if ((0.0F != DiscreteTimeIntegrator1_n) && ((int8_T)
       DiscreteTimeIntegrator1_n_0 == Integrator_g)) {
    /* Update for DiscreteIntegrator: '<S116>/Integrator' incorporates:
     *  Constant: '<S107>/Constant1'
     */
    localDW->Integrator_PREV_U_l = 0;
  } else {
    /* Update for DiscreteIntegrator: '<S116>/Integrator' */
    localDW->Integrator_PREV_U_l = rtb_IProdOut_p;
  }

  /* End of Switch: '<S107>/Switch' */

  /* Update for DiscreteIntegrator: '<S166>/Integrator' */
  localDW->Integrator_SYSTEM_ENABLE_e = 0U;
  localDW->Integrator_DSTATE = Integrator;

  /* Signum: '<S157>/SignPreSat' */
  if (rtb_SignPreSat_g < 0.0F) {
    rtb_SignPreIntegrator_bb = -1.0F;
  } else if (rtb_SignPreSat_g > 0.0F) {
    rtb_SignPreIntegrator_bb = 1.0F;
  } else {
    rtb_SignPreIntegrator_bb = rtb_SignPreSat_g;
  }

  /* End of Signum: '<S157>/SignPreSat' */

  /* Signum: '<S157>/SignPreIntegrator' */
  if (rtb_IProdOut < 0.0F) {
    DiscreteTimeIntegrator1_n = -1.0F;
  } else if (rtb_IProdOut > 0.0F) {
    DiscreteTimeIntegrator1_n = 1.0F;
  } else {
    DiscreteTimeIntegrator1_n = rtb_IProdOut;
  }

  /* End of Signum: '<S157>/SignPreIntegrator' */

  /* Switch: '<S157>/Switch' incorporates:
   *  DataTypeConversion: '<S157>/DataTypeConv1'
   *  DataTypeConversion: '<S157>/DataTypeConv2'
   *  Logic: '<S157>/AND3'
   *  RelationalOperator: '<S157>/Equal1'
   *  RelationalOperator: '<S157>/NotEqual'
   */
  if ((0.0F != rtb_SignPreSat_g) && ((int8_T)rtb_SignPreIntegrator_bb == (int8_T)
       DiscreteTimeIntegrator1_n)) {
    /* Update for DiscreteIntegrator: '<S166>/Integrator' incorporates:
     *  Constant: '<S157>/Constant1'
     */
    localDW->Integrator_PREV_U = 0.0F;
  } else {
    /* Update for DiscreteIntegrator: '<S166>/Integrator' */
    localDW->Integrator_PREV_U = rtb_IProdOut;
  }

  /* End of Switch: '<S157>/Switch' */

  /* Update for Delay: '<S13>/Delay' */
  localDW->Delay_DSTATE = rtb_Add1;

  /* Update for UnitDelay: '<S22>/Unit Delay2' */
  localDW->UnitDelay2_DSTATE = rtb_Cos;

  /* Update for UnitDelay: '<S22>/Unit Delay3' */
  localDW->UnitDelay3_DSTATE = rtb_Sin;

  /* Update for UnitDelay: '<S22>/Unit Delay4' */
  localDW->UnitDelay4_DSTATE = rtb_Sign;

  /* Update for UnitDelay: '<S22>/Unit Delay5' */
  localDW->UnitDelay5_DSTATE = rtb_Sign1;

  /* Update for DiscreteIntegrator: '<S58>/Integrator' */
  localDW->Integrator_SYSTEM_ENABLE_l = 0U;
  localDW->Integrator_DSTATE_m = Integrator_j;

  /* Signum: '<S49>/SignPreSat' */
  if (DiscreteTimeIntegrator < 0.0F) {
    rtb_Cos = -1.0F;
  } else if (DiscreteTimeIntegrator > 0.0F) {
    rtb_Cos = 1.0F;
  } else {
    rtb_Cos = DiscreteTimeIntegrator;
  }

  /* End of Signum: '<S49>/SignPreSat' */

  /* Signum: '<S49>/SignPreIntegrator' */
  if (rtb_IntegralGain_b < 0.0F) {
    rtb_Sin = -1.0F;
  } else if (rtb_IntegralGain_b > 0.0F) {
    rtb_Sin = 1.0F;
  } else {
    rtb_Sin = rtb_IntegralGain_b;
  }

  /* End of Signum: '<S49>/SignPreIntegrator' */

  /* Switch: '<S49>/Switch' incorporates:
   *  DataTypeConversion: '<S49>/DataTypeConv1'
   *  DataTypeConversion: '<S49>/DataTypeConv2'
   *  Logic: '<S49>/AND3'
   *  RelationalOperator: '<S49>/Equal1'
   *  RelationalOperator: '<S49>/NotEqual'
   */
  if ((0.0F != DiscreteTimeIntegrator) && ((int8_T)rtb_Cos == (int8_T)rtb_Sin))
  {
    /* Update for DiscreteIntegrator: '<S58>/Integrator' incorporates:
     *  Constant: '<S49>/Constant1'
     */
    localDW->Integrator_PREV_U_e = 0.0F;
  } else {
    /* Update for DiscreteIntegrator: '<S58>/Integrator' */
    localDW->Integrator_PREV_U_e = rtb_IntegralGain_b;
  }

  /* End of Switch: '<S49>/Switch' */

  /* Update for Delay: '<S20>/Delay' */
  localDW->Delay_DSTATE_k = rtb_Add1;

  /* Update for UnitDelay: '<S22>/Unit Delay' */
  localDW->UnitDelay_DSTATE_j = rtb_Sign;

  /* Update for UnitDelay: '<S22>/Unit Delay1' */
  localDW->UnitDelay1_DSTATE = rtb_Sign1;
}

/* Model step function */
void FOC_Model_step(void)
{
  real32_T PWM_HalfPeriod[3];

  /* RateTransition: '<S1>/Rate Transition3' */
  if (rtM->Timing.TaskCounters.TID[1] == 0) {
    /* RateTransition: '<S1>/Rate Transition3' */
    rtDW.RateTransition3 = rtDW.RateTransition3_Buffer0;
  }

  /* End of RateTransition: '<S1>/Rate Transition3' */

  /* S-Function (fcgen): '<S1>/Function-Call Generator1' incorporates:
   *  SubSystem: '<S1>/Curr_loop'
   */
  /* Inport: '<Root>/ia' incorporates:
   *  Inport: '<Root>/MotorOnOff'
   *  Inport: '<Root>/VDC'
   *  Inport: '<Root>/ib'
   *  Inport: '<Root>/ic'
   *  Outport: '<Root>/Tcmp2'
   *  Outport: '<Root>/Tcmp3'
   */
  Curr_loop(rtM, rtU.ia, rtU.ib, rtU.ic, rtU.MotorOnOff, rtDW.RateTransition3,
            rtU.VDC, PWM_HalfPeriod, &rtY.Tcmp2, &rtY.Tcmp3, &SMOWm,
            &rtDW.RestsSingal, &rtDW.Curr_loop_d);

  /* End of Outputs for S-Function (fcgen): '<S1>/Function-Call Generator1' */

  /* Outport: '<Root>/Tcmp1' */
  rtY.Tcmp1 = PWM_HalfPeriod[0];

  /* RateTransition: '<S1>/Rate Transition' */
  if (rtM->Timing.TaskCounters.TID[1] == 0) {
    real32_T Integrator;
    real32_T rtb_IntegralGain;
    real32_T rtb_IntegralGain_0;
    real32_T rtb_SignPreIntegrator;
    real32_T rtb_Sum;
    uint32_T Speed_loop_ELAPS_T;

    /* S-Function (fcgen): '<S1>/Function-Call Generator' incorporates:
     *  SubSystem: '<S1>/Speed_loop'
     */
    if (rtDW.Speed_loop_RESET_ELAPS_T) {
      Speed_loop_ELAPS_T = 0U;
    } else {
      Speed_loop_ELAPS_T = rtM->Timing.clockTick1 - rtDW.Speed_loop_PREV_T;
    }

    rtDW.Speed_loop_PREV_T = rtM->Timing.clockTick1;
    rtDW.Speed_loop_RESET_ELAPS_T = false;

    /* Sum: '<S3>/Sum2' incorporates:
     *  Inport: '<Root>/Speed_ref'
     */
    rtb_IntegralGain = rtU.Speed_ref - SMOWm;

    /* DiscreteIntegrator: '<S217>/Integrator' incorporates:
     *  RateTransition: '<S1>/Rate Transition2'
     */
    if (rtDW.Integrator_SYSTEM_ENABLE != 0) {
      /* DiscreteIntegrator: '<S217>/Integrator' */
      Integrator = rtDW.Integrator_DSTATE;
    } else if ((rtDW.RestsSingal > 0.0) && (rtDW.Integrator_PrevResetState <= 0))
    {
      /* [HPM 补丁] 原代码在进 RunStage 时把速度环积分清零, 交接瞬间
       * iq_ref 只有 P 项 (~0), 电机会掉速, 随后速度环大幅补 -> 超调/振荡.
       * 这里预置成维持转速所需的摩擦电流, 实现无扰切换 */
      rtDW.Integrator_DSTATE = HPM_if_iq_preload;

      /* DiscreteIntegrator: '<S217>/Integrator' */
      Integrator = rtDW.Integrator_DSTATE;
    } else {
      /* DiscreteIntegrator: '<S217>/Integrator' */
      Integrator = 0.001F * (real32_T)Speed_loop_ELAPS_T
        * rtDW.Integrator_PREV_U + rtDW.Integrator_DSTATE;
    }

    /* End of DiscreteIntegrator: '<S217>/Integrator' */

    /* Sum: '<S226>/Sum' incorporates:
     *  Gain: '<S222>/Proportional Gain'
     */
    rtb_Sum = spd_kpki.spd_kp * rtb_IntegralGain + Integrator;

    /* DeadZone: '<S210>/DeadZone' */
    if (rtb_Sum > HPM_IQ_LIMIT) {
      rtb_SignPreIntegrator = rtb_Sum - HPM_IQ_LIMIT;
    } else if (rtb_Sum >= -HPM_IQ_LIMIT) {
      rtb_SignPreIntegrator = 0.0F;
    } else {
      rtb_SignPreIntegrator = rtb_Sum - -HPM_IQ_LIMIT;
    }

    /* End of DeadZone: '<S210>/DeadZone' */

    /* Gain: '<S214>/Integral Gain' */
    rtb_IntegralGain *= spd_kpki.spd_ki;

    /* Update for DiscreteIntegrator: '<S217>/Integrator' incorporates:
     *  RateTransition: '<S1>/Rate Transition2'
     */
    rtDW.Integrator_SYSTEM_ENABLE = 0U;
    rtDW.Integrator_DSTATE = Integrator;
    if (rtDW.RestsSingal > 0.0) {
      rtDW.Integrator_PrevResetState = 1;
    } else if (rtDW.RestsSingal < 0.0) {
      rtDW.Integrator_PrevResetState = -1;
    } else if (rtDW.RestsSingal == 0.0) {
      rtDW.Integrator_PrevResetState = 0;
    } else {
      rtDW.Integrator_PrevResetState = 2;
    }

    /* Signum: '<S208>/SignPreSat' */
    if (rtb_SignPreIntegrator < 0.0F) {
      Integrator = -1.0F;
    } else if (rtb_SignPreIntegrator > 0.0F) {
      Integrator = 1.0F;
    } else {
      Integrator = rtb_SignPreIntegrator;
    }

    /* End of Signum: '<S208>/SignPreSat' */

    /* Signum: '<S208>/SignPreIntegrator' */
    if (rtb_IntegralGain < 0.0F) {
      rtb_IntegralGain_0 = -1.0F;
    } else if (rtb_IntegralGain > 0.0F) {
      rtb_IntegralGain_0 = 1.0F;
    } else {
      rtb_IntegralGain_0 = rtb_IntegralGain;
    }

    /* End of Signum: '<S208>/SignPreIntegrator' */

    /* Switch: '<S208>/Switch' incorporates:
     *  DataTypeConversion: '<S208>/DataTypeConv1'
     *  DataTypeConversion: '<S208>/DataTypeConv2'
     *  Gain: '<S208>/ZeroGain'
     *  Logic: '<S208>/AND3'
     *  RelationalOperator: '<S208>/Equal1'
     *  RelationalOperator: '<S208>/NotEqual'
     */
    if ((0.0F != rtb_SignPreIntegrator) && ((int8_T)Integrator == (int8_T)
         rtb_IntegralGain_0)) {
      /* Update for DiscreteIntegrator: '<S217>/Integrator' incorporates:
       *  Constant: '<S208>/Constant1'
       */
      rtDW.Integrator_PREV_U = 0.0F;
    } else {
      /* Update for DiscreteIntegrator: '<S217>/Integrator' */
      rtDW.Integrator_PREV_U = rtb_IntegralGain;
    }

    /* End of Switch: '<S208>/Switch' */

    /* Saturate: '<S224>/Saturation' */
    if (rtb_Sum > HPM_IQ_LIMIT) {
      /* Update for RateTransition: '<S1>/Rate Transition3' */
      rtDW.RateTransition3_Buffer0 = HPM_IQ_LIMIT;
    } else if (rtb_Sum < -HPM_IQ_LIMIT) {
      /* Update for RateTransition: '<S1>/Rate Transition3' */
      rtDW.RateTransition3_Buffer0 = -HPM_IQ_LIMIT;
    } else {
      /* Update for RateTransition: '<S1>/Rate Transition3' */
      rtDW.RateTransition3_Buffer0 = rtb_Sum;
    }

    /* End of Saturate: '<S224>/Saturation' */
    /* End of Outputs for S-Function (fcgen): '<S1>/Function-Call Generator' */
  }

  /* End of RateTransition: '<S1>/Rate Transition' */

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The resolution of this integer timer is 0.0001, which is the step size
   * of the task. Size of "clockTick0" ensures timer will not overflow during the
   * application lifespan selected.
   */
  rtM->Timing.clockTick0++;
  if (rtM->Timing.TaskCounters.TID[1] == 0) {
    /* Update absolute timer for sample time: [0.001s, 0.0s] */
    /* The "clockTick1" counts the number of times the code of this task has
     * been executed. The resolution of this integer timer is 0.001, which is the step size
     * of the task. Size of "clockTick1" ensures timer will not overflow during the
     * application lifespan selected.
     */
    rtM->Timing.clockTick1++;
  }

  rate_scheduler();
}

/* Model initialize function */
void FOC_Model_initialize(void)
{
  /* SystemInitialize for S-Function (fcgen): '<S1>/Function-Call Generator1' incorporates:
   *  SubSystem: '<S1>/Curr_loop'
   */
  Curr_loop_Init(&rtDW.RestsSingal, &rtDW.Curr_loop_d);

  /* End of SystemInitialize for S-Function (fcgen): '<S1>/Function-Call Generator1' */

  /* SystemInitialize for S-Function (fcgen): '<S1>/Function-Call Generator' incorporates:
   *  SubSystem: '<S1>/Speed_loop'
   */
  /* InitializeConditions for DiscreteIntegrator: '<S217>/Integrator' */
  rtDW.Integrator_PrevResetState = 2;

  /* End of SystemInitialize for S-Function (fcgen): '<S1>/Function-Call Generator' */

  /* Enable for S-Function (fcgen): '<S1>/Function-Call Generator1' incorporates:
   *  SubSystem: '<S1>/Curr_loop'
   */
  Curr_loop_Enable(&rtDW.Curr_loop_d);

  /* End of Enable for S-Function (fcgen): '<S1>/Function-Call Generator1' */

  /* Enable for S-Function (fcgen): '<S1>/Function-Call Generator' incorporates:
   *  SubSystem: '<S1>/Speed_loop'
   */
  rtDW.Speed_loop_RESET_ELAPS_T = true;

  /* Enable for DiscreteIntegrator: '<S217>/Integrator' */
  rtDW.Integrator_SYSTEM_ENABLE = 1U;

  /* End of Enable for S-Function (fcgen): '<S1>/Function-Call Generator' */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
