/*
 * File: FOC_Model.h
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

#ifndef RTW_HEADER_FOC_Model_h_
#define RTW_HEADER_FOC_Model_h_
#include <float.h>
#include <math.h>
#ifndef FOC_Model_COMMON_INCLUDES_
#define FOC_Model_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* FOC_Model_COMMON_INCLUDES_ */

/* Model Code Variants */

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM RT_MODEL;

/* Block signals and states (default storage) for system '<S1>/Curr_loop' */
typedef struct {
  real32_T Merge;                      /* '<S2>/Merge' */
  real32_T Merge1;                     /* '<S2>/Merge1' */
  real32_T ZReset;                     /* '<S2>/Chart1' */
  real32_T UnitDelay_DSTATE;           /* '<S24>/Unit Delay' */
  real32_T Integrator_DSTATE;          /* '<S166>/Integrator' */
  real32_T Delay_DSTATE;               /* '<S13>/Delay' */
  real32_T UnitDelay2_DSTATE;          /* '<S22>/Unit Delay2' */
  real32_T UnitDelay3_DSTATE;          /* '<S22>/Unit Delay3' */
  real32_T UnitDelay4_DSTATE;          /* '<S22>/Unit Delay4' */
  real32_T UnitDelay5_DSTATE;          /* '<S22>/Unit Delay5' */
  real32_T Integrator_DSTATE_m;        /* '<S58>/Integrator' */
  real32_T Delay_DSTATE_k;             /* '<S20>/Delay' */
  real32_T UnitDelay_DSTATE_j;         /* '<S22>/Unit Delay' */
  real32_T UnitDelay1_DSTATE;          /* '<S22>/Unit Delay1' */
  real32_T DiscreteTimeIntegrator1_DSTATE;/* '<S10>/Discrete-Time Integrator1' */
  real32_T UnitDelay_DSTATE_b;         /* '<S10>/Unit Delay' */
  real32_T DiscreteTimeIntegrator_DSTATE;/* '<S8>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator1_DSTAT_j;/* '<S8>/Discrete-Time Integrator1' */
  real32_T Integrator_PREV_U;          /* '<S166>/Integrator' */
  real32_T Integrator_PREV_U_e;        /* '<S58>/Integrator' */
  real32_T DiscreteTimeIntegrator1_PREV_U;/* '<S10>/Discrete-Time Integrator1' */
  real32_T DiscreteTimeIntegrator_PREV_U;/* '<S8>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator1_PREV__h;/* '<S8>/Discrete-Time Integrator1' */
  real32_T cnt;                        /* '<S2>/Chart1' */
  uint32_T Curr_loop_PREV_T;           /* '<S1>/Curr_loop' */
  uint32_T IfActionSubsystem4_PREV_T;  /* '<S2>/If Action Subsystem4' */
  uint32_T IfActionSubsystem2_PREV_T;  /* '<S2>/If Action Subsystem2' */
  int16_T Integrator_DSTATE_f;         /* '<S116>/Integrator' */
  int16_T Integrator_PREV_U_l;         /* '<S116>/Integrator' */
  uint16_T temporalCounter_i1;         /* '<S2>/Chart1' */
  int8_T SwitchCase1_ActiveSubsystem;  /* '<S2>/Switch Case1' */
  int8_T DiscreteTimeIntegrator_PrevRese;/* '<S8>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator1_PrevRes;/* '<S8>/Discrete-Time Integrator1' */
  uint8_T Integrator_SYSTEM_ENABLE;    /* '<S116>/Integrator' */
  uint8_T Integrator_SYSTEM_ENABLE_e;  /* '<S166>/Integrator' */
  uint8_T Integrator_SYSTEM_ENABLE_l;  /* '<S58>/Integrator' */
  uint8_T DiscreteTimeIntegrator1_SYSTEM_;/* '<S10>/Discrete-Time Integrator1' */
  uint8_T DiscreteTimeIntegrator_SYSTEM_E;/* '<S8>/Discrete-Time Integrator' */
  uint8_T DiscreteTimeIntegrator1_SYSTE_f;/* '<S8>/Discrete-Time Integrator1' */
  uint8_T is_active_c7_FOC_Model;      /* '<S2>/Chart1' */
  uint8_T is_c7_FOC_Model;             /* '<S2>/Chart1' */
  boolean_T Curr_loop_RESET_ELAPS_T;   /* '<S1>/Curr_loop' */
  boolean_T IfActionSubsystem4_RESET_ELAPS_;/* '<S2>/If Action Subsystem4' */
  boolean_T IfActionSubsystem2_RESET_ELAPS_;/* '<S2>/If Action Subsystem2' */
} DW_Curr_loop;

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  DW_Curr_loop Curr_loop_d;            /* '<S1>/Curr_loop' */
  real_T RestsSingal;                  /* '<S2>/Chart1' */
  real32_T RateTransition3;            /* '<S1>/Rate Transition3' */
  real32_T Integrator_DSTATE;          /* '<S217>/Integrator' */
  real32_T RateTransition3_Buffer0;    /* '<S1>/Rate Transition3' */
  real32_T Integrator_PREV_U;          /* '<S217>/Integrator' */
  uint32_T Speed_loop_PREV_T;          /* '<S1>/Speed_loop' */
  int8_T Integrator_PrevResetState;    /* '<S217>/Integrator' */
  uint8_T Integrator_SYSTEM_ENABLE;    /* '<S217>/Integrator' */
  boolean_T Speed_loop_RESET_ELAPS_T;  /* '<S1>/Speed_loop' */
} DW;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real32_T ia;                         /* '<Root>/ia' */
  real32_T ib;                         /* '<Root>/ib' */
  real32_T ic;                         /* '<Root>/ic' */
  real32_T Speed_ref;                  /* '<Root>/Speed_ref' */
  real32_T MotorOnOff;                 /* '<Root>/MotorOnOff' */
  real32_T VDC;                        /* '<Root>/VDC' */
} ExtU;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real32_T Tcmp1;                      /* '<Root>/Tcmp1' */
  real32_T Tcmp2;                      /* '<Root>/Tcmp2' */
  real32_T Tcmp3;                      /* '<Root>/Tcmp3' */
} ExtY;

/* Type definition for custom storage class: Struct */
typedef struct curr_kpki_tag {
  real32_T curr_d_ki;                  /* Referenced by:
                                        * '<S15>/Constant2'
                                        * '<S15>/Constant4'
                                        */
  real32_T curr_d_kp;                  /* Referenced by:
                                        * '<S15>/Constant1'
                                        * '<S15>/Constant3'
                                        */
} curr_kpki_type;

typedef struct motor_tag {
  real32_T L;                          /* Referenced by:
                                        * '<S22>/Gain'
                                        * '<S22>/Gain15'
                                        * '<S22>/Gain16'
                                        * '<S22>/Gain17'
                                        * '<S22>/Gain18'
                                        * '<S22>/Gain19'
                                        * '<S22>/Gain21'
                                        * '<S22>/Gain22'
                                        * '<S22>/Gain23'
                                        * '<S22>/Gain25'
                                        */
  real32_T Pn;                         /* Referenced by:
                                        * '<S8>/Gain'
                                        * '<S10>/Gain'
                                        * '<S13>/Gain'
                                        */
  real32_T Rs;                         /* Referenced by:
                                        * '<S22>/Gain'
                                        * '<S22>/Gain17'
                                        */
} motor_type;

typedef struct spd_kpki_tag {
  real32_T spd_ki;                   /* Referenced by: '<S214>/Integral Gain' */
  real32_T spd_kp;               /* Referenced by: '<S222>/Proportional Gain' */
} spd_kpki_type;

/* Real-time Model Data Structure */
struct tag_RTM {
  const char_T * volatile errorStatus;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    uint32_T clockTick1;
    struct {
      uint8_T TID[2];
    } TaskCounters;
  } Timing;
};

/* Block signals and states (default storage) */
extern DW rtDW;

/* External inputs (root inport signals with default storage) */
extern ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY rtY;

/*
 * Exported Global Signals
 *
 * Note: Exported global signals are block signals with an exported global
 * storage class designation.  Code generation will declare the memory for
 * these signals and export their symbols.
 *
 */
extern real32_T SMOWm;                 /* '<S13>/Gain1' */
extern real32_T SMOTheta;              /* '<S24>/Unit Delay' */
extern real32_T ialpha;                /* '<S2>/Clark' */
extern real32_T ibeta;                 /* '<S2>/Clark' */
extern real32_T ualpha;                /* '<S2>/In_park' */
extern real32_T ubeta;                 /* '<S2>/In_park' */
extern real32_T state;                 /* '<S2>/Chart1' */

/* Model entry point functions */
extern void FOC_Model_initialize(void);
extern void FOC_Model_step(void);

/* Exported data declaration */

/* Declaration for custom storage class: Struct */
extern curr_kpki_type curr_kpki;
extern motor_type motor;
extern spd_kpki_type spd_kpki;

/* Real-time Model object */
extern RT_MODEL *const rtM;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S8>/Scope' : Unused code path elimination
 * Block '<S8>/Scope1' : Unused code path elimination
 * Block '<S10>/Add1' : Unused code path elimination
 * Block '<S10>/Add2' : Unused code path elimination
 * Block '<S10>/Constant3' : Unused code path elimination
 * Block '<S10>/Mod1' : Unused code path elimination
 * Block '<S10>/Product' : Unused code path elimination
 * Block '<S10>/Scope' : Unused code path elimination
 * Block '<S10>/Scope1' : Unused code path elimination
 * Block '<S10>/Scope2' : Unused code path elimination
 * Block '<S10>/Scope3' : Unused code path elimination
 * Block '<S10>/Scope4' : Unused code path elimination
 * Block '<S11>/Scope' : Unused code path elimination
 * Block '<S13>/Scope' : Unused code path elimination
 * Block '<S13>/Scope1' : Unused code path elimination
 * Block '<S22>/Scope' : Unused code path elimination
 * Block '<S2>/Scope' : Unused code path elimination
 * Block '<S2>/Scope3' : Unused code path elimination
 * Block '<S15>/Scope' : Unused code path elimination
 * Block '<S1>/Rate Transition1' : Eliminated since input and output rates are identical
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Note that this particular code originates from a subsystem build,
 * and has its own system numbers different from the parent model.
 * Refer to the system hierarchy for this subsystem below, and use the
 * MATLAB hilite_system command to trace the generated code back
 * to the parent model.  For example,
 *
 * hilite_system('BLDC_SMO/FOC_Model')    - opens subsystem BLDC_SMO/FOC_Model
 * hilite_system('BLDC_SMO/FOC_Model/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'BLDC_SMO'
 * '<S1>'   : 'BLDC_SMO/FOC_Model'
 * '<S2>'   : 'BLDC_SMO/FOC_Model/Curr_loop'
 * '<S3>'   : 'BLDC_SMO/FOC_Model/Speed_loop'
 * '<S4>'   : 'BLDC_SMO/FOC_Model/Curr_loop/Chart1'
 * '<S5>'   : 'BLDC_SMO/FOC_Model/Curr_loop/Clark'
 * '<S6>'   : 'BLDC_SMO/FOC_Model/Curr_loop/If Action Subsystem'
 * '<S7>'   : 'BLDC_SMO/FOC_Model/Curr_loop/If Action Subsystem1'
 * '<S8>'   : 'BLDC_SMO/FOC_Model/Curr_loop/If Action Subsystem2'
 * '<S9>'   : 'BLDC_SMO/FOC_Model/Curr_loop/If Action Subsystem3'
 * '<S10>'  : 'BLDC_SMO/FOC_Model/Curr_loop/If Action Subsystem4'
 * '<S11>'  : 'BLDC_SMO/FOC_Model/Curr_loop/In_park'
 * '<S12>'  : 'BLDC_SMO/FOC_Model/Curr_loop/Park'
 * '<S13>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO'
 * '<S14>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SVPWM1'
 * '<S15>'  : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller'
 * '<S16>'  : 'BLDC_SMO/FOC_Model/Curr_loop/If Action Subsystem4/If Action Subsystem'
 * '<S17>'  : 'BLDC_SMO/FOC_Model/Curr_loop/If Action Subsystem4/If Action Subsystem1'
 * '<S18>'  : 'BLDC_SMO/FOC_Model/Curr_loop/If Action Subsystem4/If Action Subsystem2'
 * '<S19>'  : 'BLDC_SMO/FOC_Model/Curr_loop/If Action Subsystem4/If Action Subsystem3'
 * '<S20>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/LPF1'
 * '<S21>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/PLL'
 * '<S22>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/discrete smo '
 * '<S23>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/PLL/PID Controller'
 * '<S24>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/PLL/SpeedToTheta1'
 * '<S25>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/PLL/PID Controller/Anti-windup'
 * '<S26>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/PLL/PID Controller/D Gain'
 * '<S27>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/PLL/PID Controller/Filter'
 * '<S28>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/PLL/PID Controller/Filter ICs'
 * '<S29>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/PLL/PID Controller/I Gain'
 * '<S30>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/PLL/PID Controller/Ideal P Gain'
 * '<S31>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/PLL/PID Controller/Ideal P Gain Fdbk'
 * '<S32>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/PLL/PID Controller/Integrator'
 * '<S33>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/PLL/PID Controller/Integrator ICs'
 * '<S34>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/PLL/PID Controller/N Copy'
 * '<S35>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/PLL/PID Controller/N Gain'
 * '<S36>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/PLL/PID Controller/P Copy'
 * '<S37>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/PLL/PID Controller/Parallel P Gain'
 * '<S38>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/PLL/PID Controller/Reset Signal'
 * '<S39>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/PLL/PID Controller/Saturation'
 * '<S40>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/PLL/PID Controller/Saturation Fdbk'
 * '<S41>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/PLL/PID Controller/Sum'
 * '<S42>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/PLL/PID Controller/Sum Fdbk'
 * '<S43>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/PLL/PID Controller/Tracking Mode'
 * '<S44>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/PLL/PID Controller/Tracking Mode Sum'
 * '<S45>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/PLL/PID Controller/Tsamp - Integral'
 * '<S46>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/PLL/PID Controller/Tsamp - Ngain'
 * '<S47>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/PLL/PID Controller/postSat Signal'
 * '<S48>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/PLL/PID Controller/preSat Signal'
 * '<S49>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/PLL/PID Controller/Anti-windup/Disc. Clamping Parallel'
 * '<S50>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/PLL/PID Controller/Anti-windup/Disc. Clamping Parallel/Dead Zone'
 * '<S51>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/PLL/PID Controller/Anti-windup/Disc. Clamping Parallel/Dead Zone/Enabled'
 * '<S52>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/PLL/PID Controller/D Gain/Disabled'
 * '<S53>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/PLL/PID Controller/Filter/Disabled'
 * '<S54>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/PLL/PID Controller/Filter ICs/Disabled'
 * '<S55>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/PLL/PID Controller/I Gain/Internal Parameters'
 * '<S56>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/PLL/PID Controller/Ideal P Gain/Passthrough'
 * '<S57>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/PLL/PID Controller/Ideal P Gain Fdbk/Disabled'
 * '<S58>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/PLL/PID Controller/Integrator/Discrete'
 * '<S59>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/PLL/PID Controller/Integrator ICs/Internal IC'
 * '<S60>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/PLL/PID Controller/N Copy/Disabled wSignal Specification'
 * '<S61>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/PLL/PID Controller/N Gain/Disabled'
 * '<S62>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/PLL/PID Controller/P Copy/Disabled'
 * '<S63>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/PLL/PID Controller/Parallel P Gain/Internal Parameters'
 * '<S64>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/PLL/PID Controller/Reset Signal/Disabled'
 * '<S65>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/PLL/PID Controller/Saturation/Enabled'
 * '<S66>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/PLL/PID Controller/Saturation Fdbk/Disabled'
 * '<S67>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/PLL/PID Controller/Sum/Sum_PI'
 * '<S68>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/PLL/PID Controller/Sum Fdbk/Disabled'
 * '<S69>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/PLL/PID Controller/Tracking Mode/Disabled'
 * '<S70>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/PLL/PID Controller/Tracking Mode Sum/Passthrough'
 * '<S71>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/PLL/PID Controller/Tsamp - Integral/Passthrough'
 * '<S72>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/PLL/PID Controller/Tsamp - Ngain/Passthrough'
 * '<S73>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/PLL/PID Controller/postSat Signal/Forward_Path'
 * '<S74>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/PLL/PID Controller/preSat Signal/Forward_Path'
 * '<S75>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/PLL/SpeedToTheta1/Angle_Limit'
 * '<S76>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/PLL/SpeedToTheta1/Angle_Limit/If Action Subsystem1'
 * '<S77>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/PLL/SpeedToTheta1/Angle_Limit/If Action Subsystem2'
 * '<S78>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SMO/PLL/SpeedToTheta1/Angle_Limit/If Action Subsystem3'
 * '<S79>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SVPWM1/InvClark'
 * '<S80>'  : 'BLDC_SMO/FOC_Model/Curr_loop/SVPWM1/ei_t'
 * '<S81>'  : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller1'
 * '<S82>'  : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller2'
 * '<S83>'  : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller1/Anti-windup'
 * '<S84>'  : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller1/D Gain'
 * '<S85>'  : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller1/Filter'
 * '<S86>'  : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller1/Filter ICs'
 * '<S87>'  : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller1/I Gain'
 * '<S88>'  : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller1/Ideal P Gain'
 * '<S89>'  : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller1/Ideal P Gain Fdbk'
 * '<S90>'  : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller1/Integrator'
 * '<S91>'  : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller1/Integrator ICs'
 * '<S92>'  : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller1/N Copy'
 * '<S93>'  : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller1/N Gain'
 * '<S94>'  : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller1/P Copy'
 * '<S95>'  : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller1/Parallel P Gain'
 * '<S96>'  : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller1/Reset Signal'
 * '<S97>'  : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller1/Saturation'
 * '<S98>'  : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller1/Saturation Fdbk'
 * '<S99>'  : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller1/Sum'
 * '<S100>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller1/Sum Fdbk'
 * '<S101>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller1/Tracking Mode'
 * '<S102>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller1/Tracking Mode Sum'
 * '<S103>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller1/Tsamp - Integral'
 * '<S104>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller1/Tsamp - Ngain'
 * '<S105>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller1/postSat Signal'
 * '<S106>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller1/preSat Signal'
 * '<S107>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller1/Anti-windup/Disc. Clamping Parallel'
 * '<S108>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller1/Anti-windup/Disc. Clamping Parallel/Dead Zone'
 * '<S109>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller1/Anti-windup/Disc. Clamping Parallel/Dead Zone/Enabled'
 * '<S110>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller1/D Gain/Disabled'
 * '<S111>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller1/Filter/Disabled'
 * '<S112>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller1/Filter ICs/Disabled'
 * '<S113>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller1/I Gain/External Parameters'
 * '<S114>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller1/Ideal P Gain/Passthrough'
 * '<S115>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller1/Ideal P Gain Fdbk/Disabled'
 * '<S116>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller1/Integrator/Discrete'
 * '<S117>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller1/Integrator ICs/Internal IC'
 * '<S118>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller1/N Copy/Disabled wSignal Specification'
 * '<S119>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller1/N Gain/Disabled'
 * '<S120>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller1/P Copy/Disabled'
 * '<S121>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller1/Parallel P Gain/External Parameters'
 * '<S122>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller1/Reset Signal/Disabled'
 * '<S123>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller1/Saturation/Enabled'
 * '<S124>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller1/Saturation Fdbk/Disabled'
 * '<S125>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller1/Sum/Sum_PI'
 * '<S126>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller1/Sum Fdbk/Disabled'
 * '<S127>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller1/Tracking Mode/Disabled'
 * '<S128>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller1/Tracking Mode Sum/Passthrough'
 * '<S129>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller1/Tsamp - Integral/Passthrough'
 * '<S130>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller1/Tsamp - Ngain/Passthrough'
 * '<S131>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller1/postSat Signal/Forward_Path'
 * '<S132>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller1/preSat Signal/Forward_Path'
 * '<S133>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller2/Anti-windup'
 * '<S134>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller2/D Gain'
 * '<S135>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller2/Filter'
 * '<S136>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller2/Filter ICs'
 * '<S137>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller2/I Gain'
 * '<S138>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller2/Ideal P Gain'
 * '<S139>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller2/Ideal P Gain Fdbk'
 * '<S140>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller2/Integrator'
 * '<S141>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller2/Integrator ICs'
 * '<S142>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller2/N Copy'
 * '<S143>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller2/N Gain'
 * '<S144>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller2/P Copy'
 * '<S145>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller2/Parallel P Gain'
 * '<S146>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller2/Reset Signal'
 * '<S147>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller2/Saturation'
 * '<S148>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller2/Saturation Fdbk'
 * '<S149>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller2/Sum'
 * '<S150>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller2/Sum Fdbk'
 * '<S151>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller2/Tracking Mode'
 * '<S152>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller2/Tracking Mode Sum'
 * '<S153>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller2/Tsamp - Integral'
 * '<S154>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller2/Tsamp - Ngain'
 * '<S155>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller2/postSat Signal'
 * '<S156>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller2/preSat Signal'
 * '<S157>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller2/Anti-windup/Disc. Clamping Parallel'
 * '<S158>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller2/Anti-windup/Disc. Clamping Parallel/Dead Zone'
 * '<S159>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller2/Anti-windup/Disc. Clamping Parallel/Dead Zone/Enabled'
 * '<S160>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller2/D Gain/Disabled'
 * '<S161>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller2/Filter/Disabled'
 * '<S162>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller2/Filter ICs/Disabled'
 * '<S163>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller2/I Gain/External Parameters'
 * '<S164>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller2/Ideal P Gain/Passthrough'
 * '<S165>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller2/Ideal P Gain Fdbk/Disabled'
 * '<S166>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller2/Integrator/Discrete'
 * '<S167>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller2/Integrator ICs/Internal IC'
 * '<S168>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller2/N Copy/Disabled wSignal Specification'
 * '<S169>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller2/N Gain/Disabled'
 * '<S170>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller2/P Copy/Disabled'
 * '<S171>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller2/Parallel P Gain/External Parameters'
 * '<S172>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller2/Reset Signal/Disabled'
 * '<S173>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller2/Saturation/Enabled'
 * '<S174>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller2/Saturation Fdbk/Disabled'
 * '<S175>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller2/Sum/Sum_PI'
 * '<S176>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller2/Sum Fdbk/Disabled'
 * '<S177>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller2/Tracking Mode/Disabled'
 * '<S178>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller2/Tracking Mode Sum/Passthrough'
 * '<S179>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller2/Tsamp - Integral/Passthrough'
 * '<S180>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller2/Tsamp - Ngain/Passthrough'
 * '<S181>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller2/postSat Signal/Forward_Path'
 * '<S182>' : 'BLDC_SMO/FOC_Model/Curr_loop/idq_Controller/PID Controller2/preSat Signal/Forward_Path'
 * '<S183>' : 'BLDC_SMO/FOC_Model/Speed_loop/PID Controller3'
 * '<S184>' : 'BLDC_SMO/FOC_Model/Speed_loop/PID Controller3/Anti-windup'
 * '<S185>' : 'BLDC_SMO/FOC_Model/Speed_loop/PID Controller3/D Gain'
 * '<S186>' : 'BLDC_SMO/FOC_Model/Speed_loop/PID Controller3/Filter'
 * '<S187>' : 'BLDC_SMO/FOC_Model/Speed_loop/PID Controller3/Filter ICs'
 * '<S188>' : 'BLDC_SMO/FOC_Model/Speed_loop/PID Controller3/I Gain'
 * '<S189>' : 'BLDC_SMO/FOC_Model/Speed_loop/PID Controller3/Ideal P Gain'
 * '<S190>' : 'BLDC_SMO/FOC_Model/Speed_loop/PID Controller3/Ideal P Gain Fdbk'
 * '<S191>' : 'BLDC_SMO/FOC_Model/Speed_loop/PID Controller3/Integrator'
 * '<S192>' : 'BLDC_SMO/FOC_Model/Speed_loop/PID Controller3/Integrator ICs'
 * '<S193>' : 'BLDC_SMO/FOC_Model/Speed_loop/PID Controller3/N Copy'
 * '<S194>' : 'BLDC_SMO/FOC_Model/Speed_loop/PID Controller3/N Gain'
 * '<S195>' : 'BLDC_SMO/FOC_Model/Speed_loop/PID Controller3/P Copy'
 * '<S196>' : 'BLDC_SMO/FOC_Model/Speed_loop/PID Controller3/Parallel P Gain'
 * '<S197>' : 'BLDC_SMO/FOC_Model/Speed_loop/PID Controller3/Reset Signal'
 * '<S198>' : 'BLDC_SMO/FOC_Model/Speed_loop/PID Controller3/Saturation'
 * '<S199>' : 'BLDC_SMO/FOC_Model/Speed_loop/PID Controller3/Saturation Fdbk'
 * '<S200>' : 'BLDC_SMO/FOC_Model/Speed_loop/PID Controller3/Sum'
 * '<S201>' : 'BLDC_SMO/FOC_Model/Speed_loop/PID Controller3/Sum Fdbk'
 * '<S202>' : 'BLDC_SMO/FOC_Model/Speed_loop/PID Controller3/Tracking Mode'
 * '<S203>' : 'BLDC_SMO/FOC_Model/Speed_loop/PID Controller3/Tracking Mode Sum'
 * '<S204>' : 'BLDC_SMO/FOC_Model/Speed_loop/PID Controller3/Tsamp - Integral'
 * '<S205>' : 'BLDC_SMO/FOC_Model/Speed_loop/PID Controller3/Tsamp - Ngain'
 * '<S206>' : 'BLDC_SMO/FOC_Model/Speed_loop/PID Controller3/postSat Signal'
 * '<S207>' : 'BLDC_SMO/FOC_Model/Speed_loop/PID Controller3/preSat Signal'
 * '<S208>' : 'BLDC_SMO/FOC_Model/Speed_loop/PID Controller3/Anti-windup/Disc. Clamping Parallel'
 * '<S209>' : 'BLDC_SMO/FOC_Model/Speed_loop/PID Controller3/Anti-windup/Disc. Clamping Parallel/Dead Zone'
 * '<S210>' : 'BLDC_SMO/FOC_Model/Speed_loop/PID Controller3/Anti-windup/Disc. Clamping Parallel/Dead Zone/Enabled'
 * '<S211>' : 'BLDC_SMO/FOC_Model/Speed_loop/PID Controller3/D Gain/Disabled'
 * '<S212>' : 'BLDC_SMO/FOC_Model/Speed_loop/PID Controller3/Filter/Disabled'
 * '<S213>' : 'BLDC_SMO/FOC_Model/Speed_loop/PID Controller3/Filter ICs/Disabled'
 * '<S214>' : 'BLDC_SMO/FOC_Model/Speed_loop/PID Controller3/I Gain/Internal Parameters'
 * '<S215>' : 'BLDC_SMO/FOC_Model/Speed_loop/PID Controller3/Ideal P Gain/Passthrough'
 * '<S216>' : 'BLDC_SMO/FOC_Model/Speed_loop/PID Controller3/Ideal P Gain Fdbk/Disabled'
 * '<S217>' : 'BLDC_SMO/FOC_Model/Speed_loop/PID Controller3/Integrator/Discrete'
 * '<S218>' : 'BLDC_SMO/FOC_Model/Speed_loop/PID Controller3/Integrator ICs/Internal IC'
 * '<S219>' : 'BLDC_SMO/FOC_Model/Speed_loop/PID Controller3/N Copy/Disabled wSignal Specification'
 * '<S220>' : 'BLDC_SMO/FOC_Model/Speed_loop/PID Controller3/N Gain/Disabled'
 * '<S221>' : 'BLDC_SMO/FOC_Model/Speed_loop/PID Controller3/P Copy/Disabled'
 * '<S222>' : 'BLDC_SMO/FOC_Model/Speed_loop/PID Controller3/Parallel P Gain/Internal Parameters'
 * '<S223>' : 'BLDC_SMO/FOC_Model/Speed_loop/PID Controller3/Reset Signal/External Reset'
 * '<S224>' : 'BLDC_SMO/FOC_Model/Speed_loop/PID Controller3/Saturation/Enabled'
 * '<S225>' : 'BLDC_SMO/FOC_Model/Speed_loop/PID Controller3/Saturation Fdbk/Disabled'
 * '<S226>' : 'BLDC_SMO/FOC_Model/Speed_loop/PID Controller3/Sum/Sum_PI'
 * '<S227>' : 'BLDC_SMO/FOC_Model/Speed_loop/PID Controller3/Sum Fdbk/Disabled'
 * '<S228>' : 'BLDC_SMO/FOC_Model/Speed_loop/PID Controller3/Tracking Mode/Disabled'
 * '<S229>' : 'BLDC_SMO/FOC_Model/Speed_loop/PID Controller3/Tracking Mode Sum/Passthrough'
 * '<S230>' : 'BLDC_SMO/FOC_Model/Speed_loop/PID Controller3/Tsamp - Integral/Passthrough'
 * '<S231>' : 'BLDC_SMO/FOC_Model/Speed_loop/PID Controller3/Tsamp - Ngain/Passthrough'
 * '<S232>' : 'BLDC_SMO/FOC_Model/Speed_loop/PID Controller3/postSat Signal/Forward_Path'
 * '<S233>' : 'BLDC_SMO/FOC_Model/Speed_loop/PID Controller3/preSat Signal/Forward_Path'
 */
#endif                                 /* RTW_HEADER_FOC_Model_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
