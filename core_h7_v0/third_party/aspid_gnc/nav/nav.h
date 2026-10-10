/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: nav.h
 *
 * Code generated for Simulink model 'nav'.
 *
 * Model version                  : 1.51
 * Simulink Coder version         : 25.2 (R2025b) 28-Jul-2025
 * C/C++ source code generated on : Sat Oct 10 20:59:19 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef nav_h_
#define nav_h_
#ifndef nav_COMMON_INCLUDES_
#define nav_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rt_nonfinite.h"
#include "math.h"
#endif                                 /* nav_COMMON_INCLUDES_ */

#include "nav_types.h"
#include "rtGetInf.h"
#include "rtGetNaN.h"

/* Block states (default storage) for system '<Root>' */
typedef struct {
  nav_state NavState_Memory_PreviousInput;/* '<Root>/NavState_Memory' */
} DW_nav_T;

/* Constant parameters (default storage) */
typedef struct {
  /* Pooled Parameter (Expression: nav_data)
   * Referenced by:
   *   '<Root>/EKF_Prediction'
   *   '<Root>/EKF_Update'
   *   '<Root>/INS_Mechanization'
   */
  struct_bU50xOUnLbFnDvmIdHw8x pooled1;

  /* Expression: nav_state0
   * Referenced by: '<Root>/NavState_Memory'
   */
  nav_state NavState_Memory_InitialConditio;
} ConstP_nav_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  nav_bus_in nav_in;                   /* '<Root>/nav_in' */
} ExtU_nav_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  nav_bus_out nav_out;                 /* '<Root>/nav_out' */
} ExtY_nav_T;

/* Block states (default storage) */
extern DW_nav_T nav_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_nav_T nav_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_nav_T nav_Y;

/* Constant parameters (default storage) */
extern const ConstP_nav_T nav_ConstP;

/*
 * Exported Global Parameters
 *
 * Note: Exported global parameters are tunable parameters with an exported
 * global storage class designation.  Code generation will declare the memory for
 * these parameters and exports their symbols.
 *
 */
extern nav_tun_bus nav_tun;            /* Variable: nav_tun
                                        * Referenced by: '<Root>/INS_Mechanization'
                                        */

/* Model entry point functions */
extern void nav_initialize(void);
extern void nav_step(void);

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'nav'
 * '<S1>'   : 'nav/EKF_Prediction'
 * '<S2>'   : 'nav/EKF_Update'
 * '<S3>'   : 'nav/INS_Mechanization'
 */
#endif                                 /* nav_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
