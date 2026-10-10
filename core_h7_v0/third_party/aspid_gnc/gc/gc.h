/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: gc.h
 *
 * Code generated for Simulink model 'gc'.
 *
 * Model version                  : 3.46
 * Simulink Coder version         : 25.2 (R2025b) 28-Jul-2025
 * C/C++ source code generated on : Sat Oct 10 21:00:39 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef gc_h_
#define gc_h_
#ifndef gc_COMMON_INCLUDES_
#define gc_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rt_nonfinite.h"
#include "math.h"
#endif                                 /* gc_COMMON_INCLUDES_ */

#include "gc_types.h"
#include "rtGetInf.h"
#include "rtGetNaN.h"

/* Block states (default storage) for system '<Root>' */
typedef struct {
  gc_state GCState_Memory_PreviousInput;/* '<Root>/GCState_Memory' */
  real_T PrevY;                        /* '<S1>/velocidad del servo' */
  real32_T Mandoanterior_DSTATE;       /* '<Root>/Mando anterior' */
  real32_T ki_inner_DSTATE;            /* '<S1>/ki_inner' */
  int8_T ki_inner_PrevResetState;      /* '<S1>/ki_inner' */
} DW_gc_T;

/* Constant parameters (default storage) */
typedef struct {
  /* Expression: pred
   * Referenced by: '<Root>/GC_Logic'
   */
  struct_i7oMu3w6TXmEP8d3Zj9BkG GC_Logic_pred;

  /* Expression: ctrl
   * Referenced by: '<Root>/GC_Logic'
   */
  struct_rAm9KbC4xVIPKCNwU7tAqD GC_Logic_ctrl;

  /* Expression: gc_state0
   * Referenced by: '<Root>/GCState_Memory'
   */
  gc_state GCState_Memory_InitialCondition;
} ConstP_gc_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  gc_bus_in gc_in;                     /* '<Root>/gc_in' */
} ExtU_gc_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  gc_bus_out gc_out;                   /* '<Root>/gc_out' */
} ExtY_gc_T;

/* Block states (default storage) */
extern DW_gc_T gc_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_gc_T gc_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_gc_T gc_Y;

/* Constant parameters (default storage) */
extern const ConstP_gc_T gc_ConstP;

/*
 * Exported Global Parameters
 *
 * Note: Exported global parameters are tunable parameters with an exported
 * global storage class designation.  Code generation will declare the memory for
 * these parameters and exports their symbols.
 *
 */
extern gc_tun_bus gc_tun;              /* Variable: gc_tun
                                        * Referenced by: '<Root>/GC_Logic'
                                        */

/* Model entry point functions */
extern void gc_initialize(void);
extern void gc_step(void);

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S1>/a_cmd single' : Eliminate redundant data type conversion
 */

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
 * '<Root>' : 'gc'
 * '<S1>'   : 'gc/Cascada'
 * '<S2>'   : 'gc/GC_Logic'
 */
#endif                                 /* gc_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
