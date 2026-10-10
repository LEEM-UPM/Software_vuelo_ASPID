/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: state_machine.h
 *
 * Code generated for Simulink model 'state_machine'.
 *
 * Model version                  : 3.46
 * Simulink Coder version         : 25.2 (R2025b) 28-Jul-2025
 * C/C++ source code generated on : Sat Oct 10 20:59:58 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef state_machine_h_
#define state_machine_h_
#ifndef state_machine_COMMON_INCLUDES_
#define state_machine_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rt_nonfinite.h"
#include "math.h"
#endif                                 /* state_machine_COMMON_INCLUDES_ */

#include "state_machine_types.h"
#include "rtGetInf.h"

/* Block states (default storage) for system '<Root>' */
typedef struct {
  sm_state SMState_Memory_PreviousInput;/* '<Root>/SMState_Memory' */
} DW_state_machine_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  sm_bus_in sm_in;                     /* '<Root>/sm_in' */
} ExtU_state_machine_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  sm_bus_out sm_out;                   /* '<Root>/sm_out' */
} ExtY_state_machine_T;

/* Block states (default storage) */
extern DW_state_machine_T state_machine_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_state_machine_T state_machine_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_state_machine_T state_machine_Y;

/*
 * Exported Global Parameters
 *
 * Note: Exported global parameters are tunable parameters with an exported
 * global storage class designation.  Code generation will declare the memory for
 * these parameters and exports their symbols.
 *
 */
extern sm_tun_bus sm_tun;              /* Variable: sm_tun
                                        * Referenced by: '<Root>/SM_Logic'
                                        */

/* Model entry point functions */
extern void state_machine_initialize(void);
extern void state_machine_step(void);

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
 * '<Root>' : 'state_machine'
 * '<S1>'   : 'state_machine/SM_Logic'
 */
#endif                                 /* state_machine_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
