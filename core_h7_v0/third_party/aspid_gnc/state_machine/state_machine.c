/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: state_machine.c
 *
 * Code generated for Simulink model 'state_machine'.
 *
 * Model version                  : 3.48
 * Simulink Coder version         : 25.2 (R2025b) 28-Jul-2025
 * C/C++ source code generated on : Sat Oct 17 22:54:51 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "state_machine.h"
#include <math.h>
#include "rt_nonfinite.h"
#include "rtwtypes.h"
#include "state_machine_types.h"

/* Exported block parameters */
sm_tun_bus sm_tun = {
  3.0F
} ;                                    /* Variable: sm_tun
                                        * Referenced by: '<Root>/SM_Logic'
                                        */

/* Block states (default storage) */
DW_state_machine_T state_machine_DW;

/* External inputs (root inport signals with default storage) */
ExtU_state_machine_T state_machine_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_state_machine_T state_machine_Y;

/* Model step function */
void state_machine_step(void)
{
  real_T x_tmp[3];
  real_T absxk;
  real_T accel_mag;
  real_T rtb_sm_state_out_t_entry_s;
  real_T scale;
  real_T t;
  real_T t_in_state;
  int32_T b_k;
  int32_T exitg1;
  int32_T new_state;
  int32_T rtb_sm_state_out_prev_state;
  int32_T rtb_sm_state_out_state;
  int32_T state;
  uint16_T rtb_sm_state_out_err_cnt;
  uint16_T rtb_sm_state_out_launch_cnt;
  uint16_T rtb_sm_state_out_stall_cnt;
  uint16_T tmp;
  uint8_T cmd;
  uint8_T fault_code;
  uint8_T rtb_sm_state_out_error_code;
  uint8_T rtb_sm_state_out_init_done;
  uint8_T rtb_sm_state_out_nav_ever_ok;
  uint8_T rtb_sm_state_out_restore_done;
  boolean_T c[3];
  boolean_T d[3];
  boolean_T exitg2;
  boolean_T y;

  /* MATLAB Function: '<Root>/SM_Logic' incorporates:
   *  BusCreator generated from: '<Root>/SM_Logic'
   *  Inport: '<Root>/sm_in'
   *  Memory: '<Root>/SMState_Memory'
   */
  rtb_sm_state_out_state = state_machine_DW.SMState_Memory_PreviousInput.state;
  rtb_sm_state_out_prev_state =
    state_machine_DW.SMState_Memory_PreviousInput.prev_state;
  rtb_sm_state_out_t_entry_s =
    state_machine_DW.SMState_Memory_PreviousInput.t_entry_s;
  rtb_sm_state_out_launch_cnt =
    state_machine_DW.SMState_Memory_PreviousInput.launch_cnt;
  rtb_sm_state_out_err_cnt =
    state_machine_DW.SMState_Memory_PreviousInput.err_cnt;
  rtb_sm_state_out_stall_cnt =
    state_machine_DW.SMState_Memory_PreviousInput.stall_cnt;
  rtb_sm_state_out_nav_ever_ok =
    state_machine_DW.SMState_Memory_PreviousInput.nav_ever_ok;
  rtb_sm_state_out_error_code =
    state_machine_DW.SMState_Memory_PreviousInput.error_code;
  t_in_state = state_machine_DW.SMState_Memory_PreviousInput.t_prev_s;
  rtb_sm_state_out_init_done =
    state_machine_DW.SMState_Memory_PreviousInput.init_done;
  rtb_sm_state_out_restore_done =
    state_machine_DW.SMState_Memory_PreviousInput.restore_done;
  if (state_machine_DW.SMState_Memory_PreviousInput.init_done == 0) {
    rtb_sm_state_out_state = 0;
    rtb_sm_state_out_prev_state = 0;
    rtb_sm_state_out_t_entry_s = state_machine_U.sm_in.timestamp_s;
    rtb_sm_state_out_launch_cnt = 0U;
    rtb_sm_state_out_err_cnt = 0U;
    rtb_sm_state_out_stall_cnt = 0U;
    rtb_sm_state_out_nav_ever_ok = 0U;
    rtb_sm_state_out_error_code = 0U;
    t_in_state = state_machine_U.sm_in.timestamp_s;
    rtb_sm_state_out_init_done = 1U;
  }

  state = rtb_sm_state_out_state;
  new_state = rtb_sm_state_out_state;
  if ((state_machine_U.sm_in.restore.valid == 1) &&
      (state_machine_DW.SMState_Memory_PreviousInput.restore_done == 0) &&
      (state_machine_U.sm_in.timestamp_s <= 2.0) &&
      (state_machine_U.sm_in.restore.rocket_state >= 0) &&
      (state_machine_U.sm_in.restore.rocket_state <= 3)) {
    state = state_machine_U.sm_in.restore.rocket_state;
    new_state = state_machine_U.sm_in.restore.rocket_state;
    rtb_sm_state_out_state = state_machine_U.sm_in.restore.rocket_state;
    rtb_sm_state_out_prev_state = state_machine_U.sm_in.restore.rocket_state;
    rtb_sm_state_out_t_entry_s = state_machine_U.sm_in.timestamp_s;
    rtb_sm_state_out_init_done = 1U;
    rtb_sm_state_out_restore_done = 1U;
    if (state_machine_U.sm_in.restore.rocket_state == 2) {
      rtb_sm_state_out_nav_ever_ok = 1U;
    }
  }

  scale = 3.3121686421112381E-170;
  x_tmp[0] = state_machine_U.sm_in.accel_body[0];
  absxk = fabs(state_machine_U.sm_in.accel_body[0]);
  if (absxk > 3.3121686421112381E-170) {
    accel_mag = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    accel_mag = t * t;
  }

  x_tmp[1] = state_machine_U.sm_in.accel_body[1];
  absxk = fabs(state_machine_U.sm_in.accel_body[1]);
  if (absxk > scale) {
    t = scale / absxk;
    accel_mag = accel_mag * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    accel_mag += t * t;
  }

  x_tmp[2] = state_machine_U.sm_in.accel_body[2];
  absxk = fabs(state_machine_U.sm_in.accel_body[2]);
  if (absxk > scale) {
    t = scale / absxk;
    accel_mag = accel_mag * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    accel_mag += t * t;
  }

  accel_mag = scale * sqrt(accel_mag);
  if (rtIsNaN(accel_mag)) {
    b_k = 0;
    do {
      exitg1 = 0;
      if (b_k < 3) {
        if (rtIsNaN(x_tmp[b_k])) {
          exitg1 = 1;
        } else {
          b_k++;
        }
      } else {
        accel_mag = (rtInf);
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  c[0] = !rtIsInf(state_machine_U.sm_in.pos_ned[0]);
  d[0] = !rtIsNaN(state_machine_U.sm_in.pos_ned[0]);
  c[1] = !rtIsInf(state_machine_U.sm_in.pos_ned[1]);
  d[1] = !rtIsNaN(state_machine_U.sm_in.pos_ned[1]);
  c[2] = !rtIsInf(state_machine_U.sm_in.pos_ned[2]);
  d[2] = !rtIsNaN(state_machine_U.sm_in.pos_ned[2]);
  y = true;
  b_k = 0;
  exitg2 = false;
  while ((!exitg2) && (b_k < 3)) {
    if ((!c[b_k]) || (!d[b_k])) {
      y = false;
      exitg2 = true;
    } else {
      b_k++;
    }
  }

  if (y) {
    c[0] = !rtIsInf(state_machine_U.sm_in.vel_ned[0]);
    d[0] = !rtIsNaN(state_machine_U.sm_in.vel_ned[0]);
    c[1] = !rtIsInf(state_machine_U.sm_in.vel_ned[1]);
    d[1] = !rtIsNaN(state_machine_U.sm_in.vel_ned[1]);
    c[2] = !rtIsInf(state_machine_U.sm_in.vel_ned[2]);
    d[2] = !rtIsNaN(state_machine_U.sm_in.vel_ned[2]);
    y = true;
    b_k = 0;
    exitg2 = false;
    while ((!exitg2) && (b_k < 3)) {
      if ((!c[b_k]) || (!d[b_k])) {
        y = false;
        exitg2 = true;
      } else {
        b_k++;
      }
    }

    if (y) {
      c[0] = !rtIsInf(state_machine_U.sm_in.accel_body[0]);
      d[0] = !rtIsNaN(state_machine_U.sm_in.accel_body[0]);
      c[1] = !rtIsInf(state_machine_U.sm_in.accel_body[1]);
      d[1] = !rtIsNaN(state_machine_U.sm_in.accel_body[1]);
      c[2] = !rtIsInf(state_machine_U.sm_in.accel_body[2]);
      d[2] = !rtIsNaN(state_machine_U.sm_in.accel_body[2]);
      y = true;
      b_k = 0;
      exitg2 = false;
      while ((!exitg2) && (b_k < 3)) {
        if ((!c[b_k]) || (!d[b_k])) {
          y = false;
          exitg2 = true;
        } else {
          b_k++;
        }
      }

      y = (y && ((!rtIsInf(state_machine_U.sm_in.timestamp_s)) && (!rtIsNaN
             (state_machine_U.sm_in.timestamp_s))));
    } else {
      y = false;
    }
  } else {
    y = false;
  }

  cmd = 0U;
  if (state_machine_U.sm_in.telemetry_cmd.cmd_valid == 1) {
    cmd = state_machine_U.sm_in.telemetry_cmd.cmd_id;
  }

  if (state_machine_U.sm_in.nav_valid == 0) {
    if (rtb_sm_state_out_err_cnt < 65535) {
      rtb_sm_state_out_err_cnt++;
    }
  } else {
    rtb_sm_state_out_err_cnt = 0U;
    rtb_sm_state_out_nav_ever_ok = 1U;
  }

  if (state_machine_U.sm_in.timestamp_s <= t_in_state) {
    if (rtb_sm_state_out_stall_cnt < 65535) {
      rtb_sm_state_out_stall_cnt++;
    }
  } else {
    rtb_sm_state_out_stall_cnt = 0U;
  }

  fault_code = 0U;
  if (!y) {
    fault_code = 3U;
  } else if (rtb_sm_state_out_stall_cnt >= 208) {
    fault_code = 2U;
  } else {
    if (rtb_sm_state_out_nav_ever_ok == 1) {
      tmp = 416U;
    } else {
      tmp = 24960U;
    }

    if (rtb_sm_state_out_err_cnt >= tmp) {
      fault_code = 1U;
    }
  }

  if (accel_mag / 9.80665 >= sm_tun.launch_accel_g_thr) {
    if (rtb_sm_state_out_launch_cnt < 65535) {
      rtb_sm_state_out_launch_cnt++;
    }
  } else {
    rtb_sm_state_out_launch_cnt = 0U;
  }

  t_in_state = state_machine_U.sm_in.timestamp_s - rtb_sm_state_out_t_entry_s;
  switch (state) {
   case 0:
    if (rtb_sm_state_out_launch_cnt >= 25) {
      new_state = 2;
    } else if (cmd == 255) {
      new_state = 3;
      rtb_sm_state_out_error_code = 4U;
    } else if (fault_code != 0) {
      new_state = 3;
      rtb_sm_state_out_error_code = fault_code;
    } else if (cmd == 1) {
      new_state = 1;
    }
    break;

   case 1:
    if (cmd == 255) {
      new_state = 3;
      rtb_sm_state_out_error_code = 4U;
    } else if (cmd == 2) {
      new_state = 0;
    }
    break;

   case 2:
    if ((fault_code != 0) && (rtb_sm_state_out_error_code == 0)) {
      rtb_sm_state_out_error_code = fault_code;
    }
    break;

   case 3:
    if (rtb_sm_state_out_launch_cnt >= 25) {
      new_state = 2;
    } else if (cmd == 3) {
      new_state = 0;
      rtb_sm_state_out_error_code = 0U;
      rtb_sm_state_out_err_cnt = 0U;
      rtb_sm_state_out_stall_cnt = 0U;
    }
    break;

   default:
    new_state = 3;
    rtb_sm_state_out_error_code = 3U;
    break;
  }

  if (new_state != state) {
    rtb_sm_state_out_prev_state = state;
    rtb_sm_state_out_state = new_state;
    rtb_sm_state_out_t_entry_s = state_machine_U.sm_in.timestamp_s;
    rtb_sm_state_out_launch_cnt = 0U;
    t_in_state = 0.0;
  }

  /* Outport: '<Root>/sm_out' incorporates:
   *  MATLAB Function: '<Root>/SM_Logic'
   */
  state_machine_Y.sm_out.nav_cmd = 0U;

  /* MATLAB Function: '<Root>/SM_Logic' */
  if (cmd == 11) {
    /* Outport: '<Root>/sm_out' */
    state_machine_Y.sm_out.nav_cmd = 11U;
  } else if ((cmd == 10) && ((rtb_sm_state_out_state == 0) ||
              (rtb_sm_state_out_state == 1))) {
    /* Outport: '<Root>/sm_out' */
    state_machine_Y.sm_out.nav_cmd = 10U;
  }

  /* Outport: '<Root>/sm_out' incorporates:
   *  BusCreator generated from: '<Root>/SM_Logic'
   *  Inport: '<Root>/sm_in'
   *  MATLAB Function: '<Root>/SM_Logic'
   */
  state_machine_Y.sm_out.rocket_state = rtb_sm_state_out_state;
  state_machine_Y.sm_out.prev_state = rtb_sm_state_out_prev_state;
  state_machine_Y.sm_out.time_in_state_s = (real32_T)t_in_state;
  state_machine_Y.sm_out.error_code = rtb_sm_state_out_error_code;
  state_machine_Y.sm_out.timestamp_s = state_machine_U.sm_in.timestamp_s;

  /* Update for Memory: '<Root>/SMState_Memory' incorporates:
   *  BusCreator generated from: '<Root>/SM_Logic'
   *  Inport: '<Root>/sm_in'
   */
  state_machine_DW.SMState_Memory_PreviousInput.state = rtb_sm_state_out_state;
  state_machine_DW.SMState_Memory_PreviousInput.prev_state =
    rtb_sm_state_out_prev_state;
  state_machine_DW.SMState_Memory_PreviousInput.t_entry_s =
    rtb_sm_state_out_t_entry_s;
  state_machine_DW.SMState_Memory_PreviousInput.launch_cnt =
    rtb_sm_state_out_launch_cnt;
  state_machine_DW.SMState_Memory_PreviousInput.err_cnt =
    rtb_sm_state_out_err_cnt;
  state_machine_DW.SMState_Memory_PreviousInput.stall_cnt =
    rtb_sm_state_out_stall_cnt;
  state_machine_DW.SMState_Memory_PreviousInput.nav_ever_ok =
    rtb_sm_state_out_nav_ever_ok;
  state_machine_DW.SMState_Memory_PreviousInput.error_code =
    rtb_sm_state_out_error_code;
  state_machine_DW.SMState_Memory_PreviousInput.t_prev_s =
    state_machine_U.sm_in.timestamp_s;
  state_machine_DW.SMState_Memory_PreviousInput.init_done =
    rtb_sm_state_out_init_done;
  state_machine_DW.SMState_Memory_PreviousInput.restore_done =
    rtb_sm_state_out_restore_done;
}

/* Model initialize function */
void state_machine_initialize(void)
{
  /* (no initialization code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
