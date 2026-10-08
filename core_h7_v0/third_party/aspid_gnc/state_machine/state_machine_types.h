/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: state_machine_types.h
 *
 * Code generated for Simulink model 'state_machine'.
 *
 * Model version                  : 3.20
 * Simulink Coder version         : 25.2 (R2025b) 28-Jul-2025
 * C/C++ source code generated on : Tue Oct  6 00:35:58 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef state_machine_types_h_
#define state_machine_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_tel_bus_
#define DEFINED_TYPEDEF_FOR_tel_bus_

/* Auto */
typedef struct {
  uint8_T cmd_id;
  real32_T cmd_param;
  uint8_T cmd_valid;
} tel_bus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_sm_bus_in_
#define DEFINED_TYPEDEF_FOR_sm_bus_in_

/* Auto */
typedef struct {
  real32_T pos_ned[3];
  real32_T vel_ned[3];
  real32_T accel_body[3];
  real32_T gyro_body[3];
  real32_T alt_agl_m;
  real32_T gnss_age_s;
  uint8_T nav_valid;
  real_T timestamp_s;
  tel_bus telemetry_cmd;
} sm_bus_in;

#endif

#ifndef DEFINED_TYPEDEF_FOR_sm_tun_bus_
#define DEFINED_TYPEDEF_FOR_sm_tun_bus_

/* Auto */
typedef struct {
  real32_T launch_accel_g_thr;
} sm_tun_bus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_sm_state_
#define DEFINED_TYPEDEF_FOR_sm_state_

/* Auto */
typedef struct {
  int32_T state;
  int32_T prev_state;
  real_T t_entry_s;
  uint16_T launch_cnt;
  uint16_T err_cnt;
  uint16_T stall_cnt;
  uint8_T nav_ever_ok;
  uint8_T error_code;
  real_T t_prev_s;
  uint8_T init_done;
} sm_state;

#endif

#ifndef DEFINED_TYPEDEF_FOR_sm_bus_out_
#define DEFINED_TYPEDEF_FOR_sm_bus_out_

/* Auto */
typedef struct {
  int32_T rocket_state;
  int32_T prev_state;
  real32_T time_in_state_s;
  uint8_T error_code;
  real_T timestamp_s;
  uint8_T nav_cmd;
} sm_bus_out;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_L66pHEsgunnpVonnRi0qeC_
#define DEFINED_TYPEDEF_FOR_struct_L66pHEsgunnpVonnRi0qeC_

typedef struct {
  real_T dt;
  real_T g0;
  real_T launch_accel_g_thr;
  real_T launch_debounce_ms;
  uint16_T launch_debounce_n;
  real_T nav_timeout_s;
  uint16_T nav_timeout_n;
  real_T nav_boot_timeout_s;
  uint16_T nav_boot_timeout_n;
  real_T stall_timeout_s;
  uint16_T stall_timeout_n;
  uint8_T check_nav_in_maint;
} struct_L66pHEsgunnpVonnRi0qeC;

#endif
#endif                                 /* state_machine_types_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
