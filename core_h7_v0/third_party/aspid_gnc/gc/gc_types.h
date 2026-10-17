/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: gc_types.h
 *
 * Code generated for Simulink model 'gc'.
 *
 * Model version                  : 3.48
 * Simulink Coder version         : 25.2 (R2025b) 28-Jul-2025
 * C/C++ source code generated on : Sat Oct 17 22:55:20 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef gc_types_h_
#define gc_types_h_
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

#ifndef DEFINED_TYPEDEF_FOR_restore_bus_
#define DEFINED_TYPEDEF_FOR_restore_bus_

/* Auto */
typedef struct {
  uint8_T valid;
  int32_T rocket_state;
  int32_T flight_phase;
} restore_bus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_gc_bus_in_
#define DEFINED_TYPEDEF_FOR_gc_bus_in_

/* Auto */
typedef struct {
  real32_T pos_ned[3];
  real32_T vel_ned[3];
  real32_T accel_ned[3];
  real32_T accel_body[3];
  real32_T alt_agl_m;
  real32_T gnss_age_s;
  uint8_T nav_valid;
  real_T timestamp_s;
  int32_T rocket_state;
  uint8_T error_code;
  tel_bus telemetry_cmd;
  restore_bus restore;
  uint8_T nav_init_flags;
  uint8_T gnss_only;
} gc_bus_in;

#endif

#ifndef DEFINED_TYPEDEF_FOR_gc_ctrl_bus_
#define DEFINED_TYPEDEF_FOR_gc_ctrl_bus_

/* Auto */
typedef struct {
  boolean_T ctrl_enable;
  boolean_T manual_mode;
  real32_T u_manual;
  real32_T err_eff;
  real32_T tgo_eff;
  real32_T proj_factor;
  real32_T a_closed;
  real32_T authority;
  real32_T a_meas;
  real32_T u_fallback;
} gc_ctrl_bus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_gc_tun_bus_
#define DEFINED_TYPEDEF_FOR_gc_tun_bus_

/* Auto */
typedef struct {
  real32_T apogee_target_agl_m;
  real32_T min_ctrl_alt_agl_m;
  real32_T main_deploy_alt_agl_m;
  real32_T drogue_delay_s;
  real32_T drogue_inhibit_s;
  real32_T drogue_timeout_s;
  real32_T deadband_m;
} gc_tun_bus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_gc_state_
#define DEFINED_TYPEDEF_FOR_gc_state_

/* Auto */
typedef struct {
  int32_T phase;
  real32_T u_manual;
  real32_T apogee_pred_m;
  real32_T max_alt_m;
  real_T t_phase_s;
  real_T t_prev_s;
  real_T t_apogee_s;
  real_T t_drogue_s;
  real_T t_main_s;
  uint8_T apogee_detected;
  uint8_T pyro_armed;
  uint8_T drogue_armed;
  uint8_T main_armed;
  uint16_T burnout_cnt;
  uint16_T apogee_cnt;
  uint16_T land_cnt;
  real_T t_test_drogue_s;
  real_T t_test_main_s;
  uint8_T init_done;
  uint8_T restore_done;
  real_T t_airborne_s;
  real_T t_barrido_s;
  real32_T min_ctrl_alt_m;
} gc_state;

#endif

#ifndef DEFINED_TYPEDEF_FOR_gc_bus_out_
#define DEFINED_TYPEDEF_FOR_gc_bus_out_

/* Auto */
typedef struct {
  real32_T actuator_cmd;
  real32_T airbrake_deg;
  real32_T apogee_pred_m;
  real32_T apogee_error_m;
  real32_T time_to_apogee_s;
  real32_T accel_cmd_mps2;
  real32_T accel_meas_mps2;
  uint8_T drogue_deploy_flag;
  uint8_T main_deploy_flag;
  int32_T flight_phase;
  uint8_T pred_valid;
  uint8_T pyro_armed;
  real_T timestamp_s;
  uint32_T status_word;
} gc_bus_out;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_rAm9KbC4xVIPKCNwU7tAqD_
#define DEFINED_TYPEDEF_FOR_struct_rAm9KbC4xVIPKCNwU7tAqD_

typedef struct {
  real_T kp_outer;
  real_T tgo_min_s;
  real_T restore_window_s;
  real_T barrido_abierto_s;
  real_T dt;
  real_T deadband_m;
  real_T kp_inner;
  real_T ki_inner;
  real_T integ_max;
  real_T authority_min;
  real_T u_min;
  real_T u_max;
  real_T u_rate_max;
  real_T airbrake_max_deg;
  real_T coast_lockout_s;
  real_T min_ctrl_alt_agl_m;
  real_T mach_ctrl_max;
  real_T v_ctrl_min_mps;
  real_T gnss_age_max_s;
  real_T apogee_target_agl_m;
  real_T min_boost_s;
  real_T burnout_decel_thr;
  real_T burnout_debounce_s;
  uint16_T burnout_debounce_n;
  real_T apogee_debounce_s;
  uint16_T apogee_debounce_n;
  real_T apogee_alt_drop_m;
  real_T drogue_delay_s;
  real_T drogue_inhibit_s;
  real_T drogue_timeout_s;
  real_T main_deploy_alt_agl_m;
  real_T pyro_pulse_s;
  real_T main_backup_delay_s;
  real_T pyro_min_alt_agl_m;
  real_T land_speed_thr_mps;
  real_T land_alt_thr_m;
  real_T land_debounce_s;
  uint16_T land_debounce_n;
} struct_rAm9KbC4xVIPKCNwU7tAqD;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_i7oMu3w6TXmEP8d3Zj9BkG_
#define DEFINED_TYPEDEF_FOR_struct_i7oMu3w6TXmEP8d3Zj9BkG_

typedef struct {
  real_T dt;
  int32_T max_steps;
  real_T mass_kg;
  real_T S_ref;
  real_T isa_T0;
  real_T isa_p0;
  real_T isa_L;
  real_T isa_R;
  real_T isa_gamma;
  real_T g0;
  real_T alt_ref_msl_m;
  real_T mach_min;
  real_T mach_step;
  int32_T n_mach;
  real32_T cd_clean[43];
  int32_T n_deploy;
  real32_T deploy_tbl[5];
  real32_T dcd_tbl[215];
} struct_i7oMu3w6TXmEP8d3Zj9BkG;

#endif
#endif                                 /* gc_types_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
