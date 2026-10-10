/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: nav_types.h
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

#ifndef nav_types_h_
#define nav_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_lg_imu_bus_
#define DEFINED_TYPEDEF_FOR_lg_imu_bus_

/* Auto */
typedef struct {
  real32_T accel_mps2[3];
  real32_T gyro_rps[3];
  real32_T temp_C;
  real_T timestamp_s;
  uint8_T data_valid;
} lg_imu_bus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_hg_imu_bus_
#define DEFINED_TYPEDEF_FOR_hg_imu_bus_

/* Auto */
typedef struct {
  real32_T accel_mps2[3];
  real_T timestamp_s;
  uint8_T data_valid;
} hg_imu_bus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_gnss_bus_
#define DEFINED_TYPEDEF_FOR_gnss_bus_

/* Auto */
typedef struct {
  real_T pos_lla[3];
  real32_T vel_ned[3];
  uint8_T fix_type;
  uint8_T num_sats;
  uint8_T data_valid;
} gnss_bus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_baro_bus_
#define DEFINED_TYPEDEF_FOR_baro_bus_

/* Auto */
typedef struct {
  real32_T press_pa;
  real32_T alt_msl_m;
  real32_T temp_C;
  real_T timestamp_s;
  uint8_T data_valid;
} baro_bus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_nav_bus_in_
#define DEFINED_TYPEDEF_FOR_nav_bus_in_

/* Auto */
typedef struct {
  lg_imu_bus imu_lowg;
  hg_imu_bus imu_highg;
  gnss_bus gnss;
  baro_bus baro;
  int32_T rocket_state;
  uint8_T nav_cmd;
} nav_bus_in;

#endif

#ifndef DEFINED_TYPEDEF_FOR_nav_state_
#define DEFINED_TYPEDEF_FOR_nav_state_

/* Auto */
typedef struct {
  real32_T quat_nb[4];
  real32_T pos_ned[3];
  real32_T vel_ned[3];
  real32_T accel_ned[3];
  real32_T accel_body[3];
  real32_T gyro_body[3];
  real32_T accel_bias[3];
  real32_T gyro_bias[3];
  real32_T P[225];
  real32_T imu_blend_w;
  real_T origin_lla[3];
  uint8_T origin_set;
  real32_T baro_ref_pa;
  uint8_T baro_ref_set;
  uint16_T align_cnt;
  real32_T acc_sum[3];
  real32_T gyr_sum[3];
  real_T baro_sum;
  real32_T gnss_age_s;
  uint16_T reject_cnt;
  uint16_T baro_rej_cnt;
  uint8_T gnss_only;
  uint16_T descent_cnt;
  real_T timestamp_s;
  uint8_T init_done;
  real_T gnss_sum[3];
  uint16_T gnss_cnt;
  uint8_T align_ok;
} nav_state;

#endif

#ifndef DEFINED_TYPEDEF_FOR_nav_bus_out_
#define DEFINED_TYPEDEF_FOR_nav_bus_out_

/* Auto */
typedef struct {
  real32_T pos_ned[3];
  real32_T vel_ned[3];
  real32_T quat_nb[4];
  real32_T euler_rpy[3];
  real32_T accel_ned[3];
  real32_T accel_body[3];
  real32_T gyro_body[3];
  real32_T accel_bias[3];
  real32_T gyro_bias[3];
  real32_T alt_agl_m;
  real32_T imu_blend_w;
  real32_T gnss_age_s;
  real_T origin_lla[3];
  uint8_T gnss_only;
  real_T timestamp_s;
  uint8_T nav_valid;
  uint8_T nav_ready;
  uint8_T nav_init_flags;
} nav_bus_out;

#endif

#ifndef DEFINED_TYPEDEF_FOR_nav_tun_bus_
#define DEFINED_TYPEDEF_FOR_nav_tun_bus_

/* Auto */
typedef struct {
  real32_T rail_elev_deg;
  real32_T rail_azim_deg;
} nav_tun_bus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_bU50xOUnLbFnDvmIdHw8x_
#define DEFINED_TYPEDEF_FOR_struct_bU50xOUnLbFnDvmIdHw8x_

typedef struct {
  real_T dt;
  real_T g0;
  real_T gravity;
  real_T lowg_range_g;
  real_T highg_range_g;
  real_T blend_lo_g;
  real_T blend_hi_g;
  real_T lowg_sat_frac;
  real_T sigma_gyro_arw;
  real_T sigma_accel_vrw_lg;
  real_T sigma_accel_vrw_hg;
  real_T sigma_gyro_bias_rw;
  real_T sigma_accel_bias_rw;
  real_T sigma_accel_sf;
  real_T sigma_gyro_sf;
  real32_T R_gnss_pos[9];
  real32_T R_gnss_vel[9];
  real_T gate_chi2_pos;
  real_T gate_chi2_vel;
  uint8_T gnss_min_sats;
  uint8_T gnss_min_fix;
  real_T gnss_max_age_s;
  uint16_T gate_reject_max;
  real32_T gate_reset_pos_var;
  real32_T gate_reset_vel_var;
  real_T gnss_reset_age_s;
  real32_T R_baro_base;
  real32_T baro_port_cp;
  real32_T R_baro_q_coef;
  real_T gate_chi2_baro;
  real_T baro_mach_max;
  real_T isa_gamma;
  real_T alt_ref_msl_m;
  real_T baro_reject_s;
  uint16_T baro_reject_max;
  uint8_T baro_enable;
  real_T init_att_sigma_rad[3];
  real32_T P0[225];
  real_T align_time_s;
  real_T align_samples;
  real_T align_bg_drift_sigma;
  real32_T P_align[225];
  real_T align_accel_tol;
  real_T align_gyro_tol;
  real_T align_att_check_rad;
  real_T rail_survey_err_deg;
  real_T accel_bias_max_mg;
  real_T accel_bias_max;
  real_T gyro_bias_max;
  uint8_T gnss_only_after_apogee;
  real_T descent_debounce_s;
  uint16_T descent_debounce_n;
  real_T descent_vz_thr;
  real_T gnss_only_min_alt_m;
  real32_T lever_arm_gnss[3];
  uint8_T use_gnss_origin;
  real_T launch_lla[3];
  real_T rail_elev_deg;
  real_T rail_azim_deg;
} struct_bU50xOUnLbFnDvmIdHw8x;

#endif
#endif                                 /* nav_types_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
