/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: gc.c
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

#include "gc.h"
#include "rtwtypes.h"
#include "gc_types.h"
#include <math.h>
#include "rt_nonfinite.h"
#include "gc_private.h"

/* Exported block parameters */
gc_tun_bus gc_tun = {
  3000.0F,
  1500.0F,
  450.0F,
  0.5F,
  15.0F,
  30.0F,
  0.0F
} ;                                    /* Variable: gc_tun
                                        * Referenced by: '<Root>/GC_Logic'
                                        */

/* Block states (default storage) */
DW_gc_T gc_DW;

/* External inputs (root inport signals with default storage) */
ExtU_gc_T gc_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_gc_T gc_Y;

/* Forward declaration for local functions */
static real_T gc_norm(const real_T x[3]);
static void gc_isa_atmos(real_T alt_msl, real_T b_pred_isa_T0, real_T
  b_pred_isa_p0, real_T b_pred_isa_L, real_T b_pred_isa_R, real_T
  b_pred_isa_gamma, real_T b_pred_g0, real_T *rho, real_T *p, real_T *a_snd);
static real_T gc_dcd_lookup(real_T mach, real_T u, real_T b_pred_mach_min,
  real_T b_pred_mach_step, int32_T b_pred_n_mach, int32_T b_pred_n_deploy, const
  real32_T b_pred_deploy_tbl[5], const real32_T b_pred_dcd_tbl[215]);
static void gc_ballistic_deriv(real_T h_agl, const real_T v[3], real_T u, const
  struct_i7oMu3w6TXmEP8d3Zj9BkG *b_pred, real_T *dh, real_T dv[3]);
static void gc_propagate_apogee(real_T alt_agl0, const real_T vel0[3], real_T u,
  const struct_i7oMu3w6TXmEP8d3Zj9BkG *b_pred, const
  struct_rAm9KbC4xVIPKCNwU7tAqD *b_ctrl, real_T *apogee_agl, real_T *t_to_apogee,
  uint8_T *pred_valid);
static real_T gc_cd_clean_lookup(real_T mach, real_T b_pred_mach_min, real_T
  b_pred_mach_step, int32_T b_pred_n_mach, const real32_T b_pred_cd_clean[43]);
static uint32_T gc_empaquetar_estado(int32_T fase, int32_T modo, uint8_T fallo,
  uint8_T nav_flags, boolean_T nav_valid, boolean_T gnss_fresco, boolean_T
  gnss_only, boolean_T pyro, boolean_T apogeo, boolean_T lazo, boolean_T manual,
  boolean_T pred_ok, boolean_T restaurado, boolean_T inhibido, boolean_T
  freno_fallo, boolean_T barrido, boolean_T timeout);

/* Function for MATLAB Function: '<Root>/GC_Logic' */
static real_T gc_norm(const real_T x[3])
{
  real_T absxk;
  real_T scale;
  real_T t;
  real_T y;
  scale = 3.3121686421112381E-170;
  absxk = fabs(x[0]);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }

  absxk = fabs(x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  absxk = fabs(x[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  y = scale * sqrt(y);
  if (rtIsNaN(y)) {
    int32_T b_k;
    b_k = 0;
    int32_T exitg1;
    do {
      exitg1 = 0;
      if (b_k < 3) {
        if (rtIsNaN(x[b_k])) {
          exitg1 = 1;
        } else {
          b_k++;
        }
      } else {
        y = (rtInf);
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }

  return y;
}

real_T rt_powd_snf(real_T u0, real_T u1)
{
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else {
    real_T tmp;
    real_T tmp_0;
    tmp = fabs(u0);
    tmp_0 = fabs(u1);
    if (rtIsInf(u1)) {
      if (tmp == 1.0) {
        y = 1.0;
      } else if (tmp > 1.0) {
        if (u1 > 0.0) {
          y = (rtInf);
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = (rtInf);
      }
    } else if (tmp_0 == 0.0) {
      y = 1.0;
    } else if (tmp_0 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = (rtNaN);
    } else {
      y = pow(u0, u1);
    }
  }

  return y;
}

/* Function for MATLAB Function: '<Root>/GC_Logic' */
static void gc_isa_atmos(real_T alt_msl, real_T b_pred_isa_T0, real_T
  b_pred_isa_p0, real_T b_pred_isa_L, real_T b_pred_isa_R, real_T
  b_pred_isa_gamma, real_T b_pred_g0, real_T *rho, real_T *p, real_T *a_snd)
{
  real_T T;
  T = b_pred_isa_L * alt_msl + b_pred_isa_T0;
  if (T < 216.65) {
    T = 216.65;
  }

  *p = rt_powd_snf(T / b_pred_isa_T0, -b_pred_g0 / (b_pred_isa_L * b_pred_isa_R))
    * b_pred_isa_p0;
  *rho = *p / (b_pred_isa_R * T);
  *a_snd = sqrt(b_pred_isa_gamma * b_pred_isa_R * T);
}

/* Function for MATLAB Function: '<Root>/GC_Logic' */
static real_T gc_dcd_lookup(real_T mach, real_T u, real_T b_pred_mach_min,
  real_T b_pred_mach_step, int32_T b_pred_n_mach, int32_T b_pred_n_deploy, const
  real32_T b_pred_deploy_tbl[5], const real32_T b_pred_dcd_tbl[215])
{
  real_T dcd_tmp;
  real_T i0;
  real_T uu;
  real_T x;
  int32_T ia;
  int32_T jd;
  int32_T k;
  real32_T tmp;
  x = fmin(fmax((mach - b_pred_mach_min) / b_pred_mach_step, 0.0), (real_T)
           b_pred_n_mach - 1.0);
  i0 = floor(x);
  if (i0 > (real_T)b_pred_n_mach - 2.0) {
    i0 = (real_T)b_pred_n_mach - 2.0;
  }

  x -= i0;
  if (i0 >= -2.147483648E+9) {
    ia = (int32_T)i0;
  } else {
    ia = MIN_int32_T;
  }

  uu = fmin(fmax(u, 0.0), 1.0);
  jd = 1;
  for (k = 0; k <= b_pred_n_deploy - 2; k++) {
    if (uu >= b_pred_deploy_tbl[k]) {
      if ((uint32_T)k + 1U < 2147483648U) {
        jd = (int32_T)((uint32_T)k + 1U);
      } else {
        jd = MAX_int32_T;
      }
    }
  }

  if ((real_T)b_pred_n_deploy - 1.0 >= -2.147483648E+9) {
    k = b_pred_n_deploy - 1;
  } else {
    k = MIN_int32_T;
  }

  if (jd > k) {
    if ((real_T)b_pred_n_deploy - 1.0 >= -2.147483648E+9) {
      jd = b_pred_n_deploy - 1;
    } else {
      jd = MIN_int32_T;
    }
  }

  i0 = 0.0;
  tmp = b_pred_deploy_tbl[jd - 1];
  if (b_pred_deploy_tbl[jd] > tmp) {
    i0 = (uu - tmp) / ((real_T)b_pred_deploy_tbl[jd] - tmp);
  }

  k = (jd - 1) * 43 + ia;
  uu = b_pred_dcd_tbl[k];
  uu += (b_pred_dcd_tbl[k + 1] - uu) * x;
  ia += 43 * jd;
  dcd_tmp = b_pred_dcd_tbl[ia];
  return (((b_pred_dcd_tbl[ia + 1] - dcd_tmp) * x + dcd_tmp) - uu) * i0 + uu;
}

/* Function for MATLAB Function: '<Root>/GC_Logic' */
static void gc_ballistic_deriv(real_T h_agl, const real_T v[3], real_T u, const
  struct_i7oMu3w6TXmEP8d3Zj9BkG *b_pred, real_T *dh, real_T dv[3])
{
  real_T T;
  real_T i0;
  real_T mach;
  real_T sp;
  real_T x;
  int32_T ia;
  T = (b_pred->alt_ref_msl_m + h_agl) * b_pred->isa_L + b_pred->isa_T0;
  if (T < 216.65) {
    T = 216.65;
  }

  sp = gc_norm(v);
  mach = sp / sqrt(b_pred->isa_gamma * b_pred->isa_R * T);
  x = fmin(fmax((mach - b_pred->mach_min) / b_pred->mach_step, 0.0), (real_T)
           b_pred->n_mach - 1.0);
  i0 = floor(x);
  if (i0 > (real_T)b_pred->n_mach - 2.0) {
    i0 = (real_T)b_pred->n_mach - 2.0;
  }

  if (i0 >= -2.147483648E+9) {
    ia = (int32_T)i0;
  } else {
    ia = MIN_int32_T;
  }

  if (sp > 1.0E-6) {
    T = -(rt_powd_snf(T / b_pred->isa_T0, -b_pred->g0 / (b_pred->isa_L *
            b_pred->isa_R)) * b_pred->isa_p0 / (b_pred->isa_R * T) * 0.5 * sp *
          b_pred->S_ref * ((((real_T)b_pred->cd_clean[ia + 1] - b_pred->
             cd_clean[ia]) * (x - i0) + b_pred->cd_clean[ia]) + gc_dcd_lookup
           (mach, u, b_pred->mach_min, b_pred->mach_step, b_pred->n_mach,
            b_pred->n_deploy, b_pred->deploy_tbl, b_pred->dcd_tbl)) /
          b_pred->mass_kg);
    dv[0] = T * v[0];
    dv[1] = T * v[1];
    T *= v[2];
  } else {
    dv[0] = 0.0;
    dv[1] = 0.0;
    T = 0.0;
  }

  dv[2] = T + b_pred->g0;
  *dh = -v[2];
}

/* Function for MATLAB Function: '<Root>/GC_Logic' */
static void gc_propagate_apogee(real_T alt_agl0, const real_T vel0[3], real_T u,
  const struct_i7oMu3w6TXmEP8d3Zj9BkG *b_pred, const
  struct_rAm9KbC4xVIPKCNwU7tAqD *b_ctrl, real_T *apogee_agl, real_T *t_to_apogee,
  uint8_T *pred_valid)
{
  real_T dv1[3];
  real_T dv2[3];
  real_T v[3];
  real_T v_0[3];
  real_T den;
  real_T frac;
  real_T h;
  real_T h_old;
  real_T u_step;
  real_T vd_old;
  int32_T b_k;
  int32_T k;
  boolean_T b[3];
  boolean_T c[3];
  boolean_T exitg1;
  boolean_T exitg2;
  boolean_T y;
  h = alt_agl0;
  v[0] = vel0[0];
  v[1] = vel0[1];
  v[2] = vel0[2];
  *apogee_agl = alt_agl0;
  *t_to_apogee = 0.0;
  *pred_valid = 0U;
  if (-vel0[2] <= 0.0) {
    *pred_valid = 1U;
  } else {
    k = 0;
    exitg1 = false;
    while ((!exitg1) && (k <= b_pred->max_steps - 1)) {
      h_old = h;
      vd_old = v[2];
      u_step = u;
      if (-v[2] <= b_ctrl->v_ctrl_min_mps) {
        u_step = b_ctrl->u_min;
      }

      gc_ballistic_deriv(h, v, u_step, b_pred, &frac, dv1);
      v_0[0] = b_pred->dt * dv1[0] + v[0];
      v_0[1] = b_pred->dt * dv1[1] + v[1];
      v_0[2] = b_pred->dt * dv1[2] + v[2];
      gc_ballistic_deriv(h + b_pred->dt * frac, v_0, u_step, b_pred, &den, dv2);
      u_step = 0.5 * b_pred->dt;
      h += (frac + den) * u_step;
      v[0] += (dv1[0] + dv2[0]) * u_step;
      v[1] += (dv1[1] + dv2[1]) * u_step;
      v[2] += (dv1[2] + dv2[2]) * u_step;
      if (v[2] >= 0.0) {
        den = v[2] - vd_old;
        frac = 1.0;
        if (fabs(den) > 1.0E-9) {
          frac = fmin(fmax(-vd_old / den, 0.0), 1.0);
        }

        *apogee_agl = (h - h_old) * frac + h_old;
        *t_to_apogee = ((((real_T)k + 1.0) - 1.0) + frac) * b_pred->dt;
        *pred_valid = 1U;
        exitg1 = true;
      } else if (rtIsInf(h) || rtIsNaN(h)) {
        *t_to_apogee = ((real_T)k + 1.0) * b_pred->dt;
        exitg1 = true;
      } else {
        b[0] = !rtIsInf(v[0]);
        c[0] = !rtIsNaN(v[0]);
        b[1] = !rtIsInf(v[1]);
        c[1] = !rtIsNaN(v[1]);
        b[2] = !rtIsInf(v[2]);
        c[2] = !rtIsNaN(v[2]);
        y = true;
        b_k = 0;
        exitg2 = false;
        while ((!exitg2) && (b_k < 3)) {
          if ((!b[b_k]) || (!c[b_k])) {
            y = false;
            exitg2 = true;
          } else {
            b_k++;
          }
        }

        if (!y) {
          *t_to_apogee = ((real_T)k + 1.0) * b_pred->dt;
          exitg1 = true;
        } else {
          *apogee_agl = h;
          *t_to_apogee = ((real_T)k + 1.0) * b_pred->dt;
          k++;
        }
      }
    }
  }
}

/* Function for MATLAB Function: '<Root>/GC_Logic' */
static real_T gc_cd_clean_lookup(real_T mach, real_T b_pred_mach_min, real_T
  b_pred_mach_step, int32_T b_pred_n_mach, const real32_T b_pred_cd_clean[43])
{
  real_T i0;
  real_T x;
  int32_T ia;
  x = fmin(fmax((mach - b_pred_mach_min) / b_pred_mach_step, 0.0), (real_T)
           b_pred_n_mach - 1.0);
  i0 = floor(x);
  if (i0 > (real_T)b_pred_n_mach - 2.0) {
    i0 = (real_T)b_pred_n_mach - 2.0;
  }

  if (i0 >= -2.147483648E+9) {
    ia = (int32_T)i0;
  } else {
    ia = MIN_int32_T;
  }

  return ((real_T)b_pred_cd_clean[ia + 1] - b_pred_cd_clean[ia]) * (x - i0) +
    b_pred_cd_clean[ia];
}

/* Function for MATLAB Function: '<Root>/GC_Logic' */
static uint32_T gc_empaquetar_estado(int32_T fase, int32_T modo, uint8_T fallo,
  uint8_T nav_flags, boolean_T nav_valid, boolean_T gnss_fresco, boolean_T
  gnss_only, boolean_T pyro, boolean_T apogeo, boolean_T lazo, boolean_T manual,
  boolean_T pred_ok, boolean_T restaurado, boolean_T inhibido, boolean_T
  freno_fallo, boolean_T barrido, boolean_T timeout)
{
  int32_T tmp;
  int32_T tmp_0;
  uint8_T tmp_1;
  if (fase >= 0) {
    tmp = fase;
  } else {
    tmp = 0;
  }

  if (modo >= 0) {
    tmp_0 = modo;
  } else {
    tmp_0 = 0;
  }

  if (fallo <= 7) {
    tmp_1 = fallo;
  } else {
    tmp_1 = 7U;
  }

  return ((uint32_T)tmp_0 & 3U) << 3 | ((uint32_T)tmp & 7U) | (uint32_T)tmp_1 <<
    5 | (nav_flags & 15U) << 8 | (uint32_T)((nav_flags & 7U) == 7U) << 12 |
    (uint32_T)nav_valid << 13 | (uint32_T)gnss_fresco << 14 | (uint32_T)
    gnss_only << 15 | (uint32_T)pyro << 16 | (uint32_T)apogeo << 17 | (uint32_T)
    lazo << 18 | (uint32_T)manual << 19 | (uint32_T)pred_ok << 20 | (uint32_T)
    restaurado << 21 | (uint32_T)inhibido << 22 | (uint32_T)freno_fallo << 23 |
    (uint32_T)barrido << 24 | (uint32_T)timeout << 25;
}

/* Model step function */
void gc_step(void)
{
  real_T vel[3];
  real_T a_along;
  real_T apogee_err;
  real_T apogee_pred;
  real_T cmd_param;
  real_T err_eff;
  real_T mach;
  real_T proj;
  real_T rtb_gc_state_out_t_airborne_s;
  real_T rtb_gc_state_out_t_apogee_s;
  real_T rtb_gc_state_out_t_barrido_s;
  real_T rtb_gc_state_out_t_drogue_s;
  real_T rtb_gc_state_out_t_main_s;
  real_T rtb_gc_state_out_t_phase_s;
  real_T rtb_gc_state_out_t_test_drogue_;
  real_T rtb_gc_state_out_t_test_main_s;
  real_T speed;
  real_T t_to_apogee;
  int32_T new_phase;
  int32_T phase;
  int32_T rtb_gc_state_out_phase;
  real32_T rtb_correcciondeproyeccion;
  real32_T rtb_gc_ctrl_a_meas;
  real32_T rtb_gc_ctrl_authority;
  real32_T rtb_gc_ctrl_proj_factor;
  real32_T rtb_gc_ctrl_tgo_eff;
  real32_T rtb_gc_ctrl_u_fallback;
  real32_T rtb_gc_ctrl_u_manual;
  real32_T rtb_gc_state_out_max_alt_m;
  real32_T rtb_gc_state_out_u_manual;
  real32_T rtb_normalizarporautoridad;
  uint16_T rtb_gc_state_out_apogee_cnt;
  uint16_T rtb_gc_state_out_burnout_cnt;
  uint16_T rtb_gc_state_out_land_cnt;
  uint8_T cmd;
  uint8_T pred_ok;
  uint8_T rtb_gc_state_out_apogee_detecte;
  uint8_T rtb_gc_state_out_drogue_armed;
  uint8_T rtb_gc_state_out_init_done;
  uint8_T rtb_gc_state_out_main_armed;
  uint8_T rtb_gc_state_out_pyro_armed;
  uint8_T rtb_gc_state_out_restore_done;
  boolean_T apogeo_evaluable;
  boolean_T barrido_activo;
  boolean_T freno_por_fallo;
  boolean_T rtb_controldeshabilitado;
  boolean_T rtb_gc_ctrl_ctrl_enable;
  boolean_T rtb_gc_ctrl_manual_mode;
  boolean_T timeout_drogue;

  /* MATLAB Function: '<Root>/GC_Logic' incorporates:
   *  BusCreator generated from: '<Root>/GC_Logic'
   *  Inport: '<Root>/gc_in'
   *  Memory: '<Root>/GCState_Memory'
   */
  rtb_gc_state_out_phase = gc_DW.GCState_Memory_PreviousInput.phase;
  rtb_gc_state_out_u_manual = gc_DW.GCState_Memory_PreviousInput.u_manual;
  rtb_gc_state_out_max_alt_m = gc_DW.GCState_Memory_PreviousInput.max_alt_m;
  rtb_gc_state_out_t_phase_s = gc_DW.GCState_Memory_PreviousInput.t_phase_s;
  rtb_gc_state_out_t_apogee_s = gc_DW.GCState_Memory_PreviousInput.t_apogee_s;
  rtb_gc_state_out_t_drogue_s = gc_DW.GCState_Memory_PreviousInput.t_drogue_s;
  rtb_gc_state_out_t_main_s = gc_DW.GCState_Memory_PreviousInput.t_main_s;
  rtb_gc_state_out_apogee_detecte =
    gc_DW.GCState_Memory_PreviousInput.apogee_detected;
  rtb_gc_state_out_pyro_armed = gc_DW.GCState_Memory_PreviousInput.pyro_armed;
  rtb_gc_state_out_drogue_armed =
    gc_DW.GCState_Memory_PreviousInput.drogue_armed;
  rtb_gc_state_out_main_armed = gc_DW.GCState_Memory_PreviousInput.main_armed;
  rtb_gc_state_out_burnout_cnt = gc_DW.GCState_Memory_PreviousInput.burnout_cnt;
  rtb_gc_state_out_apogee_cnt = gc_DW.GCState_Memory_PreviousInput.apogee_cnt;
  rtb_gc_state_out_land_cnt = gc_DW.GCState_Memory_PreviousInput.land_cnt;
  rtb_gc_state_out_t_test_drogue_ =
    gc_DW.GCState_Memory_PreviousInput.t_test_drogue_s;
  rtb_gc_state_out_t_test_main_s =
    gc_DW.GCState_Memory_PreviousInput.t_test_main_s;
  rtb_gc_state_out_init_done = gc_DW.GCState_Memory_PreviousInput.init_done;
  rtb_gc_state_out_restore_done =
    gc_DW.GCState_Memory_PreviousInput.restore_done;
  rtb_gc_state_out_t_airborne_s =
    gc_DW.GCState_Memory_PreviousInput.t_airborne_s;
  rtb_gc_state_out_t_barrido_s = gc_DW.GCState_Memory_PreviousInput.t_barrido_s;
  if (gc_DW.GCState_Memory_PreviousInput.init_done == 0) {
    rtb_gc_state_out_phase = 0;
    rtb_gc_state_out_t_phase_s = gc_U.gc_in.timestamp_s;
    rtb_gc_state_out_init_done = 1U;
  }

  if ((gc_U.gc_in.restore.valid == 1) &&
      (gc_DW.GCState_Memory_PreviousInput.restore_done == 0) &&
      (gc_U.gc_in.timestamp_s <= 2.0) && (gc_U.gc_in.rocket_state == 2) &&
      (gc_U.gc_in.restore.flight_phase >= 0) && (gc_U.gc_in.restore.flight_phase
       <= 7)) {
    rtb_gc_state_out_phase = gc_U.gc_in.restore.flight_phase;
    rtb_gc_state_out_t_phase_s = gc_U.gc_in.timestamp_s;
    rtb_gc_state_out_init_done = 1U;
    rtb_gc_state_out_restore_done = 1U;
    if (gc_U.gc_in.restore.flight_phase >= 2) {
      rtb_gc_state_out_pyro_armed = 1U;
      rtb_gc_state_out_max_alt_m = 100.0F;
      rtb_gc_state_out_t_airborne_s = (gc_U.gc_in.timestamp_s -
        gc_tun.drogue_inhibit_s) - 1.0;
    }

    if (gc_U.gc_in.restore.flight_phase >= 4) {
      rtb_gc_state_out_apogee_detecte = 1U;
      rtb_gc_state_out_t_apogee_s = gc_U.gc_in.timestamp_s;
    }

    if (gc_U.gc_in.restore.flight_phase >= 5) {
      rtb_gc_state_out_drogue_armed = 1U;
      rtb_gc_state_out_t_drogue_s = gc_U.gc_in.timestamp_s;
    }

    if (gc_U.gc_in.restore.flight_phase >= 6) {
      rtb_gc_state_out_main_armed = 1U;
      rtb_gc_state_out_t_main_s = gc_U.gc_in.timestamp_s;
    }
  }

  phase = rtb_gc_state_out_phase;
  new_phase = rtb_gc_state_out_phase;
  vel[0] = gc_U.gc_in.vel_ned[0];
  vel[1] = gc_U.gc_in.vel_ned[1];
  vel[2] = gc_U.gc_in.vel_ned[2];
  speed = gc_norm(vel);
  gc_isa_atmos((real_T)gc_U.gc_in.alt_agl_m, 288.15, 101325.0, -0.0065, 287.05,
               1.4, 9.80665, &a_along, &apogee_pred, &apogee_err);
  mach = speed / apogee_err;
  if (gc_U.gc_in.alt_agl_m > rtb_gc_state_out_max_alt_m) {
    rtb_gc_state_out_max_alt_m = gc_U.gc_in.alt_agl_m;
  }

  rtb_controldeshabilitado = ((gc_U.gc_in.nav_valid == 1) &&
    (gc_U.gc_in.gnss_age_s <= 1.0F));
  if (rtb_gc_state_out_max_alt_m >= 100.0F) {
    rtb_gc_state_out_pyro_armed = 1U;
  }

  cmd = 0U;
  cmd_param = 0.0;
  if (gc_U.gc_in.telemetry_cmd.cmd_valid == 1) {
    cmd = gc_U.gc_in.telemetry_cmd.cmd_id;
    cmd_param = gc_U.gc_in.telemetry_cmd.cmd_param;
  }

  apogee_pred = gc_DW.GCState_Memory_PreviousInput.apogee_pred_m;
  apogee_err = 0.0;
  t_to_apogee = 0.0;

  /* Outport: '<Root>/gc_out' incorporates:
   *  MATLAB Function: '<Root>/GC_Logic'
   */
  gc_Y.gc_out.drogue_deploy_flag = 0U;

  /* MATLAB Function: '<Root>/GC_Logic' */
  freno_por_fallo = false;
  barrido_activo = false;
  timeout_drogue = false;

  /* Outport: '<Root>/gc_out' incorporates:
   *  MATLAB Function: '<Root>/GC_Logic'
   */
  gc_Y.gc_out.main_deploy_flag = 0U;

  /* MATLAB Function: '<Root>/GC_Logic' incorporates:
   *  BusCreator generated from: '<Root>/GC_Logic'
   *  Inport: '<Root>/gc_in'
   *  Memory: '<Root>/GCState_Memory'
   *  UnitDelay: '<Root>/Mando anterior'
   */
  pred_ok = 1U;
  rtb_gc_ctrl_ctrl_enable = false;
  rtb_gc_ctrl_manual_mode = false;
  rtb_gc_ctrl_u_manual = 0.0F;
  rtb_correcciondeproyeccion = 0.0F;
  rtb_gc_ctrl_tgo_eff = 4.0F;
  rtb_gc_ctrl_proj_factor = 1.0F;
  rtb_normalizarporautoridad = 0.0F;
  rtb_gc_ctrl_authority = 1.0F;
  rtb_gc_ctrl_a_meas = (real32_T)-(real_T)gc_U.gc_in.accel_body[0];
  rtb_gc_ctrl_u_fallback = 0.0F;
  switch (gc_U.gc_in.rocket_state) {
   case 0:
    new_phase = 0;
    rtb_gc_state_out_max_alt_m = 0.0F;
    rtb_gc_state_out_apogee_detecte = 0U;
    rtb_gc_state_out_pyro_armed = 0U;
    rtb_gc_state_out_drogue_armed = 0U;
    rtb_gc_state_out_main_armed = 0U;
    rtb_gc_state_out_burnout_cnt = 0U;
    rtb_gc_state_out_apogee_cnt = 0U;
    rtb_gc_state_out_land_cnt = 0U;
    rtb_gc_state_out_t_test_drogue_ = -1.0;
    rtb_gc_state_out_t_test_main_s = -1.0;
    rtb_gc_state_out_u_manual = 0.0F;
    if ((gc_DW.GCState_Memory_PreviousInput.t_barrido_s < 0.0) &&
        (rtb_gc_state_out_restore_done == 0) && ((gc_U.gc_in.nav_init_flags & 1U)
         != 0U)) {
      rtb_gc_state_out_t_barrido_s = gc_U.gc_in.timestamp_s;
    }

    if ((rtb_gc_state_out_t_barrido_s >= 0.0) && (gc_U.gc_in.timestamp_s -
         rtb_gc_state_out_t_barrido_s < 0.3425)) {
      rtb_gc_ctrl_u_fallback = 0.9F;
      barrido_activo = true;
    }
    break;

   case 1:
    new_phase = 0;
    if (cmd == 4) {
      rtb_gc_state_out_u_manual = (real32_T)fmin(fmax(cmd_param, 0.0), 0.9);
    }

    rtb_gc_ctrl_manual_mode = true;
    rtb_gc_ctrl_u_manual = rtb_gc_state_out_u_manual;
    if (cmd == 5) {
      rtb_gc_state_out_t_test_drogue_ = gc_U.gc_in.timestamp_s;
    }

    if (cmd == 6) {
      rtb_gc_state_out_t_test_main_s = gc_U.gc_in.timestamp_s;
    }

    if ((rtb_gc_state_out_t_test_drogue_ >= 0.0) && (gc_U.gc_in.timestamp_s -
         rtb_gc_state_out_t_test_drogue_ < 1.0)) {
      /* Outport: '<Root>/gc_out' */
      gc_Y.gc_out.drogue_deploy_flag = 1U;
    }

    if ((rtb_gc_state_out_t_test_main_s >= 0.0) && (gc_U.gc_in.timestamp_s -
         rtb_gc_state_out_t_test_main_s < 1.0)) {
      /* Outport: '<Root>/gc_out' */
      gc_Y.gc_out.main_deploy_flag = 1U;
    }
    break;

   case 2:
    if (rtb_gc_state_out_phase == 0) {
      phase = 1;
      new_phase = 1;
      rtb_gc_state_out_t_airborne_s = gc_U.gc_in.timestamp_s;
    }

    cmd_param = gc_U.gc_in.timestamp_s - rtb_gc_state_out_t_airborne_s;
    timeout_drogue = (cmd_param >= gc_tun.drogue_inhibit_s);
    apogeo_evaluable = ((rtb_gc_state_out_pyro_armed == 1) && timeout_drogue);
    switch (phase) {
     case 1:
      a_along = 0.0;
      if (speed > 5.0) {
        a_along = (((real_T)gc_U.gc_in.accel_ned[0] * gc_U.gc_in.vel_ned[0] +
                    (real_T)gc_U.gc_in.accel_ned[1] * gc_U.gc_in.vel_ned[1]) +
                   (real_T)gc_U.gc_in.accel_ned[2] * gc_U.gc_in.vel_ned[2]) /
          speed;
      }

      speed = gc_U.gc_in.timestamp_s - rtb_gc_state_out_t_phase_s;
      if ((speed >= 1.0) && (a_along <= -2.0)) {
        if (gc_DW.GCState_Memory_PreviousInput.burnout_cnt < 65535) {
          rtb_gc_state_out_burnout_cnt = (uint16_T)
            (gc_DW.GCState_Memory_PreviousInput.burnout_cnt + 1);
        }
      } else {
        rtb_gc_state_out_burnout_cnt = 0U;
      }

      if (rtb_gc_state_out_burnout_cnt >= 5) {
        new_phase = 2;
      }

      if ((speed >= 1.0) && (-(real_T)gc_U.gc_in.vel_ned[2] <= 0.0) &&
          apogeo_evaluable) {
        new_phase = 4;
      }
      break;

     case 2:
      if ((-(real_T)gc_U.gc_in.vel_ned[2] <= 0.0) && apogeo_evaluable) {
        new_phase = 4;
      } else if ((gc_U.gc_in.timestamp_s - rtb_gc_state_out_t_phase_s >= 1.0) &&
                 (gc_U.gc_in.alt_agl_m >= gc_tun.min_ctrl_alt_agl_m) && (mach <=
                  1.05) && (-(real_T)gc_U.gc_in.vel_ned[2] >= 30.0) &&
                 rtb_controldeshabilitado) {
        new_phase = 3;
      }
      break;

     case 3:
      gc_propagate_apogee((real_T)gc_U.gc_in.alt_agl_m, vel, (real_T)
                          gc_DW.Mandoanterior_DSTATE, &gc_ConstP.GC_Logic_pred,
                          &gc_ConstP.GC_Logic_ctrl, &apogee_pred, &t_to_apogee,
                          &pred_ok);
      apogee_err = apogee_pred - gc_tun.apogee_target_agl_m;
      err_eff = 0.0;
      if (apogee_err > gc_tun.deadband_m) {
        err_eff = apogee_err - gc_tun.deadband_m;
      } else if (apogee_err < -(real_T)gc_tun.deadband_m) {
        err_eff = apogee_err + gc_tun.deadband_m;
      }

      if ((pred_ok == 0) || (!rtb_controldeshabilitado)) {
        err_eff = 0.0;
      }

      proj = 1.0;
      if (-(real_T)gc_U.gc_in.vel_ned[2] > 1.0) {
        proj = speed / -(real_T)gc_U.gc_in.vel_ned[2];
      }

      speed = 0.5 * a_along * speed * speed * 0.013273228961416878;
      a_along = speed * gc_dcd_lookup(mach, 0.9, 0.0, 0.05, 43, 5,
        &gc_ConstP.GC_Logic_pred.deploy_tbl[0],
        &gc_ConstP.GC_Logic_pred.dcd_tbl[0]) / 20.98 / 0.9;
      rtb_gc_ctrl_ctrl_enable = (a_along > 0.5);
      if ((!rtb_gc_ctrl_ctrl_enable) && (err_eff > 0.0)) {
        rtb_gc_ctrl_u_fallback = 0.9F;
      }

      rtb_correcciondeproyeccion = (real32_T)err_eff;
      rtb_gc_ctrl_tgo_eff = (real32_T)fmax(t_to_apogee, 4.0);
      rtb_gc_ctrl_proj_factor = (real32_T)proj;
      rtb_normalizarporautoridad = (real32_T)(speed * gc_cd_clean_lookup(mach,
        0.0, 0.05, 43, &gc_ConstP.GC_Logic_pred.cd_clean[0]) / 20.98);
      rtb_gc_ctrl_authority = (real32_T)fmax(a_along, 0.5);
      rtb_gc_ctrl_a_meas = (real32_T)-(real_T)gc_U.gc_in.accel_body[0];
      if (-(real_T)gc_U.gc_in.vel_ned[2] <= 30.0) {
        new_phase = 4;
      }
      break;

     case 4:
      if (rtb_gc_state_out_apogee_detecte == 0) {
        if (apogeo_evaluable && ((-(real_T)gc_U.gc_in.vel_ned[2] <= 0.0) ||
             ((real_T)rtb_gc_state_out_max_alt_m - gc_U.gc_in.alt_agl_m >= 5.0)))
        {
          if (gc_DW.GCState_Memory_PreviousInput.apogee_cnt < 65535) {
            rtb_gc_state_out_apogee_cnt = (uint16_T)
              (gc_DW.GCState_Memory_PreviousInput.apogee_cnt + 1);
          }
        } else {
          rtb_gc_state_out_apogee_cnt = 0U;
        }

        if (rtb_gc_state_out_apogee_cnt >= 5) {
          rtb_gc_state_out_apogee_detecte = 1U;
          rtb_gc_state_out_t_apogee_s = gc_U.gc_in.timestamp_s;
        }
      }

      if ((rtb_gc_state_out_apogee_detecte == 1) && (rtb_gc_state_out_pyro_armed
           == 1) && timeout_drogue && (gc_U.gc_in.timestamp_s -
           rtb_gc_state_out_t_apogee_s >= gc_tun.drogue_delay_s)) {
        new_phase = 5;
        rtb_gc_state_out_drogue_armed = 1U;
        rtb_gc_state_out_t_drogue_s = gc_U.gc_in.timestamp_s;
      }
      break;

     case 5:
      speed = gc_U.gc_in.timestamp_s - rtb_gc_state_out_t_drogue_s;
      if (speed < 1.0) {
        /* Outport: '<Root>/gc_out' */
        gc_Y.gc_out.drogue_deploy_flag = 1U;
      }

      if ((speed >= 150.0) || ((gc_U.gc_in.alt_agl_m <=
            gc_tun.main_deploy_alt_agl_m) && (-(real_T)gc_U.gc_in.vel_ned[2] <
            0.0))) {
        new_phase = 6;
        rtb_gc_state_out_main_armed = 1U;
        rtb_gc_state_out_t_main_s = gc_U.gc_in.timestamp_s;
      }
      break;

     case 6:
      if (gc_U.gc_in.timestamp_s - rtb_gc_state_out_t_main_s < 1.0) {
        /* Outport: '<Root>/gc_out' */
        gc_Y.gc_out.main_deploy_flag = 1U;
      }

      if ((speed <= 2.0) && (gc_U.gc_in.alt_agl_m <= 30.0F)) {
        if (gc_DW.GCState_Memory_PreviousInput.land_cnt < 65535) {
          rtb_gc_state_out_land_cnt = (uint16_T)
            (gc_DW.GCState_Memory_PreviousInput.land_cnt + 1);
        }
      } else {
        rtb_gc_state_out_land_cnt = 0U;
      }

      if (rtb_gc_state_out_land_cnt >= 260) {
        new_phase = 7;
      }
      break;

     case 7:
      break;

     default:
      new_phase = 4;
      break;
    }

    if (rtb_gc_state_out_restore_done == 1) {
      cmd_param = (cmd_param - gc_tun.drogue_inhibit_s) - 1.0;
    }

    timeout_drogue = (cmd_param >= gc_tun.drogue_timeout_s);
    if (timeout_drogue && (rtb_gc_state_out_pyro_armed == 1) &&
        (rtb_gc_state_out_apogee_detecte == 0) && (rtb_gc_state_out_drogue_armed
         == 0)) {
      new_phase = 4;
      rtb_gc_state_out_apogee_detecte = 1U;
      rtb_gc_state_out_t_apogee_s = gc_U.gc_in.timestamp_s;
      rtb_gc_ctrl_ctrl_enable = false;
      rtb_gc_ctrl_u_fallback = 0.0F;
    }

    if ((gc_U.gc_in.error_code != 0) && (rtb_gc_state_out_drogue_armed == 0)) {
      rtb_gc_ctrl_ctrl_enable = false;
      rtb_gc_ctrl_u_fallback = 0.9F;
      freno_por_fallo = true;
    }
    break;

   case 3:
    new_phase = 0;
    if ((rtb_gc_state_out_pyro_armed == 1) && (rtb_gc_state_out_drogue_armed ==
         0)) {
      rtb_gc_ctrl_u_fallback = 0.9F;
      freno_por_fallo = true;
    }
    break;

   default:
    new_phase = 0;
    break;
  }

  if (new_phase != rtb_gc_state_out_phase) {
    rtb_gc_state_out_phase = new_phase;
    rtb_gc_state_out_t_phase_s = gc_U.gc_in.timestamp_s;
    rtb_gc_state_out_burnout_cnt = 0U;
  }

  /* Product: '<S1>/correccion de proyeccion' incorporates:
   *  Gain: '<S1>/kp_outer por 2'
   *  Math: '<S1>/tgo al cuadrado'
   *  Product: '<S1>/programada con tgo'
   */
  rtb_correcciondeproyeccion = 60.0F * rtb_correcciondeproyeccion /
    (rtb_gc_ctrl_tgo_eff * rtb_gc_ctrl_tgo_eff) * rtb_gc_ctrl_proj_factor;

  /* Product: '<S1>/normalizar por autoridad' incorporates:
   *  Sum: '<S1>/error de aceleracion'
   *  Sum: '<S1>/referencia de aceleracion'
   */
  rtb_normalizarporautoridad = ((rtb_normalizarporautoridad +
    rtb_correcciondeproyeccion) - rtb_gc_ctrl_a_meas) / rtb_gc_ctrl_authority;

  /* Logic: '<S1>/control deshabilitado' */
  rtb_controldeshabilitado = !rtb_gc_ctrl_ctrl_enable;

  /* DiscreteIntegrator: '<S1>/ki_inner' */
  if (rtb_controldeshabilitado || (gc_DW.ki_inner_PrevResetState != 0)) {
    gc_DW.ki_inner_DSTATE = 0.0F;
  }

  /* Switch: '<S1>/manual o automatico' incorporates:
   *  Switch: '<S1>/automatico o cerrado'
   */
  if (rtb_gc_ctrl_manual_mode) {
    speed = rtb_gc_ctrl_u_manual;
  } else if (rtb_gc_ctrl_ctrl_enable) {
    /* Sum: '<S1>/anticipacion mas PI' incorporates:
     *  DiscreteIntegrator: '<S1>/ki_inner'
     *  Gain: '<S1>/kp_inner'
     *  Product: '<S1>/anticipacion'
     *  Switch: '<S1>/automatico o cerrado'
     */
    rtb_gc_ctrl_u_manual = (rtb_correcciondeproyeccion / rtb_gc_ctrl_authority +
      0.6F * rtb_normalizarporautoridad) + gc_DW.ki_inner_DSTATE;

    /* Saturate: '<S1>/recorrido del freno' incorporates:
     *  Switch: '<S1>/automatico o cerrado'
     */
    if (rtb_gc_ctrl_u_manual > 0.9F) {
      speed = 0.89999997615814209;
    } else if (rtb_gc_ctrl_u_manual < 0.0F) {
      speed = 0.0;
    } else {
      speed = rtb_gc_ctrl_u_manual;
    }

    /* End of Saturate: '<S1>/recorrido del freno' */
  } else {
    /* Switch: '<S1>/automatico o cerrado' incorporates:
     *  DataTypeConversion: '<S1>/freno cerrado'
     */
    speed = rtb_gc_ctrl_u_fallback;
  }

  /* End of Switch: '<S1>/manual o automatico' */

  /* RateLimiter: '<S1>/velocidad del servo' */
  cmd_param = speed - gc_DW.PrevY;
  if (cmd_param > 0.059171597633136092) {
    speed = gc_DW.PrevY + 0.059171597633136092;
  } else if (cmd_param < -0.059171597633136092) {
    speed = gc_DW.PrevY - 0.059171597633136092;
  }

  gc_DW.PrevY = speed;

  /* End of RateLimiter: '<S1>/velocidad del servo' */

  /* DataTypeConversion: '<S1>/mando single' incorporates:
   *  UnitDelay: '<Root>/Mando anterior'
   */
  gc_DW.Mandoanterior_DSTATE = (real32_T)speed;

  /* Outport: '<Root>/gc_out' incorporates:
   *  BusAssignment: '<Root>/Bus Assignment'
   *  BusCreator generated from: '<Root>/GC_Logic'
   *  DataTypeConversion: '<S1>/grados single'
   *  Gain: '<S1>/a grados'
   *  Inport: '<Root>/gc_in'
   *  MATLAB Function: '<Root>/GC_Logic'
   *  UnitDelay: '<Root>/Mando anterior'
   */
  gc_Y.gc_out.actuator_cmd = gc_DW.Mandoanterior_DSTATE;
  gc_Y.gc_out.airbrake_deg = (real32_T)(45.0 * speed);
  gc_Y.gc_out.apogee_pred_m = (real32_T)apogee_pred;
  gc_Y.gc_out.apogee_error_m = (real32_T)apogee_err;
  gc_Y.gc_out.time_to_apogee_s = (real32_T)t_to_apogee;
  gc_Y.gc_out.accel_cmd_mps2 = rtb_correcciondeproyeccion;
  gc_Y.gc_out.accel_meas_mps2 = (real32_T)-(real_T)gc_U.gc_in.accel_body[0];
  gc_Y.gc_out.flight_phase = rtb_gc_state_out_phase;
  gc_Y.gc_out.pred_valid = pred_ok;
  gc_Y.gc_out.pyro_armed = rtb_gc_state_out_pyro_armed;
  gc_Y.gc_out.timestamp_s = gc_U.gc_in.timestamp_s;
  gc_Y.gc_out.status_word = gc_empaquetar_estado(rtb_gc_state_out_phase,
    gc_U.gc_in.rocket_state, gc_U.gc_in.error_code, gc_U.gc_in.nav_init_flags,
    (gc_U.gc_in.nav_valid == 1), (gc_U.gc_in.gnss_age_s <= 1.0F),
    (gc_U.gc_in.gnss_only == 1), (rtb_gc_state_out_pyro_armed == 1),
    (rtb_gc_state_out_apogee_detecte == 1), rtb_gc_ctrl_ctrl_enable,
    rtb_gc_ctrl_manual_mode, ((rtb_gc_state_out_phase == 3) && (pred_ok == 1)),
    (rtb_gc_state_out_restore_done == 1), ((gc_U.gc_in.rocket_state == 2) &&
    (gc_U.gc_in.timestamp_s - rtb_gc_state_out_t_airborne_s <
     gc_tun.drogue_inhibit_s)), freno_por_fallo, barrido_activo, timeout_drogue);

  /* Update for Memory: '<Root>/GCState_Memory' incorporates:
   *  BusCreator generated from: '<Root>/GC_Logic'
   *  Inport: '<Root>/gc_in'
   *  MATLAB Function: '<Root>/GC_Logic'
   */
  gc_DW.GCState_Memory_PreviousInput.phase = rtb_gc_state_out_phase;
  gc_DW.GCState_Memory_PreviousInput.u_manual = rtb_gc_state_out_u_manual;
  gc_DW.GCState_Memory_PreviousInput.apogee_pred_m = (real32_T)apogee_pred;
  gc_DW.GCState_Memory_PreviousInput.max_alt_m = rtb_gc_state_out_max_alt_m;
  gc_DW.GCState_Memory_PreviousInput.t_phase_s = rtb_gc_state_out_t_phase_s;
  gc_DW.GCState_Memory_PreviousInput.t_prev_s = gc_U.gc_in.timestamp_s;
  gc_DW.GCState_Memory_PreviousInput.t_apogee_s = rtb_gc_state_out_t_apogee_s;
  gc_DW.GCState_Memory_PreviousInput.t_drogue_s = rtb_gc_state_out_t_drogue_s;
  gc_DW.GCState_Memory_PreviousInput.t_main_s = rtb_gc_state_out_t_main_s;
  gc_DW.GCState_Memory_PreviousInput.apogee_detected =
    rtb_gc_state_out_apogee_detecte;
  gc_DW.GCState_Memory_PreviousInput.pyro_armed = rtb_gc_state_out_pyro_armed;
  gc_DW.GCState_Memory_PreviousInput.drogue_armed =
    rtb_gc_state_out_drogue_armed;
  gc_DW.GCState_Memory_PreviousInput.main_armed = rtb_gc_state_out_main_armed;
  gc_DW.GCState_Memory_PreviousInput.burnout_cnt = rtb_gc_state_out_burnout_cnt;
  gc_DW.GCState_Memory_PreviousInput.apogee_cnt = rtb_gc_state_out_apogee_cnt;
  gc_DW.GCState_Memory_PreviousInput.land_cnt = rtb_gc_state_out_land_cnt;
  gc_DW.GCState_Memory_PreviousInput.t_test_drogue_s =
    rtb_gc_state_out_t_test_drogue_;
  gc_DW.GCState_Memory_PreviousInput.t_test_main_s =
    rtb_gc_state_out_t_test_main_s;
  gc_DW.GCState_Memory_PreviousInput.init_done = rtb_gc_state_out_init_done;
  gc_DW.GCState_Memory_PreviousInput.restore_done =
    rtb_gc_state_out_restore_done;
  gc_DW.GCState_Memory_PreviousInput.t_airborne_s =
    rtb_gc_state_out_t_airborne_s;
  gc_DW.GCState_Memory_PreviousInput.t_barrido_s = rtb_gc_state_out_t_barrido_s;

  /* Update for DiscreteIntegrator: '<S1>/ki_inner' */
  gc_DW.ki_inner_DSTATE += 0.0153846154F * rtb_normalizarporautoridad;
  if (gc_DW.ki_inner_DSTATE > 1.0F) {
    gc_DW.ki_inner_DSTATE = 1.0F;
  } else if (gc_DW.ki_inner_DSTATE < -1.0F) {
    gc_DW.ki_inner_DSTATE = -1.0F;
  }

  gc_DW.ki_inner_PrevResetState = (int8_T)rtb_controldeshabilitado;

  /* End of Update for DiscreteIntegrator: '<S1>/ki_inner' */
}

/* Model initialize function */
void gc_initialize(void)
{
  /* InitializeConditions for Memory: '<Root>/GCState_Memory' */
  gc_DW.GCState_Memory_PreviousInput = gc_ConstP.GCState_Memory_InitialCondition;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
