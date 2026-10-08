/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: nav.c
 *
 * Code generated for Simulink model 'nav'.
 *
 * Model version                  : 1.25
 * Simulink Coder version         : 25.2 (R2025b) 28-Jul-2025
 * C/C++ source code generated on : Tue Oct  6 00:35:37 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "nav.h"
#include "rtwtypes.h"
#include "nav_types.h"
#include <math.h>
#include <string.h>
#include "nav_private.h"
#include "rt_nonfinite.h"
#include "rt_defines.h"

/* Exported block parameters */
nav_tun_bus nav_tun = {
  84.5F,
  0.0F
} ;                                    /* Variable: nav_tun
                                        * Referenced by: '<Root>/INS_Mechanization'
                                        */

/* Block states (default storage) */
DW_nav_T nav_DW;

/* External inputs (root inport signals with default storage) */
ExtU_nav_T nav_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_nav_T nav_Y;

/* Forward declaration for local functions */
static real_T nav_norm(const real_T x[3]);
static real_T nav_maximum(const real_T x[3]);
static real_T nav_norm_d4(const real_T x[4]);
static void nav_eye(real_T b_I[9]);
static void nav_mldivide(const real_T A[9], const real_T B[3], real_T Y[3]);
static void nav_diag(const real_T v[3], real_T d[9]);
static void nav_mrdiv_o(real_T A[90], const real_T B[36]);
static void nav_eye_d(real_T b_I[225]);
static void nav_apply_correction(nav_state *st, const real32_T st_ref_quat_nb[4],
  const real32_T st_ref_pos_ned[3], const real32_T st_ref_vel_ned[3], const
  real32_T st_ref_accel_body[3], const real32_T st_ref_accel_bias[3], const
  real32_T st_ref_gyro_bias[3], const real_T dx[15], const real_T P[225], real_T
  b_nav_data_g0, real_T b_nav_data_accel_bias_max, real_T
  b_nav_data_gyro_bias_max, real_T b_nav_data_bias_freeze_accel_g, boolean_T
  permite_sesgo_acel);
static boolean_T nav_all(const boolean_T x[4]);
static void nav_isfinite(const real_T x[3], boolean_T b[3]);
static boolean_T nav_all_h(const boolean_T x[3]);

/* Function for MATLAB Function: '<Root>/EKF_Prediction' */
static real_T nav_norm(const real_T x[3])
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

/* Function for MATLAB Function: '<Root>/INS_Mechanization' */
static real_T nav_maximum(const real_T x[3])
{
  real_T ex;
  int32_T idx;
  int32_T k;
  if (!rtIsNaN(x[0])) {
    idx = 1;
  } else {
    boolean_T exitg1;
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 4)) {
      if (!rtIsNaN(x[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (idx == 0) {
    ex = x[0];
  } else {
    ex = x[idx - 1];
    for (k = idx + 1; k < 4; k++) {
      real_T x_0;
      x_0 = x[k - 1];
      if (ex < x_0) {
        ex = x_0;
      }
    }
  }

  return ex;
}

/* Function for MATLAB Function: '<Root>/EKF_Update' */
static real_T nav_norm_d4(const real_T x[4])
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

  absxk = fabs(x[3]);
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
      if (b_k < 4) {
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

/* Function for MATLAB Function: '<Root>/EKF_Update' */
static void nav_eye(real_T b_I[9])
{
  memset(&b_I[0], 0, 9U * sizeof(real_T));
  b_I[0] = 1.0;
  b_I[4] = 1.0;
  b_I[8] = 1.0;
}

/* Function for MATLAB Function: '<Root>/EKF_Update' */
static void nav_mldivide(const real_T A[9], const real_T B[3], real_T Y[3])
{
  real_T b_A[9];
  real_T a21;
  real_T maxval;
  int32_T r1;
  int32_T r2;
  int32_T r3;
  memcpy(&b_A[0], &A[0], 9U * sizeof(real_T));
  r1 = 0;
  r2 = 1;
  r3 = 2;
  maxval = fabs(A[0]);
  a21 = fabs(A[1]);
  if (a21 > maxval) {
    maxval = a21;
    r1 = 1;
    r2 = 0;
  }

  if (fabs(A[2]) > maxval) {
    r1 = 2;
    r2 = 1;
    r3 = 0;
  }

  b_A[r2] = A[r2] / A[r1];
  b_A[r3] /= b_A[r1];
  b_A[r2 + 3] -= b_A[r1 + 3] * b_A[r2];
  b_A[r3 + 3] -= b_A[r1 + 3] * b_A[r3];
  b_A[r2 + 6] -= b_A[r1 + 6] * b_A[r2];
  b_A[r3 + 6] -= b_A[r1 + 6] * b_A[r3];
  if (fabs(b_A[r3 + 3]) > fabs(b_A[r2 + 3])) {
    int32_T rtemp;
    rtemp = r2;
    r2 = r3;
    r3 = rtemp;
  }

  b_A[r3 + 3] /= b_A[r2 + 3];
  b_A[r3 + 6] -= b_A[r3 + 3] * b_A[r2 + 6];
  Y[1] = B[r2] - B[r1] * b_A[r2];
  Y[2] = (B[r3] - B[r1] * b_A[r3]) - b_A[r3 + 3] * Y[1];
  Y[2] /= b_A[r3 + 6];
  Y[0] = B[r1] - b_A[r1 + 6] * Y[2];
  Y[1] -= b_A[r2 + 6] * Y[2];
  Y[1] /= b_A[r2 + 3];
  Y[0] -= b_A[r1 + 3] * Y[1];
  Y[0] /= b_A[r1];
}

/* Function for MATLAB Function: '<Root>/EKF_Update' */
static void nav_diag(const real_T v[3], real_T d[9])
{
  memset(&d[0], 0, 9U * sizeof(real_T));
  d[0] = v[0];
  d[4] = v[1];
  d[8] = v[2];
}

/* Function for MATLAB Function: '<Root>/EKF_Update' */
static void nav_mrdiv_o(real_T A[90], const real_T B[36])
{
  real_T b_A[36];
  real_T smax;
  int32_T b_jBcol;
  int32_T d_j;
  int32_T jA;
  int32_T jBcol;
  int32_T jj;
  int32_T kBcol;
  int32_T n;
  int8_T ipiv[6];
  memcpy(&b_A[0], &B[0], 36U * sizeof(real_T));
  for (d_j = 0; d_j < 6; d_j++) {
    ipiv[d_j] = (int8_T)(d_j + 1);
  }

  for (d_j = 0; d_j < 5; d_j++) {
    jj = d_j * 7;
    n = 7 - d_j;
    b_jBcol = 0;
    smax = fabs(b_A[jj]);
    for (jBcol = 2; jBcol < n; jBcol++) {
      real_T s;
      s = fabs(b_A[(jj + jBcol) - 1]);
      if (s > smax) {
        b_jBcol = jBcol - 1;
        smax = s;
      }
    }

    if (b_A[jj + b_jBcol] != 0.0) {
      if (b_jBcol != 0) {
        n = d_j + b_jBcol;
        ipiv[d_j] = (int8_T)(n + 1);
        for (jBcol = 0; jBcol < 6; jBcol++) {
          b_jBcol = jBcol * 6 + d_j;
          smax = b_A[b_jBcol];
          jA = jBcol * 6 + n;
          b_A[b_jBcol] = b_A[jA];
          b_A[jA] = smax;
        }
      }

      n = (jj - d_j) + 6;
      for (jBcol = jj + 2; jBcol <= n; jBcol++) {
        b_A[jBcol - 1] /= b_A[jj];
      }
    }

    b_jBcol = 4 - d_j;
    jA = jj + 8;
    for (jBcol = 0; jBcol <= b_jBcol; jBcol++) {
      smax = b_A[(jBcol * 6 + jj) + 6];
      if (smax != 0.0) {
        kBcol = (jA - d_j) + 4;
        for (n = jA; n <= kBcol; n++) {
          b_A[n - 1] += b_A[((jj + n) - jA) + 1] * -smax;
        }
      }

      jA += 6;
    }
  }

  for (jj = 0; jj < 6; jj++) {
    jBcol = 15 * jj;
    jA = 6 * jj;
    for (n = 0; n < jj; n++) {
      kBcol = 15 * n;
      smax = b_A[n + jA];
      if (smax != 0.0) {
        for (b_jBcol = 0; b_jBcol < 15; b_jBcol++) {
          d_j = b_jBcol + jBcol;
          A[d_j] -= A[b_jBcol + kBcol] * smax;
        }
      }
    }

    smax = 1.0 / b_A[jj + jA];
    for (n = 0; n < 15; n++) {
      d_j = n + jBcol;
      A[d_j] *= smax;
    }
  }

  for (jj = 5; jj >= 0; jj--) {
    b_jBcol = 15 * jj;
    jA = 6 * jj - 1;
    for (jBcol = jj + 2; jBcol < 7; jBcol++) {
      kBcol = (jBcol - 1) * 15;
      smax = b_A[jBcol + jA];
      if (smax != 0.0) {
        for (n = 0; n < 15; n++) {
          d_j = n + b_jBcol;
          A[d_j] -= A[n + kBcol] * smax;
        }
      }
    }
  }

  for (jj = 4; jj >= 0; jj--) {
    int8_T ipiv_0;
    ipiv_0 = ipiv[jj];
    if (jj + 1 != ipiv_0) {
      for (jBcol = 0; jBcol < 15; jBcol++) {
        n = 15 * jj + jBcol;
        smax = A[n];
        d_j = (ipiv_0 - 1) * 15 + jBcol;
        A[n] = A[d_j];
        A[d_j] = smax;
      }
    }
  }
}

/* Function for MATLAB Function: '<Root>/EKF_Update' */
static void nav_eye_d(real_T b_I[225])
{
  int32_T k;
  memset(&b_I[0], 0, 225U * sizeof(real_T));
  for (k = 0; k < 15; k++) {
    b_I[k + 15 * k] = 1.0;
  }
}

/* Function for MATLAB Function: '<Root>/EKF_Update' */
static void nav_apply_correction(nav_state *st, const real32_T st_ref_quat_nb[4],
  const real32_T st_ref_pos_ned[3], const real32_T st_ref_vel_ned[3], const
  real32_T st_ref_accel_body[3], const real32_T st_ref_accel_bias[3], const
  real32_T st_ref_gyro_bias[3], const real_T dx[15], const real_T P[225], real_T
  b_nav_data_g0, real_T b_nav_data_accel_bias_max, real_T
  b_nav_data_gyro_bias_max, real_T b_nav_data_bias_freeze_accel_g, boolean_T
  permite_sesgo_acel)
{
  real_T dq[4];
  real_T q_new[4];
  real_T st_ref_accel_body_0[3];
  real_T a_idx_0;
  real_T a_idx_1;
  real_T a_idx_2;
  real_T a_idx_3;
  real_T dba_idx_0;
  real_T dba_idx_1;
  real_T dba_idx_2;
  int32_T i;
  dba_idx_0 = dx[9];
  st_ref_accel_body_0[0] = st_ref_accel_body[0];
  dba_idx_1 = dx[10];
  st_ref_accel_body_0[1] = st_ref_accel_body[1];
  dba_idx_2 = dx[11];
  st_ref_accel_body_0[2] = st_ref_accel_body[2];
  if (nav_norm(st_ref_accel_body_0) > b_nav_data_bias_freeze_accel_g *
      b_nav_data_g0) {
    dba_idx_0 = 0.0;
    dba_idx_1 = 0.0;
    dba_idx_2 = 0.0;
  }

  if (!permite_sesgo_acel) {
    dba_idx_0 = 0.0;
    dba_idx_1 = 0.0;
    dba_idx_2 = 0.0;
  }

  dq[0] = 1.0;
  dq[1] = 0.5 * dx[0];
  dq[2] = 0.5 * dx[1];
  dq[3] = 0.5 * dx[2];
  a_idx_3 = nav_norm_d4(dq);
  a_idx_0 = 1.0 / a_idx_3;
  a_idx_1 = dq[1] / a_idx_3;
  a_idx_2 = dq[2] / a_idx_3;
  a_idx_3 = dq[3] / a_idx_3;
  q_new[0] = ((a_idx_0 * st_ref_quat_nb[0] - a_idx_1 * st_ref_quat_nb[1]) -
              a_idx_2 * st_ref_quat_nb[2]) - a_idx_3 * st_ref_quat_nb[3];
  q_new[1] = ((a_idx_0 * st_ref_quat_nb[1] + st_ref_quat_nb[0] * a_idx_1) +
              a_idx_2 * st_ref_quat_nb[3]) - st_ref_quat_nb[2] * a_idx_3;
  q_new[2] = ((a_idx_0 * st_ref_quat_nb[2] - a_idx_1 * st_ref_quat_nb[3]) +
              st_ref_quat_nb[0] * a_idx_2) + st_ref_quat_nb[1] * a_idx_3;
  q_new[3] = ((a_idx_0 * st_ref_quat_nb[3] + a_idx_1 * st_ref_quat_nb[2]) -
              st_ref_quat_nb[1] * a_idx_2) + st_ref_quat_nb[0] * a_idx_3;
  a_idx_3 = nav_norm_d4(q_new);
  q_new[0] /= a_idx_3;
  q_new[1] /= a_idx_3;
  q_new[2] /= a_idx_3;
  q_new[3] /= a_idx_3;
  st->quat_nb[0] = (real32_T)q_new[0];
  st->quat_nb[1] = (real32_T)q_new[1];
  st->quat_nb[2] = (real32_T)q_new[2];
  st->quat_nb[3] = (real32_T)q_new[3];
  st->vel_ned[0] = (real32_T)(st_ref_vel_ned[0] + dx[3]);
  st->pos_ned[0] = (real32_T)(st_ref_pos_ned[0] + dx[6]);
  st->accel_bias[0] = (real32_T)fmin(fmax(st_ref_accel_bias[0] + dba_idx_0,
    -b_nav_data_accel_bias_max), b_nav_data_accel_bias_max);
  st->gyro_bias[0] = (real32_T)fmin(fmax(st_ref_gyro_bias[0] + dx[12],
    -b_nav_data_gyro_bias_max), b_nav_data_gyro_bias_max);
  st->vel_ned[1] = (real32_T)(st_ref_vel_ned[1] + dx[4]);
  st->pos_ned[1] = (real32_T)(st_ref_pos_ned[1] + dx[7]);
  st->accel_bias[1] = (real32_T)fmin(fmax(st_ref_accel_bias[1] + dba_idx_1,
    -b_nav_data_accel_bias_max), b_nav_data_accel_bias_max);
  st->gyro_bias[1] = (real32_T)fmin(fmax(st_ref_gyro_bias[1] + dx[13],
    -b_nav_data_gyro_bias_max), b_nav_data_gyro_bias_max);
  st->vel_ned[2] = (real32_T)(st_ref_vel_ned[2] + dx[5]);
  st->pos_ned[2] = (real32_T)(st_ref_pos_ned[2] + dx[8]);
  st->accel_bias[2] = (real32_T)fmin(fmax(st_ref_accel_bias[2] + dba_idx_2,
    -b_nav_data_accel_bias_max), b_nav_data_accel_bias_max);
  st->gyro_bias[2] = (real32_T)fmin(fmax(st_ref_gyro_bias[2] + dx[14],
    -b_nav_data_gyro_bias_max), b_nav_data_gyro_bias_max);
  for (i = 0; i < 225; i++) {
    st->P[i] = (real32_T)P[i];
  }
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

/* Function for MATLAB Function: '<Root>/EKF_Update' */
static boolean_T nav_all(const boolean_T x[4])
{
  int32_T k;
  boolean_T exitg1;
  boolean_T y;
  y = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 4)) {
    if (!x[k]) {
      y = false;
      exitg1 = true;
    } else {
      k++;
    }
  }

  return y;
}

/* Function for MATLAB Function: '<Root>/EKF_Update' */
static void nav_isfinite(const real_T x[3], boolean_T b[3])
{
  b[0] = ((!rtIsInf(x[0])) && (!rtIsNaN(x[0])));
  b[1] = ((!rtIsInf(x[1])) && (!rtIsNaN(x[1])));
  b[2] = ((!rtIsInf(x[2])) && (!rtIsNaN(x[2])));
}

/* Function for MATLAB Function: '<Root>/EKF_Update' */
static boolean_T nav_all_h(const boolean_T x[3])
{
  int32_T k;
  boolean_T exitg1;
  boolean_T y;
  y = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 3)) {
    if (!x[k]) {
      y = false;
      exitg1 = true;
    } else {
      k++;
    }
  }

  return y;
}

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    int32_T tmp;
    int32_T tmp_0;
    if (u0 > 0.0) {
      tmp = 1;
    } else {
      tmp = -1;
    }

    if (u1 > 0.0) {
      tmp_0 = 1;
    } else {
      tmp_0 = -1;
    }

    y = atan2(tmp, tmp_0);
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

/* Model step function */
void nav_step(void)
{
  nav_state rtb_state_out;
  nav_state state_in;
  real_T Fdt[225];
  real_T Fdt_0[225];
  real_T P_pred[225];
  real_T Phi[225];
  real_T Phi_0[225];
  real_T Phi_1[225];
  real_T Fdt_1[90];
  real_T H[90];
  real_T H_0[90];
  real_T K[90];
  real_T R[36];
  real_T S[36];
  real_T tmp_0[36];
  real_T K_b[15];
  real_T K_b_0[15];
  real_T Cbn[9];
  real_T H_tmp[9];
  real_T z[6];
  real_T q[4];
  real_T q_mid[4];
  real_T q_new[4];
  real_T a_lg[3];
  real_T f_b[3];
  real_T omega_b[3];
  real_T theta[3];
  real_T ang;
  real_T cp;
  real_T cy;
  real_T dt;
  real_T dt_meas;
  real_T n_meas;
  real_T sp;
  real_T sy;
  int32_T Cbn_0;
  int32_T Fdt_tmp;
  int32_T Fdt_tmp_0;
  int32_T b_k;
  int32_T i;
  uint32_T tmp_1;
  int8_T S_b_tmp[15];
  int8_T S_b_tmp_0[15];
  boolean_T tmp[4];
  boolean_T hg_ok;
  boolean_T lg_ok;
  boolean_T use_pos;
  static const int8_T b_a[15] = { 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0 };

  static const int8_T c_a[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  static const real_T d[9] = { 1.0E+12, 0.0, 0.0, 0.0, 1.0E+12, 0.0, 0.0, 0.0,
    1.0E+12 };

  real_T ang_tmp;
  real_T dt_meas_tmp;
  real32_T tmp_3[4];
  real32_T tmp_4[3];
  real32_T tmp_5[3];
  real32_T tmp_6[3];
  real32_T tmp_7[3];
  boolean_T tmp_2[3];
  boolean_T guard1;

  /* MATLAB Function: '<Root>/INS_Mechanization' incorporates:
   *  BusCreator generated from: '<Root>/INS_Mechanization'
   *  Inport: '<Root>/nav_in'
   *  Memory: '<Root>/NavState_Memory'
   */
  state_in = nav_DW.NavState_Memory_PreviousInput;
  dt = 0.0025;
  if (nav_U.nav_in.imu_lowg.data_valid == 1) {
    dt_meas = nav_U.nav_in.imu_lowg.timestamp_s -
      nav_DW.NavState_Memory_PreviousInput.timestamp_s;
    if ((dt_meas > 0.00125) && (dt_meas < 0.005)) {
      dt = dt_meas;
    }
  }

  a_lg[0] = nav_U.nav_in.imu_lowg.accel_mps2[0];
  a_lg[1] = nav_U.nav_in.imu_lowg.accel_mps2[1];
  a_lg[2] = nav_U.nav_in.imu_lowg.accel_mps2[2];
  lg_ok = (nav_U.nav_in.imu_lowg.data_valid == 1);
  hg_ok = (nav_U.nav_in.imu_highg.data_valid == 1);
  dt_meas = fmin(fmax((nav_norm(a_lg) / 9.80665 - 12.0) / 4.0, 0.0), 1.0);
  theta[0] = fabs(nav_U.nav_in.imu_lowg.accel_mps2[0]);
  theta[1] = fabs(nav_U.nav_in.imu_lowg.accel_mps2[1]);
  theta[2] = fabs(nav_U.nav_in.imu_lowg.accel_mps2[2]);
  if (nav_maximum(theta) >= 153.768272) {
    dt_meas = 1.0;
  }

  use_pos = !lg_ok;
  if (use_pos && hg_ok) {
    dt_meas = 1.0;
  } else if (lg_ok && (!hg_ok)) {
    dt_meas = 0.0;
  }

  if (lg_ok || hg_ok) {
    a_lg[0] = (1.0 - dt_meas) * nav_U.nav_in.imu_lowg.accel_mps2[0] + dt_meas *
      nav_U.nav_in.imu_highg.accel_mps2[0];
    a_lg[1] = (1.0 - dt_meas) * nav_U.nav_in.imu_lowg.accel_mps2[1] + dt_meas *
      nav_U.nav_in.imu_highg.accel_mps2[1];
    a_lg[2] = (1.0 - dt_meas) * nav_U.nav_in.imu_lowg.accel_mps2[2] + dt_meas *
      nav_U.nav_in.imu_highg.accel_mps2[2];
  } else {
    a_lg[0] = nav_DW.NavState_Memory_PreviousInput.accel_body[0];
    a_lg[1] = nav_DW.NavState_Memory_PreviousInput.accel_body[1];
    a_lg[2] = nav_DW.NavState_Memory_PreviousInput.accel_body[2];
    dt_meas = nav_DW.NavState_Memory_PreviousInput.imu_blend_w;
  }

  f_b[0] = a_lg[0] - (1.0 - dt_meas) *
    nav_DW.NavState_Memory_PreviousInput.accel_bias[0];
  if (lg_ok) {
    omega_b[0] = (real_T)nav_U.nav_in.imu_lowg.gyro_rps[0] -
      nav_DW.NavState_Memory_PreviousInput.gyro_bias[0];
  } else {
    omega_b[0] = nav_DW.NavState_Memory_PreviousInput.gyro_body[0];
  }

  f_b[1] = a_lg[1] - (1.0 - dt_meas) *
    nav_DW.NavState_Memory_PreviousInput.accel_bias[1];
  if (lg_ok) {
    omega_b[1] = (real_T)nav_U.nav_in.imu_lowg.gyro_rps[1] -
      nav_DW.NavState_Memory_PreviousInput.gyro_bias[1];
  } else {
    omega_b[1] = nav_DW.NavState_Memory_PreviousInput.gyro_body[1];
  }

  f_b[2] = a_lg[2] - (1.0 - dt_meas) *
    nav_DW.NavState_Memory_PreviousInput.accel_bias[2];
  if (lg_ok) {
    omega_b[2] = (real_T)nav_U.nav_in.imu_lowg.gyro_rps[2] -
      nav_DW.NavState_Memory_PreviousInput.gyro_bias[2];
  } else {
    omega_b[2] = nav_DW.NavState_Memory_PreviousInput.gyro_body[2];
  }

  switch (nav_U.nav_in.nav_cmd) {
   case 10U:
    state_in.init_done = 0U;
    state_in.align_cnt = 0U;
    state_in.acc_sum[0] = 0.0F;
    state_in.gyr_sum[0] = 0.0F;
    state_in.acc_sum[1] = 0.0F;
    state_in.gyr_sum[1] = 0.0F;
    state_in.acc_sum[2] = 0.0F;
    state_in.gyr_sum[2] = 0.0F;
    state_in.baro_sum = 0.0F;
    state_in.baro_ref_set = 0U;
    break;

   case 11U:
    memcpy(&state_in.P[0], &nav_ConstP.pooled1.P0[0], 225U * sizeof(real32_T));
    state_in.reject_cnt = 0U;
    state_in.baro_rej_cnt = 0U;
    break;
  }

  guard1 = false;
  if ((state_in.init_done == 0) && (nav_U.nav_in.rocket_state == 0)) {
    rtb_state_out = state_in;
    rtb_state_out.timestamp_s = nav_DW.NavState_Memory_PreviousInput.timestamp_s
      + dt;
    rtb_state_out.imu_blend_w = (real32_T)dt_meas;
    rtb_state_out.accel_body[0] = (real32_T)a_lg[0];
    rtb_state_out.gyro_body[0] = (real32_T)omega_b[0];
    rtb_state_out.accel_ned[0] = 0.0F;
    rtb_state_out.accel_body[1] = (real32_T)a_lg[1];
    rtb_state_out.gyro_body[1] = (real32_T)omega_b[1];
    rtb_state_out.accel_ned[1] = 0.0F;
    rtb_state_out.accel_body[2] = (real32_T)a_lg[2];
    rtb_state_out.gyro_body[2] = (real32_T)omega_b[2];
    rtb_state_out.accel_ned[2] = 0.0F;
    rtb_state_out.gnss_age_s = (real32_T)fmin
      (nav_DW.NavState_Memory_PreviousInput.gnss_age_s + dt, 10000.0);
    if (nav_norm(omega_b) > 0.069813170079773182) {
      lg_ok = true;
    } else {
      lg_ok = (fabs(nav_norm(a_lg) - 9.80665) > 1.5);
    }

    if (lg_ok || use_pos) {
      rtb_state_out.align_cnt = 0U;
      rtb_state_out.acc_sum[0] = 0.0F;
      rtb_state_out.gyr_sum[0] = 0.0F;
      rtb_state_out.acc_sum[1] = 0.0F;
      rtb_state_out.gyr_sum[1] = 0.0F;
      rtb_state_out.acc_sum[2] = 0.0F;
      rtb_state_out.gyr_sum[2] = 0.0F;
      rtb_state_out.baro_sum = 0.0F;

      /* MATLAB Function: '<Root>/EKF_Prediction' */
      memcpy(&rtb_state_out.P[0], &nav_ConstP.pooled1.P0[0], 225U * sizeof
             (real32_T));
    } else {
      a_lg[0] += state_in.acc_sum[0];
      omega_b[0] += state_in.gyr_sum[0];
      a_lg[1] += state_in.acc_sum[1];
      omega_b[1] += state_in.gyr_sum[1];
      a_lg[2] += state_in.acc_sum[2];
      omega_b[2] += state_in.gyr_sum[2];
      dt = (real_T)state_in.baro_sum + nav_U.nav_in.baro.press_pa;
      if (state_in.align_cnt + 1 >= 200) {
        a_lg[0] /= (real_T)state_in.align_cnt + 1.0;
        a_lg[1] /= (real_T)state_in.align_cnt + 1.0;
        a_lg[2] /= (real_T)state_in.align_cnt + 1.0;
        n_meas = nav_tun.rail_elev_deg * 3.1415926535897931 / 180.0 * 0.5;
        dt_meas = cos(n_meas);
        cp = sin(n_meas);
        n_meas = nav_tun.rail_azim_deg * 3.1415926535897931 / 180.0 * 0.5;
        sp = cos(n_meas);
        cy = sin(n_meas);
        n_meas = 0.0 * cp;
        q[0] = n_meas * cy + dt_meas * sp;
        ang = 0.0 * dt_meas;
        q[1] = ang * sp - cp * cy;
        q[2] = ang * cy + cp * sp;
        q[3] = dt_meas * cy - n_meas * sp;
        n_meas = nav_norm_d4(q);
        q[0] /= n_meas;
        q[1] /= n_meas;
        q[2] /= n_meas;
        q[3] /= n_meas;
        n_meas = q[3] * q[3];
        ang = q[2] * q[2];
        H_tmp[0] = -(1.0 - (ang + n_meas) * 2.0);
        cp = q[1] * q[2];
        sp = q[0] * q[3];
        H_tmp[1] = -((cp - sp) * 2.0);
        cy = q[1] * q[3];
        sy = q[0] * q[2];
        H_tmp[2] = -((cy + sy) * 2.0);
        H_tmp[3] = -((cp + sp) * 2.0);
        cp = q[1] * q[1];
        H_tmp[4] = -(1.0 - (cp + n_meas) * 2.0);
        n_meas = q[2] * q[3];
        sp = q[0] * q[1];
        H_tmp[5] = -((n_meas - sp) * 2.0);
        H_tmp[6] = -((cy - sy) * 2.0);
        H_tmp[7] = -((n_meas + sp) * 2.0);
        H_tmp[8] = -(1.0 - (cp + ang) * 2.0);
        theta[0] = 0.0;
        theta[1] = 0.0;
        theta[2] = 9.80665;
        dt_meas = 0.0;
        ang = 0.0;
        cp = 0.0;
        for (i = 0; i < 3; i++) {
          n_meas = theta[i];
          dt_meas += H_tmp[3 * i] * n_meas;
          ang += H_tmp[3 * i + 1] * n_meas;
          cp += H_tmp[3 * i + 2] * n_meas;
        }

        theta[2] = cp;
        theta[1] = ang;
        theta[0] = dt_meas;
        n_meas = nav_norm(a_lg);
        sp = nav_norm(theta);
        cy = 1.0;
        if ((n_meas > 1.0E-6) && (sp > 1.0E-6)) {
          cy = ((a_lg[0] * dt_meas + a_lg[1] * ang) + a_lg[2] * cp) / (n_meas *
            sp);
        }

        if (acos(fmin(fmax(cy, -1.0), 1.0)) > 0.17453292519943295) {
          rtb_state_out.align_cnt = 0U;
          rtb_state_out.acc_sum[0] = 0.0F;
          rtb_state_out.gyr_sum[0] = 0.0F;
          rtb_state_out.acc_sum[1] = 0.0F;
          rtb_state_out.gyr_sum[1] = 0.0F;
          rtb_state_out.acc_sum[2] = 0.0F;
          rtb_state_out.gyr_sum[2] = 0.0F;
          rtb_state_out.baro_sum = 0.0F;

          /* MATLAB Function: '<Root>/EKF_Prediction' */
          memcpy(&rtb_state_out.P[0], &nav_ConstP.pooled1.P0[0], 225U * sizeof
                 (real32_T));
        } else {
          rtb_state_out.baro_ref_pa = (real32_T)(dt / ((real_T)
            state_in.align_cnt + 1.0));
          rtb_state_out.baro_ref_set = 1U;
          rtb_state_out.quat_nb[0] = (real32_T)q[0];
          rtb_state_out.quat_nb[1] = (real32_T)q[1];
          rtb_state_out.quat_nb[2] = (real32_T)q[2];
          rtb_state_out.quat_nb[3] = (real32_T)q[3];
          rtb_state_out.gyro_bias[0] = (real32_T)(omega_b[0] / ((real_T)
            state_in.align_cnt + 1.0));
          rtb_state_out.accel_bias[0] = (real32_T)(a_lg[0] - dt_meas);
          rtb_state_out.vel_ned[0] = 0.0F;
          rtb_state_out.gyro_bias[1] = (real32_T)(omega_b[1] / ((real_T)
            state_in.align_cnt + 1.0));
          rtb_state_out.accel_bias[1] = (real32_T)(a_lg[1] - ang);
          rtb_state_out.vel_ned[1] = 0.0F;
          rtb_state_out.gyro_bias[2] = (real32_T)(omega_b[2] / ((real_T)
            state_in.align_cnt + 1.0));
          rtb_state_out.accel_bias[2] = (real32_T)(a_lg[2] - cp);
          rtb_state_out.vel_ned[2] = 0.0F;
          memcpy(&rtb_state_out.P[0], &nav_ConstP.pooled1.P0[0], 225U * sizeof
                 (real32_T));
          rtb_state_out.init_done = 1U;
          rtb_state_out.align_cnt = 0U;
          rtb_state_out.acc_sum[0] = 0.0F;
          rtb_state_out.gyr_sum[0] = 0.0F;
          rtb_state_out.acc_sum[1] = 0.0F;
          rtb_state_out.gyr_sum[1] = 0.0F;
          rtb_state_out.acc_sum[2] = 0.0F;
          rtb_state_out.gyr_sum[2] = 0.0F;
          rtb_state_out.baro_sum = 0.0F;
          guard1 = true;
        }
      } else {
        if (state_in.align_cnt + 1 < 65536) {
          rtb_state_out.align_cnt = (uint16_T)(state_in.align_cnt + 1);
        } else {
          rtb_state_out.align_cnt = MAX_uint16_T;
        }

        rtb_state_out.acc_sum[0] = (real32_T)a_lg[0];
        rtb_state_out.gyr_sum[0] = (real32_T)omega_b[0];
        rtb_state_out.acc_sum[1] = (real32_T)a_lg[1];
        rtb_state_out.gyr_sum[1] = (real32_T)omega_b[1];
        rtb_state_out.acc_sum[2] = (real32_T)a_lg[2];
        rtb_state_out.gyr_sum[2] = (real32_T)omega_b[2];
        rtb_state_out.baro_sum = (real32_T)dt;

        /* MATLAB Function: '<Root>/EKF_Prediction' */
        memcpy(&rtb_state_out.P[0], &nav_ConstP.pooled1.P0[0], 225U * sizeof
               (real32_T));
      }
    }
  } else {
    if (state_in.init_done == 0) {
      rtb_state_out = state_in;
      n_meas = nav_tun.rail_elev_deg * 3.1415926535897931 / 180.0 * 0.5;
      cp = cos(n_meas);
      sp = sin(n_meas);
      n_meas = nav_tun.rail_azim_deg * 3.1415926535897931 / 180.0 * 0.5;
      cy = cos(n_meas);
      sy = sin(n_meas);
      n_meas = 0.0 * sp;
      q[0] = n_meas * sy + cp * cy;
      ang = 0.0 * cp;
      q[1] = ang * cy - sp * sy;
      q[2] = ang * sy + sp * cy;
      q[3] = cp * sy - n_meas * cy;
      n_meas = nav_norm_d4(q);
      rtb_state_out.quat_nb[0] = (real32_T)(q[0] / n_meas);
      rtb_state_out.quat_nb[1] = (real32_T)(q[1] / n_meas);
      rtb_state_out.quat_nb[2] = (real32_T)(q[2] / n_meas);
      rtb_state_out.quat_nb[3] = (real32_T)(q[3] / n_meas);
      memcpy(&rtb_state_out.P[0], &nav_ConstP.pooled1.P0[0], 225U * sizeof
             (real32_T));
      rtb_state_out.init_done = 1U;
      rtb_state_out.baro_ref_pa = nav_U.nav_in.baro.press_pa;
      rtb_state_out.baro_ref_set = 1U;
      rtb_state_out.timestamp_s =
        nav_DW.NavState_Memory_PreviousInput.timestamp_s + dt;
      rtb_state_out.accel_body[0] = (real32_T)a_lg[0];
      rtb_state_out.gyro_body[0] = (real32_T)omega_b[0];
      rtb_state_out.accel_body[1] = (real32_T)a_lg[1];
      rtb_state_out.gyro_body[1] = (real32_T)omega_b[1];
      rtb_state_out.accel_body[2] = (real32_T)a_lg[2];
      rtb_state_out.gyro_body[2] = (real32_T)omega_b[2];
      rtb_state_out.imu_blend_w = (real32_T)dt_meas;
    } else if (nav_DW.NavState_Memory_PreviousInput.gnss_only == 1) {
      rtb_state_out = state_in;
      theta[0] = omega_b[0] * dt;
      theta[1] = omega_b[1] * dt;
      theta[2] = omega_b[2] * dt;
      ang = nav_norm(theta);
      if (ang > 1.0E-12) {
        n_meas = 0.5 * ang;
        sy = sin(n_meas);
        n_meas = cos(n_meas);
        cp = theta[0] / ang * sy;
        sp = theta[1] / ang * sy;
        cy = theta[2] / ang * sy;
      } else {
        n_meas = 1.0;
        cp = 0.5 * theta[0];
        sp = 0.5 * theta[1];
        cy = 0.5 * theta[2];
      }

      q_new[0] = ((nav_DW.NavState_Memory_PreviousInput.quat_nb[0] * n_meas -
                   nav_DW.NavState_Memory_PreviousInput.quat_nb[1] * cp) -
                  nav_DW.NavState_Memory_PreviousInput.quat_nb[2] * sp) -
        nav_DW.NavState_Memory_PreviousInput.quat_nb[3] * cy;
      q_new[1] = ((nav_DW.NavState_Memory_PreviousInput.quat_nb[0] * cp + n_meas
                   * nav_DW.NavState_Memory_PreviousInput.quat_nb[1]) +
                  nav_DW.NavState_Memory_PreviousInput.quat_nb[2] * cy) - sp *
        nav_DW.NavState_Memory_PreviousInput.quat_nb[3];
      q_new[2] = ((nav_DW.NavState_Memory_PreviousInput.quat_nb[0] * sp -
                   nav_DW.NavState_Memory_PreviousInput.quat_nb[1] * cy) +
                  n_meas * nav_DW.NavState_Memory_PreviousInput.quat_nb[2]) + cp
        * nav_DW.NavState_Memory_PreviousInput.quat_nb[3];
      q_new[3] = ((nav_DW.NavState_Memory_PreviousInput.quat_nb[0] * cy +
                   nav_DW.NavState_Memory_PreviousInput.quat_nb[1] * sp) - cp *
                  nav_DW.NavState_Memory_PreviousInput.quat_nb[2]) + n_meas *
        nav_DW.NavState_Memory_PreviousInput.quat_nb[3];
      n_meas = nav_norm_d4(q_new);
      rtb_state_out.quat_nb[0] = (real32_T)(q_new[0] / n_meas);
      rtb_state_out.quat_nb[1] = (real32_T)(q_new[1] / n_meas);
      rtb_state_out.quat_nb[2] = (real32_T)(q_new[2] / n_meas);
      rtb_state_out.quat_nb[3] = (real32_T)(q_new[3] / n_meas);
      rtb_state_out.pos_ned[0] = (real32_T)
        (nav_DW.NavState_Memory_PreviousInput.vel_ned[0] * dt +
         nav_DW.NavState_Memory_PreviousInput.pos_ned[0]);
      rtb_state_out.accel_ned[0] = 0.0F;
      rtb_state_out.accel_body[0] = (real32_T)f_b[0];
      rtb_state_out.gyro_body[0] = (real32_T)omega_b[0];
      rtb_state_out.pos_ned[1] = (real32_T)
        (nav_DW.NavState_Memory_PreviousInput.vel_ned[1] * dt +
         nav_DW.NavState_Memory_PreviousInput.pos_ned[1]);
      rtb_state_out.accel_ned[1] = 0.0F;
      rtb_state_out.accel_body[1] = (real32_T)f_b[1];
      rtb_state_out.gyro_body[1] = (real32_T)omega_b[1];
      rtb_state_out.pos_ned[2] = (real32_T)
        (nav_DW.NavState_Memory_PreviousInput.vel_ned[2] * dt +
         nav_DW.NavState_Memory_PreviousInput.pos_ned[2]);
      rtb_state_out.accel_ned[2] = 0.0F;
      rtb_state_out.accel_body[2] = (real32_T)f_b[2];
      rtb_state_out.gyro_body[2] = (real32_T)omega_b[2];
      rtb_state_out.imu_blend_w = (real32_T)dt_meas;
      rtb_state_out.timestamp_s =
        nav_DW.NavState_Memory_PreviousInput.timestamp_s + dt;
      rtb_state_out.gnss_age_s = (real32_T)fmin
        (nav_DW.NavState_Memory_PreviousInput.gnss_age_s + dt, 10000.0);
    } else {
      rtb_state_out = state_in;
      theta[0] = omega_b[0] * dt;
      theta[1] = omega_b[1] * dt;
      theta[2] = omega_b[2] * dt;
      ang = nav_norm(theta);
      if (ang > 1.0E-12) {
        n_meas = 0.5 * ang;
        cy = sin(n_meas);
        n_meas = cos(n_meas);
        cp = theta[0] / ang * cy;
        sp = theta[1] / ang * cy;
        cy *= theta[2] / ang;
      } else {
        n_meas = 1.0;
        cp = 0.5 * theta[0];
        sp = 0.5 * theta[1];
        cy = 0.5 * theta[2];
      }

      q_new[0] = ((nav_DW.NavState_Memory_PreviousInput.quat_nb[0] * n_meas -
                   nav_DW.NavState_Memory_PreviousInput.quat_nb[1] * cp) -
                  nav_DW.NavState_Memory_PreviousInput.quat_nb[2] * sp) -
        nav_DW.NavState_Memory_PreviousInput.quat_nb[3] * cy;
      q_new[1] = ((nav_DW.NavState_Memory_PreviousInput.quat_nb[0] * cp + n_meas
                   * nav_DW.NavState_Memory_PreviousInput.quat_nb[1]) +
                  nav_DW.NavState_Memory_PreviousInput.quat_nb[2] * cy) - sp *
        nav_DW.NavState_Memory_PreviousInput.quat_nb[3];
      q_new[2] = ((nav_DW.NavState_Memory_PreviousInput.quat_nb[0] * sp -
                   nav_DW.NavState_Memory_PreviousInput.quat_nb[1] * cy) +
                  n_meas * nav_DW.NavState_Memory_PreviousInput.quat_nb[2]) + cp
        * nav_DW.NavState_Memory_PreviousInput.quat_nb[3];
      q_new[3] = ((nav_DW.NavState_Memory_PreviousInput.quat_nb[0] * cy +
                   nav_DW.NavState_Memory_PreviousInput.quat_nb[1] * sp) - cp *
                  nav_DW.NavState_Memory_PreviousInput.quat_nb[2]) + n_meas *
        nav_DW.NavState_Memory_PreviousInput.quat_nb[3];
      n_meas = nav_norm_d4(q_new);
      q_new[0] /= n_meas;
      q_new[1] /= n_meas;
      q_new[2] /= n_meas;
      q_new[3] /= n_meas;
      if (ang > 1.0E-12) {
        n_meas = 0.25 * ang;
        cy = sin(n_meas);
        n_meas = cos(n_meas);
        cp = theta[0] / ang * cy;
        sp = theta[1] / ang * cy;
        cy *= theta[2] / ang;
      } else {
        n_meas = 1.0;
        cp = 0.25 * theta[0];
        sp = 0.25 * theta[1];
        cy = 0.25 * theta[2];
      }

      q_mid[0] = ((nav_DW.NavState_Memory_PreviousInput.quat_nb[0] * n_meas -
                   nav_DW.NavState_Memory_PreviousInput.quat_nb[1] * cp) -
                  nav_DW.NavState_Memory_PreviousInput.quat_nb[2] * sp) -
        nav_DW.NavState_Memory_PreviousInput.quat_nb[3] * cy;
      q_mid[1] = ((nav_DW.NavState_Memory_PreviousInput.quat_nb[0] * cp + n_meas
                   * nav_DW.NavState_Memory_PreviousInput.quat_nb[1]) +
                  nav_DW.NavState_Memory_PreviousInput.quat_nb[2] * cy) - sp *
        nav_DW.NavState_Memory_PreviousInput.quat_nb[3];
      q_mid[2] = ((nav_DW.NavState_Memory_PreviousInput.quat_nb[0] * sp -
                   nav_DW.NavState_Memory_PreviousInput.quat_nb[1] * cy) +
                  n_meas * nav_DW.NavState_Memory_PreviousInput.quat_nb[2]) + cp
        * nav_DW.NavState_Memory_PreviousInput.quat_nb[3];
      q_mid[3] = ((nav_DW.NavState_Memory_PreviousInput.quat_nb[0] * cy +
                   nav_DW.NavState_Memory_PreviousInput.quat_nb[1] * sp) - cp *
                  nav_DW.NavState_Memory_PreviousInput.quat_nb[2]) + n_meas *
        nav_DW.NavState_Memory_PreviousInput.quat_nb[3];
      n_meas = nav_norm_d4(q_mid);
      q[0] = q_mid[0] / n_meas;
      q[1] = q_mid[1] / n_meas;
      q[2] = q_mid[2] / n_meas;
      q[3] = q_mid[3] / n_meas;
      n_meas = q[3] * q[3];
      ang = q[2] * q[2];
      H_tmp[0] = 1.0 - (ang + n_meas) * 2.0;
      cp = q[1] * q[2];
      sp = q[0] * q[3];
      H_tmp[3] = (cp - sp) * 2.0;
      cy = q[1] * q[3];
      sy = q[0] * q[2];
      H_tmp[6] = (cy + sy) * 2.0;
      H_tmp[1] = (cp + sp) * 2.0;
      cp = q[1] * q[1];
      H_tmp[4] = 1.0 - (cp + n_meas) * 2.0;
      n_meas = q[2] * q[3];
      sp = q[0] * q[1];
      H_tmp[7] = (n_meas - sp) * 2.0;
      H_tmp[2] = (cy - sy) * 2.0;
      H_tmp[5] = (n_meas + sp) * 2.0;
      H_tmp[8] = 1.0 - (cp + ang) * 2.0;
      ang = 0.0;
      cp = 0.0;
      sp = 0.0;
      for (i = 0; i < 3; i++) {
        n_meas = f_b[i];
        ang += H_tmp[3 * i] * n_meas;
        cp += H_tmp[3 * i + 1] * n_meas;
        sp += H_tmp[3 * i + 2] * n_meas;
      }

      rtb_state_out.quat_nb[0] = (real32_T)q_new[0];
      rtb_state_out.quat_nb[1] = (real32_T)q_new[1];
      rtb_state_out.quat_nb[2] = (real32_T)q_new[2];
      rtb_state_out.quat_nb[3] = (real32_T)q_new[3];
      rtb_state_out.pos_ned[0] = (real32_T)(0.5 * ang * dt * dt +
        (nav_DW.NavState_Memory_PreviousInput.vel_ned[0] * dt +
         nav_DW.NavState_Memory_PreviousInput.pos_ned[0]));
      rtb_state_out.vel_ned[0] = (real32_T)(ang * dt +
        nav_DW.NavState_Memory_PreviousInput.vel_ned[0]);
      rtb_state_out.accel_ned[0] = (real32_T)ang;
      rtb_state_out.accel_body[0] = (real32_T)f_b[0];
      rtb_state_out.gyro_body[0] = (real32_T)omega_b[0];
      rtb_state_out.pos_ned[1] = (real32_T)(0.5 * cp * dt * dt +
        (nav_DW.NavState_Memory_PreviousInput.vel_ned[1] * dt +
         nav_DW.NavState_Memory_PreviousInput.pos_ned[1]));
      rtb_state_out.vel_ned[1] = (real32_T)(cp * dt +
        nav_DW.NavState_Memory_PreviousInput.vel_ned[1]);
      rtb_state_out.accel_ned[1] = (real32_T)cp;
      rtb_state_out.accel_body[1] = (real32_T)f_b[1];
      rtb_state_out.gyro_body[1] = (real32_T)omega_b[1];
      rtb_state_out.pos_ned[2] = (real32_T)((sp + 9.81) * 0.5 * dt * dt +
        (nav_DW.NavState_Memory_PreviousInput.vel_ned[2] * dt +
         nav_DW.NavState_Memory_PreviousInput.pos_ned[2]));
      rtb_state_out.vel_ned[2] = (real32_T)((sp + 9.81) * dt +
        nav_DW.NavState_Memory_PreviousInput.vel_ned[2]);
      rtb_state_out.accel_ned[2] = (real32_T)(sp + 9.81);
      rtb_state_out.accel_body[2] = (real32_T)f_b[2];
      rtb_state_out.gyro_body[2] = (real32_T)omega_b[2];
      rtb_state_out.imu_blend_w = (real32_T)dt_meas;
      rtb_state_out.timestamp_s =
        nav_DW.NavState_Memory_PreviousInput.timestamp_s + dt;
      rtb_state_out.gnss_age_s = (real32_T)fmin
        (nav_DW.NavState_Memory_PreviousInput.gnss_age_s + dt, 10000.0);
    }

    guard1 = true;
  }

  if (guard1) {
    /* MATLAB Function: '<Root>/EKF_Prediction' */
    if (rtb_state_out.gnss_only != 1) {
      dt = (real_T)rtb_state_out.quat_nb[3] * rtb_state_out.quat_nb[3];
      dt_meas = (real_T)rtb_state_out.quat_nb[2] * rtb_state_out.quat_nb[2];
      Cbn[0] = 1.0 - (dt_meas + dt) * 2.0;
      n_meas = (real_T)rtb_state_out.quat_nb[1] * rtb_state_out.quat_nb[2];
      ang = (real_T)rtb_state_out.quat_nb[0] * rtb_state_out.quat_nb[3];
      Cbn[3] = (n_meas - ang) * 2.0;
      cp = (real_T)rtb_state_out.quat_nb[1] * rtb_state_out.quat_nb[3];
      sp = (real_T)rtb_state_out.quat_nb[0] * rtb_state_out.quat_nb[2];
      Cbn[6] = (cp + sp) * 2.0;
      Cbn[1] = (n_meas + ang) * 2.0;
      n_meas = (real_T)rtb_state_out.quat_nb[1] * rtb_state_out.quat_nb[1];
      Cbn[4] = 1.0 - (n_meas + dt) * 2.0;
      dt = (real_T)rtb_state_out.quat_nb[2] * rtb_state_out.quat_nb[3];
      ang = (real_T)rtb_state_out.quat_nb[0] * rtb_state_out.quat_nb[1];
      Cbn[7] = (dt - ang) * 2.0;
      Cbn[2] = (cp - sp) * 2.0;
      Cbn[5] = (dt + ang) * 2.0;
      Cbn[8] = 1.0 - (n_meas + dt_meas) * 2.0;
      dt = 0.0;
      dt_meas = 0.0;
      ang = 0.0;
      for (i = 0; i < 3; i++) {
        n_meas = rtb_state_out.accel_body[i];
        dt += Cbn[3 * i] * n_meas;
        dt_meas += Cbn[3 * i + 1] * n_meas;
        ang += Cbn[3 * i + 2] * n_meas;
      }

      memset(&Fdt[0], 0, 225U * sizeof(real_T));
      Fdt[3] = -0.0;
      Fdt[18] = ang;
      Fdt[33] = -dt_meas;
      Fdt[4] = -ang;
      Fdt[19] = -0.0;
      Fdt[34] = dt;
      Fdt[5] = dt_meas;
      Fdt[20] = -dt;
      Fdt[35] = -0.0;
      for (i = 0; i < 3; i++) {
        dt = Cbn[3 * i];
        Fdt_tmp = (i + 12) * 15;
        Fdt[Fdt_tmp] = -dt;
        Fdt_tmp_0 = (i + 9) * 15;
        Fdt[Fdt_tmp_0 + 3] = dt;
        dt = Cbn[3 * i + 1];
        Fdt[Fdt_tmp + 1] = -dt;
        Fdt[Fdt_tmp_0 + 4] = dt;
        dt = Cbn[3 * i + 2];
        Fdt[Fdt_tmp + 2] = -dt;
        Fdt[Fdt_tmp_0 + 5] = dt;
      }

      memset(&Cbn[0], 0, 9U * sizeof(real_T));
      Cbn[0] = 1.0;
      Cbn[4] = 1.0;
      Cbn[8] = 1.0;
      for (i = 0; i < 3; i++) {
        Fdt_tmp = (i + 3) * 15;
        Fdt[Fdt_tmp + 6] = Cbn[3 * i];
        Fdt[Fdt_tmp + 7] = Cbn[3 * i + 1];
        Fdt[Fdt_tmp + 8] = Cbn[3 * i + 2];
      }

      for (i = 0; i < 225; i++) {
        Fdt[i] *= 0.0025;
        P_pred[i] = 0.0;
      }

      for (b_k = 0; b_k < 15; b_k++) {
        P_pred[b_k + 15 * b_k] = 1.0;
        memset(&Fdt_0[b_k * 15], 0, 15U * sizeof(real_T));
        for (i = 0; i < 15; i++) {
          n_meas = Fdt[15 * b_k + i];
          for (Cbn_0 = 0; Cbn_0 < 15; Cbn_0++) {
            Fdt_tmp = 15 * b_k + Cbn_0;
            Fdt_0[Fdt_tmp] += Fdt[15 * i + Cbn_0] * n_meas;
          }
        }
      }

      omega_b[0] = rtb_state_out.accel_body[0];
      omega_b[1] = rtb_state_out.accel_body[1];
      omega_b[2] = rtb_state_out.accel_body[2];
      dt = ((1.0 - rtb_state_out.imu_blend_w) * 0.02 + rtb_state_out.imu_blend_w
            * 0.35) + 0.01 * nav_norm(omega_b);
      omega_b[0] = rtb_state_out.gyro_body[0];
      omega_b[1] = rtb_state_out.gyro_body[1];
      omega_b[2] = rtb_state_out.gyro_body[2];
      dt_meas = 0.005 * nav_norm(omega_b) + 0.00052359877559829881;
      cy = dt_meas * dt_meas;
      sy = dt * dt;
      for (i = 0; i < 225; i++) {
        Phi[i] = (P_pred[i] + Fdt[i]) + 0.5 * Fdt_0[i];
        Fdt[i] = 0.0;
      }

      for (i = 0; i < 9; i++) {
        Cbn[i] = c_a[i];
      }

      for (i = 0; i < 3; i++) {
        Cbn_0 = (int32_T)Cbn[3 * i];
        Fdt[15 * i] = (real_T)Cbn_0 * cy;
        Fdt_tmp = (i + 3) * 15;
        Fdt[Fdt_tmp + 3] = (real_T)Cbn_0 * sy;
        Fdt_tmp_0 = (i + 9) * 15;
        Fdt[Fdt_tmp_0 + 9] = (real_T)Cbn_0 * 1.0E-8;
        b_k = (i + 12) * 15;
        Fdt[b_k + 12] = (real_T)Cbn_0 * 1.0E-12;
        Cbn_0 = (int32_T)Cbn[3 * i + 1];
        Fdt[15 * i + 1] = (real_T)Cbn_0 * cy;
        Fdt[Fdt_tmp + 4] = (real_T)Cbn_0 * sy;
        Fdt[Fdt_tmp_0 + 10] = (real_T)Cbn_0 * 1.0E-8;
        Fdt[b_k + 13] = (real_T)Cbn_0 * 1.0E-12;
        Cbn_0 = (int32_T)Cbn[3 * i + 2];
        Fdt[15 * i + 2] = (real_T)Cbn_0 * cy;
        Fdt[Fdt_tmp + 5] = (real_T)Cbn_0 * sy;
        Fdt[Fdt_tmp_0 + 11] = (real_T)Cbn_0 * 1.0E-8;
        Fdt[b_k + 14] = (real_T)Cbn_0 * 1.0E-12;
      }

      for (i = 0; i < 15; i++) {
        for (Cbn_0 = 0; Cbn_0 < 15; Cbn_0++) {
          Fdt_tmp = 15 * i + Cbn_0;
          Fdt_0[Fdt_tmp] = Phi[15 * Cbn_0 + i];
          Phi_0[Fdt_tmp] = 0.0;
        }

        for (Cbn_0 = 0; Cbn_0 < 15; Cbn_0++) {
          Fdt_tmp_0 = 15 * i + Cbn_0;
          n_meas = rtb_state_out.P[Fdt_tmp_0];
          for (Fdt_tmp = 0; Fdt_tmp < 15; Fdt_tmp++) {
            b_k = 15 * i + Fdt_tmp;
            Phi_0[b_k] += Phi[15 * Cbn_0 + Fdt_tmp] * n_meas;
          }

          Phi_1[Fdt_tmp_0] = 0.0;
        }

        for (Cbn_0 = 0; Cbn_0 < 15; Cbn_0++) {
          n_meas = Fdt[15 * i + Cbn_0];
          for (Fdt_tmp_0 = 0; Fdt_tmp_0 < 15; Fdt_tmp_0++) {
            b_k = 15 * i + Fdt_tmp_0;
            Phi_1[b_k] += Phi[15 * Cbn_0 + Fdt_tmp_0] * n_meas;
          }
        }
      }

      for (i = 0; i < 15; i++) {
        for (Cbn_0 = 0; Cbn_0 < 15; Cbn_0++) {
          n_meas = 0.0;
          ang = 0.0;
          for (Fdt_tmp_0 = 0; Fdt_tmp_0 < 15; Fdt_tmp_0++) {
            dt = Fdt_0[15 * Cbn_0 + Fdt_tmp_0];
            Fdt_tmp = 15 * Fdt_tmp_0 + i;
            n_meas += Phi_1[Fdt_tmp] * dt;
            ang += Phi_0[Fdt_tmp] * dt;
          }

          b_k = 15 * Cbn_0 + i;
          P_pred[b_k] = (Fdt[b_k] + n_meas) * 0.5 * 0.0025 + ang;
        }
      }

      for (i = 0; i < 15; i++) {
        for (Cbn_0 = 0; Cbn_0 < 15; Cbn_0++) {
          Fdt_tmp_0 = 15 * i + Cbn_0;
          rtb_state_out.P[Fdt_tmp_0] = (real32_T)((P_pred[15 * Cbn_0 + i] +
            P_pred[Fdt_tmp_0]) * 0.5);
        }
      }
    }
  }

  /* End of MATLAB Function: '<Root>/INS_Mechanization' */

  /* MATLAB Function: '<Root>/EKF_Update' incorporates:
   *  BusCreator generated from: '<Root>/EKF_Update'
   *  BusCreator generated from: '<Root>/INS_Mechanization'
   *  Inport: '<Root>/nav_in'
   */
  nav_DW.NavState_Memory_PreviousInput = rtb_state_out;
  if ((rtb_state_out.origin_set == 0) && ((nav_U.nav_in.gnss.data_valid == 1) &&
       (nav_U.nav_in.gnss.fix_type >= 3))) {
    nav_DW.NavState_Memory_PreviousInput.origin_lla[0] =
      nav_U.nav_in.gnss.pos_lla[0];
    nav_DW.NavState_Memory_PreviousInput.origin_lla[1] =
      nav_U.nav_in.gnss.pos_lla[1];
    nav_DW.NavState_Memory_PreviousInput.origin_lla[2] =
      nav_U.nav_in.gnss.pos_lla[2];
    nav_DW.NavState_Memory_PreviousInput.origin_set = 1U;
  }

  if ((rtb_state_out.init_done == 1) && (rtb_state_out.gnss_only == 0)) {
    if (rtb_state_out.vel_ned[2] > 2.0F) {
      if (-(real_T)rtb_state_out.pos_ned[2] >= 100.0) {
        if (nav_DW.NavState_Memory_PreviousInput.descent_cnt < 65535) {
          tmp_1 = rtb_state_out.descent_cnt + 1U;
          if (rtb_state_out.descent_cnt + 1U > 65535U) {
            tmp_1 = 65535U;
          }

          nav_DW.NavState_Memory_PreviousInput.descent_cnt = (uint16_T)tmp_1;
        }
      } else {
        nav_DW.NavState_Memory_PreviousInput.descent_cnt = 0U;
      }
    } else {
      nav_DW.NavState_Memory_PreviousInput.descent_cnt = 0U;
    }

    if (nav_DW.NavState_Memory_PreviousInput.descent_cnt >= 200) {
      nav_DW.NavState_Memory_PreviousInput.gnss_only = 1U;
    }
  }

  use_pos = ((nav_U.nav_in.gnss.data_valid == 1) && (nav_U.nav_in.gnss.fix_type >=
              3) && (nav_U.nav_in.gnss.num_sats >= 5));
  if (rtb_state_out.gnss_only == 1) {
    if (use_pos && (nav_DW.NavState_Memory_PreviousInput.origin_set == 1)) {
      dt = sin(nav_DW.NavState_Memory_PreviousInput.origin_lla[0]);
      dt = 1.0 - 0.00669437999014 * dt * dt;
      dt_meas = sqrt(dt);
      nav_DW.NavState_Memory_PreviousInput.pos_ned[0] = (real32_T)
        ((6.3354393272928288E+6 / (dt * dt_meas) +
          nav_DW.NavState_Memory_PreviousInput.origin_lla[2]) *
         (nav_U.nav_in.gnss.pos_lla[0] -
          nav_DW.NavState_Memory_PreviousInput.origin_lla[0]));
      nav_DW.NavState_Memory_PreviousInput.pos_ned[1] = (real32_T)((6.378137E+6 /
        dt_meas + nav_DW.NavState_Memory_PreviousInput.origin_lla[2]) *
        (nav_U.nav_in.gnss.pos_lla[1] -
         nav_DW.NavState_Memory_PreviousInput.origin_lla[1]) * cos
        (nav_DW.NavState_Memory_PreviousInput.origin_lla[0]));
      nav_DW.NavState_Memory_PreviousInput.pos_ned[2] = (real32_T)
        -(nav_U.nav_in.gnss.pos_lla[2] -
          nav_DW.NavState_Memory_PreviousInput.origin_lla[2]);
      nav_DW.NavState_Memory_PreviousInput.vel_ned[0] =
        nav_U.nav_in.gnss.vel_ned[0];
      nav_DW.NavState_Memory_PreviousInput.vel_ned[1] =
        nav_U.nav_in.gnss.vel_ned[1];
      nav_DW.NavState_Memory_PreviousInput.vel_ned[2] =
        nav_U.nav_in.gnss.vel_ned[2];
      nav_DW.NavState_Memory_PreviousInput.gnss_age_s = 0.0F;
      nav_DW.NavState_Memory_PreviousInput.reject_cnt = 0U;
    }
  } else {
    if (use_pos && (nav_DW.NavState_Memory_PreviousInput.origin_set == 1) &&
        (rtb_state_out.init_done == 1)) {
      dt = (real_T)rtb_state_out.quat_nb[3] * rtb_state_out.quat_nb[3];
      dt_meas = (real_T)rtb_state_out.quat_nb[2] * rtb_state_out.quat_nb[2];
      Cbn[0] = 1.0 - (dt_meas + dt) * 2.0;
      n_meas = (real_T)rtb_state_out.quat_nb[1] * rtb_state_out.quat_nb[2];
      ang = (real_T)rtb_state_out.quat_nb[0] * rtb_state_out.quat_nb[3];
      Cbn[3] = (n_meas - ang) * 2.0;
      cp = (real_T)rtb_state_out.quat_nb[1] * rtb_state_out.quat_nb[3];
      sp = (real_T)rtb_state_out.quat_nb[0] * rtb_state_out.quat_nb[2];
      Cbn[6] = (cp + sp) * 2.0;
      Cbn[1] = (n_meas + ang) * 2.0;
      n_meas = (real_T)rtb_state_out.quat_nb[1] * rtb_state_out.quat_nb[1];
      Cbn[4] = 1.0 - (n_meas + dt) * 2.0;
      dt = (real_T)rtb_state_out.quat_nb[2] * rtb_state_out.quat_nb[3];
      ang = (real_T)rtb_state_out.quat_nb[0] * rtb_state_out.quat_nb[1];
      Cbn[7] = (dt - ang) * 2.0;
      Cbn[2] = (cp - sp) * 2.0;
      Cbn[5] = (dt + ang) * 2.0;
      Cbn[8] = 1.0 - (n_meas + dt_meas) * 2.0;
      a_lg[0] = 0.0;
      a_lg[1] = 0.0;
      a_lg[2] = 0.0;
      dt = 0.0 * rtb_state_out.gyro_body[2];
      dt_meas = rtb_state_out.gyro_body[1] * 0.0;
      theta[0] = dt_meas - dt;
      n_meas = rtb_state_out.gyro_body[0] * 0.0;
      theta[1] = dt - n_meas;
      theta[2] = n_meas - dt_meas;
      dt = 0.0;
      dt_meas = 0.0;
      ang = 0.0;
      cp = 0.0;
      sp = 0.0;
      cy = 0.0;
      for (i = 0; i < 3; i++) {
        sy = Cbn[3 * i];
        dt += sy * 0.0;
        dt_meas_tmp = Cbn[3 * i + 1];
        dt_meas += dt_meas_tmp * 0.0;
        ang_tmp = Cbn[3 * i + 2];
        ang += ang_tmp * 0.0;
        n_meas = theta[i];
        cp += sy * n_meas;
        sp += dt_meas_tmp * n_meas;
        cy += ang_tmp * n_meas;
      }

      n_meas = sin(nav_DW.NavState_Memory_PreviousInput.origin_lla[0]);
      n_meas = 1.0 - 0.00669437999014 * n_meas * n_meas;
      sy = sqrt(n_meas);
      z[0] = (6.3354393272928288E+6 / (n_meas * sy) +
              nav_DW.NavState_Memory_PreviousInput.origin_lla[2]) *
        (nav_U.nav_in.gnss.pos_lla[0] -
         nav_DW.NavState_Memory_PreviousInput.origin_lla[0]) -
        (rtb_state_out.pos_ned[0] + dt);
      z[1] = (6.378137E+6 / sy +
              nav_DW.NavState_Memory_PreviousInput.origin_lla[2]) *
        (nav_U.nav_in.gnss.pos_lla[1] -
         nav_DW.NavState_Memory_PreviousInput.origin_lla[1]) * cos
        (nav_DW.NavState_Memory_PreviousInput.origin_lla[0]) -
        (rtb_state_out.pos_ned[1] + dt_meas);
      z[2] = -(nav_U.nav_in.gnss.pos_lla[2] -
               nav_DW.NavState_Memory_PreviousInput.origin_lla[2]) -
        (rtb_state_out.pos_ned[2] + ang);
      z[3] = nav_U.nav_in.gnss.vel_ned[0] - (rtb_state_out.vel_ned[0] + cp);
      z[4] = nav_U.nav_in.gnss.vel_ned[1] - (rtb_state_out.vel_ned[1] + sp);
      z[5] = nav_U.nav_in.gnss.vel_ned[2] - (rtb_state_out.vel_ned[2] + cy);
      memset(&H[0], 0, 90U * sizeof(real_T));
      nav_eye(H_tmp);
      for (i = 0; i < 3; i++) {
        n_meas = H_tmp[3 * i];
        Fdt_tmp = (i + 6) * 6;
        H[Fdt_tmp] = n_meas;
        Cbn_0 = (i + 3) * 6;
        H[Cbn_0 + 3] = n_meas;
        n_meas = H_tmp[3 * i + 1];
        H[Fdt_tmp + 1] = n_meas;
        H[Cbn_0 + 4] = n_meas;
        n_meas = H_tmp[3 * i + 2];
        H[Fdt_tmp + 2] = n_meas;
        H[Cbn_0 + 5] = n_meas;
      }

      if (nav_norm(a_lg) > 1.0E-6) {
        H[0] = -0.0;
        H[6] = ang;
        H[12] = -dt_meas;
        H[1] = -ang;
        H[7] = -0.0;
        H[13] = dt;
        H[2] = dt_meas;
        H[8] = -dt;
        H[14] = -0.0;
        H[3] = -0.0;
        H[9] = cy;
        H[15] = -sp;
        H[4] = -cy;
        H[10] = -0.0;
        H[16] = cp;
        H[5] = sp;
        H[11] = -cp;
        H[17] = -0.0;
        for (i = 0; i < 3; i++) {
          dt = 0.0;
          dt_meas = 0.0;
          n_meas = 0.0;
          for (Cbn_0 = 0; Cbn_0 < 3; Cbn_0++) {
            dt += Cbn[3 * Cbn_0] * 0.0;
            dt_meas += Cbn[3 * Cbn_0 + 1] * 0.0;
            n_meas += Cbn[3 * Cbn_0 + 2] * 0.0;
          }

          Fdt_tmp = (i + 12) * 6;
          H[Fdt_tmp + 5] = n_meas;
          H[Fdt_tmp + 4] = dt_meas;
          H[Fdt_tmp + 3] = dt;
        }
      }

      memset(&R[0], 0, 36U * sizeof(real_T));
      for (i = 0; i < 3; i++) {
        R[6 * i] = nav_ConstP.pooled1.R_gnss_pos[3 * i];
        Cbn_0 = (i + 3) * 6;
        R[Cbn_0 + 3] = nav_ConstP.pooled1.R_gnss_vel[3 * i];
        Fdt_tmp = 3 * i + 1;
        R[6 * i + 1] = nav_ConstP.pooled1.R_gnss_pos[Fdt_tmp];
        R[Cbn_0 + 4] = nav_ConstP.pooled1.R_gnss_vel[Fdt_tmp];
        Fdt_tmp = 3 * i + 2;
        R[6 * i + 2] = nav_ConstP.pooled1.R_gnss_pos[Fdt_tmp];
        R[Cbn_0 + 5] = nav_ConstP.pooled1.R_gnss_vel[Fdt_tmp];
      }

      for (i = 0; i < 225; i++) {
        Fdt[i] = rtb_state_out.P[i];
      }

      for (i = 0; i < 6; i++) {
        for (Cbn_0 = 0; Cbn_0 < 15; Cbn_0++) {
          K[Cbn_0 + 15 * i] = H[6 * Cbn_0 + i];
        }
      }

      for (i = 0; i < 15; i++) {
        for (Cbn_0 = 0; Cbn_0 < 6; Cbn_0++) {
          H_0[Cbn_0 + 6 * i] = 0.0;
        }

        for (Cbn_0 = 0; Cbn_0 < 15; Cbn_0++) {
          n_meas = Fdt[15 * i + Cbn_0];
          for (Fdt_tmp_0 = 0; Fdt_tmp_0 < 6; Fdt_tmp_0++) {
            Fdt_tmp = 6 * i + Fdt_tmp_0;
            H_0[Fdt_tmp] += H[6 * Cbn_0 + Fdt_tmp_0] * n_meas;
          }
        }
      }

      for (i = 0; i < 6; i++) {
        for (Cbn_0 = 0; Cbn_0 < 6; Cbn_0++) {
          n_meas = 0.0;
          for (Fdt_tmp_0 = 0; Fdt_tmp_0 < 15; Fdt_tmp_0++) {
            n_meas += H_0[6 * Fdt_tmp_0 + i] * K[15 * Cbn_0 + Fdt_tmp_0];
          }

          Fdt_tmp = 6 * Cbn_0 + i;
          S[Fdt_tmp] = R[Fdt_tmp] + n_meas;
        }
      }

      for (i = 0; i < 6; i++) {
        for (Cbn_0 = 0; Cbn_0 < 6; Cbn_0++) {
          Fdt_tmp_0 = 6 * i + Cbn_0;
          tmp_0[Fdt_tmp_0] = (S[6 * Cbn_0 + i] + S[Fdt_tmp_0]) * 0.5;
        }
      }

      memcpy(&S[0], &tmp_0[0], 36U * sizeof(real_T));
      for (i = 0; i < 3; i++) {
        Cbn[3 * i] = S[6 * i];
        Cbn[3 * i + 1] = S[6 * i + 1];
        Cbn[3 * i + 2] = S[6 * i + 2];
      }

      nav_mldivide(Cbn, &z[0], theta);
      use_pos = ((z[0] * theta[0] + z[1] * theta[1]) + z[2] * theta[2] <= 60.0);
      for (i = 0; i < 3; i++) {
        Fdt_tmp = (i + 3) * 6;
        Cbn[3 * i] = S[Fdt_tmp + 3];
        Cbn[3 * i + 1] = S[Fdt_tmp + 4];
        Cbn[3 * i + 2] = S[Fdt_tmp + 5];
      }

      nav_mldivide(Cbn, &z[3], theta);
      lg_ok = ((theta[0] * z[3] + theta[1] * z[4]) + theta[2] * z[5] <= 60.0);
      hg_ok = false;
      if ((!use_pos) && (!lg_ok)) {
        if ((rtb_state_out.reject_cnt >= 20) || (rtb_state_out.gnss_age_s > 1.0F))
        {
          use_pos = true;
          lg_ok = true;
          hg_ok = true;
        } else {
          nav_DW.NavState_Memory_PreviousInput.reject_cnt = (uint16_T)
            (rtb_state_out.reject_cnt + 1);
        }
      } else {
        hg_ok = (rtb_state_out.gnss_age_s > 1.0F);
      }

      if (hg_ok) {
        theta[0] = z[3] * z[3];
        theta[1] = z[4] * z[4];
        theta[2] = z[5] * z[5];
        for (i = 0; i < 9; i++) {
          Cbn[i] = c_a[i];
        }

        nav_diag(theta, H_tmp);
        for (b_k = 0; b_k < 3; b_k++) {
          Fdt_tmp_0 = (b_k + 3) * 15;
          Fdt[Fdt_tmp_0 + 3] = (Fdt[Fdt_tmp_0 + 3] + H_tmp[3 * b_k]) + Cbn[3 *
            b_k] * 25.0;
          Fdt_tmp = 3 * b_k + 1;
          Fdt[Fdt_tmp_0 + 4] = (Fdt[Fdt_tmp_0 + 4] + H_tmp[Fdt_tmp]) +
            Cbn[Fdt_tmp] * 25.0;
          Fdt_tmp = 3 * b_k + 2;
          Fdt[Fdt_tmp_0 + 5] = (Fdt[Fdt_tmp_0 + 5] + H_tmp[Fdt_tmp]) +
            Cbn[Fdt_tmp] * 25.0;
          dt = z[b_k];
          theta[b_k] = dt * dt;
        }

        nav_diag(theta, H_tmp);
        for (i = 0; i < 3; i++) {
          Cbn_0 = (int32_T)Cbn[3 * i];
          Fdt_tmp_0 = (i + 6) * 15;
          Fdt[Fdt_tmp_0 + 6] = (Fdt[Fdt_tmp_0 + 6] + H_tmp[3 * i]) + (real_T)
            Cbn_0 * 100.0;
          Fdt_tmp = (i + 9) * 15;
          Fdt[Fdt_tmp + 9] += (real_T)Cbn_0 * 0.25;
          b_k = 3 * i + 1;
          Cbn_0 = (int32_T)Cbn[b_k];
          Fdt[Fdt_tmp_0 + 7] = (Fdt[Fdt_tmp_0 + 7] + H_tmp[b_k]) + (real_T)Cbn_0
            * 100.0;
          Fdt[Fdt_tmp + 10] += (real_T)Cbn_0 * 0.25;
          b_k = 3 * i + 2;
          Cbn_0 = (int32_T)Cbn[b_k];
          Fdt[Fdt_tmp_0 + 8] = (Fdt[Fdt_tmp_0 + 8] + H_tmp[b_k]) + (real_T)Cbn_0
            * 100.0;
          Fdt[Fdt_tmp + 11] += (real_T)Cbn_0 * 0.25;
        }

        for (i = 0; i < 15; i++) {
          for (Cbn_0 = 0; Cbn_0 < 15; Cbn_0++) {
            Fdt_tmp_0 = 15 * i + Cbn_0;
            Fdt_0[Fdt_tmp_0] = (Fdt[15 * Cbn_0 + i] + Fdt[Fdt_tmp_0]) * 0.5;
          }
        }

        memcpy(&Fdt[0], &Fdt_0[0], 225U * sizeof(real_T));
      }

      if (use_pos || lg_ok) {
        if (!use_pos) {
          for (i = 0; i < 3; i++) {
            R[6 * i] = d[3 * i];
            R[6 * i + 1] = d[3 * i + 1];
            R[6 * i + 2] = d[3 * i + 2];
          }
        }

        if (!lg_ok) {
          for (i = 0; i < 3; i++) {
            Cbn_0 = (i + 3) * 6;
            R[Cbn_0 + 3] = d[3 * i];
            R[Cbn_0 + 4] = d[3 * i + 1];
            R[Cbn_0 + 5] = d[3 * i + 2];
          }
        }

        for (i = 0; i < 15; i++) {
          for (Cbn_0 = 0; Cbn_0 < 6; Cbn_0++) {
            H_0[Cbn_0 + 6 * i] = 0.0;
          }

          for (Cbn_0 = 0; Cbn_0 < 15; Cbn_0++) {
            n_meas = Fdt[15 * i + Cbn_0];
            for (Fdt_tmp_0 = 0; Fdt_tmp_0 < 6; Fdt_tmp_0++) {
              Fdt_tmp = 6 * i + Fdt_tmp_0;
              H_0[Fdt_tmp] += H[6 * Cbn_0 + Fdt_tmp_0] * n_meas;
            }
          }
        }

        for (i = 0; i < 6; i++) {
          for (Cbn_0 = 0; Cbn_0 < 6; Cbn_0++) {
            n_meas = 0.0;
            for (Fdt_tmp_0 = 0; Fdt_tmp_0 < 15; Fdt_tmp_0++) {
              n_meas += H_0[6 * Fdt_tmp_0 + i] * K[15 * Cbn_0 + Fdt_tmp_0];
            }

            Fdt_tmp = 6 * Cbn_0 + i;
            S[Fdt_tmp] = R[Fdt_tmp] + n_meas;
          }

          memset(&Fdt_1[i * 15], 0, 15U * sizeof(real_T));
          for (Cbn_0 = 0; Cbn_0 < 15; Cbn_0++) {
            n_meas = K[15 * i + Cbn_0];
            for (Fdt_tmp_0 = 0; Fdt_tmp_0 < 15; Fdt_tmp_0++) {
              Fdt_tmp = 15 * i + Fdt_tmp_0;
              Fdt_1[Fdt_tmp] += Fdt[15 * Cbn_0 + Fdt_tmp_0] * n_meas;
            }
          }
        }

        memcpy(&K[0], &Fdt_1[0], 90U * sizeof(real_T));
        for (i = 0; i < 6; i++) {
          for (Cbn_0 = 0; Cbn_0 < 6; Cbn_0++) {
            Fdt_tmp_0 = 6 * i + Cbn_0;
            tmp_0[Fdt_tmp_0] = (S[6 * Cbn_0 + i] + S[Fdt_tmp_0]) * 0.5;
          }
        }

        nav_mrdiv_o(K, tmp_0);
        nav_eye_d(Fdt_0);
        for (i = 0; i < 15; i++) {
          for (Cbn_0 = 0; Cbn_0 < 15; Cbn_0++) {
            n_meas = 0.0;
            for (Fdt_tmp_0 = 0; Fdt_tmp_0 < 6; Fdt_tmp_0++) {
              n_meas += K[15 * Fdt_tmp_0 + i] * H[6 * Cbn_0 + Fdt_tmp_0];
            }

            b_k = 15 * Cbn_0 + i;
            P_pred[b_k] = Fdt_0[b_k] - n_meas;
            Phi[Cbn_0 + 15 * i] = 0.0;
          }
        }

        for (i = 0; i < 15; i++) {
          for (Cbn_0 = 0; Cbn_0 < 15; Cbn_0++) {
            n_meas = Fdt[15 * i + Cbn_0];
            for (Fdt_tmp_0 = 0; Fdt_tmp_0 < 15; Fdt_tmp_0++) {
              b_k = 15 * i + Fdt_tmp_0;
              Phi[b_k] += P_pred[15 * Cbn_0 + Fdt_tmp_0] * n_meas;
            }
          }
        }

        for (i = 0; i < 6; i++) {
          memset(&H[i * 15], 0, 15U * sizeof(real_T));
          for (Cbn_0 = 0; Cbn_0 < 6; Cbn_0++) {
            n_meas = R[6 * i + Cbn_0];
            for (Fdt_tmp_0 = 0; Fdt_tmp_0 < 15; Fdt_tmp_0++) {
              Fdt_tmp = 15 * i + Fdt_tmp_0;
              H[Fdt_tmp] += K[15 * Cbn_0 + Fdt_tmp_0] * n_meas;
            }
          }
        }

        for (i = 0; i < 15; i++) {
          memset(&Fdt_0[i * 15], 0, 15U * sizeof(real_T));
          for (Cbn_0 = 0; Cbn_0 < 15; Cbn_0++) {
            n_meas = P_pred[15 * Cbn_0 + i];
            for (Fdt_tmp_0 = 0; Fdt_tmp_0 < 15; Fdt_tmp_0++) {
              b_k = 15 * i + Fdt_tmp_0;
              Fdt_0[b_k] += Phi[15 * Cbn_0 + Fdt_tmp_0] * n_meas;
            }

            Phi_0[Cbn_0 + 15 * i] = 0.0;
          }

          for (Cbn_0 = 0; Cbn_0 < 6; Cbn_0++) {
            n_meas = K[15 * Cbn_0 + i];
            for (Fdt_tmp_0 = 0; Fdt_tmp_0 < 15; Fdt_tmp_0++) {
              Fdt_tmp = 15 * i + Fdt_tmp_0;
              Phi_0[Fdt_tmp] += H[15 * Cbn_0 + Fdt_tmp_0] * n_meas;
            }
          }
        }

        for (i = 0; i < 225; i++) {
          Fdt[i] = Fdt_0[i] + Phi_0[i];
        }

        memset(&K_b[0], 0, 15U * sizeof(real_T));
        for (i = 0; i < 6; i++) {
          n_meas = z[i];
          for (Cbn_0 = 0; Cbn_0 < 15; Cbn_0++) {
            K_b[Cbn_0] += K[15 * i + Cbn_0] * n_meas;
          }
        }

        for (i = 0; i < 15; i++) {
          for (Cbn_0 = 0; Cbn_0 < 15; Cbn_0++) {
            Fdt_tmp_0 = 15 * i + Cbn_0;
            Fdt_0[Fdt_tmp_0] = (Fdt[15 * Cbn_0 + i] + Fdt[Fdt_tmp_0]) * 0.5;
          }
        }

        nav_apply_correction(&nav_DW.NavState_Memory_PreviousInput,
                             rtb_state_out.quat_nb, rtb_state_out.pos_ned,
                             rtb_state_out.vel_ned, rtb_state_out.accel_body,
                             rtb_state_out.accel_bias, rtb_state_out.gyro_bias,
                             K_b, Fdt_0, 9.80665, 0.980665, 0.087266462599716474,
                             2.0, true);
        nav_DW.NavState_Memory_PreviousInput.gnss_age_s = 0.0F;
        nav_DW.NavState_Memory_PreviousInput.reject_cnt = 0U;
      }
    }

    if ((nav_U.nav_in.baro.data_valid == 1) &&
        (nav_DW.NavState_Memory_PreviousInput.baro_ref_set == 1) &&
        (rtb_state_out.init_done == 1) && (nav_U.nav_in.baro.press_pa > 1000.0F)
        && (nav_DW.NavState_Memory_PreviousInput.baro_ref_pa > 1000.0F)) {
      dt_meas = (1.0 - rt_powd_snf((real_T)nav_U.nav_in.baro.press_pa /
                  nav_DW.NavState_Memory_PreviousInput.baro_ref_pa,
                  0.19026120030795432)) * 44330.769230769227;
      dt = dt_meas - (-(real_T)nav_DW.NavState_Memory_PreviousInput.pos_ned[2]);
      theta[0] = nav_DW.NavState_Memory_PreviousInput.vel_ned[0];
      theta[1] = nav_DW.NavState_Memory_PreviousInput.vel_ned[1];
      theta[2] = nav_DW.NavState_Memory_PreviousInput.vel_ned[2];
      n_meas = nav_norm(theta);
      ang = nav_U.nav_in.baro.press_pa / 101325.0 * 1.225 * 0.5 * n_meas *
        n_meas;
      use_pos = (n_meas / sqrt(fmax(-0.0065 * dt_meas + 288.15, 150.0) * 401.87)
                 > 0.75);
      if (use_pos) {
        nav_DW.NavState_Memory_PreviousInput.baro_rej_cnt = 0U;
      }

      dt_meas = 2.7777778086601757E-6 * ang * ang + 0.25;
      for (i = 0; i < 225; i++) {
        Fdt[i] = nav_DW.NavState_Memory_PreviousInput.P[i];
      }

      for (i = 0; i < 15; i++) {
        S_b_tmp[i] = b_a[i];
        S_b_tmp_0[i] = b_a[i];
        K_b[i] = 0.0;
      }

      n_meas = 0.0;
      for (i = 0; i < 15; i++) {
        ang = K_b[i];
        for (Cbn_0 = 0; Cbn_0 < 15; Cbn_0++) {
          ang += Fdt[15 * i + Cbn_0] * (real_T)S_b_tmp[Cbn_0];
        }

        K_b[i] = ang;
        n_meas += ang * (real_T)S_b_tmp_0[i];
      }

      ang = n_meas + dt_meas;
      if ((ang > 1.0E-9) && (!use_pos)) {
        n_meas = dt * dt;
        use_pos = (n_meas / ang <= 40.0);
        if (!use_pos) {
          if (nav_DW.NavState_Memory_PreviousInput.baro_rej_cnt >= 33) {
            use_pos = true;
            Fdt[128] += n_meas;
            n_meas = 0.0;
            for (i = 0; i < 15; i++) {
              ang = 0.0;
              for (Cbn_0 = 0; Cbn_0 < 15; Cbn_0++) {
                ang += Fdt[15 * i + Cbn_0] * (real_T)S_b_tmp[Cbn_0];
              }

              n_meas += ang * (real_T)S_b_tmp_0[i];
            }

            ang = n_meas + dt_meas;
          } else {
            nav_DW.NavState_Memory_PreviousInput.baro_rej_cnt++;
          }
        }

        if (use_pos && (ang > 1.0E-9)) {
          for (i = 0; i < 15; i++) {
            n_meas = 0.0;
            for (Cbn_0 = 0; Cbn_0 < 15; Cbn_0++) {
              n_meas += Fdt[15 * Cbn_0 + i] * (real_T)S_b_tmp_0[Cbn_0];
            }

            K_b[i] = n_meas / ang;
          }

          nav_eye_d(Fdt_0);
          for (i = 0; i < 15; i++) {
            for (Cbn_0 = 0; Cbn_0 < 15; Cbn_0++) {
              b_k = 15 * i + Cbn_0;
              P_pred[b_k] = Fdt_0[b_k] - K_b[Cbn_0] * (real_T)S_b_tmp[i];
              Phi[b_k] = 0.0;
            }
          }

          for (i = 0; i < 15; i++) {
            for (Cbn_0 = 0; Cbn_0 < 15; Cbn_0++) {
              Fdt_tmp_0 = 15 * i + Cbn_0;
              n_meas = Fdt[Fdt_tmp_0];
              for (Fdt_tmp = 0; Fdt_tmp < 15; Fdt_tmp++) {
                b_k = 15 * i + Fdt_tmp;
                Phi[b_k] += P_pred[15 * Cbn_0 + Fdt_tmp] * n_meas;
              }

              Fdt_0[Fdt_tmp_0] = 0.0;
            }
          }

          for (i = 0; i < 15; i++) {
            for (Cbn_0 = 0; Cbn_0 < 15; Cbn_0++) {
              n_meas = P_pred[15 * Cbn_0 + i];
              for (Fdt_tmp_0 = 0; Fdt_tmp_0 < 15; Fdt_tmp_0++) {
                b_k = 15 * i + Fdt_tmp_0;
                Fdt_0[b_k] += Phi[15 * Cbn_0 + Fdt_tmp_0] * n_meas;
              }

              Phi_0[Cbn_0 + 15 * i] = K_b[Cbn_0] * dt_meas * K_b[i];
            }
          }

          for (i = 0; i < 225; i++) {
            Fdt[i] = Fdt_0[i] + Phi_0[i];
          }

          for (i = 0; i < 15; i++) {
            K_b_0[i] = K_b[i] * dt;
            for (Cbn_0 = 0; Cbn_0 < 15; Cbn_0++) {
              Fdt_tmp_0 = 15 * i + Cbn_0;
              Fdt_0[Fdt_tmp_0] = (Fdt[15 * Cbn_0 + i] + Fdt[Fdt_tmp_0]) * 0.5;
            }
          }

          for (i = 0; i < 4; i++) {
            tmp_3[i] = nav_DW.NavState_Memory_PreviousInput.quat_nb[i];
          }

          for (i = 0; i < 3; i++) {
            tmp_4[i] = nav_DW.NavState_Memory_PreviousInput.pos_ned[i];
          }

          for (i = 0; i < 3; i++) {
            tmp_5[i] = nav_DW.NavState_Memory_PreviousInput.vel_ned[i];
          }

          for (i = 0; i < 3; i++) {
            tmp_6[i] = nav_DW.NavState_Memory_PreviousInput.accel_bias[i];
          }

          for (i = 0; i < 3; i++) {
            tmp_7[i] = nav_DW.NavState_Memory_PreviousInput.gyro_bias[i];
          }

          nav_apply_correction(&nav_DW.NavState_Memory_PreviousInput, tmp_3,
                               tmp_4, tmp_5,
                               nav_DW.NavState_Memory_PreviousInput.accel_body,
                               tmp_6, tmp_7, K_b_0, Fdt_0, 9.80665, 0.980665,
                               0.087266462599716474, 2.0, false);
          nav_DW.NavState_Memory_PreviousInput.baro_rej_cnt = 0U;
        }
      }
    }
  }

  tmp[0] = ((!rtIsInf(nav_DW.NavState_Memory_PreviousInput.quat_nb[0])) &&
            (!rtIsNaN(nav_DW.NavState_Memory_PreviousInput.quat_nb[0])));
  tmp[1] = ((!rtIsInf(nav_DW.NavState_Memory_PreviousInput.quat_nb[1])) &&
            (!rtIsNaN(nav_DW.NavState_Memory_PreviousInput.quat_nb[1])));
  tmp[2] = ((!rtIsInf(nav_DW.NavState_Memory_PreviousInput.quat_nb[2])) &&
            (!rtIsNaN(nav_DW.NavState_Memory_PreviousInput.quat_nb[2])));
  tmp[3] = ((!rtIsInf(nav_DW.NavState_Memory_PreviousInput.quat_nb[3])) &&
            (!rtIsNaN(nav_DW.NavState_Memory_PreviousInput.quat_nb[3])));
  if (nav_all(tmp)) {
    theta[0] = nav_DW.NavState_Memory_PreviousInput.pos_ned[0];
    theta[1] = nav_DW.NavState_Memory_PreviousInput.pos_ned[1];
    theta[2] = nav_DW.NavState_Memory_PreviousInput.pos_ned[2];
    nav_isfinite(theta, tmp_2);
    if (nav_all_h(tmp_2)) {
      theta[0] = nav_DW.NavState_Memory_PreviousInput.vel_ned[0];
      theta[1] = nav_DW.NavState_Memory_PreviousInput.vel_ned[1];
      theta[2] = nav_DW.NavState_Memory_PreviousInput.vel_ned[2];
      nav_isfinite(theta, tmp_2);
      if (nav_all_h(tmp_2)) {
        theta[0] = nav_DW.NavState_Memory_PreviousInput.accel_bias[0];
        theta[1] = nav_DW.NavState_Memory_PreviousInput.accel_bias[1];
        theta[2] = nav_DW.NavState_Memory_PreviousInput.accel_bias[2];
        nav_isfinite(theta, tmp_2);
        if (nav_all_h(tmp_2)) {
          theta[0] = nav_DW.NavState_Memory_PreviousInput.gyro_bias[0];
          theta[1] = nav_DW.NavState_Memory_PreviousInput.gyro_bias[1];
          theta[2] = nav_DW.NavState_Memory_PreviousInput.gyro_bias[2];
          nav_isfinite(theta, tmp_2);
          use_pos = nav_all_h(tmp_2);
        } else {
          use_pos = false;
        }
      } else {
        use_pos = false;
      }
    } else {
      use_pos = false;
    }
  } else {
    use_pos = false;
  }

  if (!use_pos) {
    nav_DW.NavState_Memory_PreviousInput.quat_nb[0] = 1.0F;
    nav_DW.NavState_Memory_PreviousInput.quat_nb[1] = 0.0F;
    nav_DW.NavState_Memory_PreviousInput.quat_nb[2] = 0.0F;
    nav_DW.NavState_Memory_PreviousInput.quat_nb[3] = 0.0F;
    nav_DW.NavState_Memory_PreviousInput.pos_ned[0] = 0.0F;
    nav_DW.NavState_Memory_PreviousInput.vel_ned[0] = 0.0F;
    nav_DW.NavState_Memory_PreviousInput.accel_bias[0] = 0.0F;
    nav_DW.NavState_Memory_PreviousInput.gyro_bias[0] = 0.0F;
    nav_DW.NavState_Memory_PreviousInput.pos_ned[1] = 0.0F;
    nav_DW.NavState_Memory_PreviousInput.vel_ned[1] = 0.0F;
    nav_DW.NavState_Memory_PreviousInput.accel_bias[1] = 0.0F;
    nav_DW.NavState_Memory_PreviousInput.gyro_bias[1] = 0.0F;
    nav_DW.NavState_Memory_PreviousInput.pos_ned[2] = 0.0F;
    nav_DW.NavState_Memory_PreviousInput.vel_ned[2] = 0.0F;
    nav_DW.NavState_Memory_PreviousInput.accel_bias[2] = 0.0F;
    nav_DW.NavState_Memory_PreviousInput.gyro_bias[2] = 0.0F;
    memcpy(&nav_DW.NavState_Memory_PreviousInput.P[0], &nav_ConstP.pooled1.P0[0],
           225U * sizeof(real32_T));
    nav_DW.NavState_Memory_PreviousInput.init_done = 0U;
    nav_DW.NavState_Memory_PreviousInput.reject_cnt = 0U;
    nav_DW.NavState_Memory_PreviousInput.align_cnt = 0U;
    nav_DW.NavState_Memory_PreviousInput.acc_sum[0] = 0.0F;
    nav_DW.NavState_Memory_PreviousInput.gyr_sum[0] = 0.0F;
    nav_DW.NavState_Memory_PreviousInput.acc_sum[1] = 0.0F;
    nav_DW.NavState_Memory_PreviousInput.gyr_sum[1] = 0.0F;
    nav_DW.NavState_Memory_PreviousInput.acc_sum[2] = 0.0F;
    nav_DW.NavState_Memory_PreviousInput.gyr_sum[2] = 0.0F;
  }

  /* Outport: '<Root>/nav_out' incorporates:
   *  MATLAB Function: '<Root>/EKF_Update'
   */
  nav_Y.nav_out.pos_ned[0] = nav_DW.NavState_Memory_PreviousInput.pos_ned[0];
  nav_Y.nav_out.vel_ned[0] = nav_DW.NavState_Memory_PreviousInput.vel_ned[0];
  nav_Y.nav_out.pos_ned[1] = nav_DW.NavState_Memory_PreviousInput.pos_ned[1];
  nav_Y.nav_out.vel_ned[1] = nav_DW.NavState_Memory_PreviousInput.vel_ned[1];
  nav_Y.nav_out.pos_ned[2] = nav_DW.NavState_Memory_PreviousInput.pos_ned[2];
  nav_Y.nav_out.vel_ned[2] = nav_DW.NavState_Memory_PreviousInput.vel_ned[2];
  nav_Y.nav_out.quat_nb[0] = nav_DW.NavState_Memory_PreviousInput.quat_nb[0];
  nav_Y.nav_out.quat_nb[1] = nav_DW.NavState_Memory_PreviousInput.quat_nb[1];
  nav_Y.nav_out.quat_nb[2] = nav_DW.NavState_Memory_PreviousInput.quat_nb[2];
  nav_Y.nav_out.quat_nb[3] = nav_DW.NavState_Memory_PreviousInput.quat_nb[3];

  /* MATLAB Function: '<Root>/EKF_Update' */
  n_meas = (real_T)nav_DW.NavState_Memory_PreviousInput.quat_nb[2] *
    nav_DW.NavState_Memory_PreviousInput.quat_nb[2];

  /* Outport: '<Root>/nav_out' incorporates:
   *  MATLAB Function: '<Root>/EKF_Update'
   */
  nav_Y.nav_out.euler_rpy[0] = (real32_T)rt_atan2d_snf(((real_T)
    nav_DW.NavState_Memory_PreviousInput.quat_nb[2] *
    nav_DW.NavState_Memory_PreviousInput.quat_nb[3] + (real_T)
    nav_DW.NavState_Memory_PreviousInput.quat_nb[0] *
    nav_DW.NavState_Memory_PreviousInput.quat_nb[1]) * 2.0, 1.0 - ((real_T)
    nav_DW.NavState_Memory_PreviousInput.quat_nb[1] *
    nav_DW.NavState_Memory_PreviousInput.quat_nb[1] + n_meas) * 2.0);
  nav_Y.nav_out.euler_rpy[1] = (real32_T)asin(fmax(fmin(-(((real_T)
    nav_DW.NavState_Memory_PreviousInput.quat_nb[1] *
    nav_DW.NavState_Memory_PreviousInput.quat_nb[3] - (real_T)
    nav_DW.NavState_Memory_PreviousInput.quat_nb[0] *
    nav_DW.NavState_Memory_PreviousInput.quat_nb[2]) * 2.0), 1.0), -1.0));
  nav_Y.nav_out.euler_rpy[2] = (real32_T)rt_atan2d_snf(((real_T)
    nav_DW.NavState_Memory_PreviousInput.quat_nb[1] *
    nav_DW.NavState_Memory_PreviousInput.quat_nb[2] + (real_T)
    nav_DW.NavState_Memory_PreviousInput.quat_nb[0] *
    nav_DW.NavState_Memory_PreviousInput.quat_nb[3]) * 2.0, 1.0 - ((real_T)
    nav_DW.NavState_Memory_PreviousInput.quat_nb[3] *
    nav_DW.NavState_Memory_PreviousInput.quat_nb[3] + n_meas) * 2.0);
  nav_Y.nav_out.alt_agl_m = (real32_T)-(real_T)
    nav_DW.NavState_Memory_PreviousInput.pos_ned[2];
  nav_Y.nav_out.imu_blend_w = nav_DW.NavState_Memory_PreviousInput.imu_blend_w;
  nav_Y.nav_out.gnss_age_s = nav_DW.NavState_Memory_PreviousInput.gnss_age_s;
  nav_Y.nav_out.accel_ned[0] = nav_DW.NavState_Memory_PreviousInput.accel_ned[0];
  nav_Y.nav_out.accel_body[0] = nav_DW.NavState_Memory_PreviousInput.accel_body
    [0];
  nav_Y.nav_out.gyro_body[0] = nav_DW.NavState_Memory_PreviousInput.gyro_body[0];
  nav_Y.nav_out.accel_bias[0] = nav_DW.NavState_Memory_PreviousInput.accel_bias
    [0];
  nav_Y.nav_out.gyro_bias[0] = nav_DW.NavState_Memory_PreviousInput.gyro_bias[0];
  nav_Y.nav_out.origin_lla[0] = nav_DW.NavState_Memory_PreviousInput.origin_lla
    [0];
  nav_Y.nav_out.accel_ned[1] = nav_DW.NavState_Memory_PreviousInput.accel_ned[1];
  nav_Y.nav_out.accel_body[1] = nav_DW.NavState_Memory_PreviousInput.accel_body
    [1];
  nav_Y.nav_out.gyro_body[1] = nav_DW.NavState_Memory_PreviousInput.gyro_body[1];
  nav_Y.nav_out.accel_bias[1] = nav_DW.NavState_Memory_PreviousInput.accel_bias
    [1];
  nav_Y.nav_out.gyro_bias[1] = nav_DW.NavState_Memory_PreviousInput.gyro_bias[1];
  nav_Y.nav_out.origin_lla[1] = nav_DW.NavState_Memory_PreviousInput.origin_lla
    [1];
  nav_Y.nav_out.accel_ned[2] = nav_DW.NavState_Memory_PreviousInput.accel_ned[2];
  nav_Y.nav_out.accel_body[2] = nav_DW.NavState_Memory_PreviousInput.accel_body
    [2];
  nav_Y.nav_out.gyro_body[2] = nav_DW.NavState_Memory_PreviousInput.gyro_body[2];
  nav_Y.nav_out.accel_bias[2] = nav_DW.NavState_Memory_PreviousInput.accel_bias
    [2];
  nav_Y.nav_out.gyro_bias[2] = nav_DW.NavState_Memory_PreviousInput.gyro_bias[2];
  nav_Y.nav_out.origin_lla[2] = nav_DW.NavState_Memory_PreviousInput.origin_lla
    [2];
  nav_Y.nav_out.gnss_only = nav_DW.NavState_Memory_PreviousInput.gnss_only;
  nav_Y.nav_out.timestamp_s = nav_DW.NavState_Memory_PreviousInput.timestamp_s;
  nav_Y.nav_out.nav_valid = nav_DW.NavState_Memory_PreviousInput.init_done;
}

/* Model initialize function */
void nav_initialize(void)
{
  /* InitializeConditions for Memory: '<Root>/NavState_Memory' */
  nav_DW.NavState_Memory_PreviousInput =
    nav_ConstP.NavState_Memory_InitialConditio;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
