#include "pil_test.h"

#include <math.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "debug_out.h"
#include "gc.h"
#include "gnc.h"
#include "main.h"
#include "nav.h"
#include "state_machine.h"
#include "storage_stream.h"

/* === Formato de los archivos ============================================ */

#define PIL_MAGIC "LEEMPI12"
#define PIL_SD SD_CARD_1

/* Cada cuántas filas se da un paso de guiado (PIL_manifest.txt: gc a 50 Hz,
 * uno de cada 8 de nav y máquina de estados) */
#define PIL_GC_CADA GNC_GC_DECIMATION

/* PILNAVO.CSV: una fila de cada esto. Las 72609 enteras son ~40 MB de SD. */
#define PIL_NAV_CSV_CADA 4U

/* Valores de fábrica de nav_tun (nav_data.c de la v1.5). El dataset se
 * generó con los ajustables de fábrica, y gnc_init() pone los de la rampa. */
#define PIL_RAIL_ELEV_FABRICA 84.5f
#define PIL_RAIL_AZIM_FABRICA 0.0f

/* Los registros van empaquetados sin relleno, en el orden de
 * PIL_manifest.txt (el conversor tools/pil_csv_to_bin.py saca los tipos del
 * propio manifiesto). El tamaño que trae la cabecera se comprueba contra
 * sizeof() antes de leer nada. */

typedef struct __attribute__((packed)) {
  char magic[8];
  uint32_t record_bytes;
  uint32_t record_count;
  uint32_t field_count;
  char name[12];
} pil_header_t;

typedef struct __attribute__((packed)) {
  float lowg_accel[3];
  float lowg_gyro[3];
  float lowg_temp;
  double lowg_ts;
  uint8_t lowg_valid;
  float hg_accel[3];
  double hg_ts;
  uint8_t hg_valid;
  double gnss_pos_lla[3];
  float gnss_vel_ned[3];
  uint8_t fix_type;
  uint8_t num_sats;
  uint8_t gnss_valid;
  float baro_press_pa;
  float baro_alt_msl_m;
  float baro_temp_c;
  double baro_ts;
  uint8_t baro_valid;
  int32_t rocket_state;
  uint8_t nav_cmd; /* v1.5 */
} pil_nav_in_t;

typedef struct __attribute__((packed)) {
  float pos_ned[3];
  float vel_ned[3];
  float quat_nb[4];
  float euler_rpy[3];
  float accel_ned[3];
  float accel_body[3];
  float gyro_body[3];
  float accel_bias[3];
  float gyro_bias[3];
  float alt_agl_m;
  float imu_blend_w;
  float gnss_age_s;
  double origin_lla[3];
  uint8_t gnss_only;
  double timestamp_s;
  uint8_t nav_valid;
} pil_nav_exp_t;

typedef struct __attribute__((packed)) {
  float pos_ned[3];
  float vel_ned[3];
  float accel_body[3];
  float gyro_body[3];
  float alt_agl_m;
  float gnss_age_s;
  uint8_t nav_valid;
  double timestamp_s;
  uint8_t cmd_id;
  float cmd_param;
  uint8_t cmd_valid;
} pil_sm_in_t;

typedef struct __attribute__((packed)) {
  int32_t rocket_state;
  int32_t prev_state;
  float time_in_state_s;
  uint8_t error_code;
  double timestamp_s;
  uint8_t nav_cmd; /* v1.5 */
} pil_sm_exp_t;

typedef struct __attribute__((packed)) {
  float pos_ned[3];
  float vel_ned[3];
  float accel_ned[3];
  float accel_body[3];
  float alt_agl_m;
  float gnss_age_s;
  uint8_t nav_valid;
  double timestamp_s;
  int32_t rocket_state;
  uint8_t cmd_id;
  float cmd_param;
  uint8_t cmd_valid;
} pil_gc_in_t;

typedef struct __attribute__((packed)) {
  float actuator_cmd;
  float airbrake_deg;
  float apogee_pred_m;
  float apogee_error_m;
  float time_to_apogee_s;
  float accel_cmd_mps2;
  float accel_meas_mps2;
  uint8_t drogue_deploy_flag;
  uint8_t main_deploy_flag;
  int32_t flight_phase;
  uint8_t pred_valid;
  uint8_t pyro_armed;
  double timestamp_s;
} pil_gc_exp_t;

/* Lo que da tools/pil_csv_to_bin.py con el manifiesto de la v1.5 */
_Static_assert(sizeof(pil_nav_in_t) == 123U, "PILNAVI: no cuadra con el manifiesto");
_Static_assert(sizeof(pil_nav_exp_t) == 158U, "PILNAVE: no cuadra con el manifiesto");
_Static_assert(sizeof(pil_sm_in_t) == 71U, "PILSMI: no cuadra con el manifiesto");
_Static_assert(sizeof(pil_sm_exp_t) == 22U, "PILSME: no cuadra con el manifiesto");
_Static_assert(sizeof(pil_gc_in_t) == 75U, "PILGCI: no cuadra con el manifiesto");
_Static_assert(sizeof(pil_gc_exp_t) == 44U, "PILGCE: no cuadra con el manifiesto");

/* === Tolerancias (PIL_manifest.txt) ====================================== */

/* "Reales: |a-b| <= 1e-3 + 1e-4*|b|. Enteros y flags: coincidencia EXACTA."
 * Una discrepancia en un entero es una transición en otro paso: en vuelo, un
 * paracaídas abriéndose en otro momento. */
#define PIL_REL_TOL 1.0e-4
#define PIL_ABS_TOL 1.0e-3

/* === Estado ============================================================== */

static pil_test_result_t s_result;

static storage_stream_t s_summary;
static bool s_summary_open;
static uint8_t s_summary_buf[1024];
static uint8_t s_csv_buf[3][4096];
static char s_line[768];

/* Comparación de un modelo. Es una prueba SECUENCIAL: lo que importa es la
 * PRIMERA discrepancia, las siguientes suelen ser arrastre. La "peor" se mide
 * en fracción de la tolerancia, así que por debajo de 1,0 está todo dentro. */
typedef struct {
  const char *modelo;
  uint32_t pasos;
  uint32_t mismatches;
  uint32_t first_fail_step;
  const char *first_fail_name;
  double first_fail_got;
  double first_fail_want;
  const char *worst_name;
  double worst_ratio;
  uint32_t worst_step;
  uint32_t max_us;
} pil_check_t;

/* === Utilidades ========================================================== */

static uint32_t _cycles_to_us(uint32_t cycles) {
  return (uint32_t)((uint64_t)cycles / (SystemCoreClock / 1000000U));
}

static void _cycle_counter_init(void) {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/* Todo lo que sale por USB se guarda también en PILRES.TXT */
static void _report(const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  (void)vsnprintf(s_line, sizeof(s_line), fmt, ap);
  va_end(ap);

  DebugOut_Printf("%s", s_line);
  if (s_summary_open) {
    (void)storage_stream_puts(&s_summary, s_line);
  }
}

/* snprintf devuelve lo que habría escrito, no lo escrito: esto devuelve el
 * avance real y nunca se sale del buffer. */
static uint32_t _append(char *buf, uint32_t size, uint32_t used,
                        const char *fmt, ...) {
  if ((used + 1U) >= size) {
    return used;
  }
  va_list ap;
  va_start(ap, fmt);
  const int written = vsnprintf(&buf[used], size - used, fmt, ap);
  va_end(ap);
  if (written < 0) {
    return used;
  }
  const uint32_t advance = (uint32_t)written;
  return ((used + advance) < size) ? (used + advance) : (size - 1U);
}

static void _fallo(pil_check_t *c, uint32_t step, const char *name,
                   double got, double want) {
  if (c->mismatches == 0U) {
    c->first_fail_step = step;
    c->first_fail_name = name;
    c->first_fail_got = got;
    c->first_fail_want = want;
  }
  c->mismatches++;
}

static void _real(pil_check_t *c, uint32_t step, const char *name, double got,
                  double want) {
  const double tol = PIL_ABS_TOL + (PIL_REL_TOL * fabs(want));
  const double ratio = fabs(got - want) / tol;
  if (!(ratio <= c->worst_ratio)) { /* también NaN */
    c->worst_ratio = isnan(ratio) ? 1e9 : ratio;
    c->worst_name = name;
    c->worst_step = step;
  }
  if (!(ratio <= 1.0)) {
    _fallo(c, step, name, got, want);
  }
}

/* want apunta DENTRO de un registro empaquetado: puede estar sin alinear, así
 * que se lee byte a byte (memcpy). Nada de float* ahí: el Cortex-M7 da
 * HardFault al leer un float desalineado con la FPU (ver gnc_playback.c). */
static void _reals(pil_check_t *c, uint32_t step, const char *name,
                   const float *got, const void *want, uint32_t n) {
  const uint8_t *w = (const uint8_t *)want;
  for (uint32_t i = 0U; i < n; i++) {
    float v;
    memcpy(&v, &w[i * sizeof(float)], sizeof(v));
    _real(c, step, name, (double)got[i], (double)v);
  }
}

/* Dirección de un campo de un registro empaquetado, sin que el compilador
 * crea que es un float* alineado */
#define CAMPO(ptr, tipo, campo) \
  ((const void *)((const uint8_t *)(ptr) + offsetof(tipo, campo)))

static void _entero(pil_check_t *c, uint32_t step, const char *name,
                    int32_t got, int32_t want) {
  if (got != want) {
    _fallo(c, step, name, (double)got, (double)want);
  }
}

/* Abre un .BIN y comprueba que la cabecera es la que se espera */
static bool _open_vectors(storage_stream_t *st, const char *path,
                          uint32_t record_bytes, uint32_t *record_count) {
  if (!storage_stream_open_read(st, PIL_SD, path)) {
    _report("   FALTA %s en la SD1 (o no se puede abrir)\r\n", path);
    return false;
  }
  pil_header_t h;
  if (!storage_stream_read(st, &h, sizeof(h)) ||
      (memcmp(h.magic, PIL_MAGIC, sizeof(h.magic)) != 0)) {
    _report("   %s: no es un archivo PIL de la v1.5 (regenerar con"
            " pil_csv_to_bin.py)\r\n", path);
    (void)storage_stream_close(st);
    return false;
  }
  if (h.record_bytes != record_bytes) {
    _report("   %s: registro de %lu B, esperaba %lu (regenerar con"
            " pil_csv_to_bin.py)\r\n",
            path, (unsigned long)h.record_bytes, (unsigned long)record_bytes);
    (void)storage_stream_close(st);
    return false;
  }
  *record_count = h.record_count;
  return true;
}

static void _summary(const pil_check_t *c) {
  if (c->mismatches == 0U) {
    _report("   %-14s OK   %lu pasos, peor caso al %.2f de la tolerancia"
            " (%s, paso %lu), %lu us/paso max\r\n",
            c->modelo, (unsigned long)c->pasos, c->worst_ratio,
            (c->worst_name != NULL) ? c->worst_name : "-",
            (unsigned long)c->worst_step, (unsigned long)c->max_us);
  } else {
    _report("   %-14s FALLA  %lu discrepancias. LA PRIMERA (las demas suelen"
            " ser arrastre):\r\n",
            c->modelo, (unsigned long)c->mismatches);
    _report("   %-14s        paso %lu (t = %.4f s), %s: placa %.9g,"
            " esperado %.9g\r\n",
            "", (unsigned long)c->first_fail_step,
            (double)c->first_fail_step * (GNC_PERIOD_US * 1.0e-6),
            (c->first_fail_name != NULL) ? c->first_fail_name : "-",
            c->first_fail_got, c->first_fail_want);
  }
}

/* === Un paso de cada modelo ============================================== */

static void _nav_paso(const pil_nav_in_t *r, const pil_nav_exp_t *w,
                      uint32_t step, pil_check_t *c) {
  nav_bus_in *u = &nav_U.nav_in;
  memcpy(u->imu_lowg.accel_mps2, r->lowg_accel, sizeof(r->lowg_accel));
  memcpy(u->imu_lowg.gyro_rps, r->lowg_gyro, sizeof(r->lowg_gyro));
  u->imu_lowg.temp_C = r->lowg_temp;
  u->imu_lowg.timestamp_s = r->lowg_ts;
  u->imu_lowg.data_valid = r->lowg_valid;
  memcpy(u->imu_highg.accel_mps2, r->hg_accel, sizeof(r->hg_accel));
  u->imu_highg.timestamp_s = r->hg_ts;
  u->imu_highg.data_valid = r->hg_valid;
  memcpy(u->gnss.pos_lla, r->gnss_pos_lla, sizeof(r->gnss_pos_lla));
  memcpy(u->gnss.vel_ned, r->gnss_vel_ned, sizeof(r->gnss_vel_ned));
  u->gnss.fix_type = r->fix_type;
  u->gnss.num_sats = r->num_sats;
  u->gnss.data_valid = r->gnss_valid;
  u->baro.press_pa = r->baro_press_pa;
  u->baro.alt_msl_m = r->baro_alt_msl_m;
  u->baro.temp_C = r->baro_temp_c;
  u->baro.timestamp_s = r->baro_ts;
  u->baro.data_valid = r->baro_valid;
  u->rocket_state = r->rocket_state;
  u->nav_cmd = r->nav_cmd;

  const uint32_t t0 = DWT->CYCCNT;
  nav_step();
  const uint32_t us = _cycles_to_us(DWT->CYCCNT - t0);
  if (us > c->max_us) {
    c->max_us = us;
  }

  const nav_bus_out *o = &nav_Y.nav_out;
  _reals(c, step, "pos_ned", o->pos_ned, CAMPO(w, pil_nav_exp_t, pos_ned), 3U);
  _reals(c, step, "vel_ned", o->vel_ned, CAMPO(w, pil_nav_exp_t, vel_ned), 3U);
  _reals(c, step, "quat_nb", o->quat_nb, CAMPO(w, pil_nav_exp_t, quat_nb), 4U);
  _reals(c, step, "euler_rpy", o->euler_rpy, CAMPO(w, pil_nav_exp_t, euler_rpy), 3U);
  _reals(c, step, "accel_ned", o->accel_ned, CAMPO(w, pil_nav_exp_t, accel_ned), 3U);
  _reals(c, step, "accel_body", o->accel_body, CAMPO(w, pil_nav_exp_t, accel_body), 3U);
  _reals(c, step, "gyro_body", o->gyro_body, CAMPO(w, pil_nav_exp_t, gyro_body), 3U);
  _reals(c, step, "accel_bias", o->accel_bias, CAMPO(w, pil_nav_exp_t, accel_bias), 3U);
  _reals(c, step, "gyro_bias", o->gyro_bias, CAMPO(w, pil_nav_exp_t, gyro_bias), 3U);
  _real(c, step, "alt_agl_m", o->alt_agl_m, w->alt_agl_m);
  _real(c, step, "imu_blend_w", o->imu_blend_w, w->imu_blend_w);
  _real(c, step, "gnss_age_s", o->gnss_age_s, w->gnss_age_s);
  for (int i = 0; i < 3; i++) {
    _real(c, step, "origin_lla", o->origin_lla[i], w->origin_lla[i]);
  }
  _entero(c, step, "gnss_only", o->gnss_only, w->gnss_only);
  _real(c, step, "timestamp_s", o->timestamp_s, w->timestamp_s);
  _entero(c, step, "nav_valid", o->nav_valid, w->nav_valid);
}

static void _sm_paso(const pil_sm_in_t *r, const pil_sm_exp_t *w,
                     uint32_t step, pil_check_t *c) {
  sm_bus_in *u = &state_machine_U.sm_in;
  memcpy(u->pos_ned, r->pos_ned, sizeof(r->pos_ned));
  memcpy(u->vel_ned, r->vel_ned, sizeof(r->vel_ned));
  memcpy(u->accel_body, r->accel_body, sizeof(r->accel_body));
  memcpy(u->gyro_body, r->gyro_body, sizeof(r->gyro_body));
  u->alt_agl_m = r->alt_agl_m;
  u->gnss_age_s = r->gnss_age_s;
  u->nav_valid = r->nav_valid;
  u->timestamp_s = r->timestamp_s;
  u->telemetry_cmd.cmd_id = r->cmd_id;
  u->telemetry_cmd.cmd_param = r->cmd_param;
  u->telemetry_cmd.cmd_valid = r->cmd_valid;

  const uint32_t t0 = DWT->CYCCNT;
  state_machine_step();
  const uint32_t us = _cycles_to_us(DWT->CYCCNT - t0);
  if (us > c->max_us) {
    c->max_us = us;
  }

  const sm_bus_out *o = &state_machine_Y.sm_out;
  _entero(c, step, "rocket_state", o->rocket_state, w->rocket_state);
  _entero(c, step, "prev_state", o->prev_state, w->prev_state);
  _real(c, step, "time_in_state_s", o->time_in_state_s, w->time_in_state_s);
  _entero(c, step, "error_code", o->error_code, w->error_code);
  _real(c, step, "timestamp_s", o->timestamp_s, w->timestamp_s);
  _entero(c, step, "nav_cmd", o->nav_cmd, w->nav_cmd);
}

static void _gc_paso(const pil_gc_in_t *r, const pil_gc_exp_t *w,
                     uint32_t step, pil_check_t *c) {
  gc_bus_in *u = &gc_U.gc_in;
  memcpy(u->pos_ned, r->pos_ned, sizeof(r->pos_ned));
  memcpy(u->vel_ned, r->vel_ned, sizeof(r->vel_ned));
  memcpy(u->accel_ned, r->accel_ned, sizeof(r->accel_ned));
  memcpy(u->accel_body, r->accel_body, sizeof(r->accel_body));
  u->alt_agl_m = r->alt_agl_m;
  u->gnss_age_s = r->gnss_age_s;
  u->nav_valid = r->nav_valid;
  u->timestamp_s = r->timestamp_s;
  u->rocket_state = r->rocket_state;
  u->telemetry_cmd.cmd_id = r->cmd_id;
  u->telemetry_cmd.cmd_param = r->cmd_param;
  u->telemetry_cmd.cmd_valid = r->cmd_valid;

  const uint32_t t0 = DWT->CYCCNT;
  gc_step();
  const uint32_t us = _cycles_to_us(DWT->CYCCNT - t0);
  if (us > c->max_us) {
    c->max_us = us;
  }

  const gc_bus_out *o = &gc_Y.gc_out;
  _real(c, step, "actuator_cmd", o->actuator_cmd, w->actuator_cmd);
  _real(c, step, "airbrake_deg", o->airbrake_deg, w->airbrake_deg);
  _real(c, step, "apogee_pred_m", o->apogee_pred_m, w->apogee_pred_m);
  _real(c, step, "apogee_error_m", o->apogee_error_m, w->apogee_error_m);
  _real(c, step, "time_to_apogee_s", o->time_to_apogee_s, w->time_to_apogee_s);
  _real(c, step, "accel_cmd_mps2", o->accel_cmd_mps2, w->accel_cmd_mps2);
  _real(c, step, "accel_meas_mps2", o->accel_meas_mps2, w->accel_meas_mps2);
  _entero(c, step, "drogue_deploy_flag", o->drogue_deploy_flag,
          w->drogue_deploy_flag);
  _entero(c, step, "main_deploy_flag", o->main_deploy_flag,
          w->main_deploy_flag);
  _entero(c, step, "flight_phase", o->flight_phase, w->flight_phase);
  _entero(c, step, "pred_valid", o->pred_valid, w->pred_valid);
  _entero(c, step, "pyro_armed", o->pyro_armed, w->pyro_armed);
  _real(c, step, "timestamp_s", o->timestamp_s, w->timestamp_s);
}

/* === Prueba ============================================================== */

void pil_test_run(void) {
  memset(&s_result, 0, sizeof(s_result));
  _cycle_counter_init();

  s_summary_open = storage_stream_open_write(&s_summary, PIL_SD, "PILRES.TXT",
                                             s_summary_buf,
                                             sizeof(s_summary_buf));

  _report("\r\n=================================================\r\n");
  _report(" BANCO PIL DEL GNC v1.5  (nav y maquina de estados a\r\n");
  _report(" 400 Hz, guiado a 50 Hz; 181,5 s, vuelo entero)\r\n");
  _report("=================================================\r\n");
  if (!s_summary_open) {
    DebugOut_Printf("   (sin PILRES.TXT: la SD1 no esta montada)\r\n");
  }

  static const char *const archivos[6] = {"PILNAVI.BIN", "PILNAVE.BIN",
                                          "PILSMI.BIN",  "PILSME.BIN",
                                          "PILGCI.BIN",  "PILGCE.BIN"};
  static const uint32_t tamanos[6] = {
      sizeof(pil_nav_in_t), sizeof(pil_nav_exp_t), sizeof(pil_sm_in_t),
      sizeof(pil_sm_exp_t), sizeof(pil_gc_in_t),   sizeof(pil_gc_exp_t)};
  storage_stream_t st[6];
  uint32_t filas[6] = {0};
  uint32_t abiertos = 0U;
  for (; abiertos < 6U; abiertos++) {
    if (!_open_vectors(&st[abiertos], archivos[abiertos], tamanos[abiertos],
                       &filas[abiertos])) {
      break;
    }
  }

  storage_stream_t csv[3];
  bool csv_ok[3] = {false, false, false};
  static const char *const salidas[3] = {"PILNAVO.CSV", "PILSMO.CSV",
                                         "PILGCO.CSV"};
  if (abiertos == 6U) {
    for (uint32_t i = 0U; i < 3U; i++) {
      csv_ok[i] = storage_stream_open_write(&csv[i], PIL_SD, salidas[i],
                                            s_csv_buf[i],
                                            sizeof(s_csv_buf[i]));
    }
    if (csv_ok[0]) {
      (void)storage_stream_puts(
          &csv[0], "t_s,pos_ned_1,pos_ned_2,pos_ned_3,vel_ned_1,vel_ned_2,"
                   "vel_ned_3,quat_nb_1,quat_nb_2,quat_nb_3,quat_nb_4,"
                   "alt_agl_m,accel_bias_1,accel_bias_2,accel_bias_3,"
                   "gnss_only,nav_valid\r\n");
    }
    if (csv_ok[1]) {
      (void)storage_stream_puts(&csv[1], "t_s,rocket_state,prev_state,"
                                         "time_in_state_s,error_code\r\n");
    }
    if (csv_ok[2]) {
      (void)storage_stream_puts(
          &csv[2], "t_s,actuator_cmd,apogee_pred_m,apogee_error_m,"
                   "time_to_apogee_s,drogue_deploy_flag,main_deploy_flag,"
                   "flight_phase,pyro_armed\r\n");
    }
  }

  pil_check_t nav = {.modelo = "nav"};
  pil_check_t sm = {.modelo = "state_machine"};
  pil_check_t gc = {.modelo = "gc"};
  const uint32_t t_start = HAL_GetTick();

  if (abiertos == 6U) {
    /* Los tres modelos a cero, con los ajustables de fábrica (ver
     * PIL_RAIL_ELEV_FABRICA) y UN initialize() antes de la primera fila */
    memset(&nav_DW, 0, sizeof(nav_DW));
    memset(&nav_U, 0, sizeof(nav_U));
    memset(&nav_Y, 0, sizeof(nav_Y));
    memset(&state_machine_DW, 0, sizeof(state_machine_DW));
    memset(&state_machine_U, 0, sizeof(state_machine_U));
    memset(&state_machine_Y, 0, sizeof(state_machine_Y));
    memset(&gc_DW, 0, sizeof(gc_DW));
    memset(&gc_U, 0, sizeof(gc_U));
    memset(&gc_Y, 0, sizeof(gc_Y));
    nav_tun.rail_elev_deg = PIL_RAIL_ELEV_FABRICA;
    nav_tun.rail_azim_deg = PIL_RAIL_AZIM_FABRICA;
    nav_initialize();
    state_machine_initialize();
    gc_initialize();

    const uint32_t total = (filas[0] < filas[2]) ? filas[0] : filas[2];
    uint32_t fila_gc = 0U;
    uint32_t k = 0U;
    for (; k < total; k++) {
      pil_nav_in_t ni;
      pil_nav_exp_t ne;
      pil_sm_in_t si;
      pil_sm_exp_t se;
      if (!storage_stream_read(&st[0], &ni, sizeof(ni)) ||
          !storage_stream_read(&st[1], &ne, sizeof(ne)) ||
          !storage_stream_read(&st[2], &si, sizeof(si)) ||
          !storage_stream_read(&st[3], &se, sizeof(se))) {
        _report("   lectura cortada en la fila %lu\r\n", (unsigned long)k);
        break;
      }
      const double t_s = (double)k * (GNC_PERIOD_US * 1.0e-6);

      _nav_paso(&ni, &ne, k, &nav);
      _sm_paso(&si, &se, k, &sm);

      if (((k % PIL_NAV_CSV_CADA) == 0U) && csv_ok[0]) {
        const nav_bus_out *o = &nav_Y.nav_out;
        uint32_t n = _append(s_line, sizeof(s_line), 0U, "%.4f", t_s);
        for (int i = 0; i < 3; i++) {
          n = _append(s_line, sizeof(s_line), n, ",%.9g", (double)o->pos_ned[i]);
        }
        for (int i = 0; i < 3; i++) {
          n = _append(s_line, sizeof(s_line), n, ",%.9g", (double)o->vel_ned[i]);
        }
        for (int i = 0; i < 4; i++) {
          n = _append(s_line, sizeof(s_line), n, ",%.9g", (double)o->quat_nb[i]);
        }
        n = _append(s_line, sizeof(s_line), n, ",%.9g", (double)o->alt_agl_m);
        for (int i = 0; i < 3; i++) {
          n = _append(s_line, sizeof(s_line), n, ",%.9g",
                      (double)o->accel_bias[i]);
        }
        n = _append(s_line, sizeof(s_line), n, ",%u,%u\r\n",
                    (unsigned)o->gnss_only, (unsigned)o->nav_valid);
        (void)storage_stream_write(&csv[0], s_line, n);
      }
      if (csv_ok[1]) {
        const sm_bus_out *o = &state_machine_Y.sm_out;
        const uint32_t n = _append(s_line, sizeof(s_line), 0U,
                                   "%.4f,%ld,%ld,%.6g,%u\r\n", t_s,
                                   (long)o->rocket_state, (long)o->prev_state,
                                   (double)o->time_in_state_s,
                                   (unsigned)o->error_code);
        (void)storage_stream_write(&csv[1], s_line, n);
      }

      if (((k % PIL_GC_CADA) == 0U) && (fila_gc < filas[4])) {
        pil_gc_in_t gi;
        pil_gc_exp_t ge;
        if (!storage_stream_read(&st[4], &gi, sizeof(gi)) ||
            !storage_stream_read(&st[5], &ge, sizeof(ge))) {
          _report("   gc: lectura cortada en la fila %lu\r\n",
                  (unsigned long)fila_gc);
          break;
        }
        _gc_paso(&gi, &ge, fila_gc, &gc);
        fila_gc++;
        if (csv_ok[2]) {
          const gc_bus_out *o = &gc_Y.gc_out;
          const uint32_t n = _append(
              s_line, sizeof(s_line), 0U,
              "%.4f,%.9g,%.9g,%.9g,%.9g,%u,%u,%ld,%u\r\n", t_s,
              (double)o->actuator_cmd, (double)o->apogee_pred_m,
              (double)o->apogee_error_m, (double)o->time_to_apogee_s,
              (unsigned)o->drogue_deploy_flag, (unsigned)o->main_deploy_flag,
              (long)o->flight_phase, (unsigned)o->pyro_armed);
          (void)storage_stream_write(&csv[2], s_line, n);
        }
      }

      if ((k % 4000U) == 0U) {
        DebugOut_Printf("   %lu/%lu  t = %.0f s\r\n", (unsigned long)k,
                        (unsigned long)total, t_s);
      }
    }
    nav.pasos = k;
    sm.pasos = k;
    gc.pasos = fila_gc;
  }

  for (uint32_t i = 0U; i < abiertos; i++) {
    (void)storage_stream_close(&st[i]);
  }
  for (uint32_t i = 0U; i < 3U; i++) {
    if (csv_ok[i]) {
      (void)storage_stream_close(&csv[i]);
    }
  }

  s_result.ran = (nav.pasos > 0U);
  s_result.steps = nav.pasos;
  s_result.mismatches = nav.mismatches + sm.mismatches + gc.mismatches;
  s_result.duration_ms = HAL_GetTick() - t_start;
  s_result.max_step_us = nav.max_us + sm.max_us + gc.max_us;
  s_result.passed = s_result.ran && (s_result.mismatches == 0U);

  if (s_result.ran) {
    _summary(&nav);
    _summary(&sm);
    _summary(&gc);
  }
  _report("-------------------------------------------------\r\n");
  if (s_result.ran) {
    _report("   Peor tick con los tres modelos: %lu us, con %lu de"
            " presupuesto\r\n",
            (unsigned long)s_result.max_step_us,
            (unsigned long)GNC_PERIOD_US);
    _report("   %lu filas en %lu ms\r\n", (unsigned long)s_result.steps,
            (unsigned long)s_result.duration_ms);
    _report("   RESULTADO: %s\r\n",
            s_result.passed ? "CORRECTO, la placa reproduce el modelo"
                            : "HAY DIFERENCIAS (ver arriba)");
    _report("   Salidas de la placa en PILNAVO.CSV, PILSMO.CSV y"
            " PILGCO.CSV\r\n");
  } else {
    _report("   No se ejecuto: faltan los vectores en la SD1.\r\n");
    _report("   Generarlos con tools/pil_csv_to_bin.py y copiarlos a la"
            " raiz.\r\n");
  }
  _report("=================================================\r\n\r\n");

  if (s_summary_open) {
    (void)storage_stream_close(&s_summary);
    s_summary_open = false;
  }

  /* La prueba deja los modelos al final del vuelo simulado y con los
   * ajustables de fábrica: gnc_init() los deja otra vez como para volar */
  gnc_init();
}

const pil_test_result_t *pil_test_get_result(void) { return &s_result; }
