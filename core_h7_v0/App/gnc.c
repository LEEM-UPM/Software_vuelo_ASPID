#include "gnc.h"
#include "emergencia.h"

#include <math.h>
#include <string.h>

#include "gc.h"
#include "main.h"
#include "nav.h"
#include "state_machine.h"

#define GNC_RAD_TO_DEG 57.295779513f
#define GNC_DEG_TO_RAD 0.017453292519943295

/* Las IMU de la aviónica no dan temperatura por CAN2; la entrega usa esto */
#define GNC_IMU_TEMP_C 25.0f

/* Acumulador de una IMU entre dos ticks: se suman las muestras y en el tick
 * se pasa la media. Sin muestras en ese tick se repite la última (ver
 * GNC_IMU_MAX_HOLD_TICKS en gnc.h: la IMU sigue siendo válida). */
typedef struct {
  float accel_sum[3];
  float gyro_sum[3];
  uint32_t count;
  float last_accel[3]; /* media del último tick con datos */
  float last_gyro[3];
  double timestamp_s; /* de la última muestra recibida */
  bool ever;
  uint32_t tick_ultima; /* s_tick del último tick con muestra nueva */
} imu_accum_t;

static imu_accum_t s_imu1;
static imu_accum_t s_imu2;

/* Barómetro: la última muestra. La presión se pasa en TODOS los ticks (el
 * alineamiento la promedia así), y data_valid solo uno de cada
 * GNC_BARO_DECIMATION si ha llegado algo nuevo. */
static struct {
  float press_pa;
  float temp_c;
  double timestamp_s;
  bool fresh; /* ha llegado algo desde la última vez que se dio por válido */
} s_baro;

/* Del GPS se guarda lo último recibido: a 10 Hz llega una muestra cada 40
 * ticks. */
static struct {
  double pos_lla[3]; /* latitud [rad], longitud [rad], altura [m] */
  float vel_ned[3];
  uint8_t fix_type;  /* UBX, de NAV-PVT: 3 = fix 3D (ver gnc_on_gps_pvt) */
  uint8_t satellites;
  bool pvt_ever;
  uint32_t pvt_ms;   /* HAL_GetTick() del ultimo NAV-PVT */
  bool ever;
  bool vel_d_known; /* false: el receptor no dio la vertical */
  bool fresh;       /* ha llegado algo desde el ultimo tick */
} s_gps;

/*
 * Si NAV-PVT deja de llegar, el ultimo fix deja de valer pasado esto. Sin
 * esta caducidad, un GPS que siguiera mandando GGA pero no NAV-PVT tendria
 * para siempre el "fix 3D" de la ultima vez. El GPS va a 10 Hz: 1 s son diez
 * tramas seguidas perdidas.
 */
#define GNC_GPS_FIX_MAX_AGE_MS 1000U

/* Comando de telemetría pendiente: uno para la máquina de estados y otro
 * para el guiado, que van a ritmos distintos y tienen que verlo los dos */
static struct {
  uint8_t id;
  float param;
  bool sm_pending;
  bool gc_pending;
} s_cmd;

static gnc_data_t s_data;
static bool s_replay;
static bool s_replay_listo; /* hay un registro metido esperando su tick */
static uint32_t s_inits;
static uint32_t s_tick;     /* ticks desde gnc_init(), para las decimaciones */

/* Rampa: la de vuelo (sobrevive a gnc_init) y la que se está usando */
static float s_rampa_vuelo_vertical_deg;
static float s_rampa_vuelo_rumbo_deg;
static float s_rampa_vuelo_alabeo_deg = GNC_ALABEO_MEDIR;
static float s_rampa_uso_vertical_deg;
static float s_rampa_uso_rumbo_deg;
static float s_rampa_uso_alabeo_deg = GNC_ALABEO_MEDIR;
static float s_rampa_alabeo_rad;

/* Veces que se ha realineado (gnc_realinear): sobrevive a gnc_init(), que
 * es justo lo que hace realinear */
static uint32_t s_realineados;

/* Por debajo de esta inclinación medida el alabeo no se puede medir */
#define GNC_RAMPA_INCLINACION_MIN_ALABEO_DEG 2.0f

/* Media de la fuerza específica de la IMU1 en ejes de cuerpo SIN quitar el
 * alabeo, para medirlo mientras no se ha alineado (ver _quitar_alabeo) */
static float s_acc_media[3];
static bool s_acc_media_ok;
#define GNC_ALABEO_MEDIA_ALFA 0.01f /* 0,24 s a 416 Hz */

static bool _nav_alineado(void);
static void _rampa_usar(float desde_vertical_deg, float rumbo_deg,
                        float alabeo_deg);
static void _rampa_medir_alabeo(void);

/*
 * EL ALABEO DE LA RAMPA, CON LA v1.5. La navegación se alinea con la actitud
 * de la rampa (elevación y azimut, nav_tun.rail_elev_deg / rail_azim_deg) y
 * el ALABEO FIJO A 0: supone que el eje Y de cuerpo queda horizontal en la
 * rampa. Con la rampa inclinada, si la placa no está montada justo así, la
 * gravedad medida se aparta de la esperada hasta el doble de la inclinación
 * (con 5,5 grados, 11: por encima de los 10 que exige para alinearse).
 * Como en la v1.2 se le daba el alabeo y aquí se medía con la gravedad
 * (GNC_ALABEO_MEDIR), se sigue midiendo igual y se GIRAN LAS MEDIDAS sobre
 * el eje del cohete (X) para que la navegación vea alabeo 0:
 *   y' = cos(a) y - sen(a) z,   z' = sen(a) y + cos(a) z
 * Dentro del GNC todo va en esos ejes girados (el guiado solo usa el eje
 * del cohete y la vertical, que no cambian). Lo que sale hacia fuera (actitud,
 * aceleración y giro en ejes de cuerpo, sesgos) se vuelve a girar a los ejes
 * de verdad en _collect_outputs(). El alabeo se deja de medir al alinearse y
 * se vuelve a medir en cada alineación (también al realinear, comando 10).
 */
static void _quitar_alabeo(float v[3]) {
  const float c = cosf(s_rampa_alabeo_rad);
  const float s = sinf(s_rampa_alabeo_rad);
  const float y = v[1];
  const float z = v[2];
  v[1] = (c * y) - (s * z);
  v[2] = (s * y) + (c * z);
}

static void _poner_alabeo(float v[3]) {
  const float c = cosf(s_rampa_alabeo_rad);
  const float s = sinf(s_rampa_alabeo_rad);
  const float y = v[1];
  const float z = v[2];
  v[1] = (c * y) + (s * z);
  v[2] = (-s * y) + (c * z);
}

/* Planificación a 416 Hz con el contador de ciclos: HAL_GetTick() va en ms y
 * 2,404 ms no se pueden contar con él */
static uint32_t s_period_cycles;
static uint32_t s_next_cycles;
static uint32_t s_last_tick_ms; /* para no fiarse del contador si da la vuelta */

/* === Medida del tiempo =================================================== */

static void _cycle_counter_init(void) {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint32_t _cycles_to_us(uint32_t cycles) {
  const uint32_t mhz = HAL_RCC_GetSysClockFreq() / 1000000U;
  return (mhz > 0U) ? (cycles / mhz) : 0U;
}

/* === Entrada de sensores ================================================= */

static void _imu_add(imu_accum_t *acc, const float accel[3], const float gyro[3],
                     uint64_t timestamp_us) {
  for (int i = 0; i < 3; i++) {
    acc->accel_sum[i] += accel[i];
    if (gyro != NULL) {
      acc->gyro_sum[i] += gyro[i];
    }
  }
  acc->count++;
  acc->timestamp_s = (double)timestamp_us * 1e-6;
}

void gnc_on_imu1(const float accel_mps2[3], const float gyro_rps[3],
                 uint64_t timestamp_us) {
  _imu_add(&s_imu1, accel_mps2, gyro_rps, timestamp_us);
}

void gnc_on_imu2(const float accel_mps2[3], uint64_t timestamp_us) {
  _imu_add(&s_imu2, accel_mps2, NULL, timestamp_us);
}

void gnc_on_baro(float pressure_kpa, float temperature_c,
                 uint64_t timestamp_us) {
  if (!(pressure_kpa > 0.0f)) {
    return; /* también NaN: no se le da una presión imposible al EKF */
  }
  s_baro.press_pa = pressure_kpa * 1000.0f;
  s_baro.temp_c = temperature_c;
  s_baro.timestamp_s = (double)timestamp_us * 1e-6;
  s_baro.fresh = true;
}

void gnc_on_gps_position(int32_t latitude_e7, int32_t longitude_e7,
                         int32_t altitude_mm) {
  /* Campos sin valor: el GPS los manda como INT32_MIN */
  if ((latitude_e7 == INT32_MIN) || (longitude_e7 == INT32_MIN)) {
    return;
  }

  /* El modelo quiere RADIANES, no grados: multiplica la diferencia de
   * latitud y longitud directamente por un radio en metros. Si se le pasan
   * grados, cada metro de GPS le llega como 57,3 m. */
  s_gps.pos_lla[0] = (double)latitude_e7 * 1e-7 * GNC_DEG_TO_RAD;
  s_gps.pos_lla[1] = (double)longitude_e7 * 1e-7 * GNC_DEG_TO_RAD;
  if (altitude_mm != INT32_MIN) {
    s_gps.pos_lla[2] = (double)altitude_mm * 1e-3;
  }
  s_gps.ever = true;
  s_gps.fresh = true;
  s_data.gps_updates++;
}

void gnc_on_gps_pvt(int32_t vn_mmps, int32_t ve_mmps, int32_t vd_mmps,
                    uint8_t fix_type, uint8_t satellites, bool valid) {
  s_gps.pvt_ever = true;
  s_gps.pvt_ms = HAL_GetTick();
  s_gps.satellites = satellites;

  /* Sin gnssFixOK la velocidad no vale nada (sin fix el receptor da ceros o
   * lo ultimo que tuviera): no se toca, y el fix cuenta como ninguno. */
  if (!valid) {
    s_gps.fix_type = 0U;
    return;
  }
  s_gps.fix_type = fix_type;

  if ((vn_mmps == INT32_MIN) || (ve_mmps == INT32_MIN)) {
    return;
  }
  s_gps.vel_ned[0] = (float)vn_mmps * 1e-3f;
  s_gps.vel_ned[1] = (float)ve_mmps * 1e-3f;
  if (vd_mmps != INT32_MIN) {
    s_gps.vel_ned[2] = (float)vd_mmps * 1e-3f;
    s_gps.vel_d_known = true;
  } else {
    s_gps.vel_d_known = false;
  }
  s_gps.fresh = true;
}

void gnc_command(uint8_t cmd_id, float param) {
  s_cmd.id = cmd_id;
  s_cmd.param = param;
  s_cmd.sm_pending = true;
  s_cmd.gc_pending = true;
}

/* === Un tick ============================================================= */

/* Media de lo llegado desde el tick anterior, o la última si no ha llegado
 * nada. *nueva dice si había algo; devuelve si la IMU está viva (ver
 * GNC_IMU_MAX_HOLD_TICKS). */
static bool _imu_take(imu_accum_t *acc, float accel[3], float gyro[3],
                      double *timestamp_s, uint32_t *samples, bool *nueva_out) {
  *samples = acc->count;
  const bool nueva = (acc->count > 0U);
  *nueva_out = nueva;

  if (nueva) {
    acc->ever = true;
    acc->tick_ultima = s_tick;
    const float inv = 1.0f / (float)acc->count;
    for (int i = 0; i < 3; i++) {
      acc->last_accel[i] = acc->accel_sum[i] * inv;
      acc->last_gyro[i] = acc->gyro_sum[i] * inv;
      acc->accel_sum[i] = 0.0f;
      acc->gyro_sum[i] = 0.0f;
    }
    acc->count = 0U;
  }

  memcpy(accel, acc->last_accel, sizeof(acc->last_accel));
  if (gyro != NULL) {
    memcpy(gyro, acc->last_gyro, sizeof(acc->last_gyro));
  }
  *timestamp_s = acc->timestamp_s;
  return acc->ever && ((s_tick - acc->tick_ultima) <= GNC_IMU_MAX_HOLD_TICKS);
}

/*
 * De ejes de placa a ejes de cuerpo del modelo (ver el comentario de gnc.h).
 * Se aplica despues de promediar, una vez por tick en vez de una vez por
 * muestra, y deja los acumuladores en ejes de placa para que sigan cuadrando
 * con lo que imprime la consola de la placa de sensores.
 */
static void _board_to_body(float v[3]) {
#if GNC_BOARD_Z_ALONG_ROCKET
  const float board_x = v[0];
  v[0] = v[2];  /* morro   */
  v[2] = -board_x; /* abajo  */
                   /* v[1] (derecha) no cambia */
#else
  (void)v;
#endif
}

/* Atmósfera estándar: solo para rellenar baro.alt_msl_m, que nav.c no usa */
static float _altura_isa_m(float press_pa) {
  return 44330.77f * (1.0f - powf(press_pa / 101325.0f, 0.190263f));
}

static void _fill_nav_inputs(void) {
  nav_bus_in *in = &nav_U.nav_in;

  bool imu1_nueva = false;
  const bool imu1_viva =
      _imu_take(&s_imu1, in->imu_lowg.accel_mps2, in->imu_lowg.gyro_rps,
                &in->imu_lowg.timestamp_s, &s_data.imu1_samples, &imu1_nueva);
  _board_to_body(in->imu_lowg.accel_mps2);
  _board_to_body(in->imu_lowg.gyro_rps);
  /* Para medir el alabeo: la gravedad SIN girar, mientras no se alinea */
  if (imu1_nueva && !_nav_alineado()) {
    for (int i = 0; i < 3; i++) {
      s_acc_media[i] =
          s_acc_media_ok
              ? s_acc_media[i] + (GNC_ALABEO_MEDIA_ALFA *
                                  (in->imu_lowg.accel_mps2[i] - s_acc_media[i]))
              : in->imu_lowg.accel_mps2[i];
    }
    s_acc_media_ok = true;
  }
  _quitar_alabeo(in->imu_lowg.accel_mps2);
  _quitar_alabeo(in->imu_lowg.gyro_rps);
  in->imu_lowg.temp_C = GNC_IMU_TEMP_C;
  in->imu_lowg.data_valid = imu1_viva ? 1U : 0U;
  if (!imu1_nueva) {
    s_data.imu1_sin_dato++;
  }
  if (!imu1_viva) {
    s_data.imu1_caida++;
  }

  bool imu2_nueva = false;
  const bool imu2_viva =
      _imu_take(&s_imu2, in->imu_highg.accel_mps2, NULL,
                &in->imu_highg.timestamp_s, &s_data.imu2_samples, &imu2_nueva);
  _board_to_body(in->imu_highg.accel_mps2);
  _quitar_alabeo(in->imu_highg.accel_mps2);
  in->imu_highg.data_valid = imu2_viva ? 1U : 0U;
  if (!imu2_viva) {
    s_data.imu2_caida++;
  }

  /* Barómetro: la presión siempre, la validez uno de cada 6 ticks */
  in->baro.press_pa = s_baro.press_pa;
  in->baro.alt_msl_m =
      (s_baro.press_pa > 0.0f) ? _altura_isa_m(s_baro.press_pa) : 0.0f;
  in->baro.temp_C = s_baro.temp_c;
  in->baro.timestamp_s = s_baro.timestamp_s;
  if (((s_tick % GNC_BARO_DECIMATION) == 0U) && s_baro.fresh) {
    in->baro.data_valid = 1U;
    s_baro.fresh = false;
    s_data.baro_updates++;
  } else {
    in->baro.data_valid = 0U;
  }

  memcpy(in->gnss.pos_lla, s_gps.pos_lla, sizeof(in->gnss.pos_lla));
  in->gnss.vel_ned[0] = s_gps.vel_ned[0];
  in->gnss.vel_ned[1] = s_gps.vel_ned[1];
  if (s_gps.vel_d_known) {
    in->gnss.vel_ned[2] = s_gps.vel_ned[2];
  } else {
    /* Sin vertical del GPS se le pasa su propia estimación: innovación cero
     * en ese eje en vez de arrastrarlo hacia cero. */
    in->gnss.vel_ned[2] = nav_Y.nav_out.vel_ned[2];
  }
  /* fix_type en convención UBX (3 = 3D), de NAV-PVT y no del GGA: ver
   * gnc_on_gps_pvt() */
  const bool fix_reciente =
      s_gps.pvt_ever &&
      ((HAL_GetTick() - s_gps.pvt_ms) <= GNC_GPS_FIX_MAX_AGE_MS);
  in->gnss.fix_type = fix_reciente ? s_gps.fix_type : 0U;
  in->gnss.num_sats = fix_reciente ? s_gps.satellites : 0U;
  s_data.gps_fix_type = in->gnss.fix_type;
  /* 1 solo en el tick en que ha llegado algo nuevo (ver gnc.h) */
  in->gnss.data_valid = s_gps.fresh ? 1U : 0U;
  s_gps.fresh = false;

  /* El estado del cohete y la orden a la navegación del tick ANTERIOR: es
   * lo que rompe el lazo nav <-> máquina de estados (LEEME_INTEGRACION).
   * nav_cmd (v1.5): 10 realinear, 11 reiniciar la covarianza */
  in->rocket_state = state_machine_Y.sm_out.rocket_state;
  in->nav_cmd = state_machine_Y.sm_out.nav_cmd;
}

static void _fill_sm_inputs(void) {
  const nav_bus_out *nav = &nav_Y.nav_out;
  sm_bus_in *sm = &state_machine_U.sm_in;

  memcpy(sm->pos_ned, nav->pos_ned, sizeof(sm->pos_ned));
  memcpy(sm->vel_ned, nav->vel_ned, sizeof(sm->vel_ned));
  memcpy(sm->accel_body, nav->accel_body, sizeof(sm->accel_body));
  memcpy(sm->gyro_body, nav->gyro_body, sizeof(sm->gyro_body));
  sm->alt_agl_m = nav->alt_agl_m;
  sm->gnss_age_s = nav->gnss_age_s;
  sm->nav_valid = nav->nav_valid;
  sm->timestamp_s = nav->timestamp_s;
  sm->telemetry_cmd.cmd_id = s_cmd.id;
  sm->telemetry_cmd.cmd_param = s_cmd.param;
  sm->telemetry_cmd.cmd_valid = s_cmd.sm_pending ? 1U : 0U;
  s_cmd.sm_pending = false;
  /* Restauración tras un reinicio (v1.6): NO se usa. Reiniciada en el aire,
   * la core no vuelve al GNC sino al modo de emergencia (emergencia.h): el
   * filtro no se puede alinear con el cohete volando. valid = 0 es arrancar
   * normal, que es lo que pide la entrega si no se restaura. */
  sm->restore.valid = 0U;
  sm->restore.rocket_state = 0;
  sm->restore.flight_phase = 0;
}

/* COPIA COMPLETA de una vez, del mismo tick: el guiado tiene que ver todos
 * los campos del mismo ciclo (LEEME_INTEGRACION) */
static void _fill_gc_inputs(void) {
  const nav_bus_out *nav = &nav_Y.nav_out;
  gc_bus_in *gc = &gc_U.gc_in;

  memcpy(gc->pos_ned, nav->pos_ned, sizeof(gc->pos_ned));
  memcpy(gc->vel_ned, nav->vel_ned, sizeof(gc->vel_ned));
  memcpy(gc->accel_ned, nav->accel_ned, sizeof(gc->accel_ned));
  memcpy(gc->accel_body, nav->accel_body, sizeof(gc->accel_body));
  gc->alt_agl_m = nav->alt_agl_m;
  gc->gnss_age_s = nav->gnss_age_s;
  gc->nav_valid = nav->nav_valid;
  gc->timestamp_s = nav->timestamp_s;
  gc->rocket_state = state_machine_Y.sm_out.rocket_state;
  /* v1.6: el fallo en vuelo solo se ve aquí (la máquina de estados no sale
   * de EN VUELO), y con él el guiado abre el aerofreno al máximo */
  gc->error_code = state_machine_Y.sm_out.error_code;
  gc->nav_init_flags = nav->nav_init_flags;
  gc->gnss_only = nav->gnss_only;
  gc->restore.valid = 0U; /* ver _fill_sm_inputs */
  gc->restore.rocket_state = 0;
  gc->restore.flight_phase = 0;
  gc->telemetry_cmd.cmd_id = s_cmd.id;
  gc->telemetry_cmd.cmd_param = s_cmd.param;
  gc->telemetry_cmd.cmd_valid = s_cmd.gc_pending ? 1U : 0U;
  s_cmd.gc_pending = false;
}

static void _collect_outputs(void) {
  const nav_bus_out *nav = &nav_Y.nav_out;
  const sm_bus_out *sm = &state_machine_Y.sm_out;
  const gc_bus_out *gc = &gc_Y.gc_out;

  memcpy(s_data.pos_ned_m, nav->pos_ned, sizeof(s_data.pos_ned_m));
  memcpy(s_data.vel_ned_mps, nav->vel_ned, sizeof(s_data.vel_ned_mps));
  memcpy(s_data.accel_body_mps2, nav->accel_body,
         sizeof(s_data.accel_body_mps2));
  memcpy(s_data.gyro_body_rps, nav->gyro_body, sizeof(s_data.gyro_body_rps));
  memcpy(s_data.accel_bias_mps2, nav->accel_bias,
         sizeof(s_data.accel_bias_mps2));
  memcpy(s_data.gyro_bias_rps, nav->gyro_bias, sizeof(s_data.gyro_bias_rps));
  /* De los ejes girados del GNC a los de verdad (ver _quitar_alabeo) */
  _poner_alabeo(s_data.accel_body_mps2);
  _poner_alabeo(s_data.gyro_body_rps);
  _poner_alabeo(s_data.accel_bias_mps2);
  _poner_alabeo(s_data.gyro_bias_rps);
  {
    /* q_cuerpo = q_gnc * (cos(a/2), sen(a/2), 0, 0) */
    const float w = nav->quat_nb[0], x = nav->quat_nb[1];
    const float y = nav->quat_nb[2], z = nav->quat_nb[3];
    const float cw = cosf(0.5f * s_rampa_alabeo_rad);
    const float sx = sinf(0.5f * s_rampa_alabeo_rad);
    s_data.quat_nb[0] = (w * cw) - (x * sx);
    s_data.quat_nb[1] = (x * cw) + (w * sx);
    s_data.quat_nb[2] = (y * cw) + (z * sx);
    s_data.quat_nb[3] = (z * cw) - (y * sx);
  }
  for (int i = 0; i < 3; i++) {
    s_data.euler_rpy_deg[i] = nav->euler_rpy[i] * GNC_RAD_TO_DEG;
  }
  s_data.euler_rpy_deg[0] += s_rampa_alabeo_rad * GNC_RAD_TO_DEG;
  if (s_data.euler_rpy_deg[0] > 180.0f) {
    s_data.euler_rpy_deg[0] -= 360.0f;
  } else if (s_data.euler_rpy_deg[0] < -180.0f) {
    s_data.euler_rpy_deg[0] += 360.0f;
  }
  s_data.alt_agl_m = nav->alt_agl_m;
  s_data.gnss_age_s = nav->gnss_age_s;
  s_data.gnss_only = (nav->gnss_only != 0U);
  s_data.nav_valid = (nav->nav_valid != 0U);
  s_data.nav_ready = (nav->nav_ready != 0U);
  s_data.nav_init_flags = nav->nav_init_flags;

  /* Por que el EKF no se alinea: los mismos numeros que mira el modelo */
  const nav_bus_in *in = &nav_U.nav_in;
  s_data.align_count = nav_DW.NavState_Memory_PreviousInput.align_cnt;
  s_data.rampa_vertical_deg = s_rampa_uso_vertical_deg;
  s_data.rampa_rumbo_deg = s_rampa_uso_rumbo_deg;
  s_data.rampa_alabeo_deg = s_rampa_alabeo_rad * GNC_RAD_TO_DEG;
  s_data.rampa_alabeo_medido = isnan(s_rampa_uso_alabeo_deg);
  /* Simulando, la rampa es la del vuelo grabado a propósito: no es cambio */
  s_data.rampa_cambiada =
      !s_replay &&
      ((s_rampa_vuelo_vertical_deg != s_rampa_uso_vertical_deg) ||
       (s_rampa_vuelo_rumbo_deg != s_rampa_uso_rumbo_deg) ||
       (isnan(s_rampa_vuelo_alabeo_deg) != isnan(s_rampa_uso_alabeo_deg)));
  s_data.accel_norm_mps2 =
      sqrtf(in->imu_lowg.accel_mps2[0] * in->imu_lowg.accel_mps2[0] +
            in->imu_lowg.accel_mps2[1] * in->imu_lowg.accel_mps2[1] +
            in->imu_lowg.accel_mps2[2] * in->imu_lowg.accel_mps2[2]);
  s_data.gyro_norm_dps =
      sqrtf(in->imu_lowg.gyro_rps[0] * in->imu_lowg.gyro_rps[0] +
            in->imu_lowg.gyro_rps[1] * in->imu_lowg.gyro_rps[1] +
            in->imu_lowg.gyro_rps[2] * in->imu_lowg.gyro_rps[2]) *
      GNC_RAD_TO_DEG;

  s_data.rocket_state = sm->rocket_state;
  s_data.error_code = sm->error_code;

  /* Del guiado: lo del último paso (cambia a 52 Hz) */
  s_data.status_word = gc->status_word;
  s_data.flight_phase = gc->flight_phase;
  s_data.apogee_pred_m = gc->apogee_pred_m;
  s_data.apogee_error_m = gc->apogee_error_m;
  s_data.time_to_apogee_s = gc->time_to_apogee_s;
  s_data.actuator_cmd = gc->actuator_cmd;
  s_data.pyro_armed = (gc->pyro_armed != 0U);
  s_data.drogue_flag = (gc->drogue_deploy_flag != 0U);
  s_data.main_flag = (gc->main_deploy_flag != 0U);
}

/* nav + máquina de estados, y el guiado uno de cada GNC_GC_DECIMATION */
static void _tick(void) {
  const uint32_t start_cycles = DWT->CYCCNT;
  const bool toca_gc = ((s_tick % GNC_GC_DECIMATION) == 0U);

  _fill_nav_inputs();
  if (!_nav_alineado()) {
    _rampa_medir_alabeo();
  }
  nav_step();
  _fill_sm_inputs();
  state_machine_step();
  if (toca_gc) {
    _fill_gc_inputs();
    gc_step();
    s_data.gc_steps++;
  }
  _collect_outputs();
  s_tick++;

  s_data.last_step_us = _cycles_to_us(DWT->CYCCNT - start_cycles);
  if (s_data.last_step_us > s_data.max_step_us) {
    s_data.max_step_us = s_data.last_step_us;
  }
  if (toca_gc && (s_data.last_step_us > s_data.max_gc_step_us)) {
    s_data.max_gc_step_us = s_data.last_step_us;
  }
  s_data.steps++;
}

/* === API ================================================================= */

void gnc_init(void) {
  s_inits++;
  memset(&s_imu1, 0, sizeof(s_imu1));
  memset(&s_imu2, 0, sizeof(s_imu2));
  memset(&s_baro, 0, sizeof(s_baro));
  memset(&s_gps, 0, sizeof(s_gps));
  memset(&s_cmd, 0, sizeof(s_cmd));
  memset(&s_data, 0, sizeof(s_data));
  s_data.realineados = s_realineados;
  s_tick = 0U;
  s_acc_media_ok = false;

  _cycle_counter_init();

  /*
   * LOS TRES MODELOS A CERO ANTES DE INICIARLOS. No es redundante.
   *
   * Embedded Coder da por hecho que su memoria (DW) y sus puertos (U, Y)
   * empiezan a cero porque son globales sin inicializar, y a cero los deja el
   * arranque del micro. state_machine_initialize() está VACÍA ("no
   * initialization code required"). Eso vale para el primer gnc_init(), pero
   * no para uno en caliente: la simulación de ruta llama a gnc_init() al
   * empezar y al acabar, y sin esto la máquina de estados seguiría en el
   * estado de antes (pasó con la v1.1: el cohete real se quedaba "en vuelo"
   * al acabar una simulación).
   */
  memset(&nav_DW, 0, sizeof(nav_DW));
  memset(&nav_U, 0, sizeof(nav_U));
  memset(&nav_Y, 0, sizeof(nav_Y));
  memset(&state_machine_DW, 0, sizeof(state_machine_DW));
  memset(&state_machine_U, 0, sizeof(state_machine_U));
  memset(&state_machine_Y, 0, sizeof(state_machine_Y));
  memset(&gc_DW, 0, sizeof(gc_DW));
  memset(&gc_U, 0, sizeof(gc_U));
  memset(&gc_Y, 0, sizeof(gc_Y));

  nav_initialize();
  state_machine_initialize();
  gc_initialize();

  /* La rampa de vuelo (gnc_set_rampa; vertical si nadie ha dicho nada). El
   * alabeo se vuelve a medir en cada alineación. */
  _rampa_usar(s_rampa_vuelo_vertical_deg, s_rampa_vuelo_rumbo_deg,
              s_rampa_vuelo_alabeo_deg);

  s_period_cycles = HAL_RCC_GetSysClockFreq() / GNC_RATE_HZ;
  s_next_cycles = DWT->CYCCNT + s_period_cycles;
  s_last_tick_ms = HAL_GetTick();
}

static bool _nav_alineado(void) {
  return nav_DW.NavState_Memory_PreviousInput.init_done != 0U;
}

/* La actitud de la rampa al convenio del modelo (ver gnc_set_rampa en gnc.h).
 * v1.5: elevación (desde la horizontal) y azimut en grados; el alabeo se
 * quita de las medidas (_quitar_alabeo), así que para el modelo es 0. */
static void _rampa_a_modelo(void) {
  nav_tun.rail_elev_deg = 90.0f - s_rampa_uso_vertical_deg;
  nav_tun.rail_azim_deg = s_rampa_uso_rumbo_deg;
  const float roll = 0.0f;
  const float pitch = nav_tun.rail_elev_deg * (float)GNC_DEG_TO_RAD;
  const float yaw = nav_tun.rail_azim_deg * (float)GNC_DEG_TO_RAD;

  /*
   * La memoria del modelo arranca con la actitud de fábrica, y es la que sale
   * MIENTRAS el EKF no se ha alineado. Se pone la de la rampa para que antes
   * y después del alineamiento digan lo mismo (mismo orden que nav.c al
   * alinear). Con el EKF ya alineado NO se toca: es su actitud de verdad.
   */
  if (_nav_alineado()) {
    return;
  }
  const float cr = cosf(0.5f * roll), sr = sinf(0.5f * roll);
  const float cp = cosf(0.5f * pitch), sp = sinf(0.5f * pitch);
  const float cy = cosf(0.5f * yaw), sy = sinf(0.5f * yaw);
  nav_state *nav_state0 = &nav_DW.NavState_Memory_PreviousInput;
  nav_state0->quat_nb[0] = (cr * cp * cy) + (sr * sp * sy);
  nav_state0->quat_nb[1] = (sr * cp * cy) - (cr * sp * sy);
  nav_state0->quat_nb[2] = (cr * sp * cy) + (sr * cp * sy);
  nav_state0->quat_nb[3] = (cr * cp * sy) - (sr * sp * cy);
}

static void _rampa_usar(float desde_vertical_deg, float rumbo_deg,
                        float alabeo_deg) {
  s_rampa_uso_vertical_deg = desde_vertical_deg;
  s_rampa_uso_rumbo_deg = rumbo_deg;
  s_rampa_uso_alabeo_deg = alabeo_deg;
  s_rampa_alabeo_rad =
      isnan(alabeo_deg) ? 0.0f : alabeo_deg * (float)GNC_DEG_TO_RAD;
  _rampa_a_modelo();
}

void gnc_set_rampa(float desde_vertical_deg, float rumbo_deg,
                   float alabeo_deg) {
  s_rampa_vuelo_vertical_deg = desde_vertical_deg;
  s_rampa_vuelo_rumbo_deg = rumbo_deg;
  s_rampa_vuelo_alabeo_deg = alabeo_deg;
  if (s_replay) {
    return; /* simulando: se aplica al acabar (gnc_init) */
  }
  /* En tierra (POWER ON o MANTENIMIENTO) se aplica YA: si el EKF ya estaba
   * alineado, se realinea (gnc_realinear: los tres modelos de cero, NO el
   * comando 10, ver gnc.h). En vuelo o en ERROR, en el siguiente
   * gnc_init(). */
  const int32_t st = state_machine_Y.sm_out.rocket_state;
  if ((st == GNC_ESTADO_POWER_ON) || (st == GNC_ESTADO_MANTENIMIENTO)) {
    const bool alineado = _nav_alineado();
    _rampa_usar(desde_vertical_deg, rumbo_deg, alabeo_deg);
    if (alineado) {
      (void)gnc_realinear();
    }
  }
}

bool gnc_realinear(void) {
  if (s_replay) {
    return false;
  }
  const int32_t st = state_machine_Y.sm_out.rocket_state;
  const bool estado_ok = (st == GNC_ESTADO_POWER_ON) ||
                         (st == GNC_ESTADO_MANTENIMIENTO) ||
                         (st == GNC_ESTADO_ERROR);
  /* La fase, además: ERROR también puede ser en el aire */
  if (!estado_ok || (s_data.flight_phase != GNC_FASE_IDLE)) {
    return false;
  }
  s_realineados++;
  gnc_init(); /* aplica la rampa de vuelo y pone s_data.realineados */
  return true;
}

/* El valor de fábrica, leído del ajustable antes de que nadie lo cambie */
static bool s_freno_fabrica_leida;
static float s_freno_fabrica_m;

float gnc_altura_freno_fabrica_m(void) {
  if (!s_freno_fabrica_leida) {
    s_freno_fabrica_leida = true;
    s_freno_fabrica_m = gc_tun.min_ctrl_alt_agl_m;
  }
  return s_freno_fabrica_m;
}

bool gnc_altura_freno_valida(float m) {
  return (m > 0.0f) && (m < gc_tun.apogee_target_agl_m);
}

bool gnc_set_altura_freno(float m) {
  (void)gnc_altura_freno_fabrica_m(); /* antes de pisarla */
  if (!gnc_altura_freno_valida(m)) {
    return false;
  }
  gc_tun.min_ctrl_alt_agl_m = m;
  return true;
}

float gnc_altura_freno_m(void) { return gc_tun.min_ctrl_alt_agl_m; }

void gnc_set_rampa_solo_este_vuelo(float desde_vertical_deg, float rumbo_deg,
                                   float alabeo_deg) {
  _rampa_usar(desde_vertical_deg, rumbo_deg, alabeo_deg);
}

/*
 * Mientras no se ha alineado: el alabeo de la rampa sale de la gravedad
 * medida, con la misma media que usa nav.c para alinearse (acc_sum), o con la
 * muestra de este tick si aún no hay media. Con la fuerza específica en
 * ejes de cuerpo f = g (sen th, -sen ph cos th, -cos ph cos th):
 *   alabeo = atan2(-f_y, -f_z),  inclinación = acos(f_x / |f|)
 */
static void _rampa_medir_alabeo(void) {
  if (!s_acc_media_ok) {
    return;
  }
  float f[3];
  memcpy(f, s_acc_media, sizeof(f)); /* en ejes de cuerpo, sin girar */
  const float norma = sqrtf((f[0] * f[0]) + (f[1] * f[1]) + (f[2] * f[2]));
  if (!(norma > 0.0f)) {
    return;
  }
  float c = f[0] / norma;
  c = (c > 1.0f) ? 1.0f : ((c < -1.0f) ? -1.0f : c);
  s_data.inclinacion_medida_deg = 90.0f - asinf(c) * GNC_RAD_TO_DEG;

  if (isnan(s_rampa_uso_alabeo_deg)) {
    if (s_data.inclinacion_medida_deg >= GNC_RAMPA_INCLINACION_MIN_ALABEO_DEG) {
      s_rampa_alabeo_rad = atan2f(-f[1], -f[2]);
    } else {
      s_rampa_alabeo_rad = 0.0f;
    }
  }

  /* Lo mismo que comprueba nav.c al terminar de alinear: el ángulo entre la
   * gravedad medida y la que espera la actitud de la rampa (la guiñada no
   * influye). Es el que tiene que quedar por debajo de 10 grados. */
  const float th = nav_tun.rail_elev_deg * (float)GNC_DEG_TO_RAD;
  const float ph = s_rampa_alabeo_rad;
  const float e[3] = {sinf(th), -sinf(ph) * cosf(th), -cosf(ph) * cosf(th)};
  float cc = ((f[0] * e[0]) + (f[1] * e[1]) + (f[2] * e[2])) / norma;
  cc = (cc > 1.0f) ? 1.0f : ((cc < -1.0f) ? -1.0f : cc);
  s_data.rampa_error_deg = acosf(cc) * GNC_RAD_TO_DEG;
}

void gnc_set_replay(bool activa) {
  s_replay = activa;
  s_replay_listo = false;
}

void gnc_replay_muestra_lista(void) { s_replay_listo = true; }

bool gnc_replay_activa(void) { return s_replay; }

uint32_t gnc_init_count(void) { return s_inits; }

void gnc_update(void) {
  if (s_replay) {
    /* Simulando, el ritmo lo marca el archivo (un tick por registro, ver
     * gnc.h), pero el tick se da AQUI, en el mismo sitio del bucle que en
     * vuelo: lo que va detras (despliegue, aerofreno, radio) lo ve igual. */
    if (s_replay_listo) {
      s_replay_listo = false;
      _tick();
    }
    s_next_cycles = DWT->CYCCNT + s_period_cycles;
    s_last_tick_ms = HAL_GetTick();
    return;
  }

  const uint32_t ahora = DWT->CYCCNT;

  /* El contador de ciclos da la vuelta cada ~8 s: si el bucle ha estado
   * parado tanto, la resta ya no dice nada y se vuelve a empezar. */
  if ((HAL_GetTick() - s_last_tick_ms) > 1000U) {
    s_data.overruns++;
    s_next_cycles = ahora;
  }

  const int32_t retraso = (int32_t)(ahora - s_next_cycles);
  if (retraso < 0) {
    return;
  }
  if ((uint32_t)retraso >= (GNC_MAX_RETRASO_TICKS * s_period_cycles)) {
    /* Demasiado atrás: lo perdido, perdido */
    s_data.overruns++;
    s_next_cycles = ahora;
  } else if ((uint32_t)retraso >= s_period_cycles) {
    s_data.ticks_recuperados++;
  }
  s_next_cycles += s_period_cycles;
  s_last_tick_ms = HAL_GetTick();

  _tick();
}

/* Tras un reinicio en vuelo el GNC no corre: los datos los da el modo de
 * emergencia, con el mismo formato (emergencia.h) */
const gnc_data_t *gnc_get_data(void) {
  return emergencia_activa() ? emergencia_gnc() : &s_data;
}
