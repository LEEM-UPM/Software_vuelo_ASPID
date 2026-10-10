#include "gnc_playback.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "airbrake.h"
#include "despliegue.h"
#include "debug_out.h"
#include "gnc.h"
#include "main.h"
#include "storage_stream.h"

#define PLAYBACK_SD SD_CARD_1
#define PLAYBACK_FILE "VUELO.BIN"
#define PLAYBACK_LOG "VUELOLOG.CSV"
#define PLAYBACK_SUMMARY "VUELORES.TXT"
#define PLAYBACK_MAGIC "LEEMVL12" /* formato de VUELO.BIN: un registro por
                                     tick del GNC (416 Hz con la v1.6) */

/*
 * La rampa del vuelo grabado: el dataset de la entrega v1.6 (como la v1.5) se generó con los
 * ajustables de fábrica (PIL_manifest.txt), elevación 84,5 grados (5,5 desde
 * la vertical) y azimut 0 (nav_tun en nav_data.c). (La v1.2 traía 84,5007.) Si
 * se alinea con otra (la de vuelo), el EKF arrastra esa diferencia como sesgo
 * y el guiado ya no hace lo mismo que en la referencia: se vio el 2026-10-04,
 * el aerofreno oscilaba entre 0 y 25 % donde la referencia casi no lo movía.
 * Entra por el mismo camino que la rampa de vuelo (gnc_set_rampa...).
 * TODO: que viaje en la cabecera de VUELO.BIN.
 */
#define PLAYBACK_RAMPA_VERTICAL_DEG 5.5f
#define PLAYBACK_RAMPA_RUMBO_DEG 0.0f
/* El alabeo del vuelo grabado se CONOCE (0): se da en vez de medirlo. Medido
 * salía 21 grados (sesgo del IMU simulado con solo 5,5 de inclinación) y el
 * aerofreno ya no cuadraba con la referencia (20 % en vez de 17,8). */
#define PLAYBACK_RAMPA_ALABEO_DEG 0.0f

/* Un registro = un tick del GNC */
#define PASO_S (GNC_PERIOD_US * 1.0e-6)
#define TICKS_POR_S (GNC_RATE_HZ)

/* VUELOLOG.CSV: una fila cada esto (104 Hz). A 416 Hz serían 416 filas por
 * segundo escribiéndose en la SD en mitad de la simulación. */
#define LOG_CADA_TICKS 4U

/*
 * Un tick del vuelo, tal y como lo manda la aviónica por CAN2: ejes de placa,
 * el GPS en los enteros del ICD y la validez de cada sensor (solo se
 * "entrega" en los ticks en que llegó dato nuevo). Tiene que cuadrar con
 * RECORD_FMT de tools/vuelo_from_pil.py; el tamaño se comprueba contra la
 * cabecera antes de leer nada.
 */
typedef struct __attribute__((packed)) {
  float imu1_accel[3];
  float imu1_gyro[3];
  double imu1_ts_s;
  uint8_t imu1_valid;
  float imu2_accel[3];
  double imu2_ts_s;
  uint8_t imu2_valid;
  float baro_press_pa;
  float baro_temp_c;
  double baro_ts_s;
  uint8_t baro_valid;
  int32_t lat_e7;
  int32_t lon_e7;
  int32_t alt_mm;
  int32_t vn_mmps;
  int32_t ve_mmps;
  int32_t vd_mmps;
  uint8_t fix_type;  /* el del modelo: UBX, 3 = 3D */
  uint8_t satellites;
  uint8_t gps_valid; /* 1 = en este tick hay dato nuevo de GPS */
  uint8_t cmd_id;    /* comando de telemetria (ninguna entrega trae) */
  float cmd_param;
  uint8_t cmd_valid;
} playback_rec_t;
_Static_assert(sizeof(playback_rec_t) == 104U,
               "no cuadra con RECORD_FMT de tools/vuelo_from_pil.py");

typedef struct __attribute__((packed)) {
  char magic[8];
  uint32_t record_bytes;
  uint32_t record_count;
  uint32_t field_count;
  char name[12];
} playback_header_t;

static gnc_playback_result_t s_result;
static storage_stream_t s_summary;
static bool s_summary_open;
static uint8_t s_summary_buf[1024];
static uint8_t s_log_buf[4096];
static char s_line[512];

static const char *_phase_name(int32_t phase) {
  switch (phase) {
  case 0:  return "IDLE";
  case 1:  return "BOOST";
  case 2:  return "COAST sin control";
  case 3:  return "COAST con control";
  case 4:  return "APOGEO";
  case 5:  return "DROGUE";
  case 6:  return "MAIN";
  case 7:  return "EN TIERRA";
  default: return "?";
  }
}

/* Todo lo que sale por USB queda también en VUELORES.TXT */
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

/* === Estado de la reproduccion =========================================== */

static bool s_activa;
static storage_stream_t s_in;
static playback_header_t s_h;
static storage_stream_t s_log;
static bool s_log_open;
static uint32_t s_step;
static uint32_t s_t_start;
static int32_t s_last_phase;

/* Registro metido, esperando a que gnc_update() de su paso */
static bool s_esperando;
static uint32_t s_pasos_gnc;       /* gnc_data_t.steps al meterlo */
static uint32_t s_enviadas_inicio; /* airbrake_sent() al empezar */

/* Seguimiento de la navegacion: un paso de valida a invalida despues de
 * haberse alineado es un REINICIO del EKF (Navigation.c lo hace cuando su
 * estado se vuelve NaN o infinito). En vuelo no se puede realinear, asi que
 * es navegacion perdida para el resto del vuelo. */
static bool s_nav_previa;
static bool s_nav_alguna_vez;
static uint32_t s_reinicios_ekf;
static bool s_drogue_avisado;
static bool s_main_avisado;

static bool _abrir(void) {
  memset(&s_result, 0, sizeof(s_result));

  if (!storage_stream_open_read(&s_in, PLAYBACK_SD, PLAYBACK_FILE)) {
    DebugOut_Printf(" No hay " PLAYBACK_FILE " en la SD1\r\n");
    return false;
  }
  if (!storage_stream_read(&s_in, &s_h, sizeof(s_h)) ||
      (memcmp(s_h.magic, PLAYBACK_MAGIC, sizeof(s_h.magic)) != 0)) {
    DebugOut_Printf(" " PLAYBACK_FILE " no es un archivo de vuelo valido\r\n");
    (void)storage_stream_close(&s_in);
    return false;
  }
  if (s_h.record_bytes != sizeof(playback_rec_t)) {
    DebugOut_Printf(" " PLAYBACK_FILE ": registro de %lu B, esperaba %lu"
                    " (regenerar con vuelo_from_pil.py)\r\n",
                    (unsigned long)s_h.record_bytes,
                    (unsigned long)sizeof(playback_rec_t));
    (void)storage_stream_close(&s_in);
    return false;
  }

  s_summary_open = storage_stream_open_write(&s_summary, PLAYBACK_SD,
                                             PLAYBACK_SUMMARY, s_summary_buf,
                                             sizeof(s_summary_buf));
  s_log_open = storage_stream_open_write(&s_log, PLAYBACK_SD, PLAYBACK_LOG,
                                         s_log_buf, sizeof(s_log_buf));
  if (s_log_open) {
    (void)storage_stream_puts(
        &s_log, "t_s,nav_valid,rocket_state,flight_phase,pos_n_m,pos_e_m,"
                "pos_d_m,vel_d_mps,roll_deg,pitch_deg,yaw_deg,apogee_pred_m,"
                "apogee_error_m,actuator_cmd,servo_byte,can_ok\r\n");
  }

  _report("\r\n=================================================\r\n");
  _report(" REPRODUCCION DE VUELO   %lu pasos, %lu s\r\n",
          (unsigned long)s_h.record_count,
          (unsigned long)(s_h.record_count / TICKS_POR_S));
  _report(" Mismo camino que en vuelo: GNC, despliegue (1A/1B) y aerofreno\r\n"
          " al SERVO de verdad (CAN2 0x%03X)\r\n",
          (unsigned)AIRBRAKE_CAN_ID);
  _report("=================================================\r\n");
  if (!s_log_open) {
    _report("   aviso: no se puede crear " PLAYBACK_LOG "\r\n");
  }

  /* Primero el reinicio y LUEGO la puerta: si se abriera antes, entre una
   * cosa y otra podria colarse alguna medida real. */
  gnc_init();
  gnc_set_rampa_solo_este_vuelo(PLAYBACK_RAMPA_VERTICAL_DEG,
                                PLAYBACK_RAMPA_RUMBO_DEG,
                                PLAYBACK_RAMPA_ALABEO_DEG);
  gnc_set_replay(true);

  s_t_start = HAL_GetTick();
  s_nav_previa = false;
  s_nav_alguna_vez = false;
  s_reinicios_ekf = 0U;
  s_drogue_avisado = false;
  s_main_avisado = false;
  s_last_phase = -1;
  s_step = 0U;
  s_esperando = false;
  s_enviadas_inicio = airbrake_sent();
  s_activa = true;
  return true;
}

/*
 * Mete en el GNC las muestras de un registro, por las mismas funciones que
 * usa sensor_board.c en vuelo. No da el paso: lo da gnc_update() en su sitio
 * del bucle. Devuelve false si ya no quedan registros o no se pudo leer.
 */
static bool _alimentar(void) {
  if (s_step >= s_h.record_count) {
    return false;
  }
  playback_rec_t rec;
  if (!storage_stream_read(&s_in, &rec, sizeof(rec))) {
    _report("   lectura cortada en el paso %lu\r\n", (unsigned long)s_step);
    return false;
  }
  /* Por la puerta de siempre: estas son las mismas llamadas que hace
   * sensor_board.c al decodificar CAN2, así que la reproducción pasa por
   * el giro de ejes y el promediado igual que en vuelo. */
  /*
   * NUNCA pasar punteros a campos del registro: está empaquetado y los arrays
   * caen en direcciones sin alinear (imu2_accel empieza en el byte 33). El
   * Cortex-M7 no lee floats con la FPU desde una dirección desalineada: da
   * un HardFault (UNALIGNED). Pasó el 2026-10-04, la simulación colgaba el
   * core en el primer paso. Se copian a variables alineadas.
   */
  float a1[3];
  float g1[3];
  float a2[3];
  memcpy(a1, rec.imu1_accel, sizeof(a1));
  memcpy(g1, rec.imu1_gyro, sizeof(g1));
  memcpy(a2, rec.imu2_accel, sizeof(a2));
  if (rec.imu1_valid != 0U) {
    gnc_on_imu1(a1, g1, (uint64_t)(rec.imu1_ts_s * 1.0e6 + 0.5));
  }
  if (rec.imu2_valid != 0U) {
    gnc_on_imu2(a2, (uint64_t)(rec.imu2_ts_s * 1.0e6 + 0.5));
  }
  if (rec.baro_valid != 0U) {
    /* La misma que sensor_board.c con el BARO1, que llega en kPa */
    gnc_on_baro(rec.baro_press_pa * 0.001f, rec.baro_temp_c,
                (uint64_t)(rec.baro_ts_s * 1.0e6 + 0.5));
  }
  if (rec.gps_valid != 0U) {
    /* Las dos mismas llamadas que hace sensor_board.c con GGA y NAV-PVT.
     * El archivo no trae el bit gnssFixOK: el vuelo de referencia solo
     * guarda el fix_type, que ya es el que mira el modelo. */
    gnc_on_gps_position(rec.lat_e7, rec.lon_e7, rec.alt_mm);
    gnc_on_gps_pvt(rec.vn_mmps, rec.ve_mmps, rec.vd_mmps, rec.fix_type,
                   rec.satellites, true);
  }

  if (rec.cmd_valid != 0U) {
    gnc_command(rec.cmd_id, rec.cmd_param);
    _report("   t=%7.2f s   comando %u\r\n", (double)s_step * PASO_S,
            (unsigned)rec.cmd_id);
  }

  return true;
}

/*
 * Lo que ha salido del paso del registro s_step. Se llama en la vuelta
 * siguiente del bucle, cuando ya han pasado gnc_update(), despliegue_update()
 * y airbrake_update(): el byte del servo que se apunta es el que salio de
 * verdad en ese paso, por el mismo codigo que en vuelo.
 */
static void _analizar(void) {
  const uint32_t step = s_step;
  const gnc_data_t *g = gnc_get_data();
  const bool can_ok = airbrake_last_ok();
  if (!can_ok) {
    s_result.can_failures++;
  }

  /* --- Lo que ha pasado --- */
  if (g->actuator_cmd > s_result.max_actuator) {
    s_result.max_actuator = g->actuator_cmd;
  }
  if (g->actuator_cmd > 0.001f) {
    s_result.steps_actuated++;
    s_result.apogee_pred_m = g->apogee_pred_m;
  }
  if (g->actuator_cmd > (GNC_ACTUADOR_MAX - 0.001f)) {
    s_result.steps_saturated++;
  }
  const float altitude = -g->pos_ned_m[2];
  if (altitude > s_result.max_altitude_m) {
    s_result.max_altitude_m = altitude;
  }
  if (g->nav_valid != s_nav_previa) {
    if (g->nav_valid) {
      _report("   t=%7.2f s   nav VALIDA%s\r\n", (double)step * PASO_S,
              s_nav_alguna_vez ? " otra vez (realineado)" : " (alineado)");
      s_nav_alguna_vez = true;
    } else {
      s_reinicios_ekf++;
      _report("   t=%7.2f s   nav INVALIDA: REINICIO DEL EKF   fase %s  "
              "alt %.1f m  vz %.1f m/s  |a| %.1f m/s2\r\n",
              (double)step * PASO_S, _phase_name(g->flight_phase),
              (double)(-g->pos_ned_m[2]), (double)g->vel_ned_mps[2],
              (double)g->accel_norm_mps2);
    }
    s_nav_previa = g->nav_valid;
  }
  if (g->drogue_flag && !s_drogue_avisado) {
    _report("   t=%7.2f s   DROGUE: bandera del GNC -> se encienden 1A y 1B\r\n",
            (double)step * PASO_S);
    s_drogue_avisado = true;
  }
  if (g->main_flag && !s_main_avisado) {
    _report("   t=%7.2f s   MAIN: bandera del GNC -> se enciende 2A\r\n",
            (double)step * PASO_S);
    s_main_avisado = true;
  }
  if (g->flight_phase != s_last_phase) {
    _report("   t=%7.2f s   fase %s\r\n", (double)step * PASO_S,
            _phase_name(g->flight_phase));
    s_last_phase = g->flight_phase;
  }

  if (s_log_open && ((step % LOG_CADA_TICKS) == 0U)) {
    uint32_t n = _append(s_line, sizeof(s_line), 0U, "%.4f,%u,%ld,%ld",
                         (double)step * PASO_S,
                         (unsigned)(g->nav_valid ? 1U : 0U),
                         (long)g->rocket_state, (long)g->flight_phase);
    n = _append(s_line, sizeof(s_line), n, ",%.3f,%.3f,%.3f,%.3f",
                (double)g->pos_ned_m[0], (double)g->pos_ned_m[1],
                (double)g->pos_ned_m[2], (double)g->vel_ned_mps[2]);
    n = _append(s_line, sizeof(s_line), n, ",%.2f,%.2f,%.2f",
                (double)g->euler_rpy_deg[0], (double)g->euler_rpy_deg[1],
                (double)g->euler_rpy_deg[2]);
    n = _append(s_line, sizeof(s_line), n, ",%.1f,%.1f,%.4f,%u,%u\r\n",
                (double)g->apogee_pred_m, (double)g->apogee_error_m,
                (double)g->actuator_cmd, (unsigned)airbrake_last_byte(),
                (unsigned)(can_ok ? 1U : 0U));
    (void)storage_stream_write(&s_log, s_line, n);
  }

  if ((step % (10U * TICKS_POR_S)) == 0U) {
    DebugOut_Printf("   %lu/%lu  t=%.0f s  fase %s  aerofreno %3.0f %%\r\n",
                    (unsigned long)step, (unsigned long)s_h.record_count,
                    (double)step * PASO_S, _phase_name(g->flight_phase),
                    (double)(g->actuator_cmd * 100.0f));
  }
}

static void _cerrar(void) {
  (void)storage_stream_close(&s_in);
  if (s_log_open) {
    (void)storage_stream_close(&s_log);
    s_log_open = false;
  }

  s_result.ran = (s_step > 0U);
  s_result.steps = s_step;
  s_result.duration_ms = HAL_GetTick() - s_t_start;
  s_result.last_phase = s_last_phase;

  _report("-------------------------------------------------\r\n");
  if (s_step < s_h.record_count) {
    _report("   CORTADA en el paso %lu de %lu\r\n", (unsigned long)s_step,
            (unsigned long)s_h.record_count);
  }
  _report("   Reinicios del EKF     %8lu%s\r\n",
          (unsigned long)s_reinicios_ekf,
          (s_reinicios_ekf > 0U) ? "   <<< navegacion perdida en vuelo" : "");
  _report("   Altura maxima         %8.1f m\r\n",
          (double)s_result.max_altitude_m);
  _report("   Apogeo previsto       %8.1f m\r\n",
          (double)s_result.apogee_pred_m);
  _report("   Aerofreno maximo      %8.1f %%\r\n",
          (double)(s_result.max_actuator * 100.0f));
  if (s_result.steps_actuated > 0U) {
    _report("   Aerofreno activo      %8lu pasos (%.1f s), saturado %lu"
            " (%.0f %%)\r\n",
            (unsigned long)s_result.steps_actuated,
            (double)s_result.steps_actuated * PASO_S,
            (unsigned long)s_result.steps_saturated,
            100.0 * (double)s_result.steps_saturated /
                (double)s_result.steps_actuated);
  } else {
    _report("   El aerofreno no llego a salir en todo el vuelo\r\n");
  }
  _report("   Despliegue drogue 1A  %8lu ordenes%s\r\n",
          (unsigned long)despliegue_ordenes_1a(),
          (despliegue_ordenes_1a() > 0U) ? "   (canal encendido DE VERDAD)" : "");
  _report("   Despliegue drogue 1B  %8lu ordenes%s\r\n",
          (unsigned long)despliegue_ordenes_1b(),
          (despliegue_ordenes_1b() > 0U) ? "   (canal encendido DE VERDAD)" : "");
  _report("   Despliegue main 2A    %8lu ordenes%s\r\n",
          (unsigned long)despliegue_ordenes_2a(),
          (despliegue_ordenes_2a() > 0U) ? "   (canal encendido DE VERDAD)" : "");
  if (despliegue_fallos() > 0U) {
    _report("   Ordenes de despliegue rechazadas por power: %lu\r\n",
            (unsigned long)despliegue_fallos());
  }
  _report("   Ordenes al servo      %8lu enviadas, %lu sin enviar\r\n",
          (unsigned long)(airbrake_sent() - s_enviadas_inicio),
          (unsigned long)s_result.can_failures);
  _report("   %lu pasos en %lu ms\r\n", (unsigned long)s_result.steps,
          (unsigned long)s_result.duration_ms);
  _report("   Detalle paso a paso en " PLAYBACK_LOG "\r\n");
  _report("=================================================\r\n\r\n");

  if (s_summary_open) {
    (void)storage_stream_close(&s_summary);
    s_summary_open = false;
  }

  /* El vuelo reproducido deja el GNC al final del trayecto: se reinicia para
   * que lo que venga después parta de cero, y vuelve a los sensores reales.
   * Con el GNC otra vez en POWER_ON, airbrake_update() manda 0 en el
   * siguiente paso: el aerofreno se recoge solo, sin codigo de simulacion. */
  gnc_init();
  gnc_set_replay(false);
  s_activa = false;
}

/* === Avance, una vuelta del bucle cada vez ============================== */

bool gnc_playback_start(void) {
  if (s_activa) {
    return false;
  }
  return _abrir();
}

/*
 * Va ANTES de gnc_update() en el bucle, en el sitio de los sensores. Como
 * mucho un registro por vuelta: si va con retraso (una escritura larga en la
 * SD) se pone al dia en las vueltas siguientes, que duran mucho menos de
 * 10 ms, sin dejar a la radio sin turno.
 */
void gnc_playback_update(void) {
  if (!s_activa) {
    return;
  }
  if (s_esperando) {
    if (gnc_get_data()->steps == s_pasos_gnc) {
      return; /* gnc_update() aun no ha dado el paso */
    }
    s_esperando = false;
    _analizar();
    s_step++;
  }

  /* El registro n toca en t_start + 2,5(n+1) ms. HAL_GetTick() va en ms,
   * así que salen 2 o 3 ms alternos, 2,5 de media: el modelo no lo nota,
   * porque integra con su dt fijo, no con el reloj. */
  const uint32_t due =
      s_t_start + (uint32_t)(((uint64_t)(s_step + 1U) * GNC_PERIOD_US) / 1000U);
  if ((int32_t)(HAL_GetTick() - due) < 0) {
    return;
  }
  if (!_alimentar()) {
    _cerrar();
    return;
  }
  s_pasos_gnc = gnc_get_data()->steps;
  s_esperando = true;
  gnc_replay_muestra_lista();
}

void gnc_playback_stop(void) {
  if (s_activa) {
    _cerrar();
  }
}

bool gnc_playback_active(void) { return s_activa; }

uint8_t gnc_playback_progress_pct(void) {
  if (!s_activa || (s_h.record_count == 0U)) {
    return 0U;
  }
  const uint32_t pct = (s_step * 100U) / s_h.record_count;
  return (pct > 100U) ? 100U : (uint8_t)pct;
}

const gnc_playback_result_t *gnc_playback_get_result(void) { return &s_result; }
