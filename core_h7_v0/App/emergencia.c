#include "emergencia.h"

#include <math.h>
#include <string.h>

#include "aspid_data.h"
#include "baro_alt.h"
#include "can_driver.h"
#include "debug_out.h"
#include "despliegue.h"
#include "gnc_playback.h"
#include "main.h"
#include "rf_telemetry.h"
#include "sensor_board.h"

#define G0 9.80665f

static emergencia_modo_t s_modo = EMERGENCIA_INICIO;
static memoria_vuelo_t s_m; /* el vuelo, tal como se va guardando */
static bool s_rapido;
static const char *s_motivo = "";

/* Vigilancia */
static uint32_t s_t_base; /* HAL_GetTick() que corresponde al despegue */
static uint32_t s_t_guardado;

/* Emergencia */
typedef enum {
  ETAPA_SUBIDA = 0,
  ETAPA_DROGUE,
  ETAPA_MAIN,
  ETAPA_TIERRA,
} etapa_t;
static etapa_t s_etapa;
static gnc_data_t s_gnc;
static bool s_apogeo_baro;
static bool s_main_baro;
static bool s_drogue_pend;
static bool s_main_pend;
static uint32_t s_t_drogue; /* bandera levantada aquí; 0 = nunca */
static uint32_t s_t_main;
static bool s_c_apogeo, s_c_main, s_c_tierra;
static uint32_t s_t_apogeo, s_t_main_c, s_t_tierra;
static float s_freno;
static uint32_t s_gc_visto;
static bool s_motor;
static uint32_t s_t_motor; /* la última vez que se vio el motor */
static uint32_t s_imu_visto[2];
static bool s_gnc_vuelo_prev; /* para el flanco del despegue (vigilar) */

#if EMERGENCIA_PRUEBA == 1
static uint32_t s_t_prueba_fin;
#endif

static uint32_t _no_cero(uint32_t t) { return (t != 0U) ? t : 1U; }

static void _guardar(uint32_t ahora) {
  s_m.t_vuelo_ms =
      ((s_m.flags & MV_DESPEGUE) != 0U) ? (ahora - s_t_base) : 0U;
  (void)memoria_vuelo_guardar(&s_m);
  s_t_guardado = ahora;
}

/* Sin vuelo en marcha, pero sin perder la cuenta de vuelos */
static void _olvidar(uint16_t vuelo) {
  memset(&s_m, 0, sizeof(s_m));
  s_m.vuelo = vuelo;
  _guardar(HAL_GetTick());
}

/* |a| de la IMU más nueva, en g; < 0 si no ha llegado nada nuevo */
static float _acel_g(void) {
  const sensor_board_data_t *sb = sensor_board_get_data();
  float a = -1.0f;
  for (uint32_t i = 0U; i < 2U; i++) {
    const imu_data_t *m = &sb->imu[i];
    if (m->count == s_imu_visto[i]) {
      continue;
    }
    s_imu_visto[i] = m->count;
    const float n = sqrtf((m->accel_mps2[0] * m->accel_mps2[0]) +
                          (m->accel_mps2[1] * m->accel_mps2[1]) +
                          (m->accel_mps2[2] * m->accel_mps2[2])) /
                    G0;
    a = (n > a) ? n : a;
  }
  return a;
}

/* === Arranque ============================================================ */

emergencia_modo_t emergencia_arranque(void) {
  memset(&s_m, 0, sizeof(s_m));
  const bool hay = memoria_vuelo_ultimo(&s_m);
  const bool encendido = (memoria_vuelo_causa() == MEMORIA_RESET_ENCENDIDO);
  s_modo = EMERGENCIA_INICIO;
  if (!memoria_vuelo_disponible()) {
    s_motivo = "memoria de vuelo NO disponible (el firmware pisa el sector 3)";
  } else if (!hay) {
    s_motivo = "sin vuelo guardado";
  } else if ((s_m.flags & MV_SIMULACION) != 0U) {
    s_motivo = "lo guardado era de una simulacion: no cuenta";
    _olvidar(s_m.vuelo);
  } else if ((s_m.flags & MV_DESPEGUE) == 0U) {
    s_motivo = "sin vuelo en marcha";
  } else if ((s_m.flags & MV_TERMINADO) != 0U) {
    if (encendido) {
      s_motivo = "vuelo terminado y encendido de nuevo: se empieza de cero";
      _olvidar(s_m.vuelo);
    } else {
      s_modo = EMERGENCIA_POSTVUELO;
      s_motivo = "reiniciado con el vuelo ya terminado: no se toca nada";
    }
  } else {
    s_modo = EMERGENCIA_POR_VER;
    s_rapido = !encendido; /* tras un encendido arrancan todas las placas */
    s_motivo = "vuelo sin terminar: mirando si esta en el aire";
  }
  return s_modo;
}

bool emergencia_arranque_rapido(void) {
  return (s_modo == EMERGENCIA_POSTVUELO) ||
         ((s_modo == EMERGENCIA_POR_VER) && s_rapido);
}

static void _empezar(void) {
  const uint32_t ahora = HAL_GetTick();
  if (s_m.reinicios < 255U) {
    s_m.reinicios++;
  }
  s_m.flags |= MV_EMERGENCIA;
  s_t_motor = ahora;

  /* Dónde estaba el vuelo */
  int32_t fase;
  if (((s_m.flags & MV_MAIN) != 0U) || (s_m.fase == GNC_FASE_MAIN)) {
    s_etapa = ETAPA_MAIN;
    s_main_pend = (s_m.flags & MV_MAIN) == 0U;
    fase = GNC_FASE_MAIN;
  } else if (((s_m.flags & (MV_APOGEO | MV_DROGUE)) != 0U) ||
             (s_m.fase == GNC_FASE_APOGEO) || (s_m.fase == GNC_FASE_DROGUE)) {
    s_etapa = ETAPA_DROGUE;
    s_drogue_pend = (s_m.flags & MV_DROGUE) == 0U;
    fase = GNC_FASE_DROGUE;
  } else if (s_m.fase == GNC_FASE_EN_TIERRA) {
    s_etapa = ETAPA_TIERRA;
    fase = GNC_FASE_EN_TIERRA;
  } else {
    s_etapa = ETAPA_SUBIDA;
    fase = (s_m.fase >= GNC_FASE_BOOST) ? (int32_t)s_m.fase : GNC_FASE_BOOST;
    /* Ya sin motor: el aerofreno empieza fuera, sin recogerse y volver a
     * salir por el reinicio */
    if (fase >= GNC_FASE_COAST) {
      s_freno = 1.0f;
    }
  }
  memset(&s_gnc, 0, sizeof(s_gnc));
  s_gnc.rocket_state = GNC_ESTADO_EN_VUELO;
  s_gnc.flight_phase = fase;
  s_gnc.pyro_armed = true;
  s_gnc.quat_nb[0] = 0.0f; /* sin actitud: todo a 0 y nav_valid dirá */
  s_m.estado = GNC_ESTADO_EN_VUELO;
  s_m.fase = (uint8_t)fase;
  _guardar(ahora);
  rf_telemetry_pedir_estado();
  DebugOut_Printf("EMERGENCIA: reiniciado en vuelo (#%u del vuelo %u), fase "
                  "%ld, drogue %s, main %s. Solo barometro.\r\n",
                  (unsigned)s_m.reinicios, (unsigned)s_m.vuelo, (long)fase,
                  ((s_m.flags & MV_DROGUE) != 0U) ? "ya mandado" : "no",
                  ((s_m.flags & MV_MAIN) != 0U) ? "ya mandado" : "no");
}

emergencia_modo_t emergencia_confirmar(sys_state *state) {
  if (s_modo != EMERGENCIA_POR_VER) {
    return s_modo;
  }
  baro_alt_referencia(s_m.p0_pa, s_m.max_m);
  /* El reloj del vuelo sigue donde lo dejó la memoria */
  s_t_base = HAL_GetTick() - s_m.t_vuelo_ms;
  const uint32_t t0 = HAL_GetTick();
  bool aire = false;
  bool tierra = false;
  bool rara = false;
  bool quieto = false;
  uint32_t t_rara = 0U;
  uint32_t t_quieto = 0U;
  uint32_t brutas = 0U;
  uint32_t vistas[BARO_N] = {0};
  float a = -1.0f;
  while (!aire && !tierra &&
         ((HAL_GetTick() - t0) < EMERGENCIA_COMPROBAR_MS)) {
    can_rx_frame_t f;
    while (can_driver_receive(CAN_PORT_1, &f)) {
      sys_process_can1_frame(state, &f);
    }
    while (can_driver_receive(CAN_PORT_2, &f)) {
      (void)sensor_board_process_frame(f.id, f.data, f.len, f.rx_tick_ms);
    }
    const uint32_t ahora = HAL_GetTick();
    (void)baro_alt_update(ahora);

    /* Barómetros: fiable y alto, o en bruto muy alto aunque haga ruido */
    bool suelo = baro_alt_fiable();
    for (uint32_t k = 0U; k < BARO_N; k++) {
      const baro_canal_t *c = baro_alt_canal(k);
      if (c->muestras != vistas[k]) {
        vistas[k] = c->muestras;
        if ((c->p_pa > BARO_P_MIN_PA) && (c->p_pa < BARO_P_MAX_PA) &&
            (c->h_m > EMERGENCIA_AIRE_BRUTO_M)) {
          brutas++;
        }
      }
      if (c->fiable && (c->h_m > EMERGENCIA_AIRE_M)) {
        aire = true;
      }
      if (c->fiable && ((c->h_m > EMERGENCIA_AIRE_M) ||
                        (fabsf(c->vz_mps) > EMERGENCIA_TIERRA_VZ_MPS))) {
        suelo = false;
      }
    }
    if (brutas >= EMERGENCIA_AIRE_BRUTO_MUESTRAS) {
      aire = true;
    }

    /* Aceleración: lejos de 1 g un rato = no está quieto en el suelo */
    const float an = _acel_g();
    if (an >= 0.0f) {
      a = an;
      if ((a < 0.7f) || (a > 1.3f)) {
        if (!rara) {
          rara = true;
          t_rara = ahora;
        }
        if ((ahora - t_rara) >= EMERGENCIA_ACEL_RARA_MS) {
          aire = true;
        }
      } else {
        rara = false;
      }
    }
    suelo = suelo && (a >= 0.8f) && (a <= 1.2f);
    if (suelo) {
      if (!quieto) {
        quieto = true;
        t_quieto = ahora;
      }
      tierra = (ahora - t_quieto) >= EMERGENCIA_QUIETO_MS;
    } else {
      quieto = false;
    }
  }

  const bool encendido = (memoria_vuelo_causa() == MEMORIA_RESET_ENCENDIDO);
  if (aire || !tierra) {
    s_modo = EMERGENCIA_EN_VUELO;
    s_motivo = aire ? "REINICIADO EN EL AIRE: modo de emergencia"
                    : "vuelo sin terminar y sin datos para decidir: modo de "
                      "emergencia";
    _empezar();
  } else if (encendido) {
    s_modo = EMERGENCIA_INICIO;
    s_motivo = "vuelo sin terminar, pero quieto en el suelo tras un "
               "encendido: se empieza de cero";
    _olvidar(s_m.vuelo);
  } else {
    s_modo = EMERGENCIA_POSTVUELO;
    s_motivo = "vuelo sin terminar, pero quieto en el suelo: no se toca nada";
  }
  return s_modo;
}

/* === Modo de emergencia ================================================== */

/* Condición mantenida al menos ms */
static bool _mantenida(bool cond, bool *antes, uint32_t *t, uint32_t ahora,
                       uint32_t ms) {
  if (!cond) {
    *antes = false;
    return false;
  }
  if (!*antes) {
    *antes = true;
    *t = ahora;
  }
  return (ahora - *t) >= ms;
}

/* Apogeo: un barómetro fiable ha bajado 10 m de SU máxima y ninguno fiable
 * sigue subiendo */
static bool _apogeo(void) {
  bool alguno = false;
  for (uint32_t k = 0U; k < BARO_N; k++) {
    const baro_canal_t *c = baro_alt_canal(k);
    if (!c->fiable) {
      continue;
    }
    if (c->vz_mps > EMERGENCIA_SUBIENDO_MPS) {
      return false;
    }
    if ((c->max_m - c->h_m) >= EMERGENCIA_APOGEO_BAJADA_M) {
      alguno = true;
    }
  }
  return alguno;
}

static void _decidir(uint32_t ahora, bool fiable, float h, float vz) {
  switch (s_etapa) {
  case ETAPA_SUBIDA:
    if (_mantenida(_apogeo(), &s_c_apogeo, &s_t_apogeo, ahora,
                   EMERGENCIA_CONFIRMAR_MS) &&
        fiable && (h > EMERGENCIA_MIN_ALTURA_M)) {
      s_apogeo_baro = true;
      s_etapa = ETAPA_DROGUE;
      s_drogue_pend = (s_m.flags & MV_DROGUE) == 0U;
      rf_telemetry_pedir_estado();
      DebugOut_Printf("EMERGENCIA: APOGEO por el barometro a %.1f m "
                      "(maxima %.1f / %.1f)\r\n",
                      (double)h, (double)baro_alt_canal(0)->max_m,
                      (double)baro_alt_canal(1)->max_m);
    }
    break;
  case ETAPA_DROGUE:
    if (_mantenida(fiable && (h <= EMERGENCIA_MAIN_M), &s_c_main, &s_t_main_c,
                   ahora, EMERGENCIA_CONFIRMAR_MS)) {
      s_main_baro = true;
      s_etapa = ETAPA_MAIN;
      s_main_pend = (s_m.flags & MV_MAIN) == 0U;
      rf_telemetry_pedir_estado();
      DebugOut_Printf("EMERGENCIA: ALTURA DEL MAIN, %.1f m\r\n", (double)h);
    }
    break;
  case ETAPA_MAIN:
    if (_mantenida(fiable && (h < EMERGENCIA_TIERRA_M) &&
                       (fabsf(vz) < EMERGENCIA_TIERRA_VZ_MPS),
                   &s_c_tierra, &s_t_tierra, ahora, EMERGENCIA_TIERRA_MS)) {
      s_etapa = ETAPA_TIERRA;
      rf_telemetry_pedir_estado();
      DebugOut_Printf("EMERGENCIA: EN TIERRA (%.1f m)\r\n", (double)h);
    }
    break;
  default:
    break;
  }
}

/* IMU: aceleración en ejes de cuerpo (como el GNC: X morro = placa Z,
 * Y = placa Y, Z = -placa X) y si el motor empuja */
static void _imu(uint32_t ahora) {
  const sensor_board_data_t *sb = sensor_board_get_data();
  const imu_data_t *m = (sb->imu[1].count > 0U) ? &sb->imu[1] : &sb->imu[0];
  s_gnc.accel_body_mps2[0] = m->accel_mps2[2];
  s_gnc.accel_body_mps2[1] = m->accel_mps2[1];
  s_gnc.accel_body_mps2[2] = -m->accel_mps2[0];
  s_motor = m->accel_mps2[2] > (EMERGENCIA_MOTOR_G * G0);
  if (s_motor) {
    s_t_motor = ahora;
  }
}

static int32_t _fase(void) {
  switch (s_etapa) {
  case ETAPA_DROGUE:
    return GNC_FASE_DROGUE;
  case ETAPA_MAIN:
    return GNC_FASE_MAIN;
  case ETAPA_TIERRA:
    return GNC_FASE_EN_TIERRA;
  default:
    return (s_m.fase >= GNC_FASE_BOOST) && (s_m.fase <= GNC_FASE_CONTROL)
               ? (int32_t)s_m.fase
               : GNC_FASE_BOOST;
  }
}

void emergencia_update(void) {
  if (s_modo != EMERGENCIA_EN_VUELO) {
    return;
  }
  const uint32_t ahora = HAL_GetTick();
  const uint32_t nuevas = baro_alt_update(ahora);
  const bool fiable = baro_alt_fiable();
  const float h = baro_alt_altura();
  const float vz = baro_alt_vz();
  _imu(ahora);
  if (nuevas > 0U) {
    _decidir(ahora, fiable, h, vz);
  }

  /* Pirotécnicos pendientes: en cuanto el barómetro deje */
  const bool arriba = fiable && (h > EMERGENCIA_MIN_ALTURA_M);
  if (s_drogue_pend && arriba) {
    s_drogue_pend = false;
    s_t_drogue = _no_cero(ahora);
    DebugOut_Printf("EMERGENCIA: DROGUE (1A y 1B) a %.1f m\r\n", (double)h);
  }
  if (s_main_pend && arriba) {
    s_main_pend = false;
    s_t_main = _no_cero(ahora);
    DebugOut_Printf("EMERGENCIA: MAIN (2A) a %.1f m\r\n", (double)h);
  }

  /* Aerofreno: fuera del todo subiendo con el motor apagado, dentro desde
   * el apogeo. A 50 Hz y con la misma rampa que el GNC. */
  const uint32_t gc = ahora / 20U;
  if (gc != s_gc_visto) {
    s_gc_visto = gc;
    const bool abrir = (s_etapa == ETAPA_SUBIDA) && !s_motor &&
                       ((ahora - s_t_motor) >= EMERGENCIA_MOTOR_APAGADO_MS);
    float d = (abrir ? 1.0f : 0.0f) - s_freno;
    if (d > EMERGENCIA_FRENO_PASO) {
      d = EMERGENCIA_FRENO_PASO;
    } else if (d < -EMERGENCIA_FRENO_PASO) {
      d = -EMERGENCIA_FRENO_PASO;
    }
    s_freno += d;
  }

  /* Lo que verán los demás por gnc_get_data() */
  s_gnc.steps = (uint32_t)(((uint64_t)ahora * GNC_RATE_HZ) / 1000U);
  s_gnc.gc_steps = gc;             /* 50 Hz: airbrake.c manda en cada uno */
  s_gnc.rocket_state = GNC_ESTADO_EN_VUELO;
  s_gnc.flight_phase = _fase();
  s_gnc.nav_valid = fiable;
  s_gnc.alt_agl_m = h;
  s_gnc.pos_ned_m[2] = -h;
  s_gnc.vel_ned_mps[2] = -vz;
  const float m0 = baro_alt_canal(0)->max_m;
  const float m1 = baro_alt_canal(1)->max_m;
  s_gnc.apogee_pred_m = (m0 > m1) ? m0 : m1;
  /* En la escala del GNC (0..GNC_ACTUADOR_MAX), que es la que espera
   * airbrake.c; s_freno es extension 0..1 */
  s_gnc.actuator_cmd = s_freno * GNC_ACTUADOR_MAX;
  s_gnc.drogue_flag =
      (s_t_drogue != 0U) && ((ahora - s_t_drogue) < EMERGENCIA_BANDERA_MS);
  s_gnc.main_flag =
      (s_t_main != 0U) && ((ahora - s_t_main) < EMERGENCIA_BANDERA_MS);
}

/* === Lo que se guarda ==================================================== */

static void _despegue(const gnc_data_t *g, uint32_t ahora) {
  float p0[BARO_N];
  if (!baro_alt_suelo(p0)) {
    const sensor_board_data_t *sb = sensor_board_get_data();
    for (uint32_t k = 0U; k < BARO_N; k++) {
      p0[k] = sb->baro[k].pressure_kpa * 1000.0f;
    }
  }
  const uint16_t vuelo = (uint16_t)(s_m.vuelo + 1U);
  memset(&s_m, 0, sizeof(s_m));
  s_m.vuelo = vuelo;
  s_m.flags = MV_DESPEGUE;
  s_m.estado = (uint8_t)g->rocket_state;
  s_m.fase = (uint8_t)g->flight_phase;
  s_m.p0_pa[0] = p0[0];
  s_m.p0_pa[1] = p0[1];
  s_t_base = ahora;
  baro_alt_referencia(p0, NULL);
  _guardar(ahora);
}

#if EMERGENCIA_PRUEBA
static bool _prueba(uint32_t ahora);
#endif

void emergencia_vigilar(void) {
  const uint32_t ahora = HAL_GetTick();
#if EMERGENCIA_PRUEBA
  if (_prueba(ahora)) {
    return; /* la prueba se queda con la vuelta */
  }
#endif
  if (!memoria_vuelo_disponible() || (s_modo == EMERGENCIA_POSTVUELO) ||
      (s_modo == EMERGENCIA_POR_VER)) {
    return;
  }
  const gnc_data_t *g = gnc_get_data();
  /*
   * El despegue se toma por el FLANCO de EN VUELO, no por el nivel. El GNC no
   * sale de EN VUELO hasta un gnc_init() o un reinicio, así que un vuelo que
   * se olvida (no pasó de 100 m) se volvía a abrir en la vuelta siguiente:
   * abrir, olvidar, abrir... dos escrituras por vuelta hasta llenar el sector
   * de la flash. Pasó el 2026-10-07 en el banco: un golpe a la aviónica (3 g
   * durante 60 ms) le dio al GNC el despegue, en el suelo pasó a APOGEO al
   * momento, y en segundos había 4096 registros; el último, un vuelo abierto,
   * y la core arrancaba en POSTVUELO. Se apunta ANTES de mirar si se simula,
   * para que el EN VUELO de una simulación no deje un flanco al acabar.
   */
  const bool gnc_vuelo = (g->rocket_state == GNC_ESTADO_EN_VUELO);
  const bool flanco = gnc_vuelo && !s_gnc_vuelo_prev;
  s_gnc_vuelo_prev = gnc_vuelo;
  if ((s_modo == EMERGENCIA_INICIO) && gnc_playback_active()) {
    return; /* una simulación nunca se guarda */
  }
  if ((s_m.flags & MV_DESPEGUE) == 0U) {
    baro_alt_suelo_update(ahora);
    if (flanco) {
      _despegue(g, ahora);
    }
    return;
  }
  if ((s_m.flags & MV_TERMINADO) != 0U) {
    return;
  }
  if (s_modo == EMERGENCIA_INICIO) {
    (void)baro_alt_update(ahora); /* en emergencia ya lo hace el update */
  }

  bool urgente = false;
  if (((uint8_t)g->rocket_state != s_m.estado) ||
      ((uint8_t)g->flight_phase != s_m.fase)) {
    s_m.estado = (uint8_t)g->rocket_state;
    s_m.fase = (uint8_t)g->flight_phase;
    urgente = true;
  }
  const bool apogeo = ((g->flight_phase >= GNC_FASE_APOGEO) &&
                       (g->flight_phase <= GNC_FASE_EN_TIERRA)) ||
                      s_apogeo_baro;
  if (apogeo && ((s_m.flags & MV_APOGEO) == 0U)) {
    s_m.flags |= MV_APOGEO;
    urgente = true;
  }
  if (despliegue_drogue_mandado() && ((s_m.flags & MV_DROGUE) == 0U)) {
    s_m.flags |= MV_DROGUE;
    urgente = true;
  }
  if (despliegue_main_mandado() && ((s_m.flags & MV_MAIN) == 0U)) {
    s_m.flags |= MV_MAIN;
    urgente = true;
  }
  bool subio = false;
  for (uint32_t k = 0U; k < BARO_N; k++) {
    if (baro_alt_canal(k)->max_m > (s_m.max_m[k] + 10.0f)) {
      subio = true;
    }
  }
  /* El final, con las mismas reglas que las cámaras y la SD */
  const bool fin =
      (((s_m.flags & MV_APOGEO) != 0U) && g->nav_valid &&
       isfinite(g->alt_agl_m) && (g->alt_agl_m <= ASPID_DATA_FIN_ALTURA_M)) ||
      (g->flight_phase == GNC_FASE_EN_TIERRA) ||
      ((ahora - s_t_base) >= ASPID_DATA_MAX_VUELO_MS);
  if (!urgente && !fin && !(subio && ((ahora - s_t_guardado) >= 1000U))) {
    return;
  }
  for (uint32_t k = 0U; k < BARO_N; k++) {
    const float m = baro_alt_canal(k)->max_m;
    if (m > s_m.max_m[k]) {
      s_m.max_m[k] = m;
    }
  }
  if (fin) {
    const float max = (s_m.max_m[0] > s_m.max_m[1]) ? s_m.max_m[0]
                                                    : s_m.max_m[1];
    if (max < ASPID_DATA_VUELO_REAL_M) {
      _olvidar(s_m.vuelo); /* no pasó de 100 m: no era un vuelo */
      return;
    }
    s_m.flags |= MV_TERMINADO;
  }
  _guardar(ahora);
}

/* === Consultas =========================================================== */

emergencia_modo_t emergencia_modo(void) { return s_modo; }

const char *emergencia_modo_nombre(emergencia_modo_t m) {
  switch (m) {
  case EMERGENCIA_INICIO:
    return "inicio";
  case EMERGENCIA_EN_VUELO:
    return "EMERGENCIA EN VUELO";
  case EMERGENCIA_POSTVUELO:
    return "POSTVUELO";
  case EMERGENCIA_POR_VER:
    return "por ver";
  default:
    return "?";
  }
}

const char *emergencia_motivo(void) { return s_motivo; }
bool emergencia_activa(void) { return s_modo == EMERGENCIA_EN_VUELO; }
bool emergencia_postvuelo(void) { return s_modo == EMERGENCIA_POSTVUELO; }
const gnc_data_t *emergencia_gnc(void) { return &s_gnc; }
bool emergencia_apogeo_detectado(void) { return s_apogeo_baro; }
bool emergencia_main_alcanzado(void) { return s_main_baro; }
bool emergencia_drogue_hecho(void) { return (s_m.flags & MV_DROGUE) != 0U; }
bool emergencia_main_hecho(void) { return (s_m.flags & MV_MAIN) != 0U; }
float emergencia_freno(void) { return s_freno; }
bool emergencia_motor(void) { return s_motor; }
const memoria_vuelo_t *emergencia_memoria(void) { return &s_m; }

/* === Pruebas en la mesa ================================================= */

#if EMERGENCIA_PRUEBA
bool emergencia_prueba_sin_piros(void) { return s_modo == EMERGENCIA_EN_VUELO; }

/* Segundos del vuelo inventado: 0 cuando se guardó el vuelo falso (con 20 s
 * de vuelo). Sigue el reloj del vuelo, así que no se reinicia con la core. */
static float _t_prueba(void) {
  return ((float)(HAL_GetTick() - s_t_base) / 1000.0f) - 20.0f;
}

/* El vuelo inventado: sale de 1500 m a 150 m/s frenando a 15 m/s2 (apogeo a
 * 2250 m a los 10 s), cae libre hasta 40 m/s, baja a 40 m/s hasta 400 m y a
 * 15 m/s hasta el suelo. Los primeros 1,5 s con saltos de "Mach 1". El baro 2
 * con un 3 % de error de escala. */
void emergencia_prueba_presion(uint32_t k, float p0_pa, float *p_pa) {
  if (((s_modo != EMERGENCIA_EN_VUELO) && (s_modo != EMERGENCIA_POR_VER)) ||
      (p0_pa <= 0.0f)) {
    return;
  }
  static uint32_t lcg = 12345U;
  lcg = (lcg * 1664525U) + 1013904223U;
  const float azar = ((float)(lcg >> 8) / 16777216.0f) - 0.5f; /* -0,5..0,5 */
  const float t = _t_prueba();
  const float t_libre = 40.0f / 9.8f;                         /* 4,08 s */
  const float h_libre = 2250.0f - (4.9f * t_libre * t_libre); /* 2168 m */
  const float t_400 = 10.0f + t_libre + ((h_libre - 400.0f) / 40.0f);
  float h;
  if (t < 10.0f) {
    h = 1500.0f + (150.0f * t) - (7.5f * t * t);
  } else if (t < (10.0f + t_libre)) {
    const float d = t - 10.0f;
    h = 2250.0f - (4.9f * d * d);
  } else if (t < t_400) {
    h = h_libre - (40.0f * (t - 10.0f - t_libre));
  } else {
    h = 400.0f - (15.0f * (t - t_400));
  }
  if (h < 0.0f) {
    h = 0.0f;
  }
  if (k == 1U) {
    h *= 1.03f;
  }
  h += 0.3f * azar;
  if (t < 1.5f) {
    h += 600.0f * azar; /* la onda de choque pasando por la toma */
  }
  *p_pa = p0_pa * powf(1.0f - (h / 44330.77f), 5.25588f);
}
#endif

#if EMERGENCIA_PRUEBA == 1
/* La marca de "prueba hecha" que deja al final: cambia con cada compilación,
 * así que un firmware nuevo la vuelve a hacer una vez */
#define PRUEBA_DIGITO(i) ((uint32_t)(__TIME__[i] - '0'))
#define PRUEBA_HECHA                                                        \
  ((uint16_t)((((((PRUEBA_DIGITO(0) * 10U) + PRUEBA_DIGITO(1)) * 3600U) +    \
                (((PRUEBA_DIGITO(3) * 10U) + PRUEBA_DIGITO(4)) * 60U) +     \
                ((PRUEBA_DIGITO(6) * 10U) + PRUEBA_DIGITO(7))) %            \
               60000U) +                                                    \
              1000U))

/* Guarda ya lo que haya (antes de un reinicio provocado) */
static void _guardar_todo(uint32_t ahora) {
  for (uint32_t k = 0U; k < BARO_N; k++) {
    const float m = baro_alt_canal(k)->max_m;
    if (m > s_m.max_m[k]) {
      s_m.max_m[k] = m;
    }
  }
  _guardar(ahora);
}

/* Vuelo falso, reinicio, y en el aire: a los 5 s se cuelga el bucle (el
 * watchdog), a los 8 s una escritura prohibida por la MPU (fallo de memoria)
 * y bajando con el drogue un reset normal (no debe repetir el drogue).
 * Luego aterriza, POSTVUELO y limpieza. */
static bool _prueba(uint32_t ahora) {
  static uint32_t t_post;
  if (s_modo == EMERGENCIA_INICIO) {
    if ((s_m.vuelo == PRUEBA_HECHA) || (ahora < 15000U) ||
        ((s_m.flags & MV_DESPEGUE) != 0U)) {
      return false; /* ya hecha, o aún no */
    }
    const sensor_board_data_t *sb = sensor_board_get_data();
    memset(&s_m, 0, sizeof(s_m));
    s_m.vuelo = 1U;
    s_m.flags = MV_DESPEGUE;
    s_m.estado = GNC_ESTADO_EN_VUELO;
    s_m.fase = GNC_FASE_CONTROL;
    s_m.p0_pa[0] = sb->baro[0].pressure_kpa * 1000.0f;
    s_m.p0_pa[1] = sb->baro[1].pressure_kpa * 1000.0f;
    s_m.max_m[0] = 1490.0f;
    s_m.max_m[1] = 1490.0f * 1.03f;
    s_t_base = ahora - 20000U; /* 20 s de vuelo */
    _guardar(ahora);
    DebugOut_Printf("PRUEBA EMERGENCIA: vuelo falso guardado (CONTROL, "
                    "1500 m, p0 %.0f / %.0f Pa). REINICIANDO...\r\n",
                    (double)s_m.p0_pa[0], (double)s_m.p0_pa[1]);
    HAL_Delay(300); /* que salga por el USB */
    NVIC_SystemReset();
  }
  if (s_modo == EMERGENCIA_EN_VUELO) {
    const float t = _t_prueba();
    if ((s_m.reinicios == 1U) && (t >= 5.0f)) {
      _guardar_todo(ahora);
      DebugOut_Printf("PRUEBA EMERGENCIA: t %.1f s, COLGANDO EL BUCLE (el "
                      "watchdog tiene que reiniciar en 2 s)\r\n",
                      (double)t);
      HAL_Delay(100);
      while (1) {
      }
    }
    if ((s_m.reinicios == 2U) && (t >= 8.0f)) {
      _guardar_todo(ahora);
      DebugOut_Printf("PRUEBA EMERGENCIA: t %.1f s, FALLO DE MEMORIA "
                      "(escritura en 0x60000000, prohibida por la MPU)\r\n",
                      (double)t);
      HAL_Delay(100);
      *(volatile uint32_t *)0x60000000U = 0xDEADBEEFU;
      DebugOut_Printf("PRUEBA EMERGENCIA: NO HA FALLADO\r\n");
    }
    if ((s_m.reinicios == 3U) && (t >= 25.0f)) {
      _guardar_todo(ahora);
      DebugOut_Printf("PRUEBA EMERGENCIA: t %.1f s, bajando con el drogue: "
                      "RESET (no tiene que repetir el drogue)\r\n",
                      (double)t);
      HAL_Delay(100);
      NVIC_SystemReset();
    }
    if (s_etapa == ETAPA_TIERRA) {
      if (s_t_prueba_fin == 0U) {
        s_t_prueba_fin = ahora;
      }
      if ((ahora - s_t_prueba_fin) >= 10000U) {
        DebugOut_Printf("PRUEBA EMERGENCIA: aterrizado; REINICIANDO (tiene "
                        "que arrancar en POSTVUELO)...\r\n");
        HAL_Delay(300);
        NVIC_SystemReset();
      }
    }
  }
  if (s_modo == EMERGENCIA_POSTVUELO) {
    if (t_post == 0U) {
      t_post = ahora;
    }
    if ((ahora - t_post) >= 15000U) {
      DebugOut_Printf("PRUEBA EMERGENCIA: fin. Memoria limpia; REINICIANDO "
                      "(arranque normal)...\r\n");
      _olvidar(PRUEBA_HECHA);
      HAL_Delay(300);
      NVIC_SystemReset();
    }
  }
  return false;
}

#elif EMERGENCIA_PRUEBA == 2
/* Ruido de los barómetros de VERDAD en reposo, por el mismo filtro que en
 * vuelo, durante 2 minutos (en un arranque normal) */
static bool _prueba(uint32_t ahora) {
  static bool empezado, terminado;
  static uint32_t t0, t_info;
  static float r_max[BARO_N], r_suma[BARO_N], h_min[BARO_N], h_max[BARO_N];
  static float vz_max[BARO_N];
  static uint32_t r_n[BARO_N], t_fiable[BARO_N], visto[BARO_N];
  if ((s_modo != EMERGENCIA_INICIO) || terminado) {
    return false;
  }
  const sensor_board_data_t *sb = sensor_board_get_data();
  if (!empezado) {
    if ((sb->baro[0].count == 0U) || (sb->baro[1].count == 0U)) {
      return true;
    }
    const float p0[BARO_N] = {sb->baro[0].pressure_kpa * 1000.0f,
                              sb->baro[1].pressure_kpa * 1000.0f};
    baro_alt_referencia(p0, NULL);
    for (uint32_t k = 0U; k < BARO_N; k++) {
      h_min[k] = 1e9f;
      h_max[k] = -1e9f;
    }
    empezado = true;
    t0 = ahora;
    t_info = ahora;
    DebugOut_Printf("PRUEBA RUIDO: referencia %.1f / %.1f Pa, 2 minutos "
                    "quieto\r\n",
                    (double)p0[0], (double)p0[1]);
  }
  (void)baro_alt_update(ahora);
  for (uint32_t k = 0U; k < BARO_N; k++) {
    const baro_canal_t *c = baro_alt_canal(k);
    if (c->muestras == visto[k]) {
      continue;
    }
    visto[k] = c->muestras;
    if (c->n == BARO_VENTANA) {
      r_max[k] = (c->ruido_m > r_max[k]) ? c->ruido_m : r_max[k];
      r_suma[k] += c->ruido_m;
      r_n[k]++;
      vz_max[k] = (fabsf(c->vz_mps) > vz_max[k]) ? fabsf(c->vz_mps) : vz_max[k];
    }
    h_min[k] = (c->h_m < h_min[k]) ? c->h_m : h_min[k];
    h_max[k] = (c->h_m > h_max[k]) ? c->h_m : h_max[k];
    if (c->fiable && (t_fiable[k] == 0U)) {
      t_fiable[k] = _no_cero(ahora - t0);
    }
  }
  const bool fin = (ahora - t0) >= 120000U;
  if (((ahora - t_info) >= 10000U) || fin) {
    t_info = ahora;
    for (uint32_t k = 0U; k < BARO_N; k++) {
      const baro_canal_t *c = baro_alt_canal(k);
      DebugOut_Printf("PRUEBA RUIDO %s+%3lu s baro%lu: ruido ahora %.3f m, "
                      "medio %.3f, maximo %.3f (umbral %.1f)  h %.2f "
                      "(de %.2f a %.2f) m  |vz| max %.2f m/s  sucias %lu de "
                      "%lu  fiable %s (a los %lu ms)\r\n",
                      fin ? "FINAL " : "", (unsigned long)((ahora - t0) / 1000U),
                      (unsigned long)(k + 1U), (double)c->ruido_m,
                      (double)((r_n[k] > 0U) ? (r_suma[k] / (float)r_n[k]) : 0.0f),
                      (double)r_max[k], (double)BARO_RUIDO_MAX_M,
                      (double)c->h_m, (double)h_min[k], (double)h_max[k],
                      (double)vz_max[k], (unsigned long)c->sucias,
                      (unsigned long)c->muestras, c->fiable ? "SI" : "NO",
                      (unsigned long)t_fiable[k]);
    }
  }
  if (fin) {
    terminado = true;
  }
  return true;
}

#elif EMERGENCIA_PRUEBA == 3
/* Sector lleno: escribe registros hasta pasar de 3/4, deja como último el
 * vuelo 9999 y reinicia; el siguiente arranque normal tiene que borrar y
 * quedarse con ese */
static bool _prueba(uint32_t ahora) {
  static bool dicho;
  static uint16_t vuelo = 7000U;
  if (s_modo != EMERGENCIA_INICIO) {
    return false;
  }
  if ((s_m.vuelo == 9999U) && (memoria_vuelo_borrados() > 0U)) {
    if (!dicho) {
      dicho = true;
      DebugOut_Printf("PRUEBA SECTOR: tras el reinicio: borrados %lu (en %lu "
                      "ms), registros usados %lu, ultimo vuelo #%u (max %.0f), "
                      "fallos %lu\r\n",
                      (unsigned long)memoria_vuelo_borrados(),
                      (unsigned long)memoria_vuelo_borrado_ms(),
                      (unsigned long)memoria_vuelo_usados(),
                      (unsigned)s_m.vuelo, (double)s_m.max_m[0],
                      (unsigned long)memoria_vuelo_fallos());
    }
    return false;
  }
  if (ahora < 25000U) {
    return false;
  }
  const uint32_t meta = MEMORIA_VUELO_BORRAR_DESDE + 20U;
  for (uint32_t i = 0U; (i < 200U) && (memoria_vuelo_usados() < meta); i++) {
    memset(&s_m, 0, sizeof(s_m));
    vuelo++;
    s_m.vuelo = vuelo;
    s_m.max_m[0] = (float)vuelo;
    (void)memoria_vuelo_guardar(&s_m);
  }
  if (memoria_vuelo_usados() >= meta) {
    memset(&s_m, 0, sizeof(s_m));
    s_m.vuelo = 9999U;
    s_m.max_m[0] = 1234.5f;
    (void)memoria_vuelo_guardar(&s_m);
    DebugOut_Printf("PRUEBA SECTOR: lleno, %lu de %lu registros, %lu fallos; "
                    "el ultimo es el vuelo #9999 (max 1234.5). "
                    "REINICIANDO...\r\n",
                    (unsigned long)memoria_vuelo_usados(),
                    (unsigned long)MEMORIA_VUELO_REGISTROS,
                    (unsigned long)memoria_vuelo_fallos());
    HAL_Delay(300);
    NVIC_SystemReset();
  }
  return true;
}
#endif
