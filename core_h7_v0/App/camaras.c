#include "camaras.h"

#include <math.h>
#include <string.h>

#include "aspid_data.h"
#include "debug_out.h"
#include "emergencia.h"
#include "gnc.h"
#include "gnc_playback.h"
#include "main.h"
#include "rf_telemetry.h"
#include "sensor_board.h"
#include "usart.h"

#define G0 9.80665f

/* Fin del vuelo: los MISMOS valores que cierran el registro de la SD */
#define FIN_ALTURA_M ASPID_DATA_FIN_ALTURA_M
#define VUELO_REAL_M ASPID_DATA_VUELO_REAL_M
#define MAX_VUELO_MS ASPID_DATA_MAX_VUELO_MS

/* Una hora del GPS más vieja que esto no se usa */
#define HORA_VIEJA_MS 2000U

static bool s_encendidas;
static camaras_origen_t s_origen;
static uint32_t s_cambio_ms;

/* Apagado: parar primero, cortar después */
static bool s_apagando;
static camaras_origen_t s_apagar_origen;
static uint32_t s_t_apagar;
static bool s_listas;      /* todas paradas (o sin contestar)... */
static uint32_t s_t_listas; /* ...desde aquí */
static bool s_ultimo_por_plazo;
static uint32_t s_cortes_por_plazo;

/* El vuelo, visto desde aquí */
static bool s_en_vuelo;
static bool s_apogeo;
static float s_alt_max;
static uint32_t s_t_limite; /* desde aquí cuentan los 20 minutos */
static bool s_terminado; /* hubo un vuelo de verdad y acabó: ya no se
                           encienden solas en este arranque (el golpe del
                           aterrizaje no debe parecer otro despegue) */
static bool s_gnc_vuelo_prev;
static uint32_t s_steps = UINT32_MAX;
static bool s_alto;          /* una IMU por encima de 3 g... */
static uint32_t s_t_alto;    /* ...desde este HAL_GetTick() */
static bool s_alto_usado;    /* ese tramo ya disparó un despegue */

/* Las cámaras */
static camara_t s_cam[CAMARAS_N];
static uint32_t s_firma[CAMARAS_N]; /* para ver cambios */

/* Paquete de cámaras */
static bool s_paq_pendiente;
static uint32_t s_paq_ms;
static uint8_t s_paq_contador;

/* Hora del GPS, de la trama RMC */
static uint32_t s_rmc_visto;
static bool s_rmc_ok;
static bool s_rmc_falsa; /* la de la compilación, en la prueba */
static uint32_t s_rmc_tick;
static uint16_t s_rmc_anio;
static uint8_t s_rmc_mes;
static uint8_t s_rmc_dia;
static uint32_t s_rmc_ms_dia;
static char s_hora[20];
static bool s_hora_ok;
static uint32_t s_hora_tick;

static void _pedir_paquete(void) { s_paq_pendiente = true; }

static void _pin(bool on) {
  HAL_GPIO_WritePin(CAM_EN_GPIO_Port, CAM_EN_Pin,
                    on ? GPIO_PIN_SET : GPIO_PIN_RESET);
  for (uint32_t i = 0U; i < CAMARAS_N; i++) {
    camara_corriente(&s_cam[i], on);
  }
}

static float _norma(const float v[3]) {
  return sqrtf((v[0] * v[0]) + (v[1] * v[1]) + (v[2] * v[2]));
}

/* === Encender y apagar =================================================== */

/* Empieza el apagado: que paren, y ya se cortará (_vigilar_apagado) */
static void _apagar(camaras_origen_t origen) {
  if (!s_encendidas) {
    _pin(false);
    return;
  }
  if (s_apagando) {
    return;
  }
  s_apagando = true;
  s_apagar_origen = origen;
  s_t_apagar = HAL_GetTick();
  s_listas = false;
  for (uint32_t i = 0U; i < CAMARAS_N; i++) {
    camara_quiere(&s_cam[i], false);
  }
  _pedir_paquete();
}

static void _cortar(uint32_t ahora, bool por_plazo) {
  _pin(false);
  s_apagando = false;
  s_ultimo_por_plazo = por_plazo;
  if (por_plazo) {
    s_cortes_por_plazo++;
  }
  s_encendidas = false;
  s_origen = s_apagar_origen;
  s_cambio_ms = ahora;
  rf_telemetry_pedir_estado(); /* que tierra lo vea ya */
  _pedir_paquete();
}

static void _vigilar_apagado(uint32_t ahora) {
  if (!s_apagando) {
    return;
  }
  bool listas = true;
  for (uint32_t i = 0U; i < CAMARAS_N; i++) {
    listas = listas && camara_lista_para_cortar(&s_cam[i]);
  }
  if (listas && !s_listas) {
    s_t_listas = ahora;
  }
  s_listas = listas;
  if (listas && ((ahora - s_t_listas) >= CAMARAS_CIERRE_MS)) {
    _cortar(ahora, false);
  } else if ((ahora - s_t_apagar) >= CAMARAS_APAGADO_MAX_MS) {
    _cortar(ahora, true);
  }
}

void camaras_encender(camaras_origen_t origen) {
  if (s_apagando) {
    /* Se estaban apagando: que vuelvan a grabar */
    s_apagando = false;
    for (uint32_t i = 0U; i < CAMARAS_N; i++) {
      camara_quiere(&s_cam[i], true);
    }
    s_origen = origen;
    s_cambio_ms = HAL_GetTick();
    _pedir_paquete();
  }
  _pin(true);
  if (!s_encendidas) {
    s_encendidas = true;
    s_origen = origen;
    s_cambio_ms = HAL_GetTick();
    rf_telemetry_pedir_estado();
    _pedir_paquete();
  }
}

void camaras_apagar(void) { _apagar(CAMARAS_ORIGEN_TIERRA); }

/* === Hora del GPS ======================================================== */

#if CAMARAS_PRUEBA_UART
/* La hora de la compilación, como si fuera la del GPS al arrancar */
static void _hora_compilacion(void) {
  static const char meses[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
  const char *d = __DATE__; /* "Oct  6 2026" */
  const char *t = __TIME__; /* "14:30:00" */
  uint32_t mes = 1U;
  for (uint32_t i = 0U; i < 12U; i++) {
    if (strncmp(d, &meses[3U * i], 3U) == 0) {
      mes = i + 1U;
    }
  }
  s_rmc_anio = (uint16_t)(((uint32_t)(d[7] - '0') * 1000U) +
                          ((uint32_t)(d[8] - '0') * 100U) +
                          ((uint32_t)(d[9] - '0') * 10U) +
                          (uint32_t)(d[10] - '0'));
  s_rmc_mes = (uint8_t)mes;
  s_rmc_dia = (uint8_t)(((d[4] == ' ') ? 0U : ((uint32_t)(d[4] - '0') * 10U)) +
                        (uint32_t)(d[5] - '0'));
  s_rmc_ms_dia = ((((uint32_t)(t[0] - '0') * 10U) + (uint32_t)(t[1] - '0')) *
                      3600000U) +
                 ((((uint32_t)(t[3] - '0') * 10U) + (uint32_t)(t[4] - '0')) *
                  60000U) +
                 ((((uint32_t)(t[6] - '0') * 10U) + (uint32_t)(t[7] - '0')) *
                  1000U);
  s_rmc_tick = 0U;
  s_rmc_ok = true;
  s_rmc_falsa = true;
}
#endif

static char *_2d(char *p, uint32_t v) {
  *p++ = (char)('0' + ((v / 10U) % 10U));
  *p++ = (char)('0' + (v % 10U));
  return p;
}

/* "20261006T143000.0" para el reloj de las cámaras, o NULL si no se sabe */
static const char *_hora(uint32_t ahora) {
  const gps_rmc_data_t *r = &sensor_board_get_data()->gps_rmc;
  if (r->count != s_rmc_visto) {
    s_rmc_visto = r->count;
    if ((r->valid == 1U) && (r->year >= 2024U) && (r->month >= 1U) &&
        (r->month <= 12U) && (r->day >= 1U) && (r->day <= 31U) &&
        (r->time_ms < 86400000U)) {
      s_rmc_anio = r->year;
      s_rmc_mes = r->month;
      s_rmc_dia = r->day;
      s_rmc_ms_dia = r->time_ms;
      s_rmc_tick = ahora;
      s_rmc_ok = true;
      s_rmc_falsa = false;
    }
  }
#if CAMARAS_PRUEBA_UART
  if (!s_rmc_ok) {
    _hora_compilacion();
  }
#endif
  if (!s_rmc_ok || (!s_rmc_falsa && ((ahora - s_rmc_tick) > HORA_VIEJA_MS))) {
    s_hora_ok = false;
    return NULL;
  }
  if (s_hora_ok && ((ahora - s_hora_tick) < 200U)) {
    return s_hora;
  }
  const int64_t ms = (int64_t)s_rmc_ms_dia + (int64_t)(ahora - s_rmc_tick) +
                     ((int64_t)CAMARAS_HUSO_H * 3600000);
  if ((ms < 0) || (ms >= 86400000)) {
    s_hora_ok = false; /* cambio de día: hasta la siguiente trama */
    return NULL;
  }
  const uint32_t seg = (uint32_t)(ms / 1000);
  char *p = s_hora;
  p = _2d(p, s_rmc_anio / 100U);
  p = _2d(p, s_rmc_anio);
  p = _2d(p, s_rmc_mes);
  p = _2d(p, s_rmc_dia);
  *p++ = 'T';
  p = _2d(p, seg / 3600U);
  p = _2d(p, (seg / 60U) % 60U);
  p = _2d(p, seg % 60U);
  *p++ = '.';
  *p++ = '0';
  *p = '\0';
  s_hora_ok = true;
  s_hora_tick = ahora;
  return s_hora;
}

/* === Cámaras ============================================================= */

void camaras_init(void) {
  MX_UART8_Init();
  MX_UART9_Init();
  /* Recepción con pull-up: sin cámara (o sin corriente) la línea no flota
   * ni mete ruido por la interrupción */
  GPIO_InitTypeDef g = {0};
  g.Mode = GPIO_MODE_AF_PP;
  g.Pull = GPIO_PULLUP;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  g.Pin = CAM1_RX_Pin;
  g.Alternate = GPIO_AF8_UART8;
  HAL_GPIO_Init(CAM1_RX_GPIO_Port, &g);
  g.Pin = CAM2_RX_Pin;
  g.Alternate = GPIO_AF11_UART9;
  HAL_GPIO_Init(CAM2_RX_GPIO_Port, &g);

  camara_init(&s_cam[0], 1U, &huart8, UART8_IRQn);
  camara_init(&s_cam[1], 2U, &huart9, UART9_IRQn);

  /* Reiniciada tras un vuelo terminado (emergencia.h): no se encienden
   * solas otra vez (desde tierra sí) */
  if (emergencia_postvuelo()) {
    s_terminado = true;
  }
}

void camaras_uart8_irq(void) { camara_irq(&s_cam[0]); }
void camaras_uart9_irq(void) { camara_irq(&s_cam[1]); }

static uint32_t _firma(const camara_t *c) {
  return (uint32_t)c->estado | (c->no_graba ? 0x100U : 0U) |
         (c->reloj_ok ? 0x200U : 0U) | (c->sd_leida ? 0x400U : 0U) |
         (c->ajustes_leidos ? 0x800U : 0U);
}

/* Un cambio en una cámara: tierra lo ve en el siguiente paquete */
static void _vigilar_camaras(uint32_t ahora) {
  for (uint32_t i = 0U; i < CAMARAS_N; i++) {
    const camara_t *c = &s_cam[i];
    const uint32_t f = _firma(c);
    if (f == s_firma[i]) {
      continue;
    }
    s_firma[i] = f;
    _pedir_paquete();
#if CAMARAS_PRUEBA_UART
    DebugOut_Printf("PRUEBA CAMARAS +%lu ms: camara %u %s%s, %s, SD %lu/%lu "
                    "MB, quedan %lu s, reloj %s (%s), boton %lu\r\n",
                    (unsigned long)(ahora - s_cambio_ms), (unsigned)c->num,
                    camara_estado_nombre(c->estado),
                    c->no_graba ? " NO GRABA" : "",
                    c->ajustes_leidos ? c->resolucion : "?",
                    (unsigned long)c->libre_mb, (unsigned long)c->total_mb,
                    (unsigned long)c->queda_s, c->reloj_ok ? "PUESTO" : "no",
                    c->hora_camara, (unsigned long)c->pulsaciones);
#else
    (void)ahora;
#endif
  }
}

static void _paquete_tic(uint32_t ahora) {
  if (s_encendidas && ((ahora - s_paq_ms) >= CAMARAS_PAQUETE_MS)) {
    s_paq_pendiente = true;
  }
  if (s_paq_pendiente && ((ahora - s_paq_ms) >= CAMARAS_PAQUETE_MIN_MS)) {
    s_paq_pendiente = false;
    s_paq_ms = ahora;
    rf_telemetry_pedir_camaras();
  }
}

#if CAMARAS_PRUEBA_UART
/* Prueba en la mesa: ver el comentario de CAMARAS_PRUEBA_UART */
static void _prueba(uint32_t ahora) {
  static uint32_t paso;
  static uint32_t t0;
  static bool apagando_antes;
  if ((paso == 0U) && (ahora >= 25000U)) {
    paso = 1U;
    t0 = ahora;
    camaras_encender(CAMARAS_ORIGEN_TIERRA);
    DebugOut_Printf("PRUEBA CAMARAS: ENCENDER (hora falsa %s)\r\n",
                    (_hora(ahora) != NULL) ? s_hora : "?");
  }
  const uint32_t t = ahora - t0;
  if (((paso == 1U) && (t >= 40000U)) || ((paso == 3U) && (t >= 100000U))) {
    paso++;
    camaras_apagar();
    DebugOut_Printf("PRUEBA CAMARAS +%lu s: APAGAR (parando primero)\r\n",
                    (unsigned long)(t / 1000U));
  }
  if ((paso == 2U) && (t >= 60000U)) {
    paso = 3U;
    camaras_encender(CAMARAS_ORIGEN_TIERRA);
    DebugOut_Printf("PRUEBA CAMARAS +%lu s: ENCENDER otra vez\r\n",
                    (unsigned long)(t / 1000U));
  }
  if (apagando_antes && !s_apagando) {
    DebugOut_Printf("PRUEBA CAMARAS +%lu s: CORRIENTE CORTADA a los %lu ms "
                    "de pedirlo%s\r\n",
                    (unsigned long)(t / 1000U),
                    (unsigned long)(ahora - s_t_apagar),
                    s_ultimo_por_plazo ? " (POR PLAZO, sin confirmar)" : "");
  }
  apagando_antes = s_apagando;
}
#endif

/* === El vuelo ============================================================ */

static void _vuelo(uint32_t ahora) {
  const gnc_data_t *g = gnc_get_data();
  if (gnc_playback_active()) {
    s_alto = false; /* una simulación no es un vuelo */
    s_gnc_vuelo_prev = false;
    return;
  }
  const bool gnc_vuelo = (g->rocket_state == GNC_ESTADO_EN_VUELO) &&
                         (g->flight_phase != GNC_FASE_EN_TIERRA);
  const bool gnc_flanco = gnc_vuelo && !s_gnc_vuelo_prev;
  s_gnc_vuelo_prev = gnc_vuelo;

  /* Por si el GNC no lo detecta: más de 3 g en una IMU durante 100 ms.
   * Se mira una vez por tick del GNC pero se cuenta TIEMPO, no ticks vistos:
   * si el GNC se pone al día y avanza varios de golpe, contar ticks vistos
   * alargaría los 100 ms (lo cazó la prueba en el PC). */
  if (g->steps != s_steps) {
    s_steps = g->steps;
    const sensor_board_data_t *s = sensor_board_get_data();
    const float a =
        fmaxf(_norma(s->imu[0].accel_mps2), _norma(s->imu[1].accel_mps2));
    if (a >= (CAMARAS_DESPEGUE_G * G0)) {
      if (!s_alto) {
        s_alto = true;
        s_alto_usado = false;
        s_t_alto = ahora;
      }
    } else {
      s_alto = false;
    }
  }
  const bool imu_despegue = s_alto && !s_alto_usado &&
                            ((ahora - s_t_alto) >= CAMARAS_DESPEGUE_MS);
  if (imu_despegue) {
    s_alto_usado = true; /* un tramo de aceleración, un despegue */
  }

  if (s_terminado) {
    return;
  }
  if (!s_en_vuelo) {
    if (gnc_flanco || imu_despegue) {
      s_en_vuelo = true;
      s_apogeo = false;
      s_alt_max = 0.0f;
      s_t_limite = ahora;
      camaras_encender(CAMARAS_ORIGEN_DESPEGUE);
    }
    return;
  }

  /* En vuelo: ¿se ha terminado? */
  if (gnc_flanco) {
    s_t_limite = ahora; /* el GNC lo confirma: de aquí cuentan los 20 min */
  }
  if ((g->flight_phase >= GNC_FASE_APOGEO) &&
      (g->flight_phase <= GNC_FASE_EN_TIERRA)) {
    s_apogeo = true;
  }
  const float alt = g->alt_agl_m;
  if (g->nav_valid && isfinite(alt) && (alt > s_alt_max)) {
    s_alt_max = alt;
  }
  const bool fin =
      (s_apogeo && g->nav_valid && isfinite(alt) && (alt <= FIN_ALTURA_M)) ||
      (g->flight_phase == GNC_FASE_EN_TIERRA) ||
      ((ahora - s_t_limite) >= MAX_VUELO_MS);
  if (fin) {
    _apagar(CAMARAS_ORIGEN_FIN_VUELO); /* parar y luego cortar */
    s_en_vuelo = false;
    /* Si no pasó de 100 m no era un vuelo: quedan armadas para el despegue
     * de verdad. Si lo era, ya no se encienden solas (desde tierra sí). */
    s_terminado = (s_alt_max >= VUELO_REAL_M);
  }
}

void camaras_update(void) {
  const uint32_t ahora = HAL_GetTick();
  const char *hora = _hora(ahora);
  for (uint32_t i = 0U; i < CAMARAS_N; i++) {
    camara_update(&s_cam[i], ahora, hora);
  }
  _vigilar_camaras(ahora);
  _vigilar_apagado(ahora);
#if CAMARAS_PRUEBA_UART
  _prueba(ahora);
#endif
  _vuelo(ahora);
  _paquete_tic(ahora);
}

/* === Paquete y consultas ================================================= */

void camaras_paquete(uint8_t out[CAMARAS_PAQUETE_BYTES]) {
  const uint32_t ahora = HAL_GetTick();
  memset(out, 0, CAMARAS_PAQUETE_BYTES);
  out[0] = CAMARAS_PAQUETE_MARCA;
  out[1] = CAMARAS_PAQUETE_VERSION;
  out[2] = s_paq_contador++;
  out[3] = (uint8_t)((s_encendidas ? 0x01U : 0U) | (s_apagando ? 0x02U : 0U) |
                     (((uint32_t)s_origen & 0x03U) << 2) |
                     (s_en_vuelo ? 0x10U : 0U) | (s_hora_ok ? 0x20U : 0U) |
                     (s_ultimo_por_plazo ? 0x40U : 0U));
  for (uint32_t i = 0U; i < CAMARAS_N; i++) {
    camara_paquete(&s_cam[i], ahora, &out[4U + (CAMARA_PAQUETE_BYTES * i)]);
  }
}

bool camaras_encendidas(void) { return s_encendidas; }
bool camaras_apagando(void) { return s_apagando; }
camaras_origen_t camaras_origen(void) { return s_origen; }
uint32_t camaras_cambio_ms(void) { return s_cambio_ms; }
bool camaras_en_vuelo(void) { return s_en_vuelo; }
bool camaras_hora_gps(void) { return s_hora_ok; }
uint32_t camaras_cortes_por_plazo(void) { return s_cortes_por_plazo; }
const camara_t *camaras_camara(uint32_t i) {
  return &s_cam[(i < CAMARAS_N) ? i : 0U];
}
