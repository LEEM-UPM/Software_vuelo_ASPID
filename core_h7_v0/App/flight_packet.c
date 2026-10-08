#include "flight_packet.h"

#include "emergencia.h"

#include <math.h>
#include <string.h>

#include "airbrake.h"
#include "gnc.h"
#include "sensor_board.h"

/* === Escalas (ver la tabla de flight_packet.h) =========================== */

#define PK_ALT_PER_LSB_M 0.25f
#define PK_POS_PER_LSB_M 1.0f
#define PK_VVEL_PER_LSB_MPS 0.05f
#define PK_HVEL_PER_LSB_MPS 1.0f
#define PK_ACCEL_PER_LSB_G 0.25f
#define PK_APOGEE_PER_LSB_M 25.0f
#define PK_BATTERY_PER_LSB_MV 80.0f

#define PK_G_MPS2 9.80665f

/* El guiado da el aerofreno de 0 a 1; se manda en pasos de 0,4 % */
#define PK_AIRBRAKE_FULL_SCALE 250.0f

/* 5 bits para el codigo de error: el modelo usa 0 a 5 y 255 (error forzado
 * desde tierra), asi que cualquier cosa por encima de 31 se satura ahi. */
#define PK_ERROR_MAX 31U

static uint8_t s_counter;

/* === Conversiones con saturacion ========================================= */

/*
 * Redondeo al entero mas cercano. No se usa lrintf() porque arrastra el modo
 * de redondeo actual de la FPU; aqui interesa que sea siempre el mismo.
 */
static int32_t _round(float value) {
  return (int32_t)((value < 0.0f) ? (value - 0.5f) : (value + 0.5f));
}

/*
 * Un NaN se manda como cero en vez de como basura: si el EKF diverge, el
 * campo queda a 0 y el bit nav_valid dira que no hay que fiarse. Dejar pasar
 * el patron de bits de un NaN daria un valor grande y creible, que es peor.
 */
static int16_t _to_i16(float value, float per_lsb) {
  if (isnan(value)) {
    return 0;
  }
  const int32_t raw = _round(value / per_lsb);
  if (raw > 32767) {
    return 32767;
  }
  if (raw < -32768) {
    return -32768;
  }
  return (int16_t)raw;
}

static int8_t _to_i8(float value, float per_lsb) {
  if (isnan(value)) {
    return 0;
  }
  const int32_t raw = _round(value / per_lsb);
  if (raw > 127) {
    return 127;
  }
  if (raw < -128) {
    return -128;
  }
  return (int8_t)raw;
}

static uint8_t _to_u8(float value, float per_lsb) {
  if (isnan(value) || (value < 0.0f)) {
    return 0U;
  }
  const int32_t raw = _round(value / per_lsb);
  return (raw > 255) ? 255U : (uint8_t)raw;
}

static void _put_i16(uint8_t *dst, int16_t value) {
  const uint16_t raw = (uint16_t)value;
  dst[0] = (uint8_t)(raw & 0xFFU);
  dst[1] = (uint8_t)(raw >> 8);
}

/* === Construccion ======================================================== */

void flight_packet_build(const sys_state *state,
                         uint8_t out[FLIGHT_PACKET_BYTES]) {
  if (out == NULL) {
    return;
  }
  memset(out, 0, FLIGHT_PACKET_BYTES);

  const gnc_data_t *g = gnc_get_data();
  const sensor_board_data_t *sensors = sensor_board_get_data();

  /* --- 0: contador --- */
  out[0] = s_counter++;

  /* --- 1: pirotecnia + fase + validez --- */
  uint8_t pyro = 0U;
  if ((state != NULL) && state->pyro_continuity_valid) {
    pyro = (uint8_t)(state->pyro_continuity & 0x0FU);
  }
  const uint8_t phase =
      (g->flight_phase < 0) ? 0U : (uint8_t)(g->flight_phase & 0x07);
  out[1] = (uint8_t)(pyro | (uint8_t)(phase << 4) |
                     (uint8_t)(g->nav_valid ? 0x80U : 0x00U));

  /* --- 2: estado del cohete + codigo de error --- */
  /* Reiniciado en vuelo: estado 4, EMERGENCIA (emergencia.h) */
  const uint8_t rocket =
      emergencia_activa() ? (uint8_t)EMERGENCIA_ESTADO_PAQUETE
      : (g->rocket_state < 0) ? 0U
                              : (uint8_t)(g->rocket_state & 0x07);
  const uint8_t error =
      (g->error_code > PK_ERROR_MAX) ? PK_ERROR_MAX : g->error_code;
  out[2] = (uint8_t)(rocket | (uint8_t)(error << 3));

  /* --- 3 a 8: posicion. El GNC trabaja en NED (Z hacia ABAJO), asi que la
   * altura es el opuesto de la componente D: por radio se manda hacia arriba
   * positivo, que es como lo lee una persona. --- */
  _put_i16(&out[3], _to_i16(-g->pos_ned_m[2], PK_ALT_PER_LSB_M));
  _put_i16(&out[5], _to_i16(g->pos_ned_m[0], PK_POS_PER_LSB_M));
  _put_i16(&out[7], _to_i16(g->pos_ned_m[1], PK_POS_PER_LSB_M));

  /* --- 9 a 12: velocidad. Misma inversion en la vertical: positivo subiendo. --- */
  _put_i16(&out[9], _to_i16(-g->vel_ned_mps[2], PK_VVEL_PER_LSB_MPS));
  out[11] = (uint8_t)_to_i8(g->vel_ned_mps[0], PK_HVEL_PER_LSB_MPS);
  out[12] = (uint8_t)_to_i8(g->vel_ned_mps[1], PK_HVEL_PER_LSB_MPS);

  /* --- 13 a 15: aceleracion en ejes de cuerpo, en g --- */
  for (uint32_t i = 0U; i < 3U; i++) {
    out[13 + i] = (uint8_t)_to_i8(g->accel_body_mps2[i] / PK_G_MPS2,
                                  PK_ACCEL_PER_LSB_G);
  }

  /* --- 16 a 19: cuaternion, normalizado a +/-127 --- */
  for (uint32_t i = 0U; i < 4U; i++) {
    out[16 + i] = (uint8_t)_to_i8(g->quat_nb[i], 1.0f / 127.0f);
  }

  /* --- 20: apogeo previsto --- */
  out[20] = _to_u8(g->apogee_pred_m, PK_APOGEE_PER_LSB_M);

  /* --- 21: aerofreno, la EXTENSION que de verdad se mando al servo
   * (0..1 -> 0..250). No lo que pide el GNC: fuera de EN VUELO se manda 0
   * aunque el guiado pida otra cosa, y en las pruebas de tierra manda la
   * prueba. Asi tierra ve lo que esta haciendo el aerofreno. --- */
  out[21] = _to_u8(airbrake_last_command() * PK_AIRBRAKE_FULL_SCALE, 1.0f);

  /* --- 22: bateria. El rail de 11 V de la power; ver la nota de la
   * cabecera sobre por que el rango es tan ancho. --- */
  if ((state != NULL) &&
      (state->power_rails.voltage_11v_mv != SYS_POWER_RAILS_INVALID)) {
    out[22] = _to_u8((float)state->power_rails.voltage_11v_mv,
                     PK_BATTERY_PER_LSB_MV);
  }

  /* --- 23: temperatura del barometro 1 --- */
  if ((sensors != NULL) && (sensors->baro[0].count > 0U)) {
    out[23] = (uint8_t)_to_i8(sensors->baro[0].temperature_c, 1.0f);
  }
}

uint8_t flight_packet_next_counter(void) { return s_counter; }

void flight_packet_reset_counter(void) { s_counter = 0U; }
