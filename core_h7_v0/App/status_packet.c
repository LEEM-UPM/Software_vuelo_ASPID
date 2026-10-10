#include "status_packet.h"

#include "baro_alt.h"
#include "emergencia.h"

#include <string.h>

#include "camaras.h"
#include "can_driver.h"
#include "despliegue.h"
#include "gnc.h"
#include "main.h"
#include "storage.h"
#include "gnc_playback.h"
#include "uplink.h"

#define PK_MB_POR_LSB 16U
/*
 * Ventana sobre la que se mide el ritmo de los buses.
 *
 * 4 s y no 1 porque CAN1 va A RAFAGAS: el latido del wake up mete 4 tramas de
 * golpe cada 2 s sobre una base de 2/s, asi que midiendo un segundo sale 2 o
 * 6 segun el segundo que toque, con la media en 4. Un +/-50 % de capricho en
 * un numero que se manda de uvas a peras no sirve para nada. Con 4 s caben dos
 * latidos enteros y el valor sale estable.
 * CAN2 da igual: va a 837 tramas/s constantes y sale bien con cualquier
 * ventana.
 * El precio es que status_packet_medido() tarda 4 s, o sea que el primer
 * paquete de estado del arranque sale 3 s mas tarde. No importa: el arranque
 * ya tarda bastante mas que eso.
 */
#define VENTANA_HZ_MS 4000U

static uint8_t s_counter;

/* Muestreo de actividad de los buses */
static uint32_t s_ventana_ms;
static uint32_t s_rx_previo[2];
static uint32_t s_hz[2];
static bool s_arrancado;
static bool s_medido;

static uint8_t _sat_u8(uint32_t v) { return (v > 255U) ? 255U : (uint8_t)v; }

static uint16_t _sat_u16(uint32_t v) {
  return (v > 65535U) ? 65535U : (uint16_t)v;
}

static void _put_u16(uint8_t *dst, uint16_t v) {
  dst[0] = (uint8_t)(v & 0xFFU);
  dst[1] = (uint8_t)(v >> 8);
}

void status_packet_update(void) {
  const uint32_t ahora = HAL_GetTick();

  if (!s_arrancado) {
    s_arrancado = true;
    s_ventana_ms = ahora;
    for (uint32_t p = 0U; p < 2U; p++) {
      can_driver_stats_t st;
      can_driver_get_stats((can_port_t)p, &st);
      s_rx_previo[p] = st.rx_frames;
    }
    return;
  }

  const uint32_t transcurrido = ahora - s_ventana_ms;
  if (transcurrido < VENTANA_HZ_MS) {
    return;
  }

  for (uint32_t p = 0U; p < 2U; p++) {
    can_driver_stats_t st;
    can_driver_get_stats((can_port_t)p, &st);
    const uint32_t delta = st.rx_frames - s_rx_previo[p];
    s_rx_previo[p] = st.rx_frames;
    /* Se divide por el tiempo real y no por 1000 ms: si el bucle se bloquea
     * (una escritura larga en la SD), la ventana dura más y sin esto el ritmo
     * saldría inflado. */
    s_hz[p] = (delta * 1000U) / transcurrido;
  }
  s_ventana_ms = ahora;
  s_medido = true;
}

bool status_packet_medido(void) { return s_medido; }

uint32_t status_packet_can_hz(uint32_t port) {
  return (port < 2U) ? s_hz[port] : 0U;
}

/* El estado de la tarjeta en dos bits, ordenado de peor a mejor para que el
 * número se pueda leer como "cuánto de bien está". */
static uint8_t _codigo_sd(storage_sd_state_t estado) {
  switch (estado) {
  case STORAGE_SD_INIT_ERROR:
    return 1U;
  case STORAGE_SD_MOUNT_ERROR:
    return 2U;
  case STORAGE_SD_MOUNTED:
    return 3U;
  default:
    return 0U; /* sin iniciar */
  }
}

void status_packet_build(const sys_state *state,
                         uint8_t out[STATUS_PACKET_BYTES]) {
  if (out == NULL) {
    return;
  }
  memset(out, 0, STATUS_PACKET_BYTES);

  out[0] = STATUS_PACKET_MARCA;
  out[1] = STATUS_PACKET_VERSION;
  out[2] = s_counter++;

  const gnc_data_t *g = gnc_get_data();

  /* --- 3: placas y validez --- */
  uint8_t placas = 0U;
  if (state != NULL) {
    placas |= state->power ? 0x01U : 0U;
    placas |= state->avionics ? 0x02U : 0U;
    placas |= state->rf ? 0x04U : 0U;
    placas |= state->avionics_init.valid ? 0x08U : 0U;
    placas |= state->power_init.valid ? 0x10U : 0U;
    placas |= state->rf_init.valid ? 0x20U : 0U;
    placas |= (state->avionics_calibration_status == SYS_CALIBRATION_DONE)
                  ? 0x40U
                  : 0U;
  }
  placas |= (g != NULL && g->nav_valid) ? 0x80U : 0U;
  out[3] = placas;

  /* --- 4 y 5: sensores de aviónica. Los bits del ICD ya caben en un byte,
   * así que se copian tal cual sin reordenar nada. --- */
  if (state != NULL) {
    out[4] = (uint8_t)(state->avionics_init.ok & 0x7FU);
    out[5] = (uint8_t)(state->avionics_init.supported & 0x7FU);
  }

  /* --- 6 a 12: tarjetas --- */
  const storage_sd_info_t *sd1 = storage_get_info(SD_CARD_1);
  const storage_sd_info_t *sd2 = storage_get_info(SD_CARD_2);

  uint8_t tarjetas = 0U;
  if (sd1 != NULL) {
    tarjetas |= _codigo_sd(sd1->state);
    tarjetas |= (sd1->self_test == STORAGE_TEST_OK) ? 0x10U : 0U;
    tarjetas |= sd1->cd_pin_high ? 0x40U : 0U;
    out[7] = _sat_u8(sd1->boot_count);
    _put_u16(&out[9], _sat_u16(sd1->free_mb / PK_MB_POR_LSB));
  }
  if (sd2 != NULL) {
    tarjetas |= (uint8_t)(_codigo_sd(sd2->state) << 2);
    tarjetas |= (sd2->self_test == STORAGE_TEST_OK) ? 0x20U : 0U;
    tarjetas |= sd2->cd_pin_high ? 0x80U : 0U;
    out[8] = _sat_u8(sd2->boot_count);
    _put_u16(&out[11], _sat_u16(sd2->free_mb / PK_MB_POR_LSB));
  }
  out[6] = tarjetas;

  /* --- 13 a 18: buses --- */
  _put_u16(&out[13], _sat_u16(s_hz[0]));
  _put_u16(&out[15], _sat_u16(s_hz[1]));
  for (uint32_t p = 0U; p < 2U; p++) {
    can_driver_stats_t st;
    can_driver_get_stats((can_port_t)p, &st);
    out[17 + p] =
        _sat_u8(st.rx_overflows + st.fifo_msg_lost + st.tx_errors);
  }

  /* --- 19 a 23: cuentas del propio core --- */
  _put_u16(&out[19], _sat_u16(HAL_GetTick() / 1000U));
  if (state != NULL) {
    out[21] = _sat_u8(state->wake_up_retries);
    _put_u16(&out[22], _sat_u16(state->rf_packets_failed));
  }

  /* --- 24: contexto de vuelo, para saber en qué momento se tomó la foto --- */
  if (g != NULL) {
    const uint8_t fase = (g->flight_phase < 0) ? 0U
                                               : (uint8_t)(g->flight_phase & 0x07);
    const uint8_t estado =
        (g->rocket_state < 0) ? 0U : (uint8_t)(g->rocket_state & 0x07);
    const uint8_t f = g->nav_init_flags;
    out[24] = (uint8_t)(fase | (uint8_t)(estado << 3) |
                        (((f & GNC_NAV_INIT_ALINEADO) != 0U) ? 0x40U : 0U) |
                        (((f & GNC_NAV_INIT_BARO) != 0U) ? 0x80U : 0U));
  }

  /* --- 25 a 28: modo test --- */
  out[25] = (uint8_t)((uplink_modo_test() ? 0x01U : 0U) |
                      (uint8_t)((uplink_ultimo_piro() & 0x07U) << 1) |
                      (gnc_playback_active() ? 0x10U : 0U) |
                      (emergencia_postvuelo() ? 0x20U : 0U));
  if (g != NULL) {
    out[25] |= (uint8_t)((((g->nav_init_flags & GNC_NAV_INIT_GNSS) != 0U)
                              ? 0x40U : 0U) |
                         (((g->nav_init_flags & GNC_NAV_INIT_DEGRADADO) != 0U)
                              ? 0x80U : 0U));
  }
  out[26] = _sat_u8(uplink_test_restante_s());
  out[27] = _sat_u8(uplink_piros_aceptados());
  out[28] = _sat_u8(uplink_denegados());

  out[29] = gnc_playback_progress_pct();
  out[30] = (uint8_t)uplink_ultimo_denegado();

  /* --- 31: despliegue --- */
  /* Lo mandado cuenta también lo de antes de un reinicio (la memoria) */
  const bool drogue = despliegue_drogue_mandado() ||
                      (emergencia_activa() && emergencia_drogue_hecho());
  const bool main_ = despliegue_main_mandado() ||
                     (emergencia_activa() && emergencia_main_hecho());
  out[31] = (uint8_t)((drogue ? 0x01U : 0U) | (main_ ? 0x02U : 0U) |
                      ((despliegue_fallos() > 0U) ? 0x04U : 0U) |
                      (camaras_encendidas() ? 0x08U : 0U) |
                      (emergencia_activa() ? 0x10U : 0U) |
                      (emergencia_apogeo_detectado() ? 0x20U : 0U) |
                      (emergencia_main_alcanzado() ? 0x40U : 0U) |
                      ((emergencia_activa() && !baro_alt_fiable()) ? 0x80U
                                                                   : 0U));
}

uint8_t status_packet_next_counter(void) { return s_counter; }
