#include "sys_utilities.h"

#include <string.h>

#include "can_driver.h"
#include "can_protocol/can_ids.h"
#include "can_protocol/can_pack.h"
#include "can_protocol/can_protocol_version.h"
#include "main.h"
#include "sys_avionics.h"
#include "sys_power.h"
#include "sys_rf.h"

/* Wake up: se repite a 10 Hz hasta que respondan todas o pasen 5 s */
#define SYS_WAKE_UP_PERIOD_MS 100U
#define SYS_WAKE_UP_TIMEOUT_MS 5000U

/* Pasado el arranque se sigue reenviando a 1 Hz mientras falte alguna placa */
#define SYS_WAKE_UP_RETRY_PERIOD_MS 1000U

/*
 * Periodo del wake up cuando YA han contestado las tres placas.
 *
 * No es una optimizacion, arregla un agujero: antes el core dejaba de mandar
 * wake ups en cuanto las tres habian contestado una vez, y una placa que se
 * reiniciara despues se quedaba esperando un wake up que no iba a llegar
 * nunca. Pasa de verdad: el 2026-10-01, al regrabar la placa de RF, el core
 * siguio encolando 5527 paquetes con cero errores y diciendo "rf OK" mientras
 * la RF los tiraba todos porque no estaba despierta. En vuelo, una RF que se
 * reinicie por un bajon de tension se llevaria la telemetria para siempre.
 *
 * Mandarlo siempre es barato y se puede. Las tres placas CONTESTAN A TODOS los
 * wake ups, no solo al primero: su ACK es incondicional. Ademas, la avionica
 * reenvia su INIT_STATUS con cada uno (power y rf no, porque todavia no lo
 * implementan). Ninguna reinicia sensores ni recalibra: comprobado leyendo las
 * tres.
 *
 * Coste: 5 tramas cada 2 s (1 wake up + 3 ACK + 1 INIT_STATUS), o sea 2,5
 * tramas/s en un bus de 1 Mbit/s que hoy mueve 2 por segundo. Cuando power y
 * rf implementen INIT_STATUS seran 7 cada 2 s, que sigue siendo nada.
 */
#define SYS_WAKE_UP_KEEPALIVE_PERIOD_MS 2000U

/* Margen tras el wake up para recoger los INIT_STATUS de las placas */
#define SYS_INIT_STATUS_WAIT_MS 300U

static bool _all_awake(const sys_state *state);
static void _send_wake_up(void);
static void _wake_up_retry(sys_state *state);
static void _process_wake_up_ack(sys_state *state, const can_rx_frame_t *frame);
static bool _process_init_status(sys_state *state, const can_rx_frame_t *frame);

/*
 * Despierta al resto de placas por CAN1. Bloquea como máximo
 * SYS_WAKE_UP_TIMEOUT_MS. Al salir, cada campo de state indica si esa placa
 * ha respondido. Llamar después de can_driver_init().
 */
void sys_init(sys_state *state, bool rapido) {
  if (!rapido) {
    HAL_Delay(1000); /* esperar a que el resto de placas arranquen y configuren su CAN */
  }
  const uint32_t plazo = rapido ? 500U : SYS_WAKE_UP_TIMEOUT_MS;
  const uint32_t espera_init = rapido ? 50U : SYS_INIT_STATUS_WAIT_MS;
  state->power = false;
  state->avionics = false;
  state->rf = false;

  state->wake_up_retries = 0U;
  state->power_init.valid = false;
  state->avionics_init.valid = false;
  state->rf_init.valid = false;

  uint32_t start = HAL_GetTick();
  uint32_t last_tx = start - SYS_WAKE_UP_PERIOD_MS; /* el primero sale ya */

  while (!_all_awake(state) && ((HAL_GetTick() - start) < plazo)) {
    if ((HAL_GetTick() - last_tx) >= SYS_WAKE_UP_PERIOD_MS) {
      last_tx += SYS_WAKE_UP_PERIOD_MS;
      /* Si falla (nadie da ACK y la cola TX se llena) se reintenta en el
       * siguiente periodo */
      _send_wake_up();
    }

    can_rx_frame_t frame;
    while (can_driver_receive(CAN_PORT_1, &frame)) {
      /* Las placas mandan su INIT_STATUS nada más responder al wake up, así
       * que llega mientras se sigue esperando a las demás: hay que recogerlo
       * aquí o se pierde. */
      if (!_process_init_status(state, &frame)) {
        _process_wake_up_ack(state, &frame);
      }
    }
  }

  /* Las placas mandan su INIT_STATUS (ICD 6.11) justo después de responder
   * al wake up: se recogen aquí para que el informe de arranque ya los
   * tenga, en vez de aparecer un segundo más tarde. */
  const uint32_t init_status_deadline = HAL_GetTick() + espera_init;
  while ((int32_t)(HAL_GetTick() - init_status_deadline) < 0) {
    can_rx_frame_t frame;
    while (can_driver_receive(CAN_PORT_1, &frame)) {
      sys_process_can1_frame(state, &frame);
    }
  }

  sys_power_init(state);
  sys_avionics_init(state);
  sys_rf_init(state);
}

void sys_update(sys_state *state) {
  _wake_up_retry(state);
  sys_power_update(state);
  sys_avionics_update(state);
}

/*
 * INIT_STATUS (ICD 6.11): con qué componentes ha arrancado cada placa. Lo
 * envían solas tras atender el wake up, así que también llega si el core se
 * reinicia. Devuelve true si la trama era esta.
 */
static bool _process_init_status(sys_state *state,
                                 const can_rx_frame_t *frame) {
  if ((CAN_ID_GET_CLASS(frame->id) != CAN_CLASS_RESPONSE) ||
      (CAN_ID_GET_MSG(frame->id) != CAN_MSG_INIT_STATUS)) {
    return false;
  }
  if (frame->len < 8U) {
    return true; /* es INIT_STATUS, pero está mal formada */
  }

  sys_init_status *status;
  switch (CAN_ID_GET_NODE(frame->id)) {
  case CAN_NODE_POWER:
    status = &state->power_init;
    break;
  case CAN_NODE_AVIONICS:
    status = &state->avionics_init;
    break;
  case CAN_NODE_RF:
    status = &state->rf_init;
    break;
  default:
    return true;
  }

  status->ok = can_get_u32_le(&frame->data[0]);
  status->supported = can_get_u32_le(&frame->data[4]);
  status->rx_ms = HAL_GetTick();
  status->valid = true;
  return true;
}

/*
 * Tramas del bus común. Cada placa trata las suyas en su archivo (sys_power.c,
 * sys_avionics.c); los campos se leen con can_get_*() de can_pack.h.
 */
/*
 * RF_RECEIVED (ICD 6.12): bytes que la placa de RF ha oído por el aire. Aquí
 * no se interpretan; de eso se encarga uplink.c. Devuelve true si la trama
 * era esta.
 */
/* Cabecera de RF_RECEIVED (ICD 6.12): longitud, RSSI, SNR, error de
 * frecuencia (int16) y detrás el paquete */
#define RF_RX_CABECERA 5U

static bool _process_rf_received(sys_state *state,
                                 const can_rx_frame_t *frame) {
  if (frame->id != CAN_ID_RF_RECEIVED) {
    return false;
  }
  if (frame->len == 0U) {
    state->rf_rx_dropped++;
    return true;
  }

  /*
   * Dos formatos mientras dure la transición (2026-10-04): la RF nueva pone
   * la cabecera, la anterior manda el paquete pelado. Se distinguen por el
   * primer byte: en el nuevo es la longitud (como mucho 27), en el anterior
   * el primero del paquete, que en un comando de tierra es 0x5A. Cuando todas
   * las RF lleven el firmware nuevo se puede quitar el formato anterior.
   */
  const uint8_t *datos = frame->data;
  uint32_t n = frame->len;
  bool meta = false;
  if (frame->data[0] < 0x40U) {
    n = frame->data[0];
    if ((n == 0U) || ((n + RF_RX_CABECERA) > frame->len)) {
      state->rf_rx_dropped++;
      return true;
    }
    datos = &frame->data[RF_RX_CABECERA];
    meta = true;
  }
  if (n > sizeof(state->rf_rx_payload)) {
    state->rf_rx_dropped++;
    return true;
  }
  if (state->rf_rx_pending) {
    /* Llegó otro antes de que nadie leyera el anterior. Se pisa, ver la nota
     * del campo en sys_utilities.h. */
    state->rf_rx_dropped++;
  }
  memcpy(state->rf_rx_payload, datos, n);
  state->rf_rx_len = (uint8_t)n;
  state->rf_rx_meta = meta;
  state->rf_rx_rssi_dbm = meta ? (int8_t)frame->data[1] : 0;
  state->rf_rx_snr_db = meta ? (int8_t)frame->data[2] : 0;
  state->rf_rx_df_hz = meta ? can_get_i16_le(&frame->data[3]) : 0;
  state->rf_rx_pending = true;
  state->rf_rx_count++;
  state->rf_rx_ms = HAL_GetTick();
  return true;
}

bool sys_take_rf_received(sys_state *state, uint8_t *buf, uint8_t buf_size,
                          uint8_t *len, sys_rf_rx_meta_t *meta) {
  if ((state == NULL) || !state->rf_rx_pending) {
    return false;
  }
  state->rf_rx_pending = false;
  if ((buf == NULL) || (len == NULL) || (buf_size < state->rf_rx_len)) {
    return false;
  }
  memcpy(buf, state->rf_rx_payload, state->rf_rx_len);
  *len = state->rf_rx_len;
  if (meta != NULL) {
    meta->valida = state->rf_rx_meta;
    meta->rssi_dbm = state->rf_rx_rssi_dbm;
    meta->snr_db = state->rf_rx_snr_db;
    meta->df_hz = state->rf_rx_df_hz;
  }
  return true;
}

void sys_process_can1_frame(sys_state *state, const can_rx_frame_t *frame) {
  if (_process_init_status(state, frame) ||
      _process_rf_received(state, frame) ||
      sys_power_process_frame(state, frame) ||
      sys_avionics_process_frame(state, frame)) {
    return;
  }

  /* Un WAKE_UP_ACK tardío también marca la placa como despierta */
  _process_wake_up_ack(state, frame);
  /* TODO: resto de mensajes según se definan en el ICD */
}

/* CMD_WAKE_UP (0x100) con la versión del protocolo (ICD, 6.1) */
static void _send_wake_up(void) {
  uint8_t wake_up[2];
  can_put_u16_le(&wake_up[0], CAN_PROTOCOL_VERSION);
  (void)can_driver_send(CAN_PORT_1, CAN_ID_CMD_WAKE_UP, wake_up,
                        sizeof(wake_up));
}

/*
 * Manda el wake up PARA SIEMPRE: a 1 Hz mientras falte alguna placa por
 * responder, y luego como latido cada SYS_WAKE_UP_KEEPALIVE_PERIOD_MS.
 *
 * Lo segundo es lo que hace que una placa que se reinicie se recupere sola.
 * Las placas contestan a todos los wake ups, no solo al primero, pero eso no
 * sirve de nada si el core deja de mandarlos. Ver la nota de la constante.
 *
 * Solo se cuentan como reintentos los que se mandan con alguna placa
 * ausente: el latido no es un sintoma de nada y meterlo en el contador lo
 * dejaria sin significado.
 */
static void _wake_up_retry(sys_state *state) {
  static uint32_t last_ms;

  const bool todas = _all_awake(state);
  const uint32_t periodo = todas ? SYS_WAKE_UP_KEEPALIVE_PERIOD_MS
                                 : SYS_WAKE_UP_RETRY_PERIOD_MS;

  const uint32_t now = HAL_GetTick();
  if ((now - last_ms) < periodo) {
    return;
  }
  last_ms = now;

  if (!todas) {
    state->wake_up_retries++;
  }
  _send_wake_up();
}

static bool _all_awake(const sys_state *state) {
  return state->power && state->avionics && state->rf;
}

/* Cualquier otra trama que llegue durante el arranque se descarta */
static void _process_wake_up_ack(sys_state *state, const can_rx_frame_t *frame) {
  if ((CAN_ID_GET_CLASS(frame->id) != CAN_CLASS_RESPONSE) ||
      (CAN_ID_GET_MSG(frame->id) != CAN_MSG_WAKE_UP_ACK)) {
    return;
  }

  switch (CAN_ID_GET_NODE(frame->id)) {
  case CAN_NODE_POWER:
    state->power = true;
    break;
  case CAN_NODE_AVIONICS:
    state->avionics = true;
    break;
  case CAN_NODE_RF:
    state->rf = true;
    break;
  default:
    break;
  }
}
