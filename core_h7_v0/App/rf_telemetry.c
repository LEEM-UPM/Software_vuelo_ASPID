#include "rf_telemetry.h"

#include <string.h>

#include "aspid_data.h"
#include "camaras.h"
#include "emergencia.h"
#include "gnc.h"
#include "gnc_playback.h"
#include "main.h"
#include "status_packet.h"

static bool s_started;
static uint32_t s_arranque_ms; /* primera llamada */
static uint32_t s_start_ms;  /* origen de la planificación */
static uint32_t s_index;     /* cuántos paquetes tocaban ya desde ese origen */

/* Cadencia (ver el .h) */
static uint32_t s_hz = RF_TELEMETRY_HZ_TIERRA;
static bool s_lanzado; /* lanzamiento visto de verdad: 15 Hz para siempre */

static uint32_t s_sent;
static uint32_t s_failed;
static uint8_t s_last[FLIGHT_PACKET_BYTES];

/* Medida de la cadencia real */
static uint32_t s_rate_window_ms;
static uint32_t s_rate_count;
static uint32_t s_rate_hz;

/* Paquete de estado */
static bool s_estado_pendiente;
static uint32_t s_estado_enviados;
static uint32_t s_estado_arranque_restantes = RF_TELEMETRY_ESTADO_ARRANQUE;
static uint32_t s_estado_proximo_ms;
static bool s_arranque_listo;
static uint8_t s_ultimo_estado[STATUS_PACKET_BYTES];

/* Paquete de cámaras */
static bool s_camaras_pendiente;
static uint32_t s_camaras_enviados;
static uint8_t s_camaras[CAMARAS_PAQUETE_BYTES];

/* Acuse pendiente */
static uint8_t s_acuse[16]; /* UPLINK_ACUSE_BYTES (12) con margen */
static uint8_t s_acuse_len;
static uint32_t s_acuses_enviados;

/*
 * Momento teórico del paquete número 'index', contado desde el origen.
 *
 * Se calcula sobre el índice y no acumulando un periodo entero porque a 15 Hz
 * el periodo son 66,67 ms (a 3 Hz, 333,3): sumando 66 ms cada vez se
 * acumularía un error de 10 ms por segundo, y en cinco minutos la cadencia se
 * habría ido un 1 %. Con esta forma la media es exacta y solo baila un
 * milisegundo entre paquetes.
 */
static uint32_t _due_ms(uint32_t index) {
  return s_start_ms + (uint32_t)(((uint64_t)index * 1000ULL) / s_hz);
}

/* La cadencia que toca ahora (ver el .h) */
static uint32_t _cadencia(void) {
  if (s_lanzado) {
    return RF_TELEMETRY_HZ_VUELO;
  }
  const gnc_data_t *g = gnc_get_data();
  const bool gnc_vuelo = (g->rocket_state == GNC_ESTADO_EN_VUELO);
  if (gnc_playback_active()) {
    /* Simulando: como en vuelo mientras dure, sin quedarse en 15 Hz */
    return gnc_vuelo ? RF_TELEMETRY_HZ_VUELO : RF_TELEMETRY_HZ_TIERRA;
  }
  if (gnc_vuelo || camaras_en_vuelo() || emergencia_activa()) {
    s_lanzado = true;
    return RF_TELEMETRY_HZ_VUELO;
  }
  return RF_TELEMETRY_HZ_TIERRA;
}

void rf_telemetry_pedir_estado(void) { s_estado_pendiente = true; }

void rf_telemetry_pedir_camaras(void) { s_camaras_pendiente = true; }

uint32_t rf_telemetry_camaras_enviados(void) { return s_camaras_enviados; }

void rf_telemetry_mandar_acuse(const uint8_t *datos, uint8_t len) {
  if ((datos == NULL) || (len == 0U) || (len > sizeof(s_acuse))) {
    return;
  }
  memcpy(s_acuse, datos, len);
  s_acuse_len = len;
}

uint32_t rf_telemetry_acuses_enviados(void) { return s_acuses_enviados; }

/*
 * Programa los paquetes de estado del arranque.
 *
 * NO se manda nada nada más encender: en ese momento las placas todavía no
 * han contestado al wake up ni han mandado su INIT_STATUS, así que el paquete
 * saldría diciendo que no hay nada montado, que es peor que no mandarlo. Se
 * espera a que contesten las tres o, si alguna no va a contestar, a que pase
 * RF_TELEMETRY_ESTADO_ESPERA_MS, que es justo el caso en el que más falta
 * hace saberlo desde tierra.
 *
 * Y se manda varias veces porque NO HAY ACUSE: un paquete suelto que se
 * pierda deja a tierra sin estado hasta que pida uno. Tres repeticiones
 * cuestan 93 ms de aire en total.
 */
static void _arranque(const sys_state *state, uint32_t now) {
  if (s_estado_arranque_restantes == 0U) {
    return;
  }

  if (!s_arranque_listo) {
    /* Sin una ventana de medida completa, las tramas/s de los dos buses
     * saldrian a cero y pareceria que estan muertos. Ver status_packet_medido. */
    if (!status_packet_medido()) {
      return;
    }
    const bool todas = (state != NULL) && state->power && state->avionics &&
                       state->rf;
    if (!todas && ((now - s_arranque_ms) < RF_TELEMETRY_ESTADO_ESPERA_MS)) {
      return;
    }
    s_arranque_listo = true;
    s_estado_proximo_ms = now;
    s_camaras_pendiente = true; /* uno al arrancar: tierra sabe de ellas */
  }

  if ((int32_t)(now - s_estado_proximo_ms) < 0) {
    return;
  }
  s_estado_pendiente = true;
  s_estado_arranque_restantes--;
  s_estado_proximo_ms = now + RF_TELEMETRY_ESTADO_REPETICION_MS;
}

void rf_telemetry_update(sys_state *state) {
  const uint32_t now = HAL_GetTick();

  if (!s_started) {
    s_started = true;
    s_arranque_ms = now;
    s_start_ms = now;
    s_index = 0U;
    s_rate_window_ms = now;
  }

  _arranque(state, now);

  /* Cambio de cadencia: se replanifica desde ahora, y el primero sale ya (al
   * despegar, el primer paquete a 15 Hz no espera al hueco de los 3 Hz) */
  const uint32_t hz = _cadencia();
  if (hz != s_hz) {
    s_hz = hz;
    s_start_ms = now;
    s_index = 0U;
  }

  uint32_t due = _due_ms(s_index);
  if ((int32_t)(now - due) < 0) {
    return;
  }

  /* Demasiado retraso: se resincroniza en vez de recuperar (ver el .h) */
  if ((now - due) > RF_TELEMETRY_RESYNC_MS) {
    s_start_ms = now;
    s_index = 0U;
    due = now;
  }
  s_index++;

  /*
   * Si hay un paquete de estado pendiente ocupa ESTE hueco, en vez de añadir
   * uno. Así el ciclo de trabajo no cambia ni un milisegundo: se pierde un
   * paquete de vuelo de los de ese segundo y nadie lo echa de menos.
   */
  if (s_acuse_len > 0U) {
    /* Lo primero: tierra esta reintentando hasta verlo */
    const bool ok = sys_rf_transmit(state, s_acuse, s_acuse_len);
    aspid_data_tx(ASPID_TX_ACUSE, s_acuse, s_acuse_len, ok);
    if (ok) {
      s_acuses_enviados++;
    } else {
      s_failed++;
    }
    s_acuse_len = 0U;
  } else if (s_estado_pendiente) {
    s_estado_pendiente = false;
    status_packet_build(state, s_ultimo_estado);
    const bool ok = sys_rf_transmit(state, s_ultimo_estado,
                                    (uint8_t)STATUS_PACKET_BYTES);
    aspid_data_tx(ASPID_TX_ESTADO, s_ultimo_estado,
                  (uint8_t)STATUS_PACKET_BYTES, ok);
    if (ok) {
      s_estado_enviados++;
    } else {
      s_failed++;
    }
  } else if (s_camaras_pendiente) {
    s_camaras_pendiente = false;
    camaras_paquete(s_camaras);
    const bool ok =
        sys_rf_transmit(state, s_camaras, (uint8_t)CAMARAS_PAQUETE_BYTES);
    aspid_data_tx(ASPID_TX_CAMARAS, s_camaras, (uint8_t)CAMARAS_PAQUETE_BYTES,
                  ok);
    if (ok) {
      s_camaras_enviados++;
    } else {
      s_failed++;
    }
  } else {
    flight_packet_build(state, s_last);
    const bool ok = sys_rf_transmit(state, s_last, (uint8_t)FLIGHT_PACKET_BYTES);
    aspid_data_tx(ASPID_TX_VUELO, s_last, (uint8_t)FLIGHT_PACKET_BYTES, ok);
    if (ok) {
      s_sent++;
    } else {
      s_failed++;
    }
  }

  /* Cadencia real: paquetes en la última ventana de un segundo */
  s_rate_count++;
  if ((now - s_rate_window_ms) >= 1000U) {
    s_rate_hz = s_rate_count;
    s_rate_count = 0U;
    s_rate_window_ms = now;
  }
}

uint32_t rf_telemetry_sent(void) { return s_sent; }

uint32_t rf_telemetry_failed(void) { return s_failed; }

const uint8_t *rf_telemetry_last_packet(void) { return s_last; }

uint32_t rf_telemetry_measured_hz(void) { return s_rate_hz; }

uint32_t rf_telemetry_cadencia_hz(void) { return s_hz; }

bool rf_telemetry_lanzamiento_visto(void) { return s_lanzado; }

uint32_t rf_telemetry_estado_enviados(void) { return s_estado_enviados; }

const uint8_t *rf_telemetry_ultimo_estado(void) { return s_ultimo_estado; }
