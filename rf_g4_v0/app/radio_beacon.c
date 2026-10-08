/**
 * @file    radio_beacon.c
 * @brief   Periodic test packet over the SX1262 (LoRa).
 */
#include "radio_beacon.h"

#include <stdio.h>
#include <string.h>

#include "bsp_sx1262.h"
#include "main.h"
#include "sx1262.h"

/* De momento un saludo, para ver el enlace de punta a punta */
#define RADIO_BEACON_TEXT "hola"
#define RADIO_BEACON_PAYLOAD_LEN (sizeof(RADIO_BEACON_TEXT) - 1U)

static bool s_ready = false;
static uint32_t s_seq = 0;
static uint32_t s_last_tx_ms = 0;

static uint32_t s_ok_count = 0;
static uint32_t s_fail_count = 0;

/* Escucha (uplink) */
static bool s_listening = false;
static uint32_t s_rx_count = 0;
static uint32_t s_rx_crc_errors = 0;

/* Reads, clears and prints GetDeviceErrors: the direct way to know why the
 * chip is stuck. */
static void print_device_errors(void) {
  uint16_t errors = 0;
  if (!sx1262_get_device_errors(&errors)) {
    printf("  GetDeviceErrors: sin respuesta del chip\r\n");
    return;
  }
  if (errors == 0U) {
    return;
  }

  printf("  GetDeviceErrors = 0x%04X%s%s%s\r\n", errors,
         (errors & SX126X_ERRORS_XOSC_START) ? " XOSC_START(cristal)" : "",
         (errors & SX126X_ERRORS_PLL_LOCK) ? " PLL_LOCK" : "",
         (errors & SX126X_ERRORS_PA_RAMP) ? " PA_RAMP(VREG pin 7 ~1,55 V)" : "");
}

bool radio_beacon_init(void) {
  if (!sx1262_init(bsp_sx1262_get_config())) {
    printf("RADIO: configuracion del BSP incompleta\r\n");
    return false;
  }

  sx1262_result_t result = sx1262_configure();
  if (result != SX1262_OK) {
    printf("RADIO: configuracion fallida en el paso %u (error %d)\r\n",
           sx1262_get_failed_step(), (int)result);
    print_device_errors();
    return false;
  }

  printf("RADIO: OK, paquete de %u bytes cada %lu ms\r\n",
         RADIO_BEACON_PAYLOAD_LEN, (unsigned long)RADIO_BEACON_PERIOD_MS);

  s_ready = true;
  /* The first packet goes out on the first call */
  s_last_tx_ms = HAL_GetTick() - RADIO_BEACON_PERIOD_MS;
  return true;
}

/* Transmision comun: la baliza y los paquetes que manda el core (0x140) */
static bool transmit(const char *tag, const uint8_t *payload, uint8_t length) {
  if (!s_ready) {
    return false;
  }

  uint32_t tx_ms = 0;
  sx1262_result_t result = sx1262_transmit(payload, length, &tx_ms);

  bool ok = (result == SX1262_OK);
  if (ok) {
    s_ok_count++;
    /*
     * One line per packet USED to be printed here. It cannot stay now that
     * the board also listens: at 15 Hz that is 15 blocking
     * HAL_UART_Transmit() calls per second (~5 ms each at 115200), and while
     * the CPU sits inside one of those it is not polling the radio, so
     * uplink packets get missed. Now it is a summary every
     * RADIO_TX_LOG_EVERY packets.
     */
    if ((s_ok_count % RADIO_TX_LOG_EVERY) == 0U) {
      printf("TX: %lu ok / %lu fallos, ultimo %s de %u B en %lu ms\r\n",
             (unsigned long)s_ok_count, (unsigned long)s_fail_count, tag,
             length, (unsigned long)tx_ms);
    }
  } else {
    s_fail_count++;
    printf("TX %s: ERROR %d tras %lu ms (%lu ok / %lu fallos)\r\n", tag,
           (int)result, (unsigned long)tx_ms, (unsigned long)s_ok_count,
           (unsigned long)s_fail_count);
    print_device_errors();
  }

  /*
   * sx1262_transmit() leaves the chip in standby. Without this the radio
   * would go deaf for good after the first packet, instead of only for the
   * ~31 ms the transmission itself lasts.
   */
  if (s_listening) {
    (void)sx1262_start_rx();
  }
  return ok;
}

bool radio_listen(void) {
  if (!s_ready) {
    return false;
  }
  if (sx1262_start_rx() != SX1262_OK) {
    printf("RADIO: no se ha podido entrar en recepcion\r\n");
    return false;
  }
  s_listening = true;
  return true;
}

bool radio_receive_poll(uint8_t *buf, uint8_t buf_size, uint8_t *out_len,
                        sx1262_pkt_status_t *out_status) {
  if (!s_listening || buf == NULL || out_len == NULL) {
    return false;
  }

  uint8_t payload[SX1262_PAYLOAD_MAX_LEN];
  uint8_t length = 0U;
  const sx1262_rx_event_t event =
      sx1262_poll_rx(payload, &length, out_status);

  if (event == SX1262_RX_CRC_ERROR) {
    /* Counted but NOT passed on: those bytes are rubbish, and handing them to
     * the core would only give it work to end up throwing them away. */
    s_rx_crc_errors++;
    return false;
  }
  if ((event != SX1262_RX_PACKET) || (length == 0U) || (length > buf_size)) {
    return false;
  }

  s_rx_count++;
  memcpy(buf, payload, length);
  *out_len = length;
  return true;
}

uint32_t radio_rx_count(void) { return s_rx_count; }

uint32_t radio_rx_crc_errors(void) { return s_rx_crc_errors; }

bool radio_transmit(const uint8_t *payload, uint8_t length) {
  if (payload == NULL || length == 0U) {
    return false;
  }
  return transmit("CAN", payload, length);
}

bool radio_beacon_update(void) {
  if (!s_ready || ((HAL_GetTick() - s_last_tx_ms) < RADIO_BEACON_PERIOD_MS)) {
    return false;
  }
  s_last_tx_ms += RADIO_BEACON_PERIOD_MS;

  uint8_t payload[RADIO_BEACON_PAYLOAD_LEN];
  memcpy(payload, RADIO_BEACON_TEXT, RADIO_BEACON_PAYLOAD_LEN);

  char tag[20];
  snprintf(tag, sizeof(tag), "baliza #%lu", (unsigned long)s_seq);
  bool ok = transmit(tag, payload, RADIO_BEACON_PAYLOAD_LEN);

  s_seq++;
  return ok;
}

uint32_t radio_beacon_ok_count(void) { return s_ok_count; }

uint32_t radio_beacon_fail_count(void) { return s_fail_count; }
