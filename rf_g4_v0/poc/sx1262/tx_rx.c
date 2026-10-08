/**
 * @file    tx_rx.c
 * @brief   SX1262 PoC: periodic TX with on-air time check, or continuous RX.
 *
 * Select the mode with POC_MODE_TX / POC_MODE_RX. Both ends use the radio
 * configuration in bsp/sx1262/bsp_sx1262.c (868.0 MHz, SF7, BW125, CR4/5,
 * sync word 0x34).
 *
 * How to tell it really transmits: TX_DONE must arrive after the expected
 * time on air (41.2 ms for the 12-byte payload used here). If it arrives in
 * ~0 ms the chip reports "done" without having transmitted anything.
 *
 * Console: LPUART1, 115200 8N1 (TX PB11). The same counters are kept in
 * diag_* globals for the debugger.
 */
#include "board.h"
#include "bsp_sx1262.h"
#include "gpio.h"
#include "main.h"
#include "sx1262.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

#define POC_MODE_TX
// #define POC_MODE_RX

/** 4-byte sequence number + 8 characters = 12 bytes (41.2 ms on air). */
#define TX_PAYLOAD_LEN 12U

/** 868.0-868.6 MHz has a 1 % duty cycle limit: 41.2 ms on air needs a
 *  period of at least 4.1 s. */
#define TX_PERIOD_MS 5000U

/* Debugger-visible diagnostics */
volatile uint32_t diag_tx_ok = 0;
volatile uint32_t diag_tx_fail = 0;
volatile uint32_t diag_last_tx_ms = 0;
volatile uint32_t diag_rx_packets = 0;
volatile uint32_t diag_rx_crc_errors = 0;
volatile uint16_t diag_dev_errors = 0;

int __io_putchar(int ch) {
  HAL_UART_Transmit(&hlpuart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

static const char *result_name(sx1262_result_t result) {
  switch (result) {
  case SX1262_OK: return "OK";
  case SX1262_ERR_PARAM: return "parametro no valido";
  case SX1262_ERR_CHIP: return "SPI o BUSY atascado en alto";
  case SX1262_ERR_TX_TIMEOUT: return "TIMEOUT del chip (3 s)";
  case SX1262_ERR_NO_IRQ: return "ni TX_DONE ni TIMEOUT";
  default: return "?";
  }
}

/**
 * Reads, clears and decodes GetDeviceErrors: the direct way to know why the
 * chip is stuck instead of guessing.
 */
static void print_device_errors(void) {
  uint16_t errors = 0;
  if (!sx1262_get_device_errors(&errors)) {
    printf("  GetDeviceErrors: sin respuesta del chip\r\n");
    return;
  }
  diag_dev_errors = errors;

  printf("  GetDeviceErrors = 0x%04X\r\n", errors);
  if (errors == 0U) {
    return;
  }
  if (errors & SX126X_ERRORS_RC64K_CALIBRATION) printf("    RC64K_CALIBRATION\r\n");
  if (errors & SX126X_ERRORS_RC13M_CALIBRATION) printf("    RC13M_CALIBRATION\r\n");
  if (errors & SX126X_ERRORS_PLL_CALIBRATION) printf("    PLL_CALIBRATION\r\n");
  if (errors & SX126X_ERRORS_ADC_CALIBRATION) printf("    ADC_CALIBRATION\r\n");
  if (errors & SX126X_ERRORS_IMG_CALIBRATION) printf("    IMG_CALIBRATION\r\n");
  if (errors & SX126X_ERRORS_XOSC_START) printf("    XOSC_START -> revisar el cristal de 32 MHz\r\n");
  if (errors & SX126X_ERRORS_PLL_LOCK) printf("    PLL_LOCK -> sintetizador\r\n");
  if (errors & SX126X_ERRORS_PA_RAMP) printf("    PA_RAMP -> alimentacion del PA (VREG, pin 7: ~1,55 V)\r\n");
}

static void radio_start(void) {
  if (!sx1262_init(bsp_sx1262_get_config())) {
    printf("ERROR: configuracion del BSP incompleta\r\n");
    while (1) {
    }
  }

  sx1262_result_t result = sx1262_configure();
  if (result != SX1262_OK) {
    printf("ERROR: configuracion fallida en el paso %u: %s\r\n",
           sx1262_get_failed_step(), result_name(result));
    print_device_errors();
    while (1) {
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      HAL_Delay(100);
    }
  }

  sx1262_chip_status_t status;
  if (sx1262_chip_status(&status)) {
    printf("Chip OK: mode=0x%02X cmd=0x%02X\r\n", status.chip_mode,
           status.cmd_status);
  }
  print_device_errors();
}

#ifdef POC_MODE_TX
static void run_tx(void) {
  const uint32_t expected_us = sx1262_time_on_air_us(TX_PAYLOAD_LEN);
  printf("Modo: TRANSMISOR, %u bytes, tiempo en aire esperado %lu.%lu ms\r\n\r\n",
         TX_PAYLOAD_LEN, (unsigned long)(expected_us / 1000U),
         (unsigned long)((expected_us % 1000U) / 100U));

  uint32_t seq = 0;
  uint8_t payload[TX_PAYLOAD_LEN];

  while (1) {
    payload[0] = (uint8_t)(seq >> 24);
    payload[1] = (uint8_t)(seq >> 16);
    payload[2] = (uint8_t)(seq >> 8);
    payload[3] = (uint8_t)(seq);
    memcpy(&payload[4], "LEEM-RF!", 8);

    uint32_t tx_ms = 0;
    sx1262_result_t result = sx1262_transmit(payload, TX_PAYLOAD_LEN, &tx_ms);
    diag_last_tx_ms = tx_ms;

    printf("TX #%lu:\r\n", (unsigned long)seq);
    if (result == SX1262_OK) {
      diag_tx_ok++;
      printf("  TX_DONE en %lu ms\r\n", (unsigned long)tx_ms);
      if ((tx_ms * 1000U) < (expected_us / 2U)) {
        printf("  AVISO: muy por debajo del tiempo en aire, no ha transmitido de verdad\r\n");
      }
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    } else {
      diag_tx_fail++;
      printf("  ERROR tras %lu ms: %s\r\n", (unsigned long)tx_ms,
             result_name(result));
      print_device_errors();
    }
    printf("  total: %lu ok / %lu fallos\r\n", (unsigned long)diag_tx_ok,
           (unsigned long)diag_tx_fail);

    seq++;
    HAL_Delay(TX_PERIOD_MS);
  }
}
#endif

#ifdef POC_MODE_RX
static void run_rx(void) {
  printf("Modo: RECEPTOR (RX continuo)\r\n\r\n");

  if (sx1262_start_rx() != SX1262_OK) {
    printf("ERROR: no se ha podido entrar en RX\r\n");
    print_device_errors();
  }

  uint8_t buf[SX1262_PAYLOAD_MAX_LEN];
  uint8_t len = 0;
  sx1262_pkt_status_t pkt;

  while (1) {
    switch (sx1262_poll_rx(buf, &len, &pkt)) {
    case SX1262_RX_PACKET: {
      diag_rx_packets++;
      uint32_t seq = 0;
      if (len >= 4U) {
        seq = ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
              ((uint32_t)buf[2] << 8) | (uint32_t)buf[3];
      }
      printf("RX #%lu: len=%u seq=%lu rssi=%d dBm signal_rssi=%d dBm snr=%d dB texto=\"",
             (unsigned long)diag_rx_packets, len, (unsigned long)seq,
             pkt.rssi_dbm, pkt.signal_rssi_dbm, pkt.snr_db);
      for (uint8_t i = 4; i < len; i++) {
        printf("%c", (buf[i] >= 32 && buf[i] < 127) ? buf[i] : '.');
      }
      printf("\"\r\n");
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      break;
    }

    case SX1262_RX_CRC_ERROR:
      diag_rx_crc_errors++;
      printf("RX: paquete descartado por CRC (%lu)\r\n",
             (unsigned long)diag_rx_crc_errors);
      break;

    case SX1262_RX_ERROR:
      printf("RX: error de SPI/BUSY, se reinicia RX\r\n");
      print_device_errors();
      (void)sx1262_start_rx();
      break;

    case SX1262_RX_NONE:
    default:
      break;
    }

    HAL_Delay(1); /* each poll is an SPI GetIrqStatus */
  }
}
#endif

int main(void) {
  board_init();

  printf("\r\n=== SX1262 PoC ===\r\n");
  radio_start();

#if defined(POC_MODE_TX)
  run_tx();
#elif defined(POC_MODE_RX)
  run_rx();
#else
#error "Define POC_MODE_TX o POC_MODE_RX"
#endif
}
