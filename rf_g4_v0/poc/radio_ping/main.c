/**
 * @file    main.c
 * @brief   Placa RF que lanza la prueba de alcance: manda pings numerados y
 *          saca por consola la calidad del enlace EN LOS DOS SENTIDOS.
 *
 * Esta es la mitad que se queda contigo, con la consola conectada. La otra es
 * poc/radio_echo/main.c, que se lleva lejos y responde a cada ping contando
 * con qué calidad le llegó.
 *
 * Cada línea de la consola tiene las dos direcciones del enlace:
 *
 *   #42  ida: RSSI -97  SNR -8  df +1250 Hz        <- lo que midio el eco
 *        vuelta: RSSI -94  SNR -6  df -980 Hz      <- lo que medimos aqui
 *        ida y vuelta 142 ms
 *
 * Y NO tienen por qué parecerse: cada placa transmite con su potencia y recibe
 * con su antena, su LNA y su suelo de ruido. Un enlace que falla solo en un
 * sentido es información, no un error de medida: dice de qué lado está el
 * problema.
 *
 * Lo que de verdad marca el alcance máximo no es el RSSI sino el SNR y los
 * pings perdidos. A SF7 el demodulador trabaja hasta unos -7,5 dB de SNR; por
 * debajo el CRC empieza a fallar y los paquetes desaparecen. Por eso el
 * resumen cuenta perdidos: el borde de la cobertura es donde empiezan a
 * aparecer, no donde el RSSI llega a un número concreto.
 *
 * El error de frecuencia (df) dice si los cristales se están yendo. A SF7/BW125
 * el receptor aguanta del orden de +/- 30 kHz, pero cuanto más justo va el
 * enlace por distancia, más cuesta cada kHz de desajuste. Si sale un valor
 * grande y constante, retocar xtal_trim en bsp_sx1262.c.
 *
 * CICLO DE TRABAJO. Un ping de ~16 bytes a SF7/BW125 ocupa el aire unos 50 ms.
 * A PING_PERIOD_MS = 1000 sale un 5 %, por debajo del 10 % que permite la
 * banda de 869,4 a 869,65 MHz. Subir la cadencia obliga a rehacer la cuenta, y
 * ademas el eco transmite una vez por cada ping.
 *
 * NO usa el bus CAN.
 *
 * LED (PB4): destello por cada ping enviado; se queda encendido mientras
 * espera un eco que no llega.
 * Consola: LPUART1, 115200 8N1 (TX PB11).
 *
 * Para compilarlo: en CMakeLists.txt, comentar las líneas de app/ y
 * descomentar poc/radio_ping/main.c.
 */
#include <stdio.h>
#include <string.h>

#include "board.h"
#include "bsp_sx1262.h"
#include "gpio.h"
#include "main.h"
#include "sx1262.h"
#include "sx126x.h"
#include "usart.h"

/** Cada cuánto se lanza un ping */
#define PING_PERIOD_MS 1000U

/**
 * Cuánto se espera el eco antes de darlo por perdido. El eco tarda su retardo
 * (20 ms) más el tiempo en el aire de la respuesta (~60 ms); 400 ms deja
 * margen de sobra sin comerse el siguiente ping.
 */
#define ECHO_TIMEOUT_MS 400U

/** Resumen acumulado */
#define STATUS_PERIOD_MS 15000U

/* Contadores, también legibles por SWD */
volatile uint32_t diag_pings_sent;
volatile uint32_t diag_echoes_ok;
volatile uint32_t diag_echoes_lost;
volatile uint32_t diag_crc_errors;
volatile int8_t diag_last_rssi;
volatile int8_t diag_last_snr;
volatile int32_t diag_last_freq_error;
volatile uint32_t diag_last_rtt_ms;

int __io_putchar(int ch) {
  HAL_UART_Transmit(&hlpuart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

static void print_device_errors(void) {
  uint16_t errors = 0;
  if (!sx1262_get_device_errors(&errors) || (errors == 0U)) {
    return;
  }
  printf("  GetDeviceErrors = 0x%04X%s%s%s\r\n", errors,
         (errors & SX126X_ERRORS_XOSC_START) ? " XOSC_START(cristal)" : "",
         (errors & SX126X_ERRORS_PLL_LOCK) ? " PLL_LOCK" : "",
         (errors & SX126X_ERRORS_PA_RAMP) ? " PA_RAMP" : "");
}

/*
 * Saca del texto del eco el valor que sigue a una letra: "R-97" con 'R' da
 * -97. Devuelve false si esa letra no está, que es lo que pasa si el paquete
 * no viene del eco (otro cacharro en la banda) o llegó troceado.
 */
static bool parse_field(const char *text, char key, long *out) {
  for (const char *p = text; *p != '\0'; p++) {
    if ((*p != key) || ((p != text) && (p[-1] != ' '))) {
      continue;
    }
    const char *v = p + 1;
    if ((*v != '-') && (*v != '+') && ((*v < '0') || (*v > '9'))) {
      continue;
    }
    long sign = 1;
    if (*v == '-') {
      sign = -1;
      v++;
    } else if (*v == '+') {
      v++;
    }
    long value = 0;
    if ((*v < '0') || (*v > '9')) {
      continue;
    }
    while ((*v >= '0') && (*v <= '9')) {
      value = (value * 10) + (*v - '0');
      v++;
    }
    *out = sign * value;
    return true;
  }
  return false;
}

int main(void) {
  board_init();

  printf("\r\n=== Placa RF: PING para prueba de alcance ===\r\n");
  printf("Manda un ping cada %u ms y espera el eco\r\n", PING_PERIOD_MS);

  const sx1262_config_t *config = bsp_sx1262_get_config();
  if (!sx1262_init(config)) {
    printf("ERROR: configuracion del BSP incompleta\r\n");
    Error_Handler();
  }

  sx1262_result_t result = sx1262_configure();
  if (result != SX1262_OK) {
    printf("ERROR: radio no configurada, paso %u (error %d)\r\n",
           sx1262_get_failed_step(), (int)result);
    print_device_errors();
    Error_Handler();
  }

  printf("A %lu Hz, SF%u, %d dBm\r\n",
         (unsigned long)config->radio.frequency_hz, (unsigned)config->radio.sf,
         (int)config->radio.tx_power_dbm);
  printf("Perdidos y SNR son lo que marca el alcance, no el RSSI\r\n\r\n");

  while (1) {
    /* --- Ping --- */
    char ping[24];
    const int n = snprintf(ping, sizeof(ping), "PING %lu",
                           (unsigned long)(diag_pings_sent + 1U));

    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    const uint32_t t_sent = HAL_GetTick();
    const sx1262_result_t tx =
        sx1262_transmit((const uint8_t *)ping, (uint8_t)n, NULL);
    if (tx != SX1262_OK) {
      printf("#%lu  ERROR al transmitir (%d)\r\n",
             (unsigned long)(diag_pings_sent + 1U), (int)tx);
      print_device_errors();
      HAL_Delay(PING_PERIOD_MS);
      continue;
    }
    diag_pings_sent++;

    /* --- Escuchar el eco --- */
    if (sx1262_start_rx() != SX1262_OK) {
      printf("ERROR: no se pudo volver a recepcion\r\n");
      continue;
    }

    bool got_echo = false;
    while ((HAL_GetTick() - t_sent) < ECHO_TIMEOUT_MS) {
      uint8_t buf[SX1262_PAYLOAD_MAX_LEN];
      uint8_t length = 0U;
      sx1262_pkt_status_t status = {0};

      const sx1262_rx_event_t event = sx1262_poll_rx(buf, &length, &status);
      if (event == SX1262_RX_CRC_ERROR) {
        diag_crc_errors++;
        continue;
      }
      if (event != SX1262_RX_PACKET) {
        continue;
      }

      const uint32_t rtt = HAL_GetTick() - t_sent;
      buf[(length < (SX1262_PAYLOAD_MAX_LEN - 1U))
              ? length
              : (SX1262_PAYLOAD_MAX_LEN - 1U)] = '\0';

      long far_rssi = 0, far_snr = 0, far_df = 0;
      const bool understood =
          parse_field((const char *)buf, 'R', &far_rssi) &&
          parse_field((const char *)buf, 'S', &far_snr) &&
          parse_field((const char *)buf, 'F', &far_df);

      diag_echoes_ok++;
      diag_last_rssi = status.rssi_dbm;
      diag_last_snr = status.snr_db;
      diag_last_freq_error = status.freq_error_hz;
      diag_last_rtt_ms = rtt;
      got_echo = true;

      if (understood) {
        printf("#%lu  ida: RSSI %ld  SNR %ld  df %+ld Hz    "
               "vuelta: RSSI %d  SNR %d  df %+ld Hz    %lu ms\r\n",
               (unsigned long)diag_pings_sent, far_rssi, far_snr, far_df,
               (int)status.rssi_dbm, (int)status.snr_db,
               (long)status.freq_error_hz, (unsigned long)rtt);
      } else {
        /* Algo ha contestado, pero no con el formato del eco */
        printf("#%lu  respuesta no reconocida: \"%s\"   "
               "vuelta: RSSI %d  SNR %d   %lu ms\r\n",
               (unsigned long)diag_pings_sent, (const char *)buf,
               (int)status.rssi_dbm, (int)status.snr_db, (unsigned long)rtt);
      }
      break;
    }

    if (!got_echo) {
      diag_echoes_lost++;
      printf("#%lu  sin eco\r\n", (unsigned long)diag_pings_sent);
    }
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

    if ((diag_pings_sent % (STATUS_PERIOD_MS / PING_PERIOD_MS)) == 0U) {
      const unsigned long total = (unsigned long)diag_pings_sent;
      printf("-- %lu pings, %lu ecos, %lu perdidos (%lu %%), %lu CRC malo\r\n",
             total, (unsigned long)diag_echoes_ok,
             (unsigned long)diag_echoes_lost,
             (total > 0U) ? ((unsigned long)diag_echoes_lost * 100UL / total)
                          : 0UL,
             (unsigned long)diag_crc_errors);
    }

    /* Lo que falte hasta el siguiente ping */
    const uint32_t elapsed = HAL_GetTick() - t_sent;
    if (elapsed < PING_PERIOD_MS) {
      HAL_Delay(PING_PERIOD_MS - elapsed);
    }
  }
}
