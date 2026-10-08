/**
 * @file    main.c
 * @brief   Placa RF como ECO para medir alcance: responde a cada paquete que
 *          recibe contando con qué calidad le llegó.
 *
 * Esta es la mitad que se lleva lejos (la del cohete). La otra mitad es
 * poc/radio_ping/main.c, que se queda con la consola conectada.
 *
 * POR QUÉ UN ECO. Midiendo desde un solo extremo solo se conoce un sentido
 * del enlace, y no tienen por qué ser iguales: cada placa transmite con su
 * potencia y recibe con su antena, su LNA y su suelo de ruido. Con el eco,
 * quien lanza la prueba ve las DOS direcciones a la vez sin necesitar a nadie
 * al otro lado leyendo una pantalla: la calidad de ida se la cuenta el eco
 * dentro del mensaje, y la de vuelta la mide él mismo al recibirlo.
 *
 * QUÉ DEVUELVE. Un texto corto en ASCII, para que se lea sin decodificar:
 *
 *     E12 R-97 S-8 G-99 F-1250 L16
 *
 *     E   número de eco (cuántos ha respondido desde que arrancó)
 *     R   RSSI del paquete recibido [dBm]
 *     S   relación señal/ruido [dB]
 *     G   RSSI después de "despreading" [dBm]
 *     F   error de frecuencia respecto al emisor [Hz], con signo
 *     L   longitud del paquete que llegó [bytes]
 *
 * El error de frecuencia es el que dice si los dos cristales se están yendo:
 * a SF7/BW125 el receptor aguanta del orden de +/- 30 kHz, así que si sale de
 * unos pocos kHz conviene retocar xtal_trim en bsp_sx1262.c. Con el enlace ya
 * justo por distancia, ese desajuste se nota mucho más.
 *
 * CICLO DE TRABAJO. La banda de 869,4 a 869,65 MHz permite el 10 %. Un eco de
 * unos 25 bytes a SF7/BW125 ocupa el aire ~60 ms, así que respondiendo a un
 * ping por segundo se va al 6 %. Si se sube la cadencia de los pings hay que
 * rehacer la cuenta: el eco transmite tantas veces como se le pregunte.
 *
 * NO usa el bus CAN. La configuración de radio sale del BSP
 * (bsp/sx1262/bsp_sx1262.c), así que las dos placas comparten frecuencia, SF,
 * ancho de banda y sync word sin poder descuadrarse.
 *
 * LED (PB4): parpadeo lento esperando; destello por cada eco enviado.
 * Consola: LPUART1, 115200 8N1 (TX PB11). Útil en el banco, pero la prueba
 * está pensada para que esta placa vaya sola, alimentada por batería.
 *
 * Para compilarlo: en CMakeLists.txt, comentar las líneas de app/ y
 * descomentar poc/radio_echo/main.c.
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

/** Resumen periódico por consola */
#define STATUS_PERIOD_MS 10000U

/** Parpadeo mientras espera */
#define LED_WAITING_HALF_PERIOD_MS 500U

/** Destello al responder */
#define LED_ECHO_FLASH_MS 60U

/*
 * PRUEBA A/B DEL AMPLIFICADOR DE RECEPCION.
 *
 * La placa lleva un LNA externo delante del SX1262 que el BSP enciende al
 * entrar en recepcion (board_rf_switch_set_rx). A 1 esta prueba lo alterna
 * DESPUES de cada paquete, de modo que los pares y los impares se reciben en
 * condiciones opuestas, y lo dice en la respuesta con "A1" o "A0".
 *
 * Se alterna paquete a paquete, y no por tandas, a proposito: asi las dos
 * condiciones se miden intercaladas en el tiempo y a la misma distancia, y
 * cualquier deriva (movimiento, temperatura, multitrayecto) afecta por igual
 * a las dos y se cancela al comparar.
 *
 * Poner a 0 para dejar el LNA siempre encendido, que es lo normal.
 */
#define LNA_AB_TEST 0

/*
 * A 0 el eco NO responde: solo escucha y cuenta. Es el modo para medir la
 * recepcion con el LNA forzado a 5 V por hardware (R16 levantada), donde
 * transmitir seria arriesgado: con el amplificador activo, lo que se cuela
 * del PA por el aislamiento del conmutador de antena llegaria amplificado
 * +21 dB a la entrada de recepcion del SX1262, que admite del orden de
 * +10 dBm. Hacen falta 33 dB de aislamiento para estar a salvo, y eso hay
 * que comprobarlo en la hoja del 4259-63 antes de volver a poner esto a 1.
 */
#define ECHO_REPLY_ENABLE 1

/*
 * Espera antes de responder. El otro extremo necesita volver de transmisión a
 * recepción, y si el eco sale demasiado pronto se lo pierde. 20 ms sobra para
 * el cambio y no estorba al ciclo de trabajo.
 */
#define ECHO_DELAY_MS 20U

/* Contadores, también legibles por SWD cuando no hay consola */
volatile uint32_t diag_rx_packets;
volatile uint32_t diag_rx_crc_errors;
volatile uint32_t diag_echoes_sent;
volatile uint32_t diag_echoes_failed;
volatile int8_t diag_last_rssi;
volatile int8_t diag_last_snr;
volatile int8_t diag_last_signal_rssi;
volatile int32_t diag_last_freq_error;
volatile uint8_t diag_last_len;
volatile uint8_t diag_last_payload[16];
volatile uint8_t diag_chip_mode;
/* Cuentas separadas por estado del LNA: [0] apagado, [1] encendido */
volatile uint32_t diag_lna_packets[2];
volatile int32_t diag_lna_rssi_sum[2];
volatile int32_t diag_lna_snr_sum[2];
static bool s_lna_on = true;

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

int main(void) {
  board_init();

  printf("\r\n=== Placa RF: ECO para prueba de alcance ===\r\n");
  printf("Responde a cada paquete con RSSI, SNR y error de frecuencia\r\n");

  const sx1262_config_t *config = bsp_sx1262_get_config();
  if (!sx1262_init(config)) {
    printf("ERROR: configuracion del BSP incompleta\r\n");
    Error_Handler();
  }

  sx1262_chip_status_t chip;
  if (sx1262_chip_status(&chip)) {
    printf("SX1262 presente: chip_mode 0x%02X\r\n", chip.chip_mode);
  }

  sx1262_result_t result = sx1262_configure();
  if (result != SX1262_OK) {
    printf("ERROR: radio no configurada, paso %u (error %d)\r\n",
           sx1262_get_failed_step(), (int)result);
    print_device_errors();
    Error_Handler();
  }

  result = sx1262_start_rx();
  if (result != SX1262_OK) {
    printf("ERROR: no se ha podido entrar en recepcion (error %d)\r\n",
           (int)result);
    print_device_errors();
    Error_Handler();
  }

  printf("Escuchando a %lu Hz, SF%u, %u dBm de respuesta\r\n",
         (unsigned long)config->radio.frequency_hz, (unsigned)config->radio.sf,
         (unsigned)config->radio.tx_power_dbm);

  uint32_t last_status = HAL_GetTick();
  uint32_t last_blink = HAL_GetTick();
  uint32_t flash_until = 0U;
  bool led_on = false;

  while (1) {
    uint8_t payload[SX1262_PAYLOAD_MAX_LEN];
    uint8_t length = 0U;
    sx1262_pkt_status_t status = {0};

    const sx1262_rx_event_t event = sx1262_poll_rx(payload, &length, &status);

    if (event == SX1262_RX_PACKET) {
      diag_rx_packets++;
      diag_last_rssi = status.rssi_dbm;
      diag_last_snr = status.snr_db;
      diag_last_signal_rssi = status.signal_rssi_dbm;
      diag_last_freq_error = status.freq_error_hz;
      diag_last_len = length;
      memcpy((void *)diag_last_payload, payload,
             (length < sizeof(diag_last_payload))
                 ? length
                 : sizeof(diag_last_payload));

      /*
       * La respuesta se construye ANTES de esperar, para que el retardo sea
       * lo único que separa la recepción de la transmisión.
       */
      char reply[48];
      const int n =
          snprintf(reply, sizeof(reply), "E%lu R%d S%d G%d F%ld L%u A%u",
                   (unsigned long)(diag_echoes_sent + 1U),
                   (int)status.rssi_dbm, (int)status.snr_db,
                   (int)status.signal_rssi_dbm, (long)status.freq_error_hz,
                   (unsigned)length, (unsigned)(s_lna_on ? 1U : 0U));

      /* Se apunta ANTES de conmutar: este paquete llego con s_lna_on */
      const uint32_t slot = s_lna_on ? 1U : 0U;
      diag_lna_packets[slot]++;
      diag_lna_rssi_sum[slot] += (int32_t)status.rssi_dbm;
      diag_lna_snr_sum[slot] += (int32_t)status.snr_db;

#if ECHO_REPLY_ENABLE
      HAL_Delay(ECHO_DELAY_MS);

      uint32_t tx_time_ms = 0U;
      const sx1262_result_t tx =
          sx1262_transmit((const uint8_t *)reply, (uint8_t)n, &tx_time_ms);
      if (tx == SX1262_OK) {
        diag_echoes_sent++;
      } else {
        diag_echoes_failed++;
      }
#else
      (void)reply;
      (void)n;
      const uint32_t tx_time_ms = 0U;
      const sx1262_result_t tx = SX1262_OK; /* no se transmite nada */
#endif

      printf("rx #%lu  RSSI %d dBm  SNR %d dB  senal %d dBm  df %+ld Hz  "
             "%u B  ->  eco %s (%lu ms)\r\n",
             (unsigned long)diag_rx_packets, (int)status.rssi_dbm,
             (int)status.snr_db, (int)status.signal_rssi_dbm,
             (long)status.freq_error_hz, (unsigned)length,
             ECHO_REPLY_ENABLE ? ((tx == SX1262_OK) ? "enviado" : "FALLIDO")
                               : "sin responder (solo escucha)",
             (unsigned long)tx_time_ms);

#if ECHO_REPLY_ENABLE
      /* sx1262_transmit() deja el chip en standby: hay que volver a escuchar */
      (void)sx1262_start_rx();
#endif

#if LNA_AB_TEST
      /*
       * start_rx() vuelve a encender el LNA por dentro (set_rx), asi que el
       * estado para el SIGUIENTE paquete se fija justo despues.
       */
      s_lna_on = !s_lna_on;
      HAL_GPIO_WritePin(SX1262_LNA_EN_GPIO_Port, SX1262_LNA_EN_Pin,
                        s_lna_on ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif

      flash_until = HAL_GetTick() + LED_ECHO_FLASH_MS;
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
      led_on = true;
    } else if (event == SX1262_RX_CRC_ERROR) {
      /*
       * No se responde a un paquete con CRC malo: su contenido no es fiable y,
       * con el enlace al límite, estos llegan a montones. Se cuentan, que es
       * justo el dato interesante para medir alcance: el punto donde empiezan
       * a aparecer marca el borde de la cobertura.
       */
      diag_rx_crc_errors++;
    }

    const uint32_t now = HAL_GetTick();

    if (led_on && (event != SX1262_RX_PACKET) &&
        ((int32_t)(now - flash_until) >= 0)) {
      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
      led_on = false;
    }
    if (!led_on && ((now - last_blink) >= LED_WAITING_HALF_PERIOD_MS)) {
      last_blink = now;
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    }

    if ((now - last_status) >= STATUS_PERIOD_MS) {
      last_status = now;
      sx1262_chip_status_t s;
      if (sx1262_chip_status(&s)) {
        diag_chip_mode = s.chip_mode;
      }
#if LNA_AB_TEST
      for (uint32_t i = 0U; i < 2U; i++) {
        if (diag_lna_packets[i] > 0U) {
          printf("   LNA %s: %lu paquetes, RSSI medio %ld dBm, SNR medio %ld dB\r\n",
                 (i == 1U) ? "encendido" : "apagado  ",
                 (unsigned long)diag_lna_packets[i],
                 (long)(diag_lna_rssi_sum[i] / (int32_t)diag_lna_packets[i]),
                 (long)(diag_lna_snr_sum[i] / (int32_t)diag_lna_packets[i]));
        }
      }
#endif
      printf("-- %lu recibidos, %lu ecos enviados (%lu fallidos), "
             "%lu con CRC malo, chip_mode 0x%02X\r\n",
             (unsigned long)diag_rx_packets, (unsigned long)diag_echoes_sent,
             (unsigned long)diag_echoes_failed,
             (unsigned long)diag_rx_crc_errors, (unsigned)diag_chip_mode);
    }
  }
}
