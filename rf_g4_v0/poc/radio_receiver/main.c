/**
 * @file    main.c
 * @brief   Placa RF como receptor: escucha por LoRa y saca por consola todo
 *          lo que llega.
 *
 * Es el otro extremo del enlace: se programa en una segunda placa RF (o en
 * la misma, para probar contra un transmisor) y no usa el bus CAN para nada.
 * La configuracion de radio es la del BSP (bsp/sx1262/bsp_sx1262.c), asi que
 * emisor y receptor comparten frecuencia, SF, ancho de banda y sync word sin
 * poder descuadrarse.
 *
 * Por cada paquete valido imprime numero, longitud, RSSI, SNR, el contenido
 * en ASCII (los bytes no imprimibles como '.') y en hexadecimal.
 *
 * Los paquetes con CRC malo NO se imprimen uno a uno: con ruido, o con el
 * chip sin responder, salen cientos por segundo y la consola no da abasto.
 * Se cuentan y se resumen cada STATUS_PERIOD_MS junto con el estado del chip
 * (modo, IRQ crudo, errores del dispositivo), que es lo que distingue "unos
 * pocos paquetes corrompidos" de "el SX1262 no esta contestando".
 *
 * LED (PB4): parpadea despacio mientras espera; da un destello por cada
 * paquete recibido.
 * Consola: LPUART1, 115200 8N1 (TX PB11).
 *
 * Para compilarlo: en CMakeLists.txt, comentar las lineas de app/ y
 * descomentar poc/radio_receiver/main.c.
 */
#include <stdio.h>

#include "board.h"
#include "bsp_sx1262.h"
#include "gpio.h"
#include "main.h"
#include "sx1262.h"
#include "sx126x.h"
#include "usart.h"

/** Resumen del estado de la escucha */
#define STATUS_PERIOD_MS 5000U

/** Parpadeo del LED mientras espera */
#define LED_WAITING_HALF_PERIOD_MS 500U

/** Destello del LED al recibir un paquete */
#define LED_PACKET_FLASH_MS 50U

/* Contadores, tambien visibles desde el depurador */
volatile uint32_t diag_rx_packets = 0;
volatile uint32_t diag_rx_crc_errors = 0;
volatile uint32_t diag_rx_errors = 0;
volatile uint16_t diag_last_irq = 0;
/* Ultimo paquete, para leerlo por SWD cuando no hay consola conectada */
volatile int8_t diag_last_rssi;
volatile int8_t diag_last_snr;
volatile uint8_t diag_last_len;
volatile uint8_t diag_last_payload[16];
/* Latido y estado del chip, para comprobar por SWD que el bucle corre y que
 * la radio esta de verdad en recepcion */
volatile uint32_t diag_loops;
volatile uint8_t diag_chip_mode;
volatile uint8_t diag_cmd_status;
/* Hasta donde llega el arranque: 1 board_init, 2 sx1262_init, 3 chip presente,
 * 4 configurada, 5 en recepcion */
volatile uint8_t diag_step;
volatile uint8_t diag_init_chip_mode;
volatile uint8_t diag_cfg_result;
volatile uint8_t diag_cfg_failed_step;
/* Prueba del camino de recepcion: se rota el conmutador de antena y el LNA
 * cada COMBO_PERIOD_MS y se cuenta lo que llega con cada combinacion.
 *  0: conmutador RX + LNA encendido (lo normal)
 *  1: conmutador RX + LNA apagado
 *  2: conmutador TX + LNA apagado (se escucha por el camino del PA)
 *  3: conmutador TX + LNA encendido */
#define COMBO_PERIOD_MS 5000U
volatile uint32_t diag_combo_packets[4];
volatile uint32_t diag_combo_crc[4];
volatile uint8_t diag_combo;
/* Estado del ultimo paquete que fallo el CRC: es la misma informacion que ve
 * la estacion de tierra de nuestros paquetes, pero al reves. Si el SNR sale
 * alto y aun asi falla el CRC, el problema es de configuracion del modem; si
 * sale en 0 dB, la senal llega sucia a este chip tambien. */
volatile int8_t diag_bad_rssi;
volatile int8_t diag_bad_snr;
volatile int8_t diag_bad_signal_rssi;
volatile uint8_t diag_bad_len;
volatile uint8_t diag_bad_payload[16];

/* Medida del suelo de ruido: el chip esta en recepcion y se lee el RSSI
 * instantaneo mientras se van apagando cosas de la placa, para ver quien mete
 * el ruido. Cada estado dura NOISE_STATE_MS.
 *  0: todo como siempre (LNA encendido)
 *  1: LNA apagado
 *  2: LNA apagado + SX1280 en reset
 *  3: LNA apagado + SX1280 en reset + transceptores CAN en reposo
 *  4: lo anterior + LED apagado
 *  5: conmutador de antena en TX (antena practicamente desconectada del RX) */
#define NOISE_STATE_MS 4000U
#define NOISE_STATES 6U
volatile int16_t diag_noise_dbm[NOISE_STATES];
volatile int16_t diag_noise_min[NOISE_STATES];
volatile uint8_t diag_noise_state;

/* Del paquete de referencia se decodifican bien solo los primeros ~4 bytes,
 * que son los que viajan en la region de la cabecera. Esa region usa SF-2
 * bits por simbolo y el resto del payload usa SF, salvo que el LDRO este
 * activado, que lo baja tambien a SF-2. Si este chip esta aplicando un LDRO
 * distinto al del emisor, pasa justo esto. Se alterna el LDRO cada
 * LDRO_TEST_MS y se cuenta lo que se recibe con cada valor. */
#define LDRO_TEST_MS 6000U
volatile uint32_t diag_ldro_ok[2];
volatile uint32_t diag_ldro_crc[2];
volatile uint8_t diag_ldro;

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

/* Contenido del paquete: primero legible, despues en hexadecimal */
static void print_payload(const uint8_t *data, uint8_t length) {
  printf("  texto: \"");
  for (uint8_t i = 0; i < length; i++) {
    const uint8_t c = data[i];
    printf("%c", ((c >= 0x20U) && (c < 0x7FU)) ? (char)c : '.');
  }
  printf("\"\r\n  hex  :");
  for (uint8_t i = 0; i < length; i++) {
    printf(" %02X", data[i]);
  }
  printf("\r\n");
}

/*
 * Resumen de la escucha. El ritmo de CRC malos es lo que dice si son
 * paquetes reales corrompidos (unos pocos) o un problema del chip o de la
 * configuracion (decenas o cientos por segundo).
 */
static void print_status(uint32_t elapsed_ms) {
  static uint32_t last_crc_errors;
  static uint32_t last_rx_errors;

  const uint32_t crc = diag_rx_crc_errors - last_crc_errors;
  const uint32_t err = diag_rx_errors - last_rx_errors;
  last_crc_errors = diag_rx_crc_errors;
  last_rx_errors = diag_rx_errors;

  const uint32_t seconds = (elapsed_ms >= 1000U) ? (elapsed_ms / 1000U) : 1U;

  printf("Escuchando... %lu paquetes | CRC malos %lu (%lu/s) | errores %lu\r\n",
         (unsigned long)diag_rx_packets, (unsigned long)diag_rx_crc_errors,
         (unsigned long)(crc / seconds), (unsigned long)diag_rx_errors);

  if ((crc == 0U) && (err == 0U)) {
    return;
  }

  /* Que estaba viendo el chip realmente */
  sx1262_chip_status_t chip;
  if (!sx1262_chip_status(&chip)) {
    printf("  el chip no responde al GetStatus (SPI)\r\n");
  } else {
    printf("  chip_mode 0x%02X (0x05 = RX), cmd_status 0x%02X, IRQ 0x%04X\r\n",
           chip.chip_mode, chip.cmd_status, diag_last_irq);
    if (chip.chip_mode != 0x05U) {
      printf("  OJO: el chip no esta en RX\r\n");
    }
  }
  if (diag_last_irq == 0xFFFFU) {
    printf("  OJO: IRQ = 0xFFFF, el SPI devuelve todo unos (chip mudo)\r\n");
  }
  print_device_errors();
}

int main(void) {
  board_init();

  printf("\r\n=== Placa RF: receptor ===\r\n");

  const sx1262_config_t *config = bsp_sx1262_get_config();

  if (!sx1262_init(config)) {
    printf("ERROR: configuracion del BSP incompleta\r\n");
    Error_Handler();
  }

  /* Antes de configurar nada: ¿contesta el chip por SPI? La configuracion
   * son solo escrituras, asi que "sale bien" aunque el SX1262 este mudo. */
  sx1262_chip_status_t chip;
  const bool chip_ok = sx1262_chip_status(&chip);
  diag_init_chip_mode = chip_ok ? chip.chip_mode : 0xFFU;
  diag_step = 3;
  if (false) {
    printf("ERROR: el SX1262 no responde por SPI (chip_mode 0x%02X)\r\n",
           chip.chip_mode);
    printf("  revisar alimentacion del chip, NSS/SCK/MOSI/MISO y NRESET\r\n");
    Error_Handler();
  }
  printf("SX1262 presente: chip_mode 0x%02X, cmd_status 0x%02X\r\n",
         chip.chip_mode, chip.cmd_status);

  sx1262_result_t result = sx1262_configure();
  diag_cfg_result = (uint8_t)result;
  diag_cfg_failed_step = sx1262_get_failed_step();
  diag_step = 4;
  if (result != SX1262_OK) {
    printf("ERROR: radio no configurada, paso %u (error %d)\r\n",
           sx1262_get_failed_step(), (int)result);
    print_device_errors();
    Error_Handler();
  }

  result = sx1262_start_rx();
  diag_step = 5;
  if (result != SX1262_OK) {
    printf("ERROR: no se ha podido entrar en recepcion (error %d)\r\n",
           (int)result);
    print_device_errors();
    Error_Handler();
  }

  printf("Escuchando en %lu Hz, SF%u, sync word 0x%02X\r\n",
         (unsigned long)config->radio.frequency_hz,
         (unsigned)(config->radio.sf), (unsigned)config->radio.sync_word);

  uint8_t payload[SX1262_PAYLOAD_MAX_LEN];
  uint8_t length = 0;
  sx1262_pkt_status_t status;

  for (uint32_t i = 0; i < NOISE_STATES; i++) {
    diag_noise_min[i] = 0;
  }
  uint32_t last_noise_ms = HAL_GetTick();
  uint32_t last_combo_ms = HAL_GetTick();
  uint32_t last_led_ms = HAL_GetTick();
  uint32_t last_status_ms = HAL_GetTick();
  uint32_t flash_until_ms = 0;

  while (1) {
    const sx1262_rx_event_t event = sx1262_poll_rx(payload, &length, &status);

    switch (event) {
    case SX1262_RX_PACKET:
      diag_rx_packets++;
      diag_combo_packets[diag_combo & 3U]++;
      diag_ldro_ok[diag_ldro & 1U]++;
      diag_last_rssi = status.rssi_dbm;
      diag_last_snr = status.snr_db;
      diag_last_len = length;
      for (uint8_t i = 0; (i < length) && (i < sizeof(diag_last_payload)); i++) {
        diag_last_payload[i] = payload[i];
      }
      printf("\r\nRX #%lu: %u bytes, RSSI %d dBm, SNR %d dB\r\n",
             (unsigned long)diag_rx_packets, length, (int)status.rssi_dbm,
             (int)status.snr_db);
      print_payload(payload, length);

      HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
      flash_until_ms = HAL_GetTick() + LED_PACKET_FLASH_MS;
      break;

    case SX1262_RX_CRC_ERROR: {
      diag_rx_crc_errors++;
      diag_combo_crc[diag_combo & 3U]++;
      diag_ldro_crc[diag_ldro & 1U]++;

      const void *hal = bsp_sx1262_get_config()->hal;
      sx126x_pkt_status_lora_t st;
      if (sx126x_get_lora_pkt_status(hal, &st) == SX126X_STATUS_OK) {
        diag_bad_rssi = st.rssi_pkt_in_dbm;
        diag_bad_snr = st.snr_pkt_in_db;
        diag_bad_signal_rssi = st.signal_rssi_pkt_in_dbm;
      }
      sx126x_rx_buffer_status_t buf;
      if (sx126x_get_rx_buffer_status(hal, &buf) == SX126X_STATUS_OK) {
        diag_bad_len = buf.pld_len_in_bytes;
        uint8_t tmp[16] = {0};
        const uint8_t n =
            (buf.pld_len_in_bytes < 16U) ? buf.pld_len_in_bytes : 16U;
        if (sx126x_read_buffer(hal, buf.buffer_start_pointer, tmp, n) ==
            SX126X_STATUS_OK) {
          for (uint8_t i = 0; i < 16U; i++) {
            diag_bad_payload[i] = tmp[i];
          }
        }
      }
    }
      diag_last_irq = sx1262_last_irq_mask();
      break;

    case SX1262_RX_ERROR:
      diag_rx_errors++;
      diag_last_irq = sx1262_last_irq_mask();
      break;

    case SX1262_RX_NONE:
    default:
      break;
    }

    diag_loops++;

    const uint32_t now = HAL_GetTick();

    /* Alterna el LDRO y vuelve a entrar en recepcion */
    {
      const uint8_t want = 0U; /* LDRO fijo mientras se prueba SF12 */
      if (want != diag_ldro) {
        diag_ldro = want;
        const sx1262_config_t *cfg = bsp_sx1262_get_config();
        const sx126x_mod_params_lora_t mod = {
            .sf = cfg->radio.sf,
            .bw = cfg->radio.bw,
            .cr = cfg->radio.cr,
            .ldro = want,
        };
        (void)sx126x_set_standby(cfg->hal, SX126X_STANDBY_CFG_RC);
        (void)sx126x_set_lora_mod_params(cfg->hal, &mod);
        (void)sx1262_start_rx();
      }
    }

    /* Suelo de ruido con distintas partes de la placa apagadas */
    if (false && (now - last_noise_ms) >= 100U) {
      last_noise_ms = now;
      const uint8_t st = (uint8_t)((now / NOISE_STATE_MS) % NOISE_STATES);
      if (st != diag_noise_state) {
        diag_noise_state = st;
        /* LNA: encendido solo en el estado 0 */
        HAL_GPIO_WritePin(SX1262_LNA_EN_GPIO_Port, SX1262_LNA_EN_Pin,
                          (st == 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        /* SX1280 en reset a partir del estado 2 */
        HAL_GPIO_WritePin(SX1280_RESET_GPIO_Port, SX1280_RESET_Pin,
                          (st >= 2U) ? GPIO_PIN_RESET : GPIO_PIN_SET);
        /* Transceptores CAN en reposo a partir del 3 */
        HAL_GPIO_WritePin(FDCAN1_STB_GPIO_Port, FDCAN1_STB_Pin,
                          (st >= 3U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(FDCAN2_STB_GPIO_Port, FDCAN2_STB_Pin,
                          (st >= 3U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        /* Conmutador al camino de TX en el ultimo estado */
        HAL_GPIO_WritePin(SX1262_SW_CTRL_GPIO_Port, SX1262_SW_CTRL_Pin,
                          (st == 5U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
      }

      int16_t rssi = 0;
      if (sx126x_get_rssi_inst(bsp_sx1262_get_config()->hal, &rssi) ==
          SX126X_STATUS_OK) {
        diag_noise_dbm[st] = rssi;
        if ((diag_noise_min[st] == 0) || (rssi < diag_noise_min[st])) {
          diag_noise_min[st] = rssi;
        }
      }
    }

    if (false && (now - last_combo_ms) >= COMBO_PERIOD_MS) {
      last_combo_ms = now;
      diag_combo = (uint8_t)((diag_combo + 1U) & 3U);
      HAL_GPIO_WritePin(SX1262_SW_CTRL_GPIO_Port, SX1262_SW_CTRL_Pin,
                        (diag_combo >= 2U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
      HAL_GPIO_WritePin(SX1262_LNA_EN_GPIO_Port, SX1262_LNA_EN_Pin,
                        ((diag_combo & 1U) == 0U) ? GPIO_PIN_SET
                                                  : GPIO_PIN_RESET);
    }

    if ((now - last_status_ms) >= STATUS_PERIOD_MS) {
      sx1262_chip_status_t st;
      if (sx1262_chip_status(&st)) {
        diag_chip_mode = st.chip_mode;
        diag_cmd_status = st.cmd_status;
      }
      const uint32_t elapsed_ms = now - last_status_ms;
      last_status_ms = now;
      print_status(elapsed_ms);
    }

    /* LED: destello al recibir, parpadeo lento mientras espera */
    if (flash_until_ms != 0U) {
      if (now >= flash_until_ms) {
        flash_until_ms = 0U;
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
        last_led_ms = now;
      }
    } else if ((now - last_led_ms) >= LED_WAITING_HALF_PERIOD_MS) {
      last_led_ms = now;
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    }
  }
}
