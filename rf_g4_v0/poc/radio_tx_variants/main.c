/**
 * @file    main.c
 * @brief   Barrido de configuraciones de transmision para buscar por que la
 *          baliza llega al receptor con el CRC malo.
 *
 * El receptor de tierra decodifica bien la cabecera pero el payload llega con
 * bits cambiados y el SNR se queda en +1 dB con RSSI de -43 dBm. Con esa
 * senal tan fuerte el SNR deberia ser mucho mayor: la energia esta, pero no
 * correlaciona, asi que la senal sale distorsionada del emisor.
 *
 * En vez de reprogramar la placa en cada intento, esto transmite el mismo
 * paquete de 12 bytes de siempre y va rotando entre varias configuraciones,
 * VARIANT_PACKETS paquetes cada una, en bucle. Como cada variante dura
 * VARIANT_PACKETS segundos, en el log del receptor se ve por tramos cual
 * mejora el SNR:
 *
 *   0. Base: +14 dBm, rampa de 40 us, regulador LDO (lo que hay hoy).
 *   1. Rampa de 800 us: si el problema es el transitorio al arrancar el PA,
 *      subirla lo suaviza.
 *   2. Potencia minima (-9 dBm): si con poca corriente sale limpio, apunta a
 *      la alimentacion o al PA.
 *   3. +22 dBm con su configuracion de PA: el caso contrario, para ver si
 *      empeora con la corriente.
 *   4. Regulador DC-DC en vez de LDO.
 *   5. Base otra vez, para descartar que la mejora sea por calentamiento o
 *      por la hora a la que se mide.
 *
 * No usa el bus CAN: transmite desde que arranca, sin esperar al core.
 * Consola en LPUART1 a 115200 (TX PB11), por si hay un adaptador conectado.
 *
 * Para compilarlo: en CMakeLists.txt, comentar las lineas de app/ y
 * descomentar poc/radio_tx_variants/main.c.
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

#define TX_PERIOD_MS 500U /* 2 Hz: mas muestras por variante */
#define TX_PAYLOAD_LEN 12U
#define VARIANT_PACKETS 16U /* paquetes por variante (8 s a 2 Hz) */

typedef struct {
  const char *name;
  int8_t power_dbm;
  sx126x_ramp_time_t ramp;
  sx126x_reg_mod_t regulator;
  uint8_t trim_cap;  /* condensadores de ajuste XTA/XTB del cristal */
  bool wrong_switch; /* transmitir con la antena en el camino de RX */
  int32_t freq_offset_hz; /* desplazamiento sobre la frecuencia del BSP */
  sx126x_lora_cr_t cr;    /* tasa de codificacion del payload */
  uint8_t ldro;           /* optimizacion de baja tasa */
  uint8_t sw_ctrl;        /* conmutador de antena: 1 = camino de TX */
  uint8_t lna_en;         /* nivel del enable del LNA externo durante la TX */
  uint8_t keep_xosc;      /* dejar el cristal en marcha entre paquetes */
} tx_variant_t;

/* El error de frecuencia medido en tierra sube de 2,7 a 4,7 kHz en minutos:
 * demasiada deriva para un cristal bien cargado. El SX1262 lleva
 * condensadores de ajuste internos en XTA/XTB (0x00 a 0x2F, pasos de
 * ~0,47 pF, 0x12 por defecto). Barriendo ese valor se ve si el oscilador
 * responde y cual deja el error de frecuencia centrado. */
/* El SNR se queda en 0 dB con RSSI de -50 dBm y el ajuste del cristal no
 * cambia nada, asi que el problema no es el oscilador. Toca mirar el camino
 * de RF: si transmitir con el conmutador en la posicion de RX da el mismo
 * RSSI que en la de TX, es que el conmutador no esta conmutando y el PA
 * trabaja contra una carga que no es la antena, lo que puede hacerlo
 * inestable y ensuciar la senal. */
/* El conmutador de antena funciona (23 dB de diferencia entre TX y RX), pero
 * el SNR sigue en 0 dB con RSSI de -48 dBm: la senal sale distorsionada. Si
 * el PA es inestable o le falla la alimentacion, bajando la potencia deberia
 * limpiarse; si sale igual de sucia a -9 dBm que a 22, el problema esta antes
 * del PA (oscilador, PLL o la propia modulacion). */
/* Los paquetes llegan corrompidos siempre igual (no es ruido) y el chip no da
 * ningun error interno, pero el receptor mide +3,9 kHz de error de frecuencia,
 * mientras que el emisor de referencia llega casi sin error. Si el receptor no
 * compensa bien ese desvio, la cabecera aguanta (va mas protegida) pero el
 * payload no. Aqui se barre la frecuencia de transmision alrededor de la
 * nominal para ver con cual empieza a decodificar. */
/* Ni la potencia, ni el conmutador, ni el cristal, ni la frecuencia cambian
 * nada, y los errores se repiten identicos paquete a paquete: eso no es ruido
 * de radio, es que el receptor decodifica el payload con un criterio distinto
 * al que uso el emisor. La cabecera lleva la tasa de codificacion y la
 * longitud, pero se transmite siempre con 4/8; si lo que el chip anuncia en la
 * cabecera no coincide con lo que usa en el payload, pasa justo esto. Aqui se
 * prueban las cuatro tasas y el LDRO. */
/* El esquematico tiene un LNA externo (U8) y un conmutador (U7) entre el
 * SX1262 y la antena. Si el LNA sigue activo mientras se transmite, con la
 * salida del PA entrandole por detras, puede oscilar y ensuciar la senal en
 * proporcion a la potencia: justo lo que se mide. Se prueban las cuatro
 * combinaciones de conmutador y LNA. */
/* Una sola configuracion, la normal, para comprobar el enlace despues de
 * tocar el cableado de antena. */
/* El SX1262 para el cristal entre paquetes (reposo en STDBY_RC) y lo arranca
 * al transmitir. Si el oscilador va justo de margen, puede no haberse
 * estabilizado cuando empieza el paquete, y los primeros simbolos salen con
 * la frecuencia moviendose: justo lo que ve la estacion de tierra, primeros
 * bytes malos y cola buena. Se compara pararlo (como ahora) con dejarlo
 * siempre en marcha (reposo en STDBY_XOSC). */
/* Sin los condensadores externos, el ajuste interno del cristal vuelve a
 * tener efecto: se barre para ver cuanto mueve y cual deja la frecuencia
 * centrada. El numero de variante va en todos los bytes del paquete. */
#define VAR(name, trim)                                                          { name, 14, SX126X_RAMP_40_US, SX126X_REG_MODE_LDO, (trim), false, 0,            SX126X_LORA_CR_4_5, 0, 1, 0, 0 }

static const tx_variant_t s_variants[] = {
    VAR("trim 0x0D", 0x0D), VAR("trim 0x0E", 0x0E), VAR("trim 0x0F", 0x0F),
    VAR("trim 0x10", 0x10), VAR("trim 0x11", 0x11), VAR("trim 0x12", 0x12),
};

#define VARIANT_COUNT (sizeof(s_variants) / sizeof(s_variants[0]))

/* Diagnostico visible desde el depurador */
volatile uint32_t diag_variant;
volatile uint32_t diag_tx_ok;
volatile uint32_t diag_tx_fail;
volatile uint32_t diag_last_tx_ms;
/* Duracion real de la transmision medida con el contador de ciclos del
 * Cortex-M4. Para SF7, BW125, CR4/5, preambulo 8, cabecera explicita, CRC y
 * 12 bytes, el tiempo en el aire teorico son 41,2 ms. Si lo medido se desvia
 * un tanto por ciento, el reloj de referencia del SX1262 esta mal, porque el
 * chip saca de ahi la velocidad del chirp. */
volatile uint32_t diag_tx_us;
volatile uint32_t diag_tx_us_min = 0xFFFFFFFFU;
volatile uint32_t diag_tx_us_max;
volatile uint32_t diag_sysclk_hz;
/* Lo que dice el propio chip despues de transmitir: errores internos y los
 * registros de ajuste del cristal (0x0911 XTA, 0x0912 XTB), para comprobar
 * que las escrituras de ajuste llegaban de verdad. */
volatile uint16_t diag_dev_errors;
volatile uint8_t diag_trim_xta;
volatile uint8_t diag_trim_xtb;
volatile uint8_t diag_chip_mode;

int __io_putchar(int ch) {
  HAL_UART_Transmit(&hlpuart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

/* Tabla del datasheet del SX1262 ("PA optimal settings"). Es la misma que usa
 * el driver, repetida aqui para poder barrer potencias sin tocarlo. */
static void pa_config_for_power(int8_t power_dbm, sx126x_pa_cfg_params_t *out) {
  out->device_sel = 0x00; /* SX1262 */
  out->pa_lut = 0x01;

  if (power_dbm >= 22) {
    out->pa_duty_cycle = 0x04;
    out->hp_max = 0x07;
  } else if (power_dbm >= 20) {
    out->pa_duty_cycle = 0x03;
    out->hp_max = 0x05;
  } else if (power_dbm >= 17) {
    out->pa_duty_cycle = 0x02;
    out->hp_max = 0x03;
  } else {
    out->pa_duty_cycle = 0x02;
    out->hp_max = 0x02;
  }
}

/* Aplica una variante sobre la configuracion ya hecha por sx1262_configure().
 * El chip queda en STDBY_RC tras configurar, que es donde hay que estar para
 * cambiar el regulador. */
/* Conmutador y LNA segun la variante. El driver llama a set_tx() antes de
 * transmitir, asi que estas funciones sustituyen a las del BSP. */
static const tx_variant_t *s_current;

static void variant_set_tx(void) {
  HAL_GPIO_WritePin(SX1262_SW_CTRL_GPIO_Port, SX1262_SW_CTRL_Pin,
                    s_current->sw_ctrl ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SX1262_LNA_EN_GPIO_Port, SX1262_LNA_EN_Pin,
                    s_current->lna_en ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static bool apply_variant(const void *hal, const tx_variant_t *v) {
  sx126x_pa_cfg_params_t pa;
  pa_config_for_power(v->power_dbm, &pa);

  const sx126x_mod_params_lora_t mod = {
      .sf = bsp_sx1262_get_config()->radio.sf,
      .bw = bsp_sx1262_get_config()->radio.bw,
      .cr = v->cr,
      .ldro = v->ldro,
  };
  const uint32_t freq =
      (uint32_t)((int32_t)bsp_sx1262_get_config()->radio.frequency_hz +
                 v->freq_offset_hz);

  if ((sx126x_set_standby(hal, SX126X_STANDBY_CFG_RC) != SX126X_STATUS_OK) ||
      (sx126x_set_reg_mode(hal, v->regulator) != SX126X_STATUS_OK) ||
      (sx126x_set_lora_mod_params(hal, &mod) != SX126X_STATUS_OK) ||
      (sx126x_set_rf_freq(hal, freq) != SX126X_STATUS_OK) ||
      (sx126x_set_pa_cfg(hal, &pa) != SX126X_STATUS_OK) ||
      (sx126x_set_tx_params(hal, v->power_dbm, v->ramp) != SX126X_STATUS_OK)) {
    return false;
  }

  /* El chip vuelve a poner el ajuste por defecto cada vez que arranca el
   * cristal, asi que hay que escribirlo con el cristal YA en marcha y no
   * dejar que se pare: el reposo pasa a ser STDBY_XOSC. */
  if (sx126x_set_rx_tx_fallback_mode(hal, SX126X_FALLBACK_STDBY_XOSC) !=
      SX126X_STATUS_OK) {
    return false;
  }
  if (sx126x_set_standby(hal, SX126X_STANDBY_CFG_XOSC) != SX126X_STATUS_OK) {
    return false;
  }
  HAL_Delay(10);
  return sx126x_set_trimming_capacitor_values(hal, v->trim_cap, v->trim_cap) ==
         SX126X_STATUS_OK;
}


int main(void) {
  board_init();

  /* Contador de ciclos, para medir la transmision con resolucion de us */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  diag_sysclk_hz = HAL_RCC_GetSysClockFreq();

  printf("\r\n=== Placa RF: barrido de configuraciones de TX ===\r\n");

  const sx1262_config_t *config = bsp_sx1262_get_config();

  /* Dos configuraciones iguales salvo el conmutador de antena: la segunda
   * transmite con la antena puesta en el camino de recepcion. */
  static sx1262_config_t normal_cfg;
  static sx1262_config_t swapped_cfg;
  normal_cfg = *config;
  swapped_cfg = *config;
  normal_cfg.set_tx = variant_set_tx;
  swapped_cfg.set_tx = variant_set_tx;

  if (!sx1262_init(&normal_cfg)) {
    printf("ERROR: configuracion del BSP incompleta\r\n");
    Error_Handler();
  }

  sx1262_result_t result = sx1262_configure();
  if (result != SX1262_OK) {
    printf("ERROR: radio no configurada, paso %u (error %d)\r\n",
           sx1262_get_failed_step(), (int)result);
    Error_Handler();
  }

  printf("%u variantes, %lu paquetes cada una, 1 por segundo\r\n",
         (unsigned)VARIANT_COUNT, (unsigned long)VARIANT_PACKETS);

  uint32_t seq = 0;
  uint32_t last_tx_ms = HAL_GetTick() - TX_PERIOD_MS;
  uint32_t variant = VARIANT_COUNT; /* fuerza aplicar la 0 en el primer paso */

  while (1) {
    if ((HAL_GetTick() - last_tx_ms) < TX_PERIOD_MS) {
      continue;
    }
    last_tx_ms += TX_PERIOD_MS;

    const uint32_t wanted = (seq / VARIANT_PACKETS) % VARIANT_COUNT;
    if (wanted != variant) {
      variant = wanted;
      diag_variant = variant;
      s_current = &s_variants[variant];

      /* Cambiar de configuracion obliga a reconfigurar la radio entera */
      (void)sx1262_init(s_variants[variant].wrong_switch ? &swapped_cfg
                                                         : &normal_cfg);
      if (sx1262_configure() != SX1262_OK) {
        printf("ERROR reconfigurando la radio\r\n");
      }
      if (!apply_variant(config->hal, &s_variants[variant])) {
        printf("ERROR aplicando la variante %lu\r\n", (unsigned long)variant);
      }
      printf("\r\n--- variante %lu: %s ---\r\n", (unsigned long)variant,
             s_variants[variant].name);
    }

    /* Los 12 bytes con el mismo caracter, el numero de variante. Como el CRC
     * falla y no se sabe que posiciones llegaran bien, asi cualquier byte
     * correcto identifica la variante, y cuantos bytes coincidan mide cuanto
     * se corrompe el paquete. */
    uint8_t payload[TX_PAYLOAD_LEN];
    memset(payload, (int)('0' + variant), sizeof(payload));

    uint32_t tx_ms = 0;
    const uint32_t cycles_before = DWT->CYCCNT;
    result = sx1262_transmit(payload, TX_PAYLOAD_LEN, &tx_ms);
    const uint32_t cycles = DWT->CYCCNT - cycles_before;
    diag_last_tx_ms = tx_ms;
    diag_tx_us = cycles / (diag_sysclk_hz / 1000000U);
    if (diag_tx_us < diag_tx_us_min) {
      diag_tx_us_min = diag_tx_us;
    }
    if (diag_tx_us > diag_tx_us_max) {
      diag_tx_us_max = diag_tx_us;
    }

    {
      uint16_t errors = 0;
      if (sx126x_get_device_errors(config->hal, &errors) == SX126X_STATUS_OK) {
        diag_dev_errors |= errors;
      }
      uint8_t trim[2] = {0};
      if (sx126x_read_register(config->hal, 0x0911, trim, 2) ==
          SX126X_STATUS_OK) {
        diag_trim_xta = trim[0];
        diag_trim_xtb = trim[1];
      }
      sx126x_chip_status_t st;
      if (sx126x_get_status(config->hal, &st) == SX126X_STATUS_OK) {
        diag_chip_mode = (uint8_t)st.chip_mode;
      }
    }

    if (result == SX1262_OK) {
      diag_tx_ok++;
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
      printf("TX #%lu var %lu: TX_DONE en %lu ms\r\n", (unsigned long)seq,
             (unsigned long)variant, (unsigned long)tx_ms);
    } else {
      diag_tx_fail++;
      printf("TX #%lu var %lu: ERROR %d\r\n", (unsigned long)seq,
             (unsigned long)variant, (int)result);
    }

    seq++;
  }
}
