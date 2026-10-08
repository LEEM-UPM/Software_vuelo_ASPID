/**
 * @file    sx1262.c
 * @brief   Mid-level driver for the SX1262 sub-GHz radio chip.
 *
 * This file contains zero board-specific references. No STM32 HAL headers,
 * no GPIO macros, no SPI handles. All hardware access goes through the
 * sx1262_config_t injected by the BSP at initialisation time.
 *
 * The init and TX sequences are the ones verified on the RF board (26/26
 * packets, TX_DONE at 41 ms vs 41.2 ms expected). Do not reorder steps or
 * swap values for the "typical" ones from Semtech examples.
 */

#include "sx1262.h"
#include <stddef.h>

/* -------------------------------------------------------------------------
 * Fixed link parameters (both ends of the link must use the same)
 * ------------------------------------------------------------------------- */
#define SX1262_PREAMBLE_SYMBOLS 8U
#define SX1262_CRC_ON true

/** SetOcpValue in steps of 2.5 mA: 0x38 = 56 * 2.5 = 140 mA (SX1262). */
#define SX1262_OCP_VALUE 0x38U

/** SetTx timeout. The chip raises TIMEOUT if TX has not finished by then. */
#define SX1262_TX_TIMEOUT_MS 3000U

/** Software deadline on top of the chip's own TX timeout. */
#define SX1262_TX_POLL_MARGIN_MS 500U

/** Poll period of GetIrqStatus while waiting for TX_DONE. */
#define SX1262_TX_POLL_PERIOD_MS 1U

/** Configuration injected by the BSP. Valid after sx1262_init() returns true. */
static const sx1262_config_t *s_config = NULL;

/** true while in continuous RX (DIO IRQ mask set for RX). */
static bool s_rx_active = false;

/** Step (1..16) of sx1262_configure() that failed last; 0 = none. */
static uint8_t s_failed_step = 0;

/** Diagnostic counter — number of DIO1 edges seen since reset. */
volatile uint32_t sx1262_isr_count = 0;

/* -------------------------------------------------------------------------
 * Private helpers
 * ------------------------------------------------------------------------- */

/**
 * SetPaConfig for the SX1262, datasheet table "PA optimal settings". Any
 * other combination may behave erratically or damage the PA, so powers not
 * in the table are rejected.
 *
 * OJO con los niveles por debajo de +22 dBm: aqui se pasa el mismo valor a
 * SetTxParams, como hace el driver de referencia de Semtech, pero la tabla
 * del datasheet supone SetTxParams = +22 dBm en todas sus filas. Medido el
 * 2026-09-22 contra la estacion de tierra: al pasar de "+14 dBm" (0x02/0x02
 * con SetTxParams 14) a +22 dBm (0x04/0x07 con SetTxParams 22) el RSSI subio
 * 24 dB, no los 8 que tocarian, asi que aquel "+14 dBm" eran en realidad
 * unos -2 dBm. Si alguna vez hace falta un nivel intermedio de verdad, hay
 * que medirlo antes de fiarse del numero.
 */
static bool pa_config_for_power(int8_t power_dbm, sx126x_pa_cfg_params_t *out) {
  out->device_sel = 0x00; /* SX1262 */
  out->pa_lut = 0x01;

  switch (power_dbm) {
  case 14:
    out->pa_duty_cycle = 0x02;
    out->hp_max = 0x02;
    return true;
  case 17:
    out->pa_duty_cycle = 0x02;
    out->hp_max = 0x03;
    return true;
  case 20:
    out->pa_duty_cycle = 0x03;
    out->hp_max = 0x05;
    return true;
  case 22:
    out->pa_duty_cycle = 0x04;
    out->hp_max = 0x07;
    return true;
  default:
    return false;
  }
}

static uint32_t lora_bw_in_hz(sx126x_lora_bw_t bw) {
  switch (bw) {
  case SX126X_LORA_BW_007: return 7810U;
  case SX126X_LORA_BW_010: return 10420U;
  case SX126X_LORA_BW_015: return 15630U;
  case SX126X_LORA_BW_020: return 20830U;
  case SX126X_LORA_BW_031: return 31250U;
  case SX126X_LORA_BW_041: return 41670U;
  case SX126X_LORA_BW_062: return 62500U;
  case SX126X_LORA_BW_125: return 125000U;
  case SX126X_LORA_BW_250: return 250000U;
  case SX126X_LORA_BW_500: return 500000U;
  default: return 125000U;
  }
}

static sx126x_status_t set_pkt_params(uint8_t payload_length) {
  const sx126x_pkt_params_lora_t pkt = {
      .preamble_len_in_symb = SX1262_PREAMBLE_SYMBOLS,
      .header_type = SX126X_LORA_PKT_EXPLICIT,
      .pld_len_in_bytes = payload_length,
      .crc_is_on = SX1262_CRC_ON,
      .invert_iq_is_on = false,
  };
  return sx126x_set_lora_pkt_params(s_config->hal, &pkt);
}

/*
 * LoRa frequency error indicator. The chip leaves it in three registers as a
 * 20-bit two's complement number, valid only for the packet just received.
 *
 * Datasheet conversion:  Hz = raw * (2^24 / F_XTAL) * (BW / 500000)
 * with F_XTAL = 32 MHz, so the constant is 0.524288 * BW / 500000.
 */
#define SX126X_REG_FREQ_ERROR 0x076BU

static int32_t read_freq_error_hz(void) {
  const void *hal = s_config->hal;
  uint8_t efe[3] = {0U, 0U, 0U};
  if (sx126x_read_register(hal, SX126X_REG_FREQ_ERROR, efe, sizeof(efe)) !=
      SX126X_STATUS_OK) {
    return 0;
  }

  int32_t raw = (int32_t)(((uint32_t)(efe[0] & 0x0FU) << 16) |
                          ((uint32_t)efe[1] << 8) | (uint32_t)efe[2]);
  if ((raw & 0x00080000) != 0) {
    raw -= 0x00100000; /* 20 bits en complemento a dos */
  }

  const uint32_t bw_hz = lora_bw_in_hz(s_config->radio.bw);
  return (int32_t)(((double)raw * 0.524288 * (double)bw_hz) / 500000.0);
}

static sx126x_status_t set_irq_tx(void) {
  return sx126x_set_dio_irq_params(
      s_config->hal, SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT,
      SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT, SX126X_IRQ_NONE,
      SX126X_IRQ_NONE);
}

/** Runs one step of sx1262_configure(); returns from it on failure. */
#define CONFIG_STEP(step, call)                                                \
  do {                                                                         \
    s_failed_step = (step);                                                    \
    if ((call) != SX126X_STATUS_OK) {                                          \
      return SX1262_ERR_CHIP;                                                  \
    }                                                                          \
  } while (0)

/* -------------------------------------------------------------------------
 * Public API
 * ------------------------------------------------------------------------- */

bool sx1262_init(const sx1262_config_t *config) {
  if ((config == NULL) || (config->hal == NULL) || (config->set_tx == NULL) ||
      (config->set_rx == NULL) || (config->get_tick_ms == NULL) ||
      (config->delay_ms == NULL)) {
    return false;
  }

  s_config = config;
  s_rx_active = false;

  if (config->register_irq_handler != NULL) {
    config->register_irq_handler(sx1262_dio1_irq_handler);
  }

  return true;
}

sx1262_result_t sx1262_configure(void) {
  if (s_config == NULL) {
    return SX1262_ERR_PARAM;
  }

  const void *hal = s_config->hal;

  sx126x_pa_cfg_params_t pa_cfg;
  if (!pa_config_for_power(s_config->radio.tx_power_dbm, &pa_cfg)) {
    s_failed_step = 8;
    return SX1262_ERR_PARAM;
  }

  const sx126x_mod_params_lora_t mod = {
      .sf = s_config->radio.sf,
      .bw = s_config->radio.bw,
      .cr = s_config->radio.cr,
      /* LDRO off: symbol time 2^SF/BW is far below 16 ms at SF7/BW125. It
       * must be turned on if SF/BW ever push the symbol time above that. */
      .ldro = 0,
  };

  s_rx_active = false;

  /* 1. Reset. The HAL waits for BUSY to drop before returning. */
  CONFIG_STEP(1, sx126x_reset(hal));
  /* 2. Regulator mode can only be changed in STDBY_RC. */
  CONFIG_STEP(2, sx126x_set_standby(hal, SX126X_STANDBY_CFG_RC));
  /* 3. LDO: the DC-DC node exists on the board but is not reliable. */
  CONFIG_STEP(3, sx126x_set_reg_mode(hal, SX126X_REG_MODE_LDO));
  /* 4. Calibration under the final regulator. No SetDIO3AsTcxoCtrl: the
   *    reference is a crystal and a TCXO setting makes this step hang. */
  CONFIG_STEP(4, sx126x_cal(hal, SX126X_CAL_ALL));
  /* 5. Mandatory after Calibrate; SetRfFrequency does not do it. */
  CONFIG_STEP(5, sx126x_cal_img_in_mhz(hal, s_config->radio.cal_img_freq1_mhz,
                                       s_config->radio.cal_img_freq2_mhz));
  CONFIG_STEP(6, sx126x_set_pkt_type(hal, SX126X_PKT_TYPE_LORA));
  CONFIG_STEP(7, sx126x_set_rf_freq(hal, s_config->radio.frequency_hz));
  CONFIG_STEP(8, sx126x_set_pa_cfg(hal, &pa_cfg));
  /* 9. Datasheet workaround (reg 0x08D8): protects the PA against a
   *    mismatched antenna. Re-armed before every TX as well. */
  CONFIG_STEP(9, sx126x_cfg_tx_clamp(hal));
  CONFIG_STEP(10, sx126x_set_ocp_value(hal, SX1262_OCP_VALUE));
  CONFIG_STEP(11, sx126x_set_tx_params(hal, s_config->radio.tx_power_dbm,
                                       SX126X_RAMP_40_US));
  /* 12. The Semtech driver also applies the 0x0889 modulation workaround. */
  CONFIG_STEP(12, sx126x_set_lora_mod_params(hal, &mod));
  CONFIG_STEP(13, sx126x_set_lora_sync_word(hal, s_config->radio.sync_word));
  /* 14. Reposo con el cristal en marcha: al pararlo, el chip vuelve a poner
   *     el ajuste de los condensadores por defecto (0x12) y la frecuencia se
   *     iria unas +15 ppm. Ademas, asi no hay que esperar a que arranque. */
  CONFIG_STEP(14,
              sx126x_set_rx_tx_fallback_mode(hal, SX126X_FALLBACK_STDBY_XOSC));
  /* 15. El ajuste del cristal solo se puede escribir con el oscilador en
   *     marcha, y hay que dejarlo en marcha para que no se pierda. */
  CONFIG_STEP(15, sx126x_set_standby(hal, SX126X_STANDBY_CFG_XOSC));
  s_config->delay_ms(10);
  CONFIG_STEP(15, sx126x_set_trimming_capacitor_values(
                      hal, s_config->radio.xtal_trim, s_config->radio.xtal_trim));
  CONFIG_STEP(16, set_irq_tx());
  CONFIG_STEP(17, sx126x_clear_irq_status(hal, SX126X_IRQ_ALL));
  CONFIG_STEP(17, sx126x_clear_device_errors(hal));

  s_failed_step = 0;
  return SX1262_OK;
}

uint8_t sx1262_get_failed_step(void) { return s_failed_step; }

sx1262_result_t sx1262_transmit(const uint8_t *payload, uint8_t length,
                                uint32_t *tx_time_ms) {
  if ((s_config == NULL) || (payload == NULL) || (length == 0U)) {
    return SX1262_ERR_PARAM;
  }

  const void *hal = s_config->hal;

  /* Coming from continuous RX: leave it and restore the TX IRQ mask. */
  if (s_rx_active) {
    if ((sx126x_set_standby(hal, SX126X_STANDBY_CFG_XOSC) != SX126X_STATUS_OK) ||
        (set_irq_tx() != SX126X_STATUS_OK)) {
      return SX1262_ERR_CHIP;
    }
    s_rx_active = false;
  }

  /* 1. payloadLen = real length of THIS packet (explicit header). */
  if ((set_pkt_params(length) != SX126X_STATUS_OK) ||
      /* 2. */
      (sx126x_set_buffer_base_address(hal, 0x00, 0x00) != SX126X_STATUS_OK) ||
      /* 3. */
      (sx126x_write_buffer(hal, 0x00, payload, length) != SX126X_STATUS_OK) ||
      /* 4. */
      (sx126x_clear_irq_status(hal, SX126X_IRQ_ALL) != SX126X_STATUS_OK) ||
      /* 5. TxClamp has to be re-armed before every TX. */
      (sx126x_cfg_tx_clamp(hal) != SX126X_STATUS_OK)) {
    return SX1262_ERR_CHIP;
  }

  /* 6. Antenna switch to TX, LNA off. */
  s_config->set_tx();

  /* 7. */
  if (sx126x_set_tx(hal, SX1262_TX_TIMEOUT_MS) != SX126X_STATUS_OK) {
    return SX1262_ERR_CHIP;
  }
  uint32_t start = s_config->get_tick_ms();

  /* 8. Poll GetIrqStatus until TX_DONE or TIMEOUT. */
  sx1262_result_t result = SX1262_ERR_NO_IRQ;
  while ((s_config->get_tick_ms() - start) <
         (SX1262_TX_TIMEOUT_MS + SX1262_TX_POLL_MARGIN_MS)) {
    sx126x_irq_mask_t irq = 0;
    if (sx126x_get_irq_status(hal, &irq) != SX126X_STATUS_OK) {
      result = SX1262_ERR_CHIP;
      break;
    }
    if ((irq & SX126X_IRQ_TX_DONE) != 0U) {
      result = SX1262_OK;
      break;
    }
    if ((irq & SX126X_IRQ_TIMEOUT) != 0U) {
      result = SX1262_ERR_TX_TIMEOUT;
      break;
    }
    s_config->delay_ms(SX1262_TX_POLL_PERIOD_MS);
  }

  if (tx_time_ms != NULL) {
    *tx_time_ms = s_config->get_tick_ms() - start;
  }

  /* No event at all: the chip is stuck in TX, take it out. */
  if (result == SX1262_ERR_NO_IRQ) {
    (void)sx126x_set_standby(hal, SX126X_STANDBY_CFG_XOSC);
  }

  /* 9. */
  (void)sx126x_clear_irq_status(hal, SX126X_IRQ_ALL);

  return result;
}

sx1262_result_t sx1262_start_rx(void) {
  if (s_config == NULL) {
    return SX1262_ERR_PARAM;
  }

  const void *hal = s_config->hal;

  /* Explicit header: the real length comes in the packet. */
  if ((set_pkt_params(SX1262_PAYLOAD_MAX_LEN) != SX126X_STATUS_OK) ||
      (sx126x_cfg_rx_boosted(hal, true) != SX126X_STATUS_OK) ||
      (sx126x_set_dio_irq_params(hal, SX126X_IRQ_RX_DONE | SX126X_IRQ_CRC_ERROR,
                                 SX126X_IRQ_RX_DONE | SX126X_IRQ_CRC_ERROR,
                                 SX126X_IRQ_NONE, SX126X_IRQ_NONE) !=
       SX126X_STATUS_OK) ||
      (sx126x_set_buffer_base_address(hal, 0x00, 0x00) != SX126X_STATUS_OK) ||
      (sx126x_clear_irq_status(hal, SX126X_IRQ_ALL) != SX126X_STATUS_OK)) {
    return SX1262_ERR_CHIP;
  }

  /* Antenna switch to RX, LNA on. */
  s_config->set_rx();

  /* Continuous RX: no re-arming between packets. */
  if (sx126x_set_rx_with_timeout_in_rtc_step(hal, SX126X_RX_CONTINUOUS) !=
      SX126X_STATUS_OK) {
    return SX1262_ERR_CHIP;
  }

  s_rx_active = true;
  return SX1262_OK;
}

/* Ultimo GetIrqStatus leido: 0xFFFF suele significar que el SPI no responde
 * (todos los bits a 1), no que hayan llegado todas esas interrupciones. */
static uint16_t s_last_irq_mask;

uint16_t sx1262_last_irq_mask(void) { return s_last_irq_mask; }

sx1262_rx_event_t sx1262_poll_rx(uint8_t *out_buf, uint8_t *out_length,
                                 sx1262_pkt_status_t *out_status) {
  if ((s_config == NULL) || (out_buf == NULL) || (out_length == NULL) ||
      !s_rx_active) {
    return SX1262_RX_NONE;
  }

  const void *hal = s_config->hal;

  sx126x_irq_mask_t irq = 0;
  if (sx126x_get_irq_status(hal, &irq) != SX126X_STATUS_OK) {
    return SX1262_RX_ERROR;
  }
  s_last_irq_mask = (uint16_t)irq;

  /* Todos los bits a 1 no son 16 interrupciones a la vez: es una lectura SPI
   * sin respuesta (MISO en alto). Si se tomara por buena, el bit de CRC
   * estaria siempre puesto y cada vuelta del bucle daria un paquete malo. */
  if ((uint16_t)irq == 0xFFFFU) {
    return SX1262_RX_ERROR;
  }

  /* On a corrupted packet the chip raises CRC_ERROR together with RX_DONE:
   * check it first so a bad payload is never reported as valid. */
  if ((irq & SX126X_IRQ_CRC_ERROR) != 0U) {
    (void)sx126x_clear_irq_status(hal, SX126X_IRQ_ALL);
    return SX1262_RX_CRC_ERROR;
  }

  if ((irq & SX126X_IRQ_RX_DONE) == 0U) {
    return SX1262_RX_NONE;
  }

  sx1262_rx_event_t event = SX1262_RX_PACKET;

  sx126x_rx_buffer_status_t buf_status;
  if ((sx126x_get_rx_buffer_status(hal, &buf_status) != SX126X_STATUS_OK) ||
      (sx126x_read_buffer(hal, buf_status.buffer_start_pointer, out_buf,
                          buf_status.pld_len_in_bytes) != SX126X_STATUS_OK)) {
    event = SX1262_RX_ERROR;
  } else {
    *out_length = buf_status.pld_len_in_bytes;
  }

  if (event == SX1262_RX_PACKET) {
    sx126x_pkt_status_lora_t raw;
    if (sx126x_get_lora_pkt_status(hal, &raw) != SX126X_STATUS_OK) {
      event = SX1262_RX_ERROR;
    } else if (out_status != NULL) {
      out_status->rssi_dbm = raw.rssi_pkt_in_dbm;
      out_status->snr_db = raw.snr_pkt_in_db;
      out_status->signal_rssi_dbm = raw.signal_rssi_pkt_in_dbm;
      out_status->freq_error_hz = read_freq_error_hz();
    }
  }

  (void)sx126x_handle_rx_done(hal);
  (void)sx126x_clear_irq_status(hal, SX126X_IRQ_ALL);

  return event;
}

bool sx1262_chip_status(sx1262_chip_status_t *out) {
  if ((s_config == NULL) || (out == NULL)) {
    return false;
  }

  sx126x_chip_status_t raw;
  if (sx126x_get_status(s_config->hal, &raw) != SX126X_STATUS_OK) {
    return false;
  }
  if (raw.chip_mode == 0x00) {
    return false; /* chip did not respond */
  }

  out->chip_mode = (uint8_t)raw.chip_mode;
  out->cmd_status = (uint8_t)raw.cmd_status;
  return true;
}

bool sx1262_get_device_errors(uint16_t *out_errors) {
  if ((s_config == NULL) || (out_errors == NULL)) {
    return false;
  }

  sx126x_errors_mask_t raw;
  if (sx126x_get_device_errors(s_config->hal, &raw) != SX126X_STATUS_OK) {
    return false;
  }

  *out_errors = (uint16_t)raw;
  (void)sx126x_clear_device_errors(s_config->hal);
  return true;
}

/*
 * LoRa time on air (datasheet §6.1.4), explicit header, LDRO off:
 *   Tsym     = 2^SF / BW
 *   preamble = (Npreamble + 4.25) symbols
 *   payload  = 8 + ceil((8*PL - 4*SF + 28 + 16*CRC) / (4*SF)) * (CR + 4)
 * Example: SF7, BW125, CR4/5, preamble 8, CRC on, 12 bytes -> 41.2 ms.
 */
uint32_t sx1262_time_on_air_us(uint8_t payload_length) {
  if (s_config == NULL) {
    return 0U;
  }

  const int32_t sf = (int32_t)s_config->radio.sf;
  const int32_t cr = (int32_t)s_config->radio.cr; /* 1..4 = 4/5..4/8 */
  const int32_t crc_bits = SX1262_CRC_ON ? 16 : 0;

  int32_t num = 8 * (int32_t)payload_length - 4 * sf + 28 + crc_bits;
  int32_t den = 4 * sf;
  int32_t blocks = (num > 0) ? ((num + den - 1) / den) : 0;
  int32_t payload_symbols = 8 + blocks * (cr + 4);

  /* In quarter symbols to keep the 4.25 exact */
  uint64_t quarter_symbols = (uint64_t)(4 * (int32_t)SX1262_PREAMBLE_SYMBOLS +
                                        17 + 4 * payload_symbols);
  uint64_t symbol_us =
      ((uint64_t)1U << sf) * 1000000ULL / lora_bw_in_hz(s_config->radio.bw);

  return (uint32_t)((quarter_symbols * symbol_us) / 4U);
}

void sx1262_dio1_irq_handler(void) { sx1262_isr_count++; }
