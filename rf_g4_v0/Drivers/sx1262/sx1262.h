/**
 * @file    sx1262.h
 * @brief   Mid-level driver for the SX1262 sub-GHz radio chip.
 *
 *   The Semtech sx126x_driver is a thin, chip-level library. It knows how
 *   to encode every command the chip understands, but it has no opinion on
 *   how those commands should be sequenced. This module owns:
 *     - The initialisation sequence verified on this board (see
 *       sx1262_configure()).
 *     - The per-packet TX sequence and the continuous RX loop.
 *     - The RF switch control, delegated to the BSP via callbacks.
 *
 *   Crucially, this module contains NO board-specific pin references.
 *   All hardware access is performed through the sx1262_config_t that the
 *   BSP constructs and injects via sx1262_init().
 *
 *   Things this driver deliberately NEVER does (each one is a real failure
 *   already debugged on the RF board):
 *     - SetDIO2AsRfSwitchCtrl: DIO2 goes to the MCU; the antenna switch is
 *       driven by a GPIO through the set_tx/set_rx callbacks.
 *     - SetDIO3AsTcxoCtrl: the reference is a 32 MHz crystal, not a TCXO.
 *       Configuring a TCXO makes calibration fail and BUSY stay high forever.
 *     - SetPaConfig outside the datasheet table: only +14/+17/+20/+22 dBm
 *       are accepted (sx1262_configure() fails otherwise).
 *     - A fixed payloadLen in the packet params: with an explicit header it
 *       is that length that goes on air, so it is set before every TX.
 *
 *   Dependency rule:
 *   This file MUST NOT include any STM32 HAL headers, gpio.h, spi.h, or
 *   any BSP header. The only allowed external includes are:
 *     - sx126x.h           (Semtech driver public API)
 *     - Standard C headers (stdint.h, stdbool.h, stddef.h)
 */

#ifndef DRIVERS_SX1262_SX1262_H_
#define DRIVERS_SX1262_SX1262_H_

#include "sx126x.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Callback type for RF switch control (implemented by the BSP). */
typedef void (*sx1262_rf_switch_fn_t)(void);

/**
 * @brief Complete configuration for the SX1262 driver.
 *
 * Constructed by the BSP and passed once to sx1262_init().
 * Must remain valid for the entire lifetime of the driver.
 */
typedef struct {
  /** Hardware context for the Semtech sx126x HAL (SPI, NSS, RESET, BUSY). */
  const void *hal;

  /**
   * Antenna path, board-specific. set_tx routes the antenna to the PA and
   * disables the LNA; set_rx routes it to the LNA and enables it.
   * Both are mandatory: this board has no DIO2-controlled switch.
   */
  sx1262_rf_switch_fn_t set_tx;
  sx1262_rf_switch_fn_t set_rx;

  /** Time base (the driver cannot include the STM32 HAL). Mandatory. */
  uint32_t (*get_tick_ms)(void);
  void (*delay_ms)(uint32_t ms);

  /**
   * Optional: connects sx1262_dio1_irq_handler() to the DIO1 EXTI. The
   * driver polls GetIrqStatus and does not depend on it; the handler only
   * counts edges (sx1262_isr_count) for diagnostics. May be NULL.
   */
  void (*register_irq_handler)(void (*handler)(void));

  struct {
    uint32_t frequency_hz;
    uint16_t cal_img_freq1_mhz; /**< CalibrateImage band, lower edge. */
    uint16_t cal_img_freq2_mhz; /**< CalibrateImage band, upper edge. */
    sx126x_lora_sf_t sf;
    sx126x_lora_bw_t bw;
    sx126x_lora_cr_t cr;
    int8_t tx_power_dbm; /**< Only +14, +17, +20 or +22. */
    uint8_t sync_word;
    /**
     * Crystal trimming capacitors (XTA and XTB), 0x00 to 0x2F in ~0.47 pF
     * steps. These ARE the crystal's load capacitors: the SX1262 has them
     * built in, so the board must not add external ones. The chip resets
     * this register to 0x12 every time the crystal starts, which is why the
     * driver keeps the oscillator running (fallback STDBY_XOSC) instead of
     * stopping it between packets.
     */
    uint8_t xtal_trim;
  } radio;
} sx1262_config_t;

/** Result of a driver operation. */
typedef enum {
  SX1262_OK = 0,
  SX1262_ERR_PARAM,      /**< Invalid argument or configuration.          */
  SX1262_ERR_CHIP,       /**< SPI failed or BUSY stuck high (100 ms).     */
  SX1262_ERR_TX_TIMEOUT, /**< The chip raised TIMEOUT (3 s SetTx timeout). */
  SX1262_ERR_NO_IRQ,     /**< Neither TX_DONE nor TIMEOUT arrived in time. */
} sx1262_result_t;

/** Snapshot of the SX1262 internal state, as reported by GetStatus. */
typedef struct {
  uint8_t chip_mode;  /**< 0x02 STDBY_RC, 0x03 STDBY_XOSC, 0x04 FS, 0x05 RX, 0x06 TX */
  uint8_t cmd_status; /**< 0x02 data available, 0x03 timeout, 0x04 error,
                           0x05 failure to execute, 0x06 TX done */
} sx1262_chip_status_t;

/** Link quality of the last packet received (GetPacketStatus). */
typedef struct {
  int8_t rssi_dbm;        /**< RSSI averaged over the last packet.        */
  int8_t snr_db;          /**< SNR of the last packet.                    */
  int8_t signal_rssi_dbm; /**< RSSI after despreading.                    */
  /**
   * Frequency error between this receiver and the transmitter, in Hz
   * (positive = the transmitter is above us). Read from the LoRa FEI
   * registers after RxDone; it is only meaningful for the packet that was
   * just received.
   *
   * What it is good for: a large, consistent offset means the two crystals
   * disagree, which costs sensitivity. At SF7/BW125 the receiver tolerates
   * roughly +/- 30 kHz, so anything past a few kHz is worth trimming
   * (see xtal_trim in bsp_sx1262.c).
   */
  int32_t freq_error_hz;
} sx1262_pkt_status_t;

/** What sx1262_poll_rx() found. */
typedef enum {
  SX1262_RX_NONE = 0,  /**< Nothing received yet.                         */
  SX1262_RX_PACKET,    /**< A packet was received and copied.             */
  SX1262_RX_CRC_ERROR, /**< A packet arrived with a bad CRC (discarded).  */
  SX1262_RX_ERROR,     /**< SPI failed or BUSY stuck high.                */
} sx1262_rx_event_t;

/** Maximum payload length supported by the LoRa packet format. */
#define SX1262_PAYLOAD_MAX_LEN 255U

/**
 * @brief Store the board configuration. Does not touch the hardware.
 * @return false if config or a mandatory field is NULL.
 */
bool sx1262_init(const sx1262_config_t *config);

/**
 * @brief Full initialisation sequence. The order matters:
 *
 *   1.  reset() (waits for BUSY)       9.  TxClampConfig (reg 0x08D8)
 *   2.  SetStandby(STDBY_RC)           10. SetOcpValue(0x38) = 140 mA
 *   3.  SetRegulatorMode(LDO)          11. SetTxParams(power, 40 µs ramp)
 *   4.  Calibrate(ALL)                 12. SetModulationParams(SF, BW, CR, LDRO off)
 *   5.  CalibrateImage(band)           13. SetLoRaSyncWord
 *   6.  SetPacketType(LORA)            14. SetRxTxFallbackMode(STDBY_RC)
 *   7.  SetRfFrequency                 15. SetDioIrqParams(TX_DONE | TIMEOUT)
 *   8.  SetPaConfig (datasheet table)  16. ClearIrqStatus(ALL) + ClearDeviceErrors
 *
 * Stops at the first failing step (see sx1262_get_failed_step()).
 */
sx1262_result_t sx1262_configure(void);

/** Step (1..16) where the last sx1262_configure() failed; 0 if none. */
uint8_t sx1262_get_failed_step(void);

/**
 * @brief Transmit one packet and wait for it to finish (blocking).
 *
 * Per packet: SetPacketParams (payloadLen = length), SetBufferBaseAddress,
 * WriteBuffer, ClearIrqStatus, TxClampConfig, antenna to TX, SetTx(3000 ms),
 * poll GetIrqStatus until TX_DONE or TIMEOUT, ClearIrqStatus.
 *
 * If the radio was left in RX by sx1262_start_rx(), it is first put in
 * STDBY_RC and the DIO IRQ mask is switched back to TX_DONE | TIMEOUT.
 *
 * @param[out] tx_time_ms  Time from SetTx until TX_DONE was seen. May be
 *                         NULL. Compare it with the expected time on air:
 *                         ~0 ms means the chip reported done without
 *                         actually transmitting.
 */
sx1262_result_t sx1262_transmit(const uint8_t *payload, uint8_t length,
                                uint32_t *tx_time_ms);

/**
 * @brief Enter continuous RX (no re-arming needed between packets).
 *
 * SetPacketParams (payloadLen = 255), CfgRxBoosted(true),
 * SetDioIrqParams(RX_DONE | CRC_ERROR), antenna to RX,
 * SetRx(0x00FFFFFF = continuous).
 */
sx1262_result_t sx1262_start_rx(void);

/**
 * @brief Check for a received packet (polls GetIrqStatus over SPI).
 *
 * On RX_DONE: GetRxBufferStatus, ReadBuffer from bufferStartPointer,
 * GetPacketStatus and sx126x_handle_rx_done() (stops the RX timer).
 *
 * @param[out] out_buf     At least SX1262_PAYLOAD_MAX_LEN bytes.
 * @param[out] out_length  Received length.
 * @param[out] out_status  RSSI/SNR of the packet. May be NULL.
 */
sx1262_rx_event_t sx1262_poll_rx(uint8_t *out_buf, uint8_t *out_length,
                                 sx1262_pkt_status_t *out_status);

/**
 * @brief Raw IRQ mask of the last sx1262_poll_rx() (GetIrqStatus).
 *
 * Diagnostics: 0xFFFF means the SPI read came back as all ones (the chip is
 * not answering), not that every interrupt fired at once.
 */
uint16_t sx1262_last_irq_mask(void);

/** @brief Read the current chip status (GetStatus). */
bool sx1262_chip_status(sx1262_chip_status_t *out);

/**
 * @brief Read and clear the chip's operation-error flags (GetDeviceErrors).
 *
 * This is the direct way to know why the chip is stuck. Most useful bits
 * (SX126X_ERRORS_*): XOSC_START -> crystal; PA_RAMP -> PA supply (VREG,
 * pin 7, should read ~1.55 V); PLL_LOCK -> synthesiser.
 */
bool sx1262_get_device_errors(uint16_t *out_errors);

/**
 * @brief Expected LoRa time on air, in microseconds, for the configured
 * SF/BW/CR with preamble 8, explicit header and CRC on (LDRO off).
 */
uint32_t sx1262_time_on_air_us(uint8_t payload_length);

/** @brief DIO1 EXTI handler: only counts edges (diagnostics). */
void sx1262_dio1_irq_handler(void);

/** Diagnostic counter — number of DIO1 edges observed since reset. */
extern volatile uint32_t sx1262_isr_count;

#ifdef __cplusplus
}
#endif

#endif /* DRIVERS_SX1262_SX1262_H_ */
