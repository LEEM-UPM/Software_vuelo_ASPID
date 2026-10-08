/**
 * @file    bsp_sx1262.c
 * @brief   Board-specific configuration for the SX1262 on the RF board.
 *
 * This is the ONLY file in the project allowed to reference:
 *   - Concrete SPI handles       (hspi1)
 *   - Concrete GPIO ports/pins   (SX1262_CS_GPIO_Port, etc.)
 *   - RF switch GPIO control     (SX1262_SW_CTRL_GPIO_Port)
 *   - External LNA enable        (SX1262_LNA_EN_GPIO_Port)
 *   - Radio link parameters      (frequency, SF, BW, PA power)
 *
 * If the hardware changes (pin reassignment, new board revision, different
 * frequency band), this is the only file that needs to be modified.
 *
 * Hardware (verified on the board):
 *   - SX1262IMLTRT with a 32 MHz CRYSTAL on XTA/XTB. There is NO TCXO: never
 *     call SetDIO3AsTcxoCtrl. The 10 pF external load capacitors the board
 *     came with (C37 and C38) HAVE TO BE REMOVED: the SX1262 has its own
 *     trimming capacitors and, with both, the oscillator ran at its limit --
 *     everything transmitted and received came out with so much phase noise
 *     that no packet passed the CRC (SNR 0 dB at any power). Without them,
 *     SNR is +12 dB and the trim (radio.xtal_trim) works.
 *   - Regulator: LDO. The DC-DC node (DCC_SW, L7 15 uH, C41 470 nF) exists
 *     but is not reliable on this board.
 *   - SPI1 PA5/PA6/PA7 (AF5), mode 0, MSB first, /2 = 8 MHz, NSS by
 *     software on PA2. NRESET PA0, BUSY PA1, DIO1 PC15 (EXTI15, rising).
 *   - DIO2 (PC14) and DIO3 (PC13) are plain MCU inputs: DIO2 is NOT the RF
 *     switch control and DIO3 is NOT a TCXO supply.
 *   - Antenna switch CTRL (U7, 4259-63) on PB6, driven by the MCU.
 *     LNA_ENABLE on PA3. The external LNA (U8, TSY-83LN+) is deliberately
 *     kept in BYPASS: it never amplifies on this board and enabling it costs
 *     15 dB. Full measurements and reasoning above board_lna_bypass().
 *
 * Nothing above the BSP layer (drivers, services, app) may include
 * this file or reference anything defined here.
 */

#include "bsp_sx1262.h"
#include "board_internal.h"
#include "gpio.h"
#include "spi.h"
#include "sx126x.h"
#include "sx126x_hal_context_stm32.h"

/* -------------------------------------------------------------------------
 * External LNA: kept in BYPASS on purpose
 *
 * The TSY-83LN+ (U8) never amplifies on this board. Measured on 2026-09-24
 * with the ping/echo range-test firmware, normalising each reading against
 * the opposite link direction (reciprocity cancels path and antennas, and
 * both numbers come from the same packet):
 *
 *   VEN = 0 V   (bypass)   -21 dB   <- the best of the three
 *   VEN = 3.3 V            -36 dB      15 dB worse
 *   VEN = 5 V              -40 dB      19 dB worse
 *
 * The root cause found in the schematic is that VEN is driven straight from
 * a 3.3 V MCU pin through R16 (0R), while the datasheet requires VEN = VDD =
 * +5 V for Amplifier-ON mode (note 11). At 3.3 V the part sits in neither
 * state. But feeding a real +5 V by hand (R16 lifted) did NOT fix it -- it
 * came out 4 dB worse still -- so the level was only part of the story and
 * the device is most likely damaged. Pending: measure the +5 V line current
 * (datasheet: 76 mA amplifying, 3 mA in bypass, ~0 = dead).
 *
 * So the amplifier is kept in bypass, which costs ~1.3 dB of insertion loss
 * and is 15 dB BETTER than what the board was doing. Note that a healthy LNA
 * would buy about 5 dB of sensitivity, not its 21 dB of gain: what improves
 * sensitivity is the chain's noise figure, and the gain only makes the first
 * stage dominate it.
 *
 * None of this affects telemetry range: the LNA sits in the rocket's RECEIVE
 * path, and telemetry goes rocket -> ground. It only limits the uplink.
 *
 * When the hardware gets a level shifter for VEN (a 5 V-powered TTL-threshold
 * buffer, e.g. 74HCT1G125), restore the enable in set_rx() and keep the
 * bypass in set_tx(): the input takes +22 dBm amplifying and +29 dBm in
 * bypass, so dropping to bypass while transmitting at 22 dBm protects the
 * part -- which is what the mode is for.
 * ------------------------------------------------------------------------- */
static void board_lna_bypass(void) {
  HAL_GPIO_WritePin(SX1262_LNA_EN_GPIO_Port, SX1262_LNA_EN_Pin, GPIO_PIN_RESET);
}

/* -------------------------------------------------------------------------
 * RF switch control
 *
 * The 4259-63 switch (U7) routes the antenna between the TX path (PA
 * output, RF1) and the RX path (external LNA input, RF2). Its CTRL pin is
 * driven by PB6; the LNA has its own enable on PA3. DIO2 is NOT used for
 * this (no SetDIO2AsRfSwitchCtrl): if it were, nobody would drive the
 * switch and the PA would transmit into an undefined path.
 *
 * Single-pin control mode (VDD tied to +3V3, only CTRL driven):
 *   CTRL = HIGH → RFC connected to RF1 → TX path active
 *   CTRL = LOW  → RFC connected to RF2 → RX path active
 * ------------------------------------------------------------------------- */
/*
 * The LNA is left in BYPASS in both directions -- measured 2026-09-24, see the
 * note above board_lna_bypass().
 */
static void board_rf_switch_set_tx(void) {
  HAL_GPIO_WritePin(SX1262_SW_CTRL_GPIO_Port, SX1262_SW_CTRL_Pin, GPIO_PIN_SET);
  board_lna_bypass();
}

static void board_rf_switch_set_rx(void) {
  HAL_GPIO_WritePin(SX1262_SW_CTRL_GPIO_Port, SX1262_SW_CTRL_Pin, GPIO_PIN_RESET);
  board_lna_bypass();
}

/* -------------------------------------------------------------------------
 * Time base for the driver (it cannot include the STM32 HAL)
 * ------------------------------------------------------------------------- */
static uint32_t board_get_tick_ms(void) { return HAL_GetTick(); }

static void board_delay_ms(uint32_t ms) { HAL_Delay(ms); }

/* -------------------------------------------------------------------------
 * IRQ registration
 *
 * The driver polls GetIrqStatus; DIO1 only feeds a diagnostic edge counter
 * (sx1262_isr_count) through the EXTI dispatch table in board.c.
 * ------------------------------------------------------------------------- */
static void bsp_sx1262_register_irq_handler(void (*handler)(void)) {
  board_register_exti_callback(SX1262_DIO1_Pin, (board_exti_callback_t)handler);
}

/* -------------------------------------------------------------------------
 * HAL context
 *
 * Pin assignments must match the CubeMX configuration in rf_g4_v0.ioc.
 * ------------------------------------------------------------------------- */
static const sx126x_hal_context_t s_hal_ctx = {
    .spi = &hspi1,
    .nss = {.port = SX1262_CS_GPIO_Port, .pin = SX1262_CS_Pin},
    .reset = {.port = SX1262_RESET_GPIO_Port, .pin = SX1262_RESET_Pin},
    .busy = {.port = SX1262_BUSY_GPIO_Port, .pin = SX1262_BUSY_Pin},
    .dio1 = {.port = SX1262_DIO1_GPIO_Port, .pin = SX1262_DIO1_Pin},
};

/* -------------------------------------------------------------------------
 * Radio configuration
 *
 * Verified on the board: 26/26 transmissions, no device errors, TX_DONE at
 * 41 ms vs 41.2 ms expected for a 12-byte payload. Do not replace these
 * values with the "typical" ones from examples.
 *
 *   869.525 MHz, image calibration band 863..870 MHz.
 *
 *   BAND: 869.4-869.65 MHz (ERC 70-03 / EN 300 220): 500 mW (27 dBm) e.r.p.
 *   and 10 % duty cycle, against 25 mW and 1 % in 868.0-868.6 MHz where this
 *   started. BW125 centred at 869.525 MHz occupies 869.4625-869.5875 MHz, so
 *   it fits with margin (it is the channel LoRaWAN uses for RX2).
 *   SF7 + BW125 + CR4/5 (LDRO off, set in the driver).
 *   +22 dBm, el maximo del SX1262, con SetPaConfig 0x04/0x07 (el driver solo
 *     acepta +14/+17/+20/+22, cada uno con los valores de PA del datasheet).
 *     Consume unos 118 mA transmitiendo, frente a los ~45 mA de +14 dBm: la
 *     alimentacion de la placa tiene que aguantarlo sin hundirse.
 *     La banda 869,4-869,65 permite 500 mW e.r.p. (27 dBm), asi que sobra
 *     margen legal; 22 dBm son 158 mW.
 *   Sync word 0x34. The receiver must use the same value.
 * ------------------------------------------------------------------------- */
static const sx1262_config_t s_sx1262_config = {
    .hal = &s_hal_ctx,
    .set_tx = board_rf_switch_set_tx,
    .set_rx = board_rf_switch_set_rx,
    .get_tick_ms = board_get_tick_ms,
    .delay_ms = board_delay_ms,
    .register_irq_handler = bsp_sx1262_register_irq_handler,
    .radio =
        {
            .frequency_hz = 869525000U,
            .cal_img_freq1_mhz = 863U,
            .cal_img_freq2_mhz = 870U,
            .sf = SX126X_LORA_SF7,
            /*
             * BW 250 kHz: la mitad de tiempo en el aire que 125 (un paquete
             * de 20 B pasa de ~57 a ~28 ms) a cambio de 3 dB de
             * sensibilidad, o sea un 71 % del alcance.
             *
             * OJO: la banda de 869,4 a 869,65 MHz mide 250 kHz JUSTOS, asi
             * que centrados en 869,525 se ocupa de borde a borde y no queda
             * guarda ninguna. Funciona (un ancho de banda mayor tolera mejor
             * el error de frecuencia, y medimos 5,4 kHz entre placas), pero
             * regulatoriamente esta al limite. Para volar de verdad,
             * reconsiderar 125 kHz o bajar el SF en su lugar.
             */
            .bw = SX126X_LORA_BW_250,
            .cr = SX126X_LORA_CR_4_5,
            .tx_power_dbm = 22,
            .sync_word = 0x34,
            /* Medido contra la estacion de tierra el 2026-09-22, cada paso
             * son ~5,4 ppm. A 22 dBm, 0x10 deja el error en +1,2 ppm; 0x0F
             * seria -4,8 ppm. A potencias bajas el optimo era 0x0F: los
             * 118 mA de la transmision hunden un poco la alimentacion y tiran
             * del cristal unos -4 kHz. Vale solo con los condensadores
             * externos del cristal (C37 y C38) quitados: con ellos puestos el
             * oscilador iba al limite, con tanto ruido de fase que ningun
             * paquete pasaba el CRC. */
            .xtal_trim = 0x10,
        },
};

const sx1262_config_t *bsp_sx1262_get_config(void) { return &s_sx1262_config; }
