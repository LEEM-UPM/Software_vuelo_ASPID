/**
 * @file    radio_beacon.h
 * @brief   SX1262 (LoRa) transmission: periodic test packet and packets
 *          commanded by the core.
 *
 * Sends a 12-byte packet (4-byte sequence number, big-endian, + "LEEM-RF!")
 * every RADIO_BEACON_PERIOD_MS with the verified radio configuration in
 * bsp/sx1262/bsp_sx1262.c (869.525 MHz, SF7, BW125, CR4/5, sync word 0x34).
 * Each packet takes ~41 ms on air; sx1262_transmit() blocks for that long.
 *
 * DUTY CYCLE: 869.4-869.65 MHz allows 10 % duty cycle. 41 ms every 1 s is
 * ~4.1 %, within the limit -- but it is shared with whatever the core asks
 * for with CMD_RF_TRANSMIT, so keep an eye on the total.
 */
#ifndef APP_RADIO_BEACON_H_
#define APP_RADIO_BEACON_H_

#include <stdbool.h>
#include <stdint.h>

#include "sx1262.h"

#ifdef __cplusplus
extern "C" {
#endif

#define RADIO_BEACON_PERIOD_MS 1000U

/**
 * One console line every this many transmitted packets. NOT one per packet:
 * at 15 Hz that would be 15 blocking UART writes per second, and the radio is
 * not being polled while the CPU is inside one of them.
 */
#define RADIO_TX_LOG_EVERY 150U

/**
 * @brief Enter continuous receive: the uplink, ground -> rocket.
 *
 * Call once after radio_beacon_init(). From then on the radio listens
 * whenever it is not transmitting; transmit() puts it back into RX on its
 * own when it is done.
 *
 * @return false if the radio is not configured or could not enter RX.
 */
bool radio_listen(void);

/**
 * @brief Check for a packet received from the air.
 *
 * Call often: the chip is polled over SPI, there is no interrupt driving
 * this. Packets that fail the LoRa CRC are counted and discarded here, they
 * never reach the caller.
 *
 * @param out_status  if not NULL, how well that packet was received (RSSI,
 *                    SNR, frequency error); only valid when it returns true.
 * @return true if a good packet was copied into buf.
 */
bool radio_receive_poll(uint8_t *buf, uint8_t buf_size, uint8_t *out_len,
                        sx1262_pkt_status_t *out_status);

uint32_t radio_rx_count(void);       /**< good packets received  */
uint32_t radio_rx_crc_errors(void);  /**< arrived, but corrupt   */

/**
 * @brief Initialise and configure the SX1262.
 * @return false if the radio could not be configured (the reason is printed
 *         to the console). radio_beacon_update() then does nothing.
 */
bool radio_beacon_init(void);

/**
 * @brief Send one packet right away, as given.
 *
 * Used for CMD_RF_TRANSMIT (ICD 6.10): the bytes the core sends over CAN go
 * on the air untouched. Blocks until TX_DONE (~41 ms for 12 bytes, more the
 * longer the payload). Counts towards the same ok/fail counters.
 *
 * @return true if the packet was transmitted (radio configured and TX_DONE).
 */
bool radio_transmit(const uint8_t *payload, uint8_t length);

/**
 * @brief Send the next packet if the period has elapsed.
 *
 * Call on every pass of the main loop.
 * @return true if a packet was transmitted successfully in this call.
 */
bool radio_beacon_update(void);

/* Diagnostics (watch them in the debugger). */
uint32_t radio_beacon_ok_count(void);
uint32_t radio_beacon_fail_count(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_RADIO_BEACON_H_ */
