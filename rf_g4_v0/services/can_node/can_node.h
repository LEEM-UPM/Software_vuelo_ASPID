/**
 * @file    can_node.h
 * @brief   This board as a node of the common CAN1 bus between boards.
 *
 * Everything on CAN1 follows the ICD in third_party/can_protocol: 1 Mbit/s,
 * CAN FD without BRS, 11-bit IDs built with CAN_ID(), little-endian fields.
 * This board is CAN_NODE_RF (0x4).
 *
 * Start-up handshake (ICD 6.1 and 6.2):
 *  - The core broadcasts CMD_WAKE_UP (0x100, 2 bytes: its
 *    CAN_PROTOCOL_VERSION) at 10 Hz for up to 5 s after it boots.
 *  - This board waits until it gets one, answering every CMD_WAKE_UP with
 *    WAKE_UP_ACK (0x240, no data) -- also once it is already running, so a
 *    core that resets sees it again.
 *
 * Radio command (ICD 6.10):
 *  - CMD_RF_TRANSMIT (0x140, core -> rf): its data are the bytes to put on
 *    the air, copied verbatim into a LoRa packet. Only the latest one is
 *    kept: if a second arrives before the application has sent the first,
 *    the older one is dropped (see can_node_radio_dropped_count()).
 *
 * Accepted IDs: broadcast commands (0x100-0x10F) and commands to this board
 * (0x140-0x14F), with the single filter of ICD 5.3 (0x100 / 0x7B0).
 *
 * Layering: sits on top of bsp_can1 (raw frames). It contains no STM32 HAL
 * references.
 */
#ifndef SERVICES_CAN_NODE_CAN_NODE_H_
#define SERVICES_CAN_NODE_CAN_NODE_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Arm the CAN1 filter for this node and start the peripheral.
 *
 * Call once, after board_init().
 * @return true on success.
 */
bool can_node_init(void);

/**
 * @brief Drain the CAN1 RX FIFO and handle what arrived.
 *
 * Every valid CMD_WAKE_UP is answered right away with WAKE_UP_ACK. Call on
 * every pass of the main loop: the G4's RX FIFO only holds 3 frames.
 */
void can_node_poll(void);

/**
 * @brief can_node_poll() and then: true if a valid CMD_WAKE_UP arrived since
 * the previous call. Meant for the start-up wait loop:
 *
 *   while (!can_node_poll_wake_up()) {}
 *
 * A CMD_WAKE_UP whose protocol version is not compatible
 * (CAN_PROTOCOL_IS_COMPATIBLE()) is neither answered nor reported, so the
 * board keeps waiting and the core sees it as not awake.
 */
bool can_node_poll_wake_up(void);

/** @brief true once a valid CMD_WAKE_UP has been received. */
bool can_node_is_awake(void);

/** Largest CMD_RF_TRANSMIT payload (maximum data length of the ICD). */
#define CAN_NODE_RADIO_PAYLOAD_MAX 32U

/**
 * @brief Takes the payload of the last CMD_RF_TRANSMIT, if any.
 *
 * @param buf      where the bytes are copied.
 * @param buf_size size of buf; the payload is dropped if it does not fit.
 * @param len      out: number of bytes copied.
 * @return true if there was a payload waiting (and it is consumed).
 */
bool can_node_take_radio_payload(uint8_t *buf, uint8_t buf_size, uint8_t *len);

/* Diagnostics (watch them in the debugger). */
/* RF_RECEIVED (ICD 6.12): header in front of the payload */
#define CAN_NODE_RX_HEADER_BYTES 5U
#define CAN_NODE_RX_PAYLOAD_MAX \
  (CAN_NODE_RADIO_PAYLOAD_MAX - CAN_NODE_RX_HEADER_BYTES) /* 27 */

/**
 * @brief Forward a packet received over the air to the core (RF_RECEIVED,
 *        ICD 6.12). This is the uplink: ground -> rocket.
 *
 * The payload goes up untouched, behind a 5-byte header with how well it
 * was received, so the ground can see the uplink from the rocket's side:
 *   [0] payload length   [1] RSSI, int8 dBm   [2] SNR, int8 dB
 *   [3..4] frequency error, int16 Hz little-endian (+ = ground transmits
 *          above us), saturated
 *   [5..] payload
 * The header carries the length because CAN FD pads the frame to the next
 * valid size. Only packets that passed the LoRa CRC should be passed here.
 *
 * @return false if not awake yet, the length is 0 or over
 *         CAN_NODE_RX_PAYLOAD_MAX, or the frame could not be queued.
 */
bool can_node_send_received(const uint8_t *payload, uint8_t length,
                            int8_t rssi_dbm, int8_t snr_db,
                            int32_t freq_error_hz);

uint32_t can_node_rx_forwarded_count(void); /**< packets passed to the core */
uint32_t can_node_rx_dropped_count(void);   /**< too long, asleep or TX full */

uint32_t can_node_wake_up_count(void);    /**< valid CMD_WAKE_UP received  */
uint32_t can_node_incompatible_count(void); /**< CMD_WAKE_UP, other version */
uint32_t can_node_tx_error_count(void);   /**< frames that could not be queued */
uint32_t can_node_radio_count(void);      /**< CMD_RF_TRANSMIT received      */
uint32_t can_node_radio_dropped_count(void); /**< overwritten or too long    */
uint32_t can_node_bus_off_count(void);    /**< FDCAN1 pulled out of bus-off */

#ifdef __cplusplus
}
#endif

#endif /* SERVICES_CAN_NODE_CAN_NODE_H_ */
