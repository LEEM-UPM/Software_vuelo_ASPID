/**
 * @file    bsp_can1.h
 * @brief   Board-specific access to CAN1 (FDCAN1), the common bus between
 *          boards.
 *
 * This is the ONLY module allowed to touch the raw FDCAN1 HAL handle
 * (hfdcan1) and its filter configuration. Everything above this layer
 * works with plain uint32_t IDs and byte buffers — no HAL types leak out,
 * matching the rule in board.h.
 *
 * Bus parameters (third_party/can_protocol ICD, section 2): 1 Mbit/s, CAN FD
 * without BRS, 11-bit IDs. Bit timing is owned by CubeMX / MX_FDCAN1_Init()
 * in Core/Src/fdcan.c, which also checks it against the ICD at start-up.
 * This module only adds what CubeMX does not generate: the acceptance
 * filter, starting the peripheral, and frame TX/RX.
 *
 * What the IDs mean is not decided here: that is the job of the layer above
 * (services/can_node), which follows the ICD.
 *
 * RX is polling-based by design (no FDCAN NVIC interrupt is wired up):
 * call bsp_can1_poll_rx() from the main loop. The G4's RX FIFO only holds 3
 * frames, so poll it on every pass.
 */
#ifndef BSP_CAN_BSP_CAN1_H_
#define BSP_CAN_BSP_CAN1_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Largest CAN FD data field. Both TX and RX buffers use this size: HAL
 *  copies as many bytes as the received DLC says. */
#define BSP_CAN1_MAX_DATA_BYTES 64U

/** One data frame, standard (11-bit) identifier only. */
typedef struct {
  uint32_t id;                            /**< Standard identifier, 0-0x7FF. */
  uint8_t len;                            /**< Data bytes, per the DLC.      */
  uint8_t data[BSP_CAN1_MAX_DATA_BYTES];  /**< Payload, len bytes valid.     */
} bsp_can1_frame_t;

/**
 * @brief Arm the acceptance filter and start FDCAN1.
 *
 * Must be called once, after board_init() (which runs MX_FDCAN1_Init() but
 * does not start the peripheral — filters must be set before starting).
 *
 * One standard-ID mask filter (hfdcan1.Init.StdFiltersNbr = 1): a frame is
 * accepted if (id & mask) == (filter_id & mask). Everything else — other
 * standard IDs, all extended IDs and remote frames — is rejected in
 * hardware.
 *
 * @return true if the filter was set and the peripheral started.
 */
bool bsp_can1_start(uint32_t filter_id, uint32_t filter_mask);

/**
 * @brief Pop one received frame (non-blocking).
 *
 * @param[out] out_frame  Oldest pending frame on success.
 * @return true  if a frame was waiting and has been popped.
 * @return false if out_frame is NULL, the RX FIFO was empty or the HAL
 *               failed to read it.
 */
bool bsp_can1_poll_rx(bsp_can1_frame_t *out_frame);

/**
 * @brief Queue a data frame: standard ID, CAN FD format, no BRS.
 *
 * If len is not a valid CAN FD length it is zero-padded up to the next one
 * (e.g. 10 -> 12), as the ICD requires. Non-blocking: returns once the
 * frame is in the TX FIFO, not when it has gone out on the bus.
 *
 * @param data  Payload bytes. May be NULL only if len == 0.
 * @param len   0-64 bytes.
 * @return false if the arguments are invalid or the TX FIFO is full.
 */
bool bsp_can1_send(uint32_t id, const uint8_t *data, uint8_t len);

/**
 * @brief Bring FDCAN1 back from bus-off, if it is there.
 *
 * A node whose transmit error counter passes 255 (some 32 failed frames in a
 * row: a bad contact, CANH-CANL short, noise) disconnects itself from the
 * bus. The STM32 FDCAN then stays in INIT and DOES NOT come back on its own:
 * software has to clear CCCR.INIT, after which it waits for 128 idle
 * sequences (~1.4 ms at 1 Mbit/s) and rejoins.
 *
 * Call on every pass of the main loop -- also while waiting for the wake
 * up, which would otherwise never arrive. Two register reads.
 *
 * @return true if it has just pulled FDCAN1 out of bus-off (once per event).
 */
bool bsp_can1_recover_bus_off(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_CAN_BSP_CAN1_H_ */
