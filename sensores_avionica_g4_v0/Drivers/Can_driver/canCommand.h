/**
 ******************************************************************************
 * @file    canCommand.h
 * @brief   Reception of the one-byte command frame the core H7 board sends
 *          over the telemetry CAN bus.
 *
 * The frame (as transmitted by the H7, STM32H723):
 *  - CAN FD without BRS, standard ID CAN_COMMAND_ID (0x200), data frame.
 *  - 1 data byte: a uint8_t, 0..255. No header, timestamp or padding.
 *  - Sent every 20 ms (50 Hz, +-1 ms), with automatic retransmission on the
 *    H7 side. Frames can still go missing; the latest one received wins.
 *  - Test mode at the time of writing: toggles between 0 and 255 once a
 *    second. Nothing here assumes those are the only values.
 *
 * 0x200 sits above the telemetry range (0x101-0x105, see canTelemetry.h) on
 * purpose, so the sensor frames -- which are never retransmitted -- always
 * win arbitration against it.
 *
 * Receiving is interrupt-driven and never blocks the transmit side: the Rx
 * FIFO 0 interrupt (HAL_FDCAN_RxFifo0Callback(), forwarded to
 * CanCommand_OnRxFifo0()) only copies the byte out and raises a flag; the
 * main loop picks it up with CanCommand_TakeLatest().
 ******************************************************************************
 */

#ifndef __CANCOMMAND_H__
#define __CANCOMMAND_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "main.h"

/* Standard CAN identifier of the H7's command frame. */
#define CAN_COMMAND_ID 0x200u

/* No frame for this long means the link is down: 5 missed 20 ms periods. */
#define CAN_COMMAND_LINK_TIMEOUT_MS 100u

/* === API ================================================================== */

/* Whitelists CAN_COMMAND_ID into Rx FIFO 0 (standard filter index 0) and
 * enables the Rx FIFO 0 new-message interrupt. Everything else stays
 * rejected by the global filter CanTelemetry_Init() set up -- which is why
 * this must be called AFTER CanTelemetry_Init() on the same peripheral.
 * The FDCAN2_IT0 NVIC line itself is enabled in fdcan.c's MspInit. Returns
 * true on success. */
bool CanCommand_Init(FDCAN_HandleTypeDef *hfdcan);

/* Call from HAL_FDCAN_RxFifo0Callback() (interrupt context). Drains Rx
 * FIFO 0, keeping the last valid command frame. Ignores other peripherals,
 * so it is safe to forward every callback. */
void CanCommand_OnRxFifo0(FDCAN_HandleTypeDef *hfdcan);

/* If a command frame arrived since the previous call, writes its value to
 * `*command` and returns true; otherwise returns false and leaves `*command`
 * untouched. Only the latest value is kept -- frames received in between
 * two calls are superseded, not queued. */
bool CanCommand_TakeLatest(uint8_t *command);

/* True if a command frame arrived within the last
 * CAN_COMMAND_LINK_TIMEOUT_MS (false before the first one). */
bool CanCommand_IsLinkAlive(void);

/* Total command frames received since CanCommand_Init() (diagnostic). */
uint32_t CanCommand_RxCount(void);

#ifdef __cplusplus
}
#endif

#endif /* __CANCOMMAND_H__ */
