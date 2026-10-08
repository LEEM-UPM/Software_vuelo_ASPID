/**
 ******************************************************************************
 * @file    canCommand.c
 * @brief   Reception of the one-byte command frame the core H7 board sends
 *          over the telemetry CAN bus.
 ******************************************************************************
 */

#include "canCommand.h"

/* G4 FDCAN message RAM elements are always 64 bytes, and
 * HAL_FDCAN_GetRxMessage() copies as many bytes as the frame's DLC says --
 * so the buffer must fit the largest frame, even though only 1 byte of a
 * valid command is used. */
#define CAN_COMMAND_RX_BUFFER_BYTES 64u

static FDCAN_HandleTypeDef *canCommandHandle;

/* Written in interrupt context (CanCommand_OnRxFifo0()), read from the main
 * loop -- see CanCommand_TakeLatest() for how the two are kept consistent. */
static volatile uint8_t latestCommand;
static volatile bool commandPending;
static volatile uint32_t lastRxTick;
static volatile uint32_t rxCount;

/* === API =================================================================== */

bool CanCommand_Init(FDCAN_HandleTypeDef *hfdcan) {
  canCommandHandle = hfdcan;

  FDCAN_FilterTypeDef filter = {0};
  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterIndex = 0; /* hfdcan2.Init.StdFiltersNbr = 1 in fdcan.c */
  filter.FilterType = FDCAN_FILTER_DUAL;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1 = CAN_COMMAND_ID;
  filter.FilterID2 = CAN_COMMAND_ID; /* dual filter: both slots, same ID */
  if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK) {
    return false;
  }

  return HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                        0) == HAL_OK;
}

void CanCommand_OnRxFifo0(FDCAN_HandleTypeDef *hfdcan) {
  if (hfdcan != canCommandHandle) {
    return;
  }

  /* Drain the FIFO rather than read a single frame: if more than one
   * arrived before this ran, only the newest matters. Checking the fill
   * level first keeps GetRxMessage() from ever hitting an empty FIFO (which
   * it would flag in hfdcan->ErrorCode). */
  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0u) {
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[CAN_COMMAND_RX_BUFFER_BYTES];
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, data) !=
        HAL_OK) {
      return;
    }

    /* The hardware filter only lets CAN_COMMAND_ID through; still reject
     * anything malformed rather than trust that blindly. */
    if (header.IdType != FDCAN_STANDARD_ID ||
        header.Identifier != CAN_COMMAND_ID ||
        header.RxFrameType != FDCAN_DATA_FRAME ||
        header.DataLength == FDCAN_DLC_BYTES_0) {
      continue;
    }

    latestCommand = data[0];
    commandPending = true;
    lastRxTick = HAL_GetTick();
    rxCount++;
  }
}

bool CanCommand_TakeLatest(uint8_t *command) {
  /* Value and flag are taken together with interrupts masked, so a frame
   * landing in between can't be half-read or have its flag cleared unseen. */
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  bool pending = commandPending;
  uint8_t value = latestCommand;
  commandPending = false;
  __set_PRIMASK(primask);

  if (pending) {
    *command = value;
  }
  return pending;
}

bool CanCommand_IsLinkAlive(void) {
  if (rxCount == 0u) {
    return false;
  }
  /* Unsigned subtraction: correct across the HAL tick wraparound. */
  return (HAL_GetTick() - lastRxTick) < CAN_COMMAND_LINK_TIMEOUT_MS;
}

uint32_t CanCommand_RxCount(void) { return rxCount; }

/* === HAL interrupt callbacks ============================================= */

/* Rx FIFO 0 of any FDCAN with that interrupt enabled. Only FDCAN2 has it
 * (the H7's command frame, the only one its filter lets through): CAN1 is
 * polled by canNode.c. If another module ever needs this interrupt, the
 * dispatch goes here. */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs) {
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0u) {
    CanCommand_OnRxFifo0(hfdcan);
  }
}
