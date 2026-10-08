/**
 * @file    bsp_can1.c
 * @brief   Board-specific access to CAN1 (FDCAN1), the common bus between
 *          boards.
 *
 * See bsp_can1.h for the module's responsibilities and boundaries.
 */
#include "bsp_can1.h"

#include <stddef.h>
#include <string.h>

#include "fdcan.h"

/** Data bytes of each DLC code (in this HAL, FDCAN_DLC_BYTES_x = 0..15). */
static const uint8_t s_dlc_to_bytes[16] = {0,  1,  2,  3,  4,  5,  6,  7,
                                           8,  12, 16, 20, 24, 32, 48, 64};

bool bsp_can1_start(uint32_t filter_id, uint32_t filter_mask) {
  FDCAN_FilterTypeDef filter = {0};
  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterIndex = 0; /* hfdcan1.Init.StdFiltersNbr = 1 in fdcan.c */
  filter.FilterType = FDCAN_FILTER_MASK;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1 = filter_id;
  filter.FilterID2 = filter_mask;
  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) != HAL_OK) {
    return false;
  }

  /* The global filter can only be set before the peripheral is started. */
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
                                   FDCAN_REJECT_REMOTE,
                                   FDCAN_REJECT_REMOTE) != HAL_OK) {
    return false;
  }

  return HAL_FDCAN_Start(&hfdcan1) == HAL_OK;
}

bool bsp_can1_recover_bus_off(void) {
  /* With INIT cleared, BO stays set until the recovery ends: checking INIT
   * too counts each event once instead of once per loop pass. */
  if (((FDCAN1->PSR & FDCAN_PSR_BO) != 0U) &&
      ((FDCAN1->CCCR & FDCAN_CCCR_INIT) != 0U)) {
    CLEAR_BIT(FDCAN1->CCCR, FDCAN_CCCR_INIT);
    return true;
  }
  return false;
}

bool bsp_can1_poll_rx(bsp_can1_frame_t *out_frame) {
  if (out_frame == NULL) {
    return false;
  }

  /* Checking the fill level first keeps GetRxMessage() from ever hitting an
   * empty FIFO (which it would flag in hfdcan1.ErrorCode). */
  if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) == 0U) {
    return false;
  }

  FDCAN_RxHeaderTypeDef rx_header;
  if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rx_header,
                             out_frame->data) != HAL_OK) {
    return false;
  }

  out_frame->id = rx_header.Identifier;
  out_frame->len = s_dlc_to_bytes[rx_header.DataLength & 0xFU];
  return true;
}

bool bsp_can1_send(uint32_t id, const uint8_t *data, uint8_t len) {
  if ((len > BSP_CAN1_MAX_DATA_BYTES) || ((data == NULL) && (len > 0U))) {
    return false;
  }

  /* Smallest DLC that holds len; the padding stays at 0 */
  uint32_t dlc = 0U;
  while (s_dlc_to_bytes[dlc] < len) {
    dlc++;
  }

  uint8_t tx_data[BSP_CAN1_MAX_DATA_BYTES] = {0};
  if (len > 0U) {
    memcpy(tx_data, data, len);
  }

  FDCAN_TxHeaderTypeDef tx_header = {0};
  tx_header.Identifier = id;
  tx_header.IdType = FDCAN_STANDARD_ID;
  tx_header.TxFrameType = FDCAN_DATA_FRAME;
  tx_header.DataLength = dlc;
  tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx_header.BitRateSwitch = FDCAN_BRS_OFF; /* bus is FD without BRS */
  tx_header.FDFormat = FDCAN_FD_CAN;
  tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tx_header.MessageMarker = 0;

  return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, tx_data) ==
         HAL_OK;
}
