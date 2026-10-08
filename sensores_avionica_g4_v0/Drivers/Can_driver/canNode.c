/**
 ******************************************************************************
 * @file    canNode.c
 * @brief   This board as a node of the common CAN1 bus between boards.
 ******************************************************************************
 */

#include "canNode.h"

#include "can_protocol/can_ids.h"
#include "can_protocol/can_pack.h"
#include "can_protocol/can_protocol_version.h"

/* G4 FDCAN message RAM elements are always 64 bytes, and
 * HAL_FDCAN_GetRxMessage() copies as many bytes as the frame's DLC says --
 * so the buffer must fit the largest frame. */
#define CAN_NODE_RX_BUFFER_BYTES 64u

/* Lengths from the ICD */
#define CMD_WAKE_UP_LEN_DLC           FDCAN_DLC_BYTES_2
#define CMD_AVIONICS_CALIBRATE_DLC    FDCAN_DLC_BYTES_4
#define AVIONICS_CALIBRATION_LEN      32u

static FDCAN_HandleTypeDef *canNodeHandle;

/* Only touched from the main loop. */
static bool awake;
static bool wakeUpPending;
static bool calibrationPending;
static float calibrationTiltDeg;

static uint32_t wakeUpCount;
static uint32_t incompatibleCount;
static uint32_t rejectedCount;
static uint32_t txErrorCount;

/* === Private helpers ====================================================== */

/* Queues one data frame, CAN FD without BRS. `dlc` must match `data`. */
static bool CanNode_Send(uint32_t id, uint32_t dlc, const uint8_t *data) {
  FDCAN_TxHeaderTypeDef header = {0};
  header.Identifier = id;
  header.IdType = FDCAN_STANDARD_ID;
  header.TxFrameType = FDCAN_DATA_FRAME;
  header.DataLength = dlc;
  header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  header.BitRateSwitch = FDCAN_BRS_OFF; /* bus is FD without BRS */
  header.FDFormat = FDCAN_FD_CAN;
  header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  header.MessageMarker = 0;

  if (HAL_FDCAN_AddMessageToTxFifoQ(canNodeHandle, &header,
                                    (uint8_t *)data) != HAL_OK) {
    txErrorCount++;
    return false;
  }
  return true;
}

static bool CanNode_SetFilter(uint32_t index, uint32_t id, uint32_t mask) {
  FDCAN_FilterTypeDef filter = {0};
  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterIndex = index;
  filter.FilterType = FDCAN_FILTER_MASK;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1 = id;
  filter.FilterID2 = mask;
  return HAL_FDCAN_ConfigFilter(canNodeHandle, &filter) == HAL_OK;
}

static void CanNode_OnWakeUp(const FDCAN_RxHeaderTypeDef *header,
                             const uint8_t *data) {
  if (header->DataLength < CMD_WAKE_UP_LEN_DLC) {
    rejectedCount++;
    return;
  }

  if (!CAN_PROTOCOL_IS_COMPATIBLE(can_get_u16_le(&data[0]))) {
    incompatibleCount++;
    return;
  }

  wakeUpCount++;
  awake = true;
  wakeUpPending = true;

  /* No data; nothing is copied, but the HAL still wants a buffer. */
  uint8_t unused = 0;
  (void)CanNode_Send(CAN_ID_WAKE_UP_ACK(CAN_NODE_AVIONICS), FDCAN_DLC_BYTES_0,
                     &unused);
}

static void CanNode_OnCalibrate(const FDCAN_RxHeaderTypeDef *header,
                                const uint8_t *data) {
  if (!awake || header->DataLength < CMD_AVIONICS_CALIBRATE_DLC) {
    rejectedCount++;
    return;
  }

  calibrationTiltDeg = can_get_f32_le(&data[0]);
  calibrationPending = true;
}

/* int16 fields: 2 bytes each, X Y Z */
static void CanNode_PutAxes(uint8_t *buf, const int16_t axes[3]) {
  can_put_i16_le(&buf[0], axes[0]);
  can_put_i16_le(&buf[2], axes[1]);
  can_put_i16_le(&buf[4], axes[2]);
}

/* === API =================================================================== */

bool CanNode_Init(FDCAN_HandleTypeDef *hfdcan) {
  canNodeHandle = hfdcan;

  /* hfdcan1.Init.StdFiltersNbr = 2 in fdcan.c */
  const uint32_t mask = CAN_ID_CLASS_MASK | CAN_ID_NODE_MASK; /* 0x7F0 */
  if (!CanNode_SetFilter(0u, CAN_ID(CAN_CLASS_COMMAND, CAN_NODE_BROADCAST, 0u),
                         mask) ||                                  /* 0x100 */
      !CanNode_SetFilter(1u, CAN_ID(CAN_CLASS_COMMAND, CAN_NODE_AVIONICS, 0u),
                         mask)) {                                  /* 0x130 */
    return false;
  }

  /* The global filter can only be set before the peripheral is started. */
  if (HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT,
                                   FDCAN_REJECT_REMOTE,
                                   FDCAN_REJECT_REMOTE) != HAL_OK) {
    return false;
  }

  return HAL_FDCAN_Start(hfdcan) == HAL_OK;
}

void CanNode_Poll(void) {
  if (canNodeHandle == NULL) {
    return;
  }

  /* Checking the fill level first keeps GetRxMessage() from ever hitting an
   * empty FIFO (which it would flag in hfdcan->ErrorCode). */
  while (HAL_FDCAN_GetRxFifoFillLevel(canNodeHandle, FDCAN_RX_FIFO0) > 0u) {
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[CAN_NODE_RX_BUFFER_BYTES];
    if (HAL_FDCAN_GetRxMessage(canNodeHandle, FDCAN_RX_FIFO0, &header, data) !=
        HAL_OK) {
      break;
    }

    if (header.IdType != FDCAN_STANDARD_ID ||
        header.RxFrameType != FDCAN_DATA_FRAME) {
      rejectedCount++;
      continue;
    }

    switch (header.Identifier) {
      case CAN_ID_CMD_WAKE_UP:
        CanNode_OnWakeUp(&header, data);
        break;
      case CAN_ID_CMD_AVIONICS_CALIBRATE:
        CanNode_OnCalibrate(&header, data);
        break;
      default:
        break; /* other commands through the filters: not handled yet */
    }
  }
}

bool CanNode_PollWakeUp(void) {
  CanNode_Poll();

  bool pending = wakeUpPending;
  wakeUpPending = false;
  return pending;
}

bool CanNode_TakeCalibrationRequest(float *tiltDeg) {
  if (!calibrationPending) {
    return false;
  }

  calibrationPending = false;
  if (tiltDeg != NULL) {
    *tiltDeg = calibrationTiltDeg;
  }
  return true;
}

bool CanNode_SendCalibration(const CanNode_Calibration_t *calibration) {
  if (calibration == NULL) {
    return false;
  }

  /* ICD 6.9: 24 bytes of int16 + 2 x float32 = 32 bytes, little-endian */
  uint8_t data[AVIONICS_CALIBRATION_LEN] = {0};
  CanNode_PutAxes(&data[0], calibration->imu1_accel);
  CanNode_PutAxes(&data[6], calibration->imu1_gyro);
  CanNode_PutAxes(&data[12], calibration->imu2_accel);
  CanNode_PutAxes(&data[18], calibration->imu2_gyro);
  can_put_f32_le(&data[24], calibration->baro1_kPa);
  can_put_f32_le(&data[28], calibration->baro2_kPa);

  return CanNode_Send(CAN_ID_AVIONICS_CALIBRATION, FDCAN_DLC_BYTES_32, data);
}

bool CanNode_SendInitStatus(uint32_t ok_mask, uint32_t supported_mask) {
  uint8_t data[8];
  can_put_u32_le(&data[0], ok_mask);
  can_put_u32_le(&data[4], supported_mask);
  return CanNode_Send(CAN_ID_INIT_STATUS(CAN_NODE_AVIONICS),
                      FDCAN_DLC_BYTES_8, data);
}

uint32_t CanNode_WakeUpCount(void) { return wakeUpCount; }

uint32_t CanNode_IncompatibleCount(void) { return incompatibleCount; }

uint32_t CanNode_RejectedCount(void) { return rejectedCount; }

uint32_t CanNode_TxErrorCount(void) { return txErrorCount; }
