/**
 ******************************************************************************
 * @file    canTelemetry.c
 * @brief   Sensor telemetry over FDCAN.
 ******************************************************************************
 */

#include "canTelemetry.h"

#include <string.h>

/* 1 id byte + 4 timestamp bytes + CAN_TELEMETRY_MAX_VALUES floats, rounded
 * up to the 64-byte CAN FD maximum so the padding below always has somewhere
 * to go. */
#define CAN_TELEMETRY_MAX_FRAME_BYTES 64u

#define CAN_TELEMETRY_HEADER_BYTES 5u /* sensor id + uint32 timestamp */

static FDCAN_HandleTypeDef *canTelemetryHandle;

/* === Private helpers ====================================================== */

/* Maps a byte count to the smallest CAN FD frame that holds it, returning
 * both the HAL's DLC code and the padded length. CAN FD only allows 0-8,
 * 12, 16, 20, 24, 32, 48 and 64-byte frames. Returns false if `length`
 * exceeds 64. */
static bool CanTelemetry_LengthToDlc(uint8_t length, uint32_t *dlc,
                                      uint8_t *paddedLength) {
  static const struct {
    uint8_t bytes;
    uint32_t dlc;
  } sizes[] = {
      {8u, FDCAN_DLC_BYTES_8},   {12u, FDCAN_DLC_BYTES_12},
      {16u, FDCAN_DLC_BYTES_16}, {20u, FDCAN_DLC_BYTES_20},
      {24u, FDCAN_DLC_BYTES_24}, {32u, FDCAN_DLC_BYTES_32},
      {48u, FDCAN_DLC_BYTES_48}, {64u, FDCAN_DLC_BYTES_64},
  };

  for (uint32_t i = 0; i < (sizeof(sizes) / sizeof(sizes[0])); i++) {
    if (length <= sizes[i].bytes) {
      *dlc = sizes[i].dlc;
      *paddedLength = sizes[i].bytes;
      return true;
    }
  }

  return false;
}

/* === API =================================================================== */

bool CanTelemetry_Init(FDCAN_HandleTypeDef *hfdcan) {
  canTelemetryHandle = hfdcan;

  /* Drop everything no explicit filter accepts, rather than let it
   * accumulate in the Rx FIFOs -- see the header. */
  if (HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT,
                                    FDCAN_REJECT_REMOTE,
                                    FDCAN_REJECT_REMOTE) != HAL_OK) {
    return false;
  }

  return HAL_FDCAN_Start(hfdcan) == HAL_OK;
}

bool CanTelemetry_Send(CanTelemetry_SensorId_t sensorId, uint32_t timestampUs,
                        const float *values, uint8_t valueCount) {
  if (values == NULL || valueCount > CAN_TELEMETRY_MAX_VALUES) {
    return false;
  }
  /* The Cortex-M4 is little-endian, so the floats' memory already is the
   * little-endian wire format the header documents. */
  return CanTelemetry_SendBytes(sensorId, timestampUs, (const uint8_t *)values,
                                (uint8_t)(valueCount * sizeof(float)));
}

bool CanTelemetry_SendBytes(CanTelemetry_SensorId_t sensorId,
                             uint32_t timestampUs, const uint8_t *bytes,
                             uint8_t length) {
  if (canTelemetryHandle == NULL || bytes == NULL ||
      length > (CAN_TELEMETRY_MAX_FRAME_BYTES - CAN_TELEMETRY_HEADER_BYTES)) {
    return false;
  }

  /* Drop rather than block when the FIFO has backed up -- see the header. */
  if (HAL_FDCAN_GetTxFifoFreeLevel(canTelemetryHandle) == 0u) {
    return false;
  }

  uint8_t payload[CAN_TELEMETRY_MAX_FRAME_BYTES] = {0};
  payload[0] = (uint8_t)sensorId;
  memcpy(&payload[1], &timestampUs, sizeof(timestampUs));

  uint8_t used = (uint8_t)(CAN_TELEMETRY_HEADER_BYTES + length);
  memcpy(&payload[CAN_TELEMETRY_HEADER_BYTES], bytes, length);

  uint32_t dlc = 0;
  uint8_t paddedLength = 0;
  if (!CanTelemetry_LengthToDlc(used, &dlc, &paddedLength)) {
    return false;
  }
  /* payload[] is already zeroed, so the bytes between `used` and
   * `paddedLength` are the required zero padding. */
  (void)paddedLength;

  FDCAN_TxHeaderTypeDef header = {0};
  header.Identifier = CAN_TELEMETRY_ID_BASE + (uint32_t)sensorId;
  header.IdType = FDCAN_STANDARD_ID;
  header.TxFrameType = FDCAN_DATA_FRAME;
  header.DataLength = dlc;
  header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  header.BitRateSwitch = FDCAN_BRS_OFF; /* bus is FD without BRS */
  header.FDFormat = FDCAN_FD_CAN;
  header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  header.MessageMarker = 0;

  return HAL_FDCAN_AddMessageToTxFifoQ(canTelemetryHandle, &header, payload) ==
         HAL_OK;
}
