/**
 ******************************************************************************
 * @file    canNode.h
 * @brief   This board as a node of the common CAN1 bus between boards.
 *
 * Everything on CAN1 follows the ICD in third_party/can_protocol: 1 Mbit/s,
 * CAN FD without BRS, 11-bit IDs built with CAN_ID(), little-endian fields.
 * This board is CAN_NODE_AVIONICS (0x3).
 *
 * Messages handled here (ICD section 6):
 *  - CMD_WAKE_UP (0x100, core -> all): answered with WAKE_UP_ACK (0x230,
 *    no data) -- also once already running, so a core that resets sees this
 *    board again.
 *  - CMD_AVIONICS_CALIBRATE (0x131, core -> avionics): float32 tilt angle in
 *    degrees. Answered with AVIONICS_CALIBRATION (0x231) once the calibration
 *    routine (App/calibration.c) has finished.
 *
 * Commands other than CMD_WAKE_UP are ignored until the first valid
 * CMD_WAKE_UP has arrived.
 *
 * Reception is polled, not interrupt-driven: CanNode_Poll() reads the FDCAN1
 * Rx FIFO 0 directly. The G4's FIFO only holds 3 frames, so call it on every
 * pass of the main loop. It only records what arrived; the main loop picks it
 * up with the CanNode_Take*() functions.
 ******************************************************************************
 */

#ifndef __CANNODE_H__
#define __CANNODE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "main.h"

/* Field of AVIONICS_CALIBRATION that could not be calibrated (ICD 6.9) */
#define CAN_NODE_CALIBRATION_INVALID_I16 ((int16_t)0x7FFF)
/* The pressure fields are float32: "not calibrated" is NaN */

/* AVIONICS_CALIBRATION contents, already in wire units (ICD 6.9):
 *  - accel: 0.001 m/s^2 per unit, X Y Z
 *  - gyro:  0.0001 rad/s per unit, X Y Z
 *  - baro_kPa: mean pressure at the base altitude, kPa */
typedef struct {
  int16_t imu1_accel[3];
  int16_t imu1_gyro[3];
  int16_t imu2_accel[3];
  int16_t imu2_gyro[3];
  float baro1_kPa;
  float baro2_kPa;
} CanNode_Calibration_t;

/* === API ================================================================== */

/* Associates this module with FDCAN1 (already initialised by
 * MX_FDCAN1_Init()) and starts it with two filters
 * (hfdcan1.Init.StdFiltersNbr = 2 in fdcan.c):
 *  - index 0: broadcast commands, 0x100-0x10F
 *  - index 1: commands to this board, 0x130-0x13F
 * Avionics (0011) has two bits set, so both cannot share one filter
 * (ICD 5.3). Everything else is rejected. Returns true on success. */
bool CanNode_Init(FDCAN_HandleTypeDef *hfdcan);

/* Drains Rx FIFO 0. Answers every valid CMD_WAKE_UP right away and records
 * the other commands for the CanNode_Take*() functions below. */
void CanNode_Poll(void);

/* CanNode_Poll() and then: true if a valid CMD_WAKE_UP arrived since the
 * previous call. Meant for the start-up wait loop:
 *   while (!CanNode_PollWakeUp()) {}
 * A CMD_WAKE_UP whose protocol version is not compatible
 * (CAN_PROTOCOL_IS_COMPATIBLE()) is neither answered nor reported, so the
 * board keeps waiting and the core sees it as not awake. */
bool CanNode_PollWakeUp(void);

/* True if a CMD_AVIONICS_CALIBRATE arrived since the previous call; its tilt
 * angle (degrees) is written to *tiltDeg. */
bool CanNode_TakeCalibrationRequest(float *tiltDeg);

/* Sends AVIONICS_CALIBRATION (0x231, 32 bytes). Returns false if it could not
 * be queued. */
bool CanNode_SendCalibration(const CanNode_Calibration_t *calibration);

/* Bits de INIT_STATUS (ICD 6.11) de esta placa */
#define CAN_NODE_INIT_BARO1 (1u << 0)
#define CAN_NODE_INIT_BARO2 (1u << 1)
#define CAN_NODE_INIT_IMU1 (1u << 2)
#define CAN_NODE_INIT_IMU2 (1u << 3)
#define CAN_NODE_INIT_MAG (1u << 4)
#define CAN_NODE_INIT_GPS (1u << 5)
#define CAN_NODE_INIT_VECTORNAV (1u << 6)

/* Componentes que tiene esta placa (campo supported de INIT_STATUS) */
#define CAN_NODE_INIT_SUPPORTED                                                \
  (CAN_NODE_INIT_BARO1 | CAN_NODE_INIT_BARO2 | CAN_NODE_INIT_IMU1 |            \
   CAN_NODE_INIT_IMU2 | CAN_NODE_INIT_MAG | CAN_NODE_INIT_GPS |                \
   CAN_NODE_INIT_VECTORNAV)

/* Le dice al core con que componentes ha arrancado la placa (INIT_STATUS,
 * ICD 6.11). Se envia tras atender cada CMD_WAKE_UP, asi que un core que se
 * reinicie lo vuelve a recibir. */
bool CanNode_SendInitStatus(uint32_t ok_mask, uint32_t supported_mask);

/* Diagnostics (watch them in the debugger). */
uint32_t CanNode_WakeUpCount(void);       /* valid CMD_WAKE_UP received */
uint32_t CanNode_IncompatibleCount(void); /* CMD_WAKE_UP with another version */
uint32_t CanNode_RejectedCount(void);     /* malformed or before wake-up */
uint32_t CanNode_TxErrorCount(void);      /* frames that could not be queued */

#ifdef __cplusplus
}
#endif

#endif /* __CANNODE_H__ */
