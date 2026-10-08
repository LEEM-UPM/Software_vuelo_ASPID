/**
 ******************************************************************************
 * @file    calibration.h
 * @brief   Ground calibration of the IMUs and barometers, commanded by the
 *          core over CAN1 (ICD 6.8 CMD_AVIONICS_CALIBRATE / 6.9
 *          AVIONICS_CALIBRATION).
 *
 * One blocking routine, all sensors sampled AT THE SAME TIME (one loop that
 * reads whichever sensor has a sample ready, not one sensor after another):
 *  - Barometers: mean pressure over CALIBRATION_BARO_TIME_MS = pressure at the
 *    base altitude. Sent to the core; NOT subtracted from the readings.
 *  - Gyroscopes: mean over CALIBRATION_IMU_TIME_MS = offset, subtracted from
 *    every later reading.
 *  - Accelerometers: mean over CALIBRATION_IMU_TIME_MS minus the gravity
 *    projection expected for the tilt angle sent by the core = offset,
 *    subtracted from every later reading.
 *
 * While it runs nothing else happens on this board: no telemetry, no CAN.
 * It lasts CALIBRATION_IMU_TIME_MS (2 s), well inside the core's 5 s timeout.
 * The offsets live in RAM only and are lost on reset.
 ******************************************************************************
 */

#ifndef __CALIBRATION_H__
#define __CALIBRATION_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "canNode.h"

#define CALIBRATION_BARO_TIME_MS 1000u
#define CALIBRATION_IMU_TIME_MS  2000u

/* If the core has sent CMD_AVIONICS_CALIBRATE: runs the calibration (blocks
 * for CALIBRATION_IMU_TIME_MS), applies the offsets and answers with
 * AVIONICS_CALIBRATION. Call on every pass of the main loop, after
 * CanNode_Poll(). */
void Calibration_Update(void);

/* Runs the calibration for a rocket tilted `tiltDeg` degrees, applies the
 * IMU offsets and writes the values to send in *result. */
void Calibration_Run(float tiltDeg, CanNode_Calibration_t *result);

#ifdef __cplusplus
}
#endif

#endif /* __CALIBRATION_H__ */
