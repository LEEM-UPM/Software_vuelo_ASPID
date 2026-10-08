/**
 ******************************************************************************
 * @file    canTelemetry.h
 * @brief   Sensor telemetry over FDCAN.
 *
 * Sends one CAN FD frame per sensor sample: a sensor identifier byte, the
 * sample's timestamp, then the sample's values as little-endian 32-bit
 * floats. Frames
 * go out on their own CAN identifier per sensor as well (see
 * CAN_TELEMETRY_ID_BASE), so a receiver can filter in hardware instead of
 * having to inspect the payload -- the identifier byte is there too, as the
 * authoritative tag travelling with the data.
 *
 * Payload layout:
 *
 *   byte 0        sensor id (CanTelemetry_SensorId_t)
 *   bytes 1..4    timestamp  (uint32, little-endian, microseconds -- see
 *                 timestamp.h; wraps every ~71.6 min)
 *   bytes 5..8    value[0]   (float, little-endian)
 *   bytes 9..12   value[1]
 *   ...           up to CAN_TELEMETRY_MAX_VALUES
 *   (padding)     zero-filled up to the next valid CAN FD length
 *
 * The timestamp is whatever the caller passes in. In App/sensorsI2c.c
 * it is captured inside the sensor's data-ready interrupt, i.e. the moment
 * the sample became available, not when the frame was queued.
 *
 * CAN FD only allows certain frame lengths (0-8, 12, 16, 20, 24, 32, 48,
 * 64), so the frame is the smallest of those that fits, zero-padded. The
 * bus is configured for FD without bit rate switching (see fdcan.c), so
 * the whole frame travels at the nominal 2 Mbit/s.
 *
 * Which values each sensor sends is decided by the caller, not here -- see
 * the call sites in App/sensorsI2c.c.
 ******************************************************************************
 */

#ifndef __CANTELEMETRY_H__
#define __CANTELEMETRY_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "main.h"

/* Base standard CAN identifier. Each sensor transmits on
 * CAN_TELEMETRY_ID_BASE + its CanTelemetry_SensorId_t value. */
#define CAN_TELEMETRY_ID_BASE 0x100u

/* Largest number of floats one frame can carry: 1 id byte + 4 timestamp
 * bytes + 14 * 4 bytes = 61, which fits the 64-byte CAN FD maximum. */
#define CAN_TELEMETRY_MAX_VALUES 14u

/* First payload byte: which sensor the frame came from. */
typedef enum {
  CanTelemetry_Barometer1 = 0x01,   /* WSEN-PADS on I2C1 */
  CanTelemetry_Barometer2 = 0x02,   /* WSEN-PADS on I2C2 */
  CanTelemetry_Imu1 = 0x03,         /* WSEN-ISDS */
  CanTelemetry_Imu2 = 0x04,         /* LSM6DSV32X */
  CanTelemetry_Magnetometer = 0x05, /* MLX90393 */
  CanTelemetry_GpsGga = 0x06,       /* MAX-M10S, NMEA GGA (integers) */
  CanTelemetry_GpsRmc = 0x07,       /* MAX-M10S, NMEA RMC (integers) */
  CanTelemetry_GpsPvt = 0x08,       /* MAX-M10S, UBX NAV-PVT (velocity NED) */
} CanTelemetry_SensorId_t;

/* GPS frames don't carry floats: after the same 5-byte header (sensor id +
 * timestamp) they carry fixed-point integers, little-endian. Offsets below
 * are from the start of the payload. Invalid fields: int32 0x80000000,
 * uint16 0xFFFF, time 0xFFFFFFFF.
 *
 *  0x106 GGA (25 bytes -> 32)          0x107 RMC (28 bytes -> 32)
 *   5  uint32 UTC time of day [ms]      5  uint32 UTC time of day [ms]
 *   9  int32  latitude  [1e-7 deg]      9  int32  latitude  [1e-7 deg]
 *  13  int32  longitude [1e-7 deg]     13  int32  longitude [1e-7 deg]
 *  17  int32  altitude MSL [mm]        17  int32  ground speed [mm/s]
 *  21  uint8  fix quality (0 = none)   21  uint16 course [0.01 deg]
 *  22  uint8  satellites used          23  uint16 year (0 = unknown)
 *  23  uint16 HDOP x100                25  uint8  month
 *                                      26  uint8  day
 *                                      27  uint8  status (1 = 'A' valid)
 *
 * 0x108 NAV-PVT (15 bytes -> 20) carries what NMEA cannot: the VERTICAL
 * velocity. GGA has no velocity and RMC only gives speed over ground and
 * course, both horizontal, so without this frame the flight computer's EKF
 * never sees the rocket climb and calls apogee at motor burnout.
 *
 *   5  int32  velocity north [mm/s]
 *   9  int32  velocity east  [mm/s]
 *  13  int32  velocity down  [mm/s]   (positive = descending)
 *  17  uint8  fix type (0 none, 2 = 2D, 3 = 3D)
 *  18  uint8  satellites used
 *  19  uint8  1 = the receiver considers the fix valid
 */
#define CAN_TELEMETRY_GPS_GGA_BYTES 20u /* after the header */
#define CAN_TELEMETRY_GPS_RMC_BYTES 23u
#define CAN_TELEMETRY_GPS_PVT_BYTES 15u

/* === API ================================================================== */

/* Associates this module with an already-initialised FDCAN peripheral (see
 * MX_FDCAN1_Init()/MX_FDCAN2_Init()), sets the global filter to reject every
 * incoming frame that no explicit filter accepts, and starts the peripheral.
 * Call once before any CanTelemetry_Send(). Returns true on success.
 *
 * Nothing in this module receives. Frames that should get through are
 * whitelisted afterwards by their own module (see canCommand.h) -- which has
 * to come after this call, since the global filter can only be set before
 * the peripheral is started.
 *
 * The board's CAN transceivers are enabled by MX_GPIO_Init(), which drives
 * CAN1_STB/CAN2_STB low. */
bool CanTelemetry_Init(FDCAN_HandleTypeDef *hfdcan);

/* Queues one frame: `sensorId`, `timestampUs`, then `valueCount` floats.
 * Non-blocking -- if the transmit FIFO is full the frame is dropped and
 * false is returned, rather than stalling the caller. Telemetry must never
 * be allowed to hold up the sensor loop.
 *
 * Delivery is best-effort: automatic retransmission is disabled on the
 * telemetry bus (FDCAN2, see fdcan.c), so a frame that loses arbitration, is
 * not acknowledged or hits a bus error is sent once and dropped. A stale
 * sample is worth less than the next fresh one, and this keeps a bus with
 * no receiver on it from backing up the FIFO. Note that a true return only
 * means the frame was queued, not that it was delivered.
 *
 * Returns false if the FIFO is full, if valueCount exceeds
 * CAN_TELEMETRY_MAX_VALUES, or if the HAL rejects the frame. */
bool CanTelemetry_Send(CanTelemetry_SensorId_t sensorId, uint32_t timestampUs,
                        const float *values, uint8_t valueCount);

/* Same as CanTelemetry_Send(), but the payload after the header is `length`
 * raw bytes already packed by the caller (at most 59). Used by the GPS. */
bool CanTelemetry_SendBytes(CanTelemetry_SensorId_t sensorId,
                             uint32_t timestampUs, const uint8_t *bytes,
                             uint8_t length);

#ifdef __cplusplus
}
#endif

#endif /* __CANTELEMETRY_H__ */
