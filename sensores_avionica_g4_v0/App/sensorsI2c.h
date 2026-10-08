/**
 ******************************************************************************
 * @file    sensorsI2c.h
 * @brief   I2C sensors: read on data-ready and forwarded over CAN telemetry.
 *
 *  - I2C1 (PB7 SDA / PB8 SCL): PADS1 (barometer 1), WSEN-ISDS (main IMU),
 *    MLX90393 magnetometer.
 *  - I2C2 (PA8 SDA / PA9 SCL): PADS2 (barometer 2), LSM6DSV32X (high-g
 *    IMU), WSEN-HIDS (humidity, not yet implemented).
 *
 * Every sensor is read (a normal blocking call to e.g. Barometer_Read())
 * only when its data-ready interrupt says there is a sample (sensorIrq.h),
 * and each successful read goes straight out over CAN (canTelemetry.h).
 * The MLX90393 runs in Single Measurement mode (master-triggered), so it
 * also needs re-arming after each read -- the others free-run on their own
 * ODR and don't.
 *
 * Every sensor's latest sample is kept in a static in sensorsI2c.c, meant
 * to be inspected live from the debugger (Live Expressions / Watch).
 *
 * NO SENSOR CAN STOP THE BOARD. A sensor that does not answer at start-up,
 * or that stops answering later, is simply marked as down: it is skipped
 * (so it costs no bus time) and re-initialised every
 * SENSORS_I2C_RETRY_PERIOD_MS in case it comes back. If every sensor on a
 * bus fails at once, the I2C peripheral is reinitialised -- the usual cure
 * for a bus left stuck by a sensor that was reset mid-transfer. Which
 * sensors are working is in SensorsI2c_ActiveMask().
 ******************************************************************************
 */

#ifndef __SENSORSI2C_H__
#define __SENSORSI2C_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "barometer.h"

/* Sensores del I2C, un bit por sensor en SensorsI2c_ActiveMask() */
typedef enum {
  SensorsI2c_Barometer1 = 0, /* PADS1, I2C1      */
  SensorsI2c_Barometer2,     /* PADS2, I2C2      */
  SensorsI2c_Imu1,           /* WSEN-ISDS, I2C1  */
  SensorsI2c_Imu2,           /* LSM6DSV32X, I2C2 */
  SensorsI2c_Magnetometer,   /* MLX90393, I2C1   */
  SensorsI2c_Count
} SensorsI2c_Id_t;

#define SENSORS_I2C_BIT(id) (1u << (id))

/* One IMU sample or offset set, X Y Z. IMU 1 = WSEN-ISDS, IMU 2 = LSM6DSV32X. */
typedef struct {
  float accel_mps2[3];
  float gyro_radps[3];
} SensorsI2c_ImuSample_t;

typedef SensorsI2c_ImuSample_t SensorsI2c_ImuOffsets_t;

/* Configures and initialises the five I2C sensors. After MX_I2C1_Init() and
 * MX_I2C2_Init(). */
void SensorsI2c_Init(void);

/* Arms every data-ready interrupt and triggers the magnetometer's first
 * conversion. From here on HAL_GPIO_EXTI_Callback() (sensorIrq.c) runs. */
void SensorsI2c_Start(void);

/* Reads every sensor with a sample waiting and sends it over CAN. Call on
 * every pass of the main loop. */
void SensorsI2c_Service(void);

/* True while every I2C sensor has produced a successful read within the
 * last I2C_SENSOR_TIMEOUT_MS (sensorsI2c.c). */
bool SensorsI2c_Healthy(void);

/* One bit per sensor (SENSORS_I2C_BIT(SensorsI2c_Imu1)...): 1 = initialised
 * and reading. A sensor that fails clears its bit and gets it back if a
 * later retry works. */
uint8_t SensorsI2c_ActiveMask(void);

/* True if that sensor is initialised and answering. */
bool SensorsI2c_IsActive(SensorsI2c_Id_t id);

/* Diagnostics: failed reads and re-initialisation attempts of one sensor. */
uint32_t SensorsI2c_ErrorCount(SensorsI2c_Id_t id);
uint32_t SensorsI2c_RetryCount(SensorsI2c_Id_t id);

/* I2C buses reinitialised because every sensor on them went down. */
uint32_t SensorsI2c_BusResetCount(void);

/* --- For the calibration (App/calibration.c) ----------------------------- */

/* Reads barometer `index` (1 or 2) if it has a sample waiting. Nothing is
 * sent over CAN. Returns true if a sample was read. */
bool SensorsI2c_ReadBarometerRaw(uint8_t index, Barometer_Sample_t *sample);

/* Reads IMU `index` (1 or 2) if it has a sample waiting, WITHOUT the
 * calibration offsets. Nothing is sent over CAN. Returns true if read. */
bool SensorsI2c_ReadImuRaw(uint8_t index, SensorsI2c_ImuSample_t *sample);

/* Offsets subtracted from every later sample of IMU `index` (1 or 2) before
 * it is sent. RAM only: lost on reset. */
void SensorsI2c_SetImuOffsets(uint8_t index,
                              const SensorsI2c_ImuOffsets_t *offsets);

#ifdef __cplusplus
}
#endif

#endif /* __SENSORSI2C_H__ */
