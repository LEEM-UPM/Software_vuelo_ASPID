/**
 ******************************************************************************
 * @file    imu.h
 * @brief   Board-level driver for the WSEN-ISDS 6-axis IMU (accel + gyro).
 *
 * Wraps the low-level WSEN_ISDS_2536030320001 SDK (third_party/WESensorsSDK)
 * into a small API for simple, polled readings (no FIFO, no interrupts),
 * following the same pattern as barometer.h and gps.h.
 *
 * NOTE: the underlying WE driver tracks the currently configured full-scale
 * range in module-global state (not per WE_sensorInterface_t instance), so
 * it only supports driving one WSEN-ISDS at a time -- which matches this
 * board, where WSEN-ISDS is the single main IMU (the high-g IMU is a
 * different chip, LSM6DSV32X, not covered by this driver).
 ******************************************************************************
 */

#ifndef __IMU_H__
#define __IMU_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "main.h"
#include "WSEN_ISDS_2536030320001.h"
#include "WeSensorsSDK.h"

/* One accelerometer + gyroscope + temperature reading, in SI units. */
typedef struct {
  float accelX_mps2; /* m/s^2 */
  float accelY_mps2; /* m/s^2 */
  float accelZ_mps2; /* m/s^2 */

  float gyroX_radps; /* rad/s */
  float gyroY_radps; /* rad/s */
  float gyroZ_radps; /* rad/s */

  float temperature_degC;
} Imu_Sample_t;

/* One WSEN-ISDS instance. */
typedef struct {
  WE_sensorInterface_t sensorInterface;
} Imu_t;

/* === API ================================================================== */

/* Fills in the sensor interface for one instance. Must be called before
 * Imu_Init(). Does not touch the hardware. */
void Imu_Config(Imu_t *imu, I2C_HandleTypeDef *hi2c, uint8_t i2cAddress);

/* Probes the sensor (device ID check), applies the default power-on
 * configuration (block data update, +-16 g / +-2000 dps full scale,
 * 416 Hz output data rate for both accelerometer and gyroscope, and an
 * accelerometer anti-aliasing setup of 400 Hz analog chain + ODR/4 digital
 * LPF, i.e. ~104 Hz of bandwidth). Both stay in high-performance mode.
 * See Imu_Init() for the reasoning behind each. Returns true on success. */
bool Imu_Init(Imu_t *imu);

/* Blocking read: waits for a new accelerometer + gyroscope sample to be
 * ready and reads it out (temperature is read opportunistically, whether
 * or not its own data-ready flag is set). Converts the WE SDK's native
 * mg / mdps readings to SI units (m/s^2, rad/s) -- see Imu_Sample_t.
 * Returns true on success. */
bool Imu_Read(Imu_t *imu, Imu_Sample_t *sample);

/* Configures the INT0 pin to assert (active-high, push-pull -- matching
 * this board's IMU1_INT0 EXTI config, GPIO_MODE_IT_RISING in gpio.c) when
 * a new accelerometer + gyroscope sample is ready. Call after Imu_Init().
 * This does not touch the MCU's EXTI/NVIC config (already set up by
 * CubeMX for IMU1_INT0); the application still needs to react to the
 * resulting HAL_GPIO_EXTI_Callback() itself (e.g. set a flag there and
 * call Imu_Read() from the main loop once it's seen -- Imu_Read()'s own
 * data-ready wait will then return immediately). Returns true on success.
 */
bool Imu_EnableDataReadyInterrupt(Imu_t *imu);

#ifdef __cplusplus
}
#endif

#endif /* __IMU_H__ */
