/**
 ******************************************************************************
 * @file    imu2.h
 * @brief   Board-level driver for the LSM6DSV32X high-g 6-axis IMU
 *          (accel + gyro) -- "IMU 2" on this board's schematic.
 *
 * Wraps STMicroelectronics' official register-level driver (vendored in
 * third_party/ST_LSM6DSV32X) into the same small API as imu.h, so both
 * IMUs are used the same way: Config -> Init -> Read, plus an optional
 * EnableDataReadyInterrupt().
 *
 * The part is capable of +-32 g / +-4000 dps. It is configured at +-16 g
 * (the same as the WSEN-ISDS, imu.h) / +-1000 dps: the core's EKF uses it
 * as its high-g IMU above 12 g, so it must not saturate before the ISDS
 * does. See Imu2_Init() for the details.
 * Readings come out in SI units, same as Imu_Sample_t.
 *
 * Unlike the WSEN-ISDS driver, this one keeps all its state per instance
 * (the ST driver takes a context struct), so several LSM6DSV32X could be
 * driven at once if the board ever carried more than one.
 ******************************************************************************
 */

#ifndef __IMU2_H__
#define __IMU2_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "main.h"
#include "lsm6dsv32x_reg.h"

/* Confirmed with the i2c_scan PoC: this board's LSM6DSV32X answers at 0x6B
 * on I2C2, i.e. SDO/SA0 strapped high (the ST driver spells the same thing
 * as LSM6DSV32X_I2C_ADD_H = 0xD7, an already-shifted 8-bit address -- this
 * one is the plain 7-bit form the rest of this project uses). */
#define LSM6DSV32X_ADDRESS_I2C_BOARD 0x6Bu

/* One accelerometer + gyroscope + temperature reading, in SI units --
 * same layout and units as imu.h's Imu_Sample_t. */
typedef struct {
  float accelX_mps2; /* m/s^2 */
  float accelY_mps2; /* m/s^2 */
  float accelZ_mps2; /* m/s^2 */

  float gyroX_radps; /* rad/s */
  float gyroY_radps; /* rad/s */
  float gyroZ_radps; /* rad/s */

  float temperature_degC;
} Imu2_Sample_t;

/* One LSM6DSV32X instance. */
typedef struct {
  stmdev_ctx_t ctx;   /* ST driver context; its handle points back here */
  I2C_HandleTypeDef *hi2c;
  uint8_t i2cAddress; /* 7-bit, not shifted */
} Imu2_t;

/* === API ================================================================== */

/* Fills in the ST driver context for one instance. Must be called before
 * Imu2_Init(). Does not touch the hardware. */
void Imu2_Config(Imu2_t *imu2, I2C_HandleTypeDef *hi2c, uint8_t i2cAddress);

/* Probes the sensor (WHO_AM_I check), software-resets it, then applies the
 * default configuration: block data update, +-16 g / +-1000 dps full scale
 * (see the note at the top of this file about why not the part's maximum),
 * 480 Hz output data rate and high-performance mode for both accelerometer
 * and gyroscope. Returns true on success. */
bool Imu2_Init(Imu2_t *imu2);

/* Blocking read: waits for a new accelerometer + gyroscope sample to be
 * ready and reads it out (temperature is read opportunistically, whether
 * or not its own data-ready flag is set). Converts the ST driver's native
 * mg / mdps readings to SI units (m/s^2, rad/s) -- see Imu2_Sample_t.
 * Returns true on success. */
bool Imu2_Read(Imu2_t *imu2, Imu2_Sample_t *sample);

/* Routes accelerometer and gyroscope data-ready to the sensor's INT1 pad
 * and enables interrupts. Call after Imu2_Init().
 *
 * WATCH THE PIN NAMING: the sensor's INT1 pad is this board's IMU2_INT1
 * net, which the .ioc/main.h call IMU2_INT0_Pin (PA10) -- the CubeMX
 * labels are off by one against the schematic (schematic IMU2_INT1/INT2 =
 * macros IMU2_INT0_Pin/IMU2_INT1_Pin). So the EXTI that fires for this is
 * IMU2_INT0_Pin. The pin's electrical config is left at the sensor's
 * defaults (IF_CFG: push-pull, active high), which is what this board's
 * GPIO_MODE_IT_RISING setup in gpio.c expects.
 *
 * This does not touch the MCU's EXTI/NVIC config (already set up by CubeMX
 * for both IMU2 pins); the application still needs to react to the
 * resulting HAL_GPIO_EXTI_Callback() itself (e.g. set a flag there and
 * call Imu2_Read() from the main loop once it's seen -- Imu2_Read()'s own
 * data-ready wait will then return immediately). Returns true on success.
 */
bool Imu2_EnableDataReadyInterrupt(Imu2_t *imu2);

#ifdef __cplusplus
}
#endif

#endif /* __IMU2_H__ */
