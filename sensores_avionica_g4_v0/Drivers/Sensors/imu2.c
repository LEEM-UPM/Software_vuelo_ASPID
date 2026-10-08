/**
 ******************************************************************************
 * @file    imu2.c
 * @brief   Board-level driver for the LSM6DSV32X high-g 6-axis IMU.
 ******************************************************************************
 */

#include "imu2.h"

/* A read is well under a millisecond at 400 kHz: longer than this means the
 * bus is in trouble, and waiting a second for it would stall the loop. */
#define IMU2_I2C_TIMEOUT_MS 20u

/* Data-ready poll in Imu2_Read() (ODR is 480 Hz) */
#define IMU2_DATA_TIMEOUT_MS 10u

/* The ST driver reports mg / mdps; convert to SI (m/s^2, rad/s), same as
 * imu.c does for the WSEN-ISDS. */
#define IMU2_MG_TO_MPS2 (9.80665e-3f)
#define IMU2_MDPS_TO_RADPS (1.745329252e-5f) /* (pi/180) / 1000 */

/* === ST driver platform glue ==============================================
 * stmdev_ctx_t's handle points at the owning Imu2_t, so these can reach
 * both the I2C peripheral and the device address. */

static int32_t Imu2_PlatformWrite(void *handle, uint8_t reg, const uint8_t *buf,
                                   uint16_t len) {
  Imu2_t *imu2 = (Imu2_t *)handle;
  return (HAL_I2C_Mem_Write(imu2->hi2c, (uint16_t)(imu2->i2cAddress << 1), reg,
                             I2C_MEMADD_SIZE_8BIT, (uint8_t *)buf, len,
                             IMU2_I2C_TIMEOUT_MS) == HAL_OK)
             ? 0
             : -1;
}

static int32_t Imu2_PlatformRead(void *handle, uint8_t reg, uint8_t *buf,
                                  uint16_t len) {
  Imu2_t *imu2 = (Imu2_t *)handle;
  return (HAL_I2C_Mem_Read(imu2->hi2c, (uint16_t)(imu2->i2cAddress << 1), reg,
                            I2C_MEMADD_SIZE_8BIT, buf, len,
                            IMU2_I2C_TIMEOUT_MS) == HAL_OK)
             ? 0
             : -1;
}

/* === API =================================================================== */

void Imu2_Config(Imu2_t *imu2, I2C_HandleTypeDef *hi2c, uint8_t i2cAddress) {
  imu2->hi2c = hi2c;
  imu2->i2cAddress = i2cAddress;

  imu2->ctx.write_reg = Imu2_PlatformWrite;
  imu2->ctx.read_reg = Imu2_PlatformRead;
  imu2->ctx.mdelay = HAL_Delay; /* required: lsm6dsv32x_sw_reset() fails
                                 * outright if this is left NULL */
  imu2->ctx.handle = imu2;
}

bool Imu2_Init(Imu2_t *imu2) {
  HAL_Delay(50);

  uint8_t deviceId = 0;
  if (lsm6dsv32x_device_id_get(&imu2->ctx, &deviceId) != 0) {
    return false;
  }

  if (deviceId != LSM6DSV32X_ID) {
    return false;
  }

  /* Software reset. Unlike the WSEN sensors' soft reset, the ST driver
   * powers both sensors down, sets the reset bit and polls for completion
   * internally (bounded by its own retry count). */
  if (lsm6dsv32x_sw_reset(&imu2->ctx) != 0) {
    return false;
  }

  if (lsm6dsv32x_block_data_update_set(&imu2->ctx, 1) != 0) {
    return false;
  }

  /* Accelerometer +-16 g, the same as the WSEN-ISDS (imu.h).
   *
   * The core's GNC (ASPID v1.2 EKF) uses this part as its HIGH-G IMU: below
   * 12 g it only reads the WSEN-ISDS, between 12 and 16 g it blends towards
   * this one, and once the ISDS gets close to its 16 g ceiling it uses only
   * this one. Until 2026-10-06 it was set to +-8 g for resolution, which
   * meant the EKF would switch to a SATURATED sensor exactly when it needed
   * it (above 12 g). At +-16 g both read correctly over the whole range we
   * expect to fly; the part can go to +-32 g (LSM6DSV32X_32g, which is what
   * the EKF is tuned for) if the flight ever needs it. Resolution is
   * 0.488 mg/LSB, plenty: the EKF only uses this sensor above 12 g.
   *
   * Gyro stays at +-1000 dps (35 mdps/LSB): the GNC takes its rates from the
   * WSEN-ISDS only.
   *
   * The conversion helpers in Imu2_Read() must match whatever is set here
   * -- getting them out of sync silently scales every reading. */
  if (lsm6dsv32x_xl_full_scale_set(&imu2->ctx, LSM6DSV32X_16g) != 0) {
    return false;
  }
  if (lsm6dsv32x_gy_full_scale_set(&imu2->ctx, LSM6DSV32X_1000dps) != 0) {
    return false;
  }

  if (lsm6dsv32x_xl_mode_set(&imu2->ctx, LSM6DSV32X_XL_HIGH_PERFORMANCE_MD) !=
      0) {
    return false;
  }
  if (lsm6dsv32x_gy_mode_set(&imu2->ctx, LSM6DSV32X_GY_HIGH_PERFORMANCE_MD) !=
      0) {
    return false;
  }

  /* 480 Hz, from the standard ODR family (the HA01/HA02 families offer
   * rounder rates like 400 or 500 Hz, but need HIGH_ACCURACY_ODR mode).
   *
   * On the budget: an Imu2_Read() is ~0.6 ms with the bus at 400 kHz, so
   * this costs ~29% of the main loop. I2C2 itself is nearly idle (only
   * PADS2 shares it), but the loop is single-threaded and also services
   * I2C1, so the real ceiling is the loop, not the bus -- 960 Hz would
   * push the total past 100% and start dropping samples everywhere. */
  if (lsm6dsv32x_xl_data_rate_set(&imu2->ctx, LSM6DSV32X_ODR_AT_480Hz) != 0) {
    return false;
  }
  if (lsm6dsv32x_gy_data_rate_set(&imu2->ctx, LSM6DSV32X_ODR_AT_480Hz) != 0) {
    return false;
  }

  return true;
}

bool Imu2_Read(Imu2_t *imu2, Imu2_Sample_t *sample) {
  lsm6dsv32x_data_ready_t ready = {0};

  /* Bounded: data-ready already fired, so the data is there -- if it is not,
   * the sample is given up on instead of blocking the main loop. */
  const uint32_t start = HAL_GetTick();
  do {
    if (lsm6dsv32x_flag_data_ready_get(&imu2->ctx, &ready) != 0) {
      return false;
    }
    if ((HAL_GetTick() - start) >= IMU2_DATA_TIMEOUT_MS) {
      return false;
    }
  } while (!ready.drdy_xl || !ready.drdy_gy);

  int16_t rawAccel[3];
  if (lsm6dsv32x_acceleration_raw_get(&imu2->ctx, rawAccel) != 0) {
    return false;
  }
  /* from_fs16/from_fs1000 must match the full scales set in Imu2_Init(). */
  sample->accelX_mps2 =
      lsm6dsv32x_from_fs16_to_mg(rawAccel[0]) * IMU2_MG_TO_MPS2;
  sample->accelY_mps2 =
      lsm6dsv32x_from_fs16_to_mg(rawAccel[1]) * IMU2_MG_TO_MPS2;
  sample->accelZ_mps2 =
      lsm6dsv32x_from_fs16_to_mg(rawAccel[2]) * IMU2_MG_TO_MPS2;

  int16_t rawGyro[3];
  if (lsm6dsv32x_angular_rate_raw_get(&imu2->ctx, rawGyro) != 0) {
    return false;
  }
  sample->gyroX_radps =
      lsm6dsv32x_from_fs1000_to_mdps(rawGyro[0]) * IMU2_MDPS_TO_RADPS;
  sample->gyroY_radps =
      lsm6dsv32x_from_fs1000_to_mdps(rawGyro[1]) * IMU2_MDPS_TO_RADPS;
  sample->gyroZ_radps =
      lsm6dsv32x_from_fs1000_to_mdps(rawGyro[2]) * IMU2_MDPS_TO_RADPS;

  /* Temperature has its own (slower) data rate; read it opportunistically
   * without failing the whole sample if it is not ready yet. */
  if (ready.drdy_temp) {
    int16_t rawTemp = 0;
    if (lsm6dsv32x_temperature_raw_get(&imu2->ctx, &rawTemp) == 0) {
      sample->temperature_degC = lsm6dsv32x_from_lsb_to_celsius(rawTemp);
    }
  }

  return true;
}

bool Imu2_EnableDataReadyInterrupt(Imu2_t *imu2) {
  /* Route accelerometer and gyroscope data-ready to the INT1 pad (this
   * board's IMU2_INT0_Pin / PA10 -- see the header about the naming). */
  lsm6dsv32x_pin_int_route_t route = {0};
  route.drdy_xl = 1;
  route.drdy_g = 1;
  if (lsm6dsv32x_pin_int1_route_set(&imu2->ctx, &route) != 0) {
    return false;
  }

  /* Master interrupt enable, pulsed (not latched) so each new sample gives
   * a fresh edge without needing the source register read to clear it. */
  lsm6dsv32x_interrupt_mode_t interruptMode = {0};
  interruptMode.enable = 1;
  interruptMode.lir = 0;
  return lsm6dsv32x_interrupt_enable_set(&imu2->ctx, interruptMode) == 0;
}
