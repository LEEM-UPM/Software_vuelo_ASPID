/**
 ******************************************************************************
 * @file    imu.c
 * @brief   Board-level driver for the WSEN-ISDS 6-axis IMU (accel + gyro).
 ******************************************************************************
 */

#include "imu.h"

#include "platform.h"

/* SDK getters below return mg / mdps; convert to SI (m/s^2, rad/s). */
#define IMU_MG_TO_MPS2 (9.80665e-3f)
#define IMU_MDPS_TO_RADPS (1.745329252e-5f) /* (pi/180) / 1000 */

/* Nothing here may wait forever: a sensor that does not answer must fail and
 * let the main loop carry on with the others. */
#define IMU_READY_TIMEOUT_MS 20u /* interface probe / soft reset          */
#define IMU_DATA_TIMEOUT_MS 10u  /* data-ready poll (ODR is 416 Hz)       */

/* A read is ~0.6 ms at 400 kHz: longer than this means the bus is in
 * trouble, and waiting a second for it would stall the whole loop. */
#define IMU_I2C_TIMEOUT_MS 20u

/*
 * MOUNTING: this sensor sits rotated 180 degrees about the board's Z axis
 * with respect to the LSM6DSV32X (imu2.c) and to the axes the rest of the
 * system expects, so its X and Y point the other way. Turning the sample
 * here, in the driver, means everybody downstream -- the CAN telemetry, the
 * calibration and the core's GNC -- sees both IMUs in the same frame; doing
 * it in only one of them would leave the others disagreeing.
 *
 * 180 degrees about Z: (x, y, z) -> (-x, -y, z), for acceleration and for
 * angular rate alike. If the board is ever rebuilt with both IMUs aligned,
 * this is the only place to change.
 */
#define IMU_MOUNT_X(v) (-(v))
#define IMU_MOUNT_Y(v) (-(v))

void Imu_Config(Imu_t *imu, I2C_HandleTypeDef *hi2c, uint8_t i2cAddress) {
  imu->sensorInterface.sensorType = WE_ISDS;
  imu->sensorInterface.interfaceType = WE_i2c;
  imu->sensorInterface.options.i2c.address = i2cAddress;
  imu->sensorInterface.options.i2c.burstMode = 1;
  imu->sensorInterface.options.i2c.slaveTransmitterMode = 0;
  imu->sensorInterface.options.i2c.useRegAddrMsbForMultiBytesRead = 0;
  imu->sensorInterface.options.i2c.reserved = 0;
  imu->sensorInterface.options.readTimeout = IMU_I2C_TIMEOUT_MS;
  imu->sensorInterface.options.writeTimeout = IMU_I2C_TIMEOUT_MS;
  imu->sensorInterface.handle = hi2c;
}

bool Imu_Init(Imu_t *imu) {
  HAL_Delay(50);

  uint32_t start = HAL_GetTick();
  while (WE_isSensorInterfaceReady(&imu->sensorInterface) != WE_SUCCESS) {
    if ((HAL_GetTick() - start) >= IMU_READY_TIMEOUT_MS) {
      return false; /* not on the bus: the caller retries later */
    }
  }

  uint8_t deviceId = 0;
  if (ISDS_getDeviceID(&imu->sensorInterface, &deviceId) != WE_SUCCESS) {
    return false;
  }

  if (deviceId != ISDS_DEVICE_ID_VALUE) {
    return false;
  }

  /* Soft reset */
  ISDS_softReset(&imu->sensorInterface, ISDS_enable);
  ISDS_state_t resetState;
  start = HAL_GetTick();
  do {
    if (ISDS_getSoftResetState(&imu->sensorInterface, &resetState) !=
        WE_SUCCESS) {
      return false;
    }
    if ((HAL_GetTick() - start) >= IMU_READY_TIMEOUT_MS) {
      return false; /* the reset never finished */
    }
  } while (resetState);

  /* Sensor configuration */
  ISDS_enableAutoIncrement(&imu->sensorInterface, ISDS_enable);
  ISDS_enableBlockDataUpdate(&imu->sensorInterface, ISDS_enable);

  /* Full scale: the widest this sensor offers, +-16 g / +-2000 dps.
   * +-2000 dps is a hard ceiling here -- if the vehicle spins past that,
   * the usable gyro reading is the high-g IMU's (imu2.h, +-4000 dps).
   * Setting these also primes the driver's internal full-scale state,
   * which the *_float getters below rely on. */
  ISDS_setAccFullScale(&imu->sensorInterface, ISDS_accFullScaleSixteenG);
  ISDS_setGyroFullScale(&imu->sensorInterface, ISDS_gyroFullScale2000dps);

  /* 416 Hz ODR for both. On the I2C budget: one Imu_Read() is ~0.6 ms with
   * the bus at 400 kHz, so this costs ~25% of I2C1 (shared with PADS1 and
   * the magnetometer). 833 Hz would still fit at ~50%; past that the
   * blocking read-per-sample approach runs out of bus and the FIFO would
   * have to be used instead. */
  ISDS_setAccOutputDataRate(&imu->sensorInterface, ISDS_accOdr416Hz);
  ISDS_setGyroOutputDataRate(&imu->sensorInterface, ISDS_gyroOdr416Hz);

  /* Anti-aliasing for the accelerometer: narrow the analog chain from its
   * 1.5 kHz default to 400 Hz, and take the digital LPF1 down from ODR/2
   * (208 Hz, right at Nyquist) to ODR/4, i.e. ~104 Hz of output bandwidth
   * with margin to spare. The cost is that vibration content between
   * ~104 and 208 Hz no longer shows up -- a deliberate trade, since at
   * 416 Hz that band cannot be represented cleanly anyway and would
   * otherwise fold back onto the signal as aliasing noise.
   *
   * The gyro gets no equivalent treatment: its LPF1 bandwidth options
   * (ISDS_gyroLPF_t) only start at an 800 Hz ODR, so at 416 Hz there is
   * nothing to select -- its bandwidth is already set by the decimation.
   *
   * Both sensors stay in high-performance mode (the default), where the
   * internal conversion runs faster than the ODR and is filtered down to
   * it -- the sensor's only form of oversampling, and not adjustable. */
  ISDS_setAccAnalogChainBandwidth(&imu->sensorInterface,
                                   ISDS_accAnalogChainBandwidth400Hz);
  ISDS_setAccDigitalLpfBandwidth(&imu->sensorInterface,
                                  ISDS_accDigitalLpfBandwidthOdrDiv4);

  return true;
}

bool Imu_Read(Imu_t *imu, Imu_Sample_t *sample) {
  ISDS_state_t tempReady = ISDS_disable;
  ISDS_state_t accReady = ISDS_disable;
  ISDS_state_t gyroReady = ISDS_disable;

  const uint32_t start = HAL_GetTick();
  do {
    if (ISDS_isDataReady(&imu->sensorInterface, &tempReady, &accReady,
                          &gyroReady) != WE_SUCCESS) {
      return false;
    }
    if ((HAL_GetTick() - start) >= IMU_DATA_TIMEOUT_MS) {
      return false; /* data-ready fired but the data never showed up */
    }
  } while (!accReady || !gyroReady);

  float accelX_mg, accelY_mg, accelZ_mg;
  if (ISDS_getAccelerations_float(&imu->sensorInterface, &accelX_mg,
                                   &accelY_mg, &accelZ_mg) != WE_SUCCESS) {
    return false;
  }
  sample->accelX_mps2 = IMU_MOUNT_X(accelX_mg) * IMU_MG_TO_MPS2;
  sample->accelY_mps2 = IMU_MOUNT_Y(accelY_mg) * IMU_MG_TO_MPS2;
  sample->accelZ_mps2 = accelZ_mg * IMU_MG_TO_MPS2;

  float gyroX_mdps, gyroY_mdps, gyroZ_mdps;
  if (ISDS_getAngularRates_float(&imu->sensorInterface, &gyroX_mdps,
                                  &gyroY_mdps, &gyroZ_mdps) != WE_SUCCESS) {
    return false;
  }
  sample->gyroX_radps = IMU_MOUNT_X(gyroX_mdps) * IMU_MDPS_TO_RADPS;
  sample->gyroY_radps = IMU_MOUNT_Y(gyroY_mdps) * IMU_MDPS_TO_RADPS;
  sample->gyroZ_radps = gyroZ_mdps * IMU_MDPS_TO_RADPS;

  /* Temperature has its own (slower) data rate; read it opportunistically
   * without failing the whole sample if it is not ready yet. */
  if (tempReady) {
    ISDS_getTemperature_float(&imu->sensorInterface,
                               &sample->temperature_degC);
  }

  return true;
}

bool Imu_EnableDataReadyInterrupt(Imu_t *imu) {
  if (ISDS_setInterruptActiveLevel(&imu->sensorInterface, ISDS_activeHigh) !=
      WE_SUCCESS) {
    return false;
  }

  if (ISDS_setInterruptPinType(&imu->sensorInterface, ISDS_pushPull) !=
      WE_SUCCESS) {
    return false;
  }

  if (ISDS_enableAccDataReadyINT0(&imu->sensorInterface, ISDS_enable) !=
      WE_SUCCESS) {
    return false;
  }

  if (ISDS_enableGyroDataReadyINT0(&imu->sensorInterface, ISDS_enable) !=
      WE_SUCCESS) {
    return false;
  }

  return ISDS_enableInterrupts(&imu->sensorInterface, ISDS_enable) ==
         WE_SUCCESS;
}
