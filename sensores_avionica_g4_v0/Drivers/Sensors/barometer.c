/**
 ******************************************************************************
 * @file    barometer.c
 * @brief   Board-level driver for the WSEN-PADS barometer(s).
 ******************************************************************************
 */

#include "barometer.h"

#include "platform.h"

/* Nothing here may wait forever: a sensor that does not answer must fail and
 * let the main loop carry on with the others. */
#define BAROMETER_READY_TIMEOUT_MS 20u /* interface probe / soft reset      */
#define BAROMETER_DATA_TIMEOUT_MS 10u  /* data-ready poll (ODR is 200 Hz)   */

/* I2C transfer timeouts of the WSEN driver. A read is ~0.35 ms at 400 kHz,
 * so anything longer than this means the bus is in trouble; waiting a second
 * for it (the value this used to have) would stall the whole loop. */
#define BAROMETER_I2C_TIMEOUT_MS 20u

void Barometer_Config(Barometer_t *baro, I2C_HandleTypeDef *hi2c,
                       uint8_t i2cAddress) {
  baro->sensorInterface.sensorType = WE_PADS;
  baro->sensorInterface.interfaceType = WE_i2c;
  baro->sensorInterface.options.i2c.address = i2cAddress;
  baro->sensorInterface.options.i2c.burstMode = 1;
  baro->sensorInterface.options.i2c.slaveTransmitterMode = 0;
  baro->sensorInterface.options.i2c.useRegAddrMsbForMultiBytesRead = 0;
  baro->sensorInterface.options.i2c.reserved = 0;
  baro->sensorInterface.options.readTimeout = BAROMETER_I2C_TIMEOUT_MS;
  baro->sensorInterface.options.writeTimeout = BAROMETER_I2C_TIMEOUT_MS;
  baro->sensorInterface.handle = hi2c;
}

bool Barometer_Init(Barometer_t *baro) {
  HAL_Delay(50);

  uint32_t start = HAL_GetTick();
  while (WE_isSensorInterfaceReady(&baro->sensorInterface) != WE_SUCCESS) {
    if ((HAL_GetTick() - start) >= BAROMETER_READY_TIMEOUT_MS) {
      return false; /* not on the bus: the caller retries later */
    }
  }

  uint8_t deviceId = 0;
  if (PADS_getDeviceID(&baro->sensorInterface, &deviceId) != WE_SUCCESS) {
    return false;
  }

  if (deviceId != PADS_DEVICE_ID_VALUE) {
    return false;
  }

  /* Soft reset */
  PADS_softReset(&baro->sensorInterface, PADS_enable);
  PADS_state_t resetState;
  start = HAL_GetTick();
  do {
    if (PADS_getSoftResetState(&baro->sensorInterface, &resetState) !=
        WE_SUCCESS) {
      return false;
    }
    if ((HAL_GetTick() - start) >= BAROMETER_READY_TIMEOUT_MS) {
      return false; /* the reset never finished */
    }
  } while (resetState);

  /* Sensor configuration.
   *
   * NOTE on low-noise mode: the sensor only honours LOW_NOISE_EN below
   * 100 Hz (see the register documentation in the SDK header), so at the
   * 200 Hz ODR set below it is silently ignored and the sensor runs in
   * low-power mode -- i.e. more noise per sample, the price paid for the
   * rate. It is set anyway so that it takes effect again if the ODR is
   * ever dropped back below 100 Hz. */
  PADS_setPowerMode(&baro->sensorInterface, PADS_lowNoise);
  PADS_enableAutoIncrement(&baro->sensorInterface, PADS_enable);

  /* Low-pass filter at ODR/9 (the wider of the two settings), i.e. ~22 Hz
   * of bandwidth at the 200 Hz ODR below. The narrower ODR/20 option would
   * cut that to ~10 Hz, which throws away much of the point of running
   * fast. Disabling the filter entirely would give ODR/2. */
  PADS_enableLowPassFilter(&baro->sensorInterface, PADS_enable);
  PADS_setLowPassFilterConfig(&baro->sensorInterface, PADS_lpFilterBW1);
  PADS_enableBlockDataUpdate(&baro->sensorInterface, PADS_enable);

  /* No FIFO: just free-running conversions at a fixed rate, read out from
   * the output registers when the data-ready interrupt fires (see
   * Barometer_EnableDataReadyInterrupt()).
   *
   * 200 Hz is this sensor's maximum. On the I2C budget: one
   * Barometer_Read() is ~0.35 ms with both buses at 400 kHz (see
   * i2c.c/the .ioc), so 200 Hz costs ~7% of the bus -- comfortable even on
   * I2C1, where this barometer shares with the WSEN-ISDS (~13% at 208 Hz)
   * and the magnetometer. It was ~28% back when the buses ran at 100 kHz,
   * which is why they were raised. Note 400 kHz is the system ceiling: the
   * MLX90393 on I2C1 tops out there. */
  PADS_setFifoMode(&baro->sensorInterface, PADS_bypassMode);
  PADS_setOutputDataRate(&baro->sensorInterface, PADS_outputDataRate200Hz);

  return true;
}

bool Barometer_Read(Barometer_t *baro, Barometer_Sample_t *sample) {
  PADS_state_t pressureReady = PADS_disable;
  PADS_state_t temperatureReady = PADS_disable;

  const uint32_t start = HAL_GetTick();
  do {
    if (PADS_isDataAvailable(&baro->sensorInterface, &temperatureReady,
                              &pressureReady) != WE_SUCCESS) {
      return false;
    }
    if ((HAL_GetTick() - start) >= BAROMETER_DATA_TIMEOUT_MS) {
      return false; /* data-ready fired but the data never showed up */
    }
  } while (!pressureReady || !temperatureReady);

  if (PADS_getPressure_float(&baro->sensorInterface, &sample->pressure_kPa) !=
      WE_SUCCESS) {
    return false;
  }

  if (PADS_getTemperature_float(&baro->sensorInterface,
                                 &sample->temperature_degC) != WE_SUCCESS) {
    return false;
  }

  return true;
}

bool Barometer_EnableDataReadyInterrupt(Barometer_t *baro) {
  if (PADS_setInterruptActiveLevel(&baro->sensorInterface, PADS_activeHigh) !=
      WE_SUCCESS) {
    return false;
  }

  if (PADS_setInterruptPinType(&baro->sensorInterface, PADS_pushPull) !=
      WE_SUCCESS) {
    return false;
  }

  return PADS_enableDataReadyInterrupt(&baro->sensorInterface, PADS_enable) ==
         WE_SUCCESS;
}
