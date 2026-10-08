/**
 ******************************************************************************
 * @file    barometer.h
 * @brief   Board-level driver for the WSEN-PADS barometer(s).
 *
 * Wraps the low-level WSEN_PADS_2511020213301 SDK (third_party/WESensorsSDK)
 * into a small instance-based API for simple, polled pressure/temperature
 * readings (no FIFO, no interrupts).
 *
 * The board carries two redundant WSEN-PADS barometers, so every function
 * takes a Barometer_t* instance instead of relying on globals -- create one
 * instance per physical sensor.
 ******************************************************************************
 */

#ifndef __BAROMETER_H__
#define __BAROMETER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "main.h"
#include "WSEN_PADS_2511020213301.h"
#include "WeSensorsSDK.h"

/* One pressure/temperature reading. */
typedef struct {
  float pressure_kPa;
  float temperature_degC;
} Barometer_Sample_t;

/* One WSEN-PADS barometer instance. */
typedef struct {
  WE_sensorInterface_t sensorInterface;
} Barometer_t;

/* === API ================================================================== */

/* Fills in the sensor interface for one instance. Must be called before
 * Barometer_Init(). Does not touch the hardware. */
void Barometer_Config(Barometer_t *baro, I2C_HandleTypeDef *hi2c,
                       uint8_t i2cAddress);

/* Probes the sensor (device ID check), applies the default power-on
 * configuration (block data update, FIFO in bypass mode, low-pass filter
 * at ODR/9, and the sensor's maximum 200 Hz output data rate -- so ~22 Hz
 * of bandwidth) and starts sampling. Note that low-noise mode is requested
 * but has no effect at 200 Hz; see the comment in Barometer_Init(), which
 * also covers what running this fast costs on the I2C bus.
 * Returns true on success. */
bool Barometer_Init(Barometer_t *baro);

/* Blocking read: waits for a new sample to be ready and reads it out.
 * Returns true on success. */
bool Barometer_Read(Barometer_t *baro, Barometer_Sample_t *sample);

/* Configures the INT1 pin to assert (active-high, push-pull -- matching
 * this board's PADS1_INT1/PADS2_INT1 EXTI config, GPIO_MODE_IT_RISING in
 * gpio.c) when a new pressure+temperature sample is ready. Call after
 * Barometer_Init(). This does not touch the MCU's EXTI/NVIC config
 * (already set up in gpio.c for both PADS1_INT1 and PADS2_INT1); the
 * application still needs to react to the resulting HAL_GPIO_EXTI_
 * Callback() itself (e.g. set a flag there and call Barometer_Read() from
 * the main loop once it's seen -- Barometer_Read()'s own data-ready wait
 * will then return immediately). Returns true on success. */
bool Barometer_EnableDataReadyInterrupt(Barometer_t *baro);

#ifdef __cplusplus
}
#endif

#endif /* __BAROMETER_H__ */
