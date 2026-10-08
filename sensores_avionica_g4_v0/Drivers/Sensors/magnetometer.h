/**
 ******************************************************************************
 * @file    magnetometer.h
 * @brief   Board-level driver for the MLX90393 3-axis magnetometer.
 *
 * There is no vendored SDK for this chip (unlike the WSEN-PADS/WSEN-ISDS,
 * which wrap Wurth Elektronik's WESensorsSDK), so this talks I2C directly
 * using the command set from the MLX90393 datasheet (section 15.4, "I2C
 * Communication"): Start Single Measurement (SM) to trigger one conversion,
 * then Read Measurement (RM) to read it back. No FIFO/burst/interrupt mode,
 * following the same "simple polled reading" pattern as barometer.h and
 * imu.h.
 *
 * The only configuration this driver writes is the oversampling/filtering
 * pair (DIG_FILT = 5, OSR = 1 -- see magnetometer.c, which also explains
 * why the power-on defaults could not be left alone). Everything else stays
 * at the factory values: GAIN_SEL=0x7, HALLCONF=0xC, RES_XYZ=0x00,
 * TCMP_EN=0 (datasheet Table 16). The conversion to Tesla in
 * Magnetometer_ReadMeasurement() is only valid for that gain/resolution
 * pair -- see datasheet Table 17 for the sensitivity at other GAIN_SEL /
 * RES_XYZ settings, if this driver is ever extended to change them.
 ******************************************************************************
 */

#ifndef __MAGNETOMETER_H__
#define __MAGNETOMETER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "main.h"

/* Confirmed with the i2c_scan PoC: this board's MLX90393
 * (MLX90393ELW-ABA-014-SP per the README) answers at 0x18. The 5 MSBs
 * (00110) come from the ABA-014 option code, the 2 LSBs from how A0/A1 are
 * strapped (A0 = GND on this board) -- see the datasheet's I2C address table
 * (section 15.4) if a board revision ever changes that strapping. */
#define MLX90393_ADDRESS_I2C_BOARD 0x18u

/* One magnetic field + temperature reading, in SI units. */
typedef struct {
  float fieldX_T; /* Tesla */
  float fieldY_T; /* Tesla */
  float fieldZ_T; /* Tesla */

  float temperature_degC;
} Magnetometer_Sample_t;

/* How far Magnetometer_Init() got -- the last step that succeeded. If Init
 * fails, the step AFTER the one recorded here is the one that failed. */
typedef enum {
  Magnetometer_InitNotStarted = 0,
  Magnetometer_InitProbed = 1,     /* ACKed its I2C address */
  Magnetometer_InitIdle = 2,       /* accepted Exit Mode */
  Magnetometer_InitFilterRead = 3, /* filter register read back */
  Magnetometer_InitDone = 4,       /* filter register written */
} Magnetometer_InitStep_t;

/* One MLX90393 instance. */
typedef struct {
  I2C_HandleTypeDef *hi2c;
  uint8_t i2cAddress; /* 7-bit, not shifted */

  /* Status byte from the most recent command the sensor answered -- the
   * only way it reports whether a command was accepted. Diagnostic only.
   * Bits (datasheet 15.2): 7 BURST_MODE, 6 WOC_MODE, 5 SM_MODE, 4 ERROR,
   * 3 SED, 2 RS, 1:0 D. E.g. after a successful Start Single Measurement
   * expect SM_MODE (0x20) set and ERROR (0x10) clear. */
  uint8_t lastStatus;

  /* Diagnostic: see Magnetometer_InitStep_t. */
  Magnetometer_InitStep_t initStep;

  /* Diagnostic: the raw filter register value read back during Init, and
   * the value written to it (only the low 5 bits should differ). */
  uint16_t filterRegRead;
  uint16_t filterRegWritten;
} Magnetometer_t;

/* === API ================================================================== */

/* Fills in the sensor interface for one instance. Must be called before
 * Magnetometer_Init(). Does not touch the hardware. */
void Magnetometer_Config(Magnetometer_t *mag, I2C_HandleTypeDef *hi2c,
                          uint8_t i2cAddress);

/* Probes the sensor, puts it in the IDLE state (Exit Mode command) and
 * configures its oversampling/filtering. Bounded (unlike Barometer_Init()/
 * Imu_Init()'s retry-forever probe loop): a readiness check plus a handful
 * of command/response round trips, each with the HAL's own timeout.
 * Returns true on success. */
bool Magnetometer_Init(Magnetometer_t *mag);

/* Triggers one on-demand X/Y/Z/temperature conversion (Single Measurement
 * mode), waits out its worst-case conversion time (see
 * MAG_CONVERSION_DELAY_MS in magnetometer.c), then reads it back. Just
 * Magnetometer_StartMeasurement() + a delay + Magnetometer_ReadMeasurement()
 * -- see those for interrupt-driven use, without the blocking delay.
 * Returns true on success (including the sensor's own status byte not
 * flagging an error). */
bool Magnetometer_Read(Magnetometer_t *mag, Magnetometer_Sample_t *sample);

/* Triggers one on-demand X/Y/Z/temperature conversion (Single Measurement
 * mode) without waiting for or reading back the result -- pairs with
 * Magnetometer_ReadMeasurement() for interrupt-driven use: the sensor
 * raises its INT pin (MAG_INT / PC13 on this board, GPIO_MODE_IT_RISING in
 * gpio.c) once the conversion completes, no separate configuration step
 * needed (datasheet section 14.2). Returns true if the command was sent
 * and acknowledged without error. */
bool Magnetometer_StartMeasurement(Magnetometer_t *mag);

/* Reads back the measurement triggered by the most recent
 * Magnetometer_StartMeasurement() call. Does not wait -- call only once
 * the conversion is known to be done (e.g. the MAG_INT interrupt fired).
 * Returns true on success. */
bool Magnetometer_ReadMeasurement(Magnetometer_t *mag,
                                   Magnetometer_Sample_t *sample);

/* Exit Mode: cancels any measurement in progress and returns the sensor to
 * IDLE, clearing its data-ready. Useful to recover a sensor that is stuck
 * (e.g. a trigger that was rejected or lost) before starting over with
 * Magnetometer_StartMeasurement(). Returns true if the sensor accepted it. */
bool Magnetometer_Exit(Magnetometer_t *mag);

#ifdef __cplusplus
}
#endif

#endif /* __MAGNETOMETER_H__ */
