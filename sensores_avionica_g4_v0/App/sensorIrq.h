/**
 ******************************************************************************
 * @file    sensorIrq.h
 * @brief   Data-ready interrupts of the I2C sensors.
 *
 * PADS1, PADS2, WSEN-ISDS, LSM6DSV32X and MLX90393 each have a data-ready
 * pin wired to an EXTI (see gpio.c/stm32g4xx_it.c -- CubeMX had this for
 * everything except PADS1_INT1 (PA15), which had to be added by hand, see
 * main.h's comment on IMU1_INT1 about the EXTI15 line it had to give up for
 * it). HAL_GPIO_EXTI_Callback() (in sensorIrq.c) only sets one flag per
 * sensor and captures the timestamp of the edge; the main loop reads the
 * sensor when SensorIrq_Take() says there is something to read.
 ******************************************************************************
 */

#ifndef __SENSORIRQ_H__
#define __SENSORIRQ_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  SensorIrq_Barometer1 = 0, /* PADS1_INT1 */
  SensorIrq_Barometer2,     /* PADS2_INT1 */
  SensorIrq_Imu1,           /* IMU1_INT0 */
  SensorIrq_Imu2,           /* IMU2_INT0 */
  SensorIrq_Magnetometer,   /* MAG_INT */
  SensorIrq_Count
} SensorIrq_Id_t;

/* True if a sensor has a sample waiting: its EXTI flag is set, or its
 * (active-high, level) data-ready line is still high. Clears the flag, and
 * writes the timestamp to send with the sample to `*sampleUs`.
 *
 * Both checks are needed: the EXTIs are edge triggered, but every one of
 * these data-ready lines is level -- it rises when a sample is ready and only
 * falls once that sample is read. If the next sample lands before the loop
 * gets round to reading the previous one, the line never drops, there is no
 * new edge, and without the level check that sensor would never be read
 * again. At 416-480 Hz with this much blocking I2C in the loop, that happens
 * within milliseconds of boot.
 *
 * `*sampleUs` is normally the timestamp captured in the EXTI interrupt. If
 * only the level is high (missed edge), there is no interrupt timestamp and
 * "now" is used instead: an upper bound on when the sample became ready. */
bool SensorIrq_Take(SensorIrq_Id_t id, uint32_t *sampleUs);

/* Drops a pending flag without reading it (used on the magnetometer's
 * re-arm timeout). */
void SensorIrq_Clear(SensorIrq_Id_t id);

#ifdef __cplusplus
}
#endif

#endif /* __SENSORIRQ_H__ */
