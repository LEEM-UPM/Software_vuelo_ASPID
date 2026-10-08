/**
 ******************************************************************************
 * @file    calibration.c
 * @brief   Ground calibration of the IMUs and barometers.
 ******************************************************************************
 */

#include "calibration.h"
#include "watchdog.h"

#include <math.h>
#include <stdbool.h>

#include "main.h"
#include "sensorsI2c.h"

/* === Mounting (TO BE CONFIRMED on the rocket) ==============================
 *
 * Assumed: the board's +Z axis points to the nose of the rocket, so with the
 * rocket vertical (tilt 0) the accelerometers read +g on Z and 0 on X and Y.
 *
 * CALIBRATION_TILT_TOWARD: towards which of the 4 sides of the board the
 * rocket leans when tiltDeg > 0 (the nose moves towards that axis). Chosen
 * arbitrarily for now; the core sends 0 degrees, where it makes no
 * difference.
 *
 * Expected accelerometer reading (a resting accelerometer measures +g
 * "upwards"), g = CALIBRATION_GRAVITY_MPS2, a = tilt:
 *   TOWARD_POS_X: ( -g sin a,        0, g cos a )
 *   TOWARD_NEG_X: ( +g sin a,        0, g cos a )
 *   TOWARD_POS_Y: (        0, -g sin a, g cos a )
 *   TOWARD_NEG_Y: (        0, +g sin a, g cos a )
 * Both IMUs are assumed to have their axes aligned with the board's.
 * ======================================================================== */
#define CALIBRATION_TILT_TOWARD_POS_X 0
#define CALIBRATION_TILT_TOWARD_NEG_X 1
#define CALIBRATION_TILT_TOWARD_POS_Y 2
#define CALIBRATION_TILT_TOWARD_NEG_Y 3

#define CALIBRATION_TILT_TOWARD CALIBRATION_TILT_TOWARD_POS_X

#define CALIBRATION_GRAVITY_MPS2 9.80665f

/* Wire scales of AVIONICS_CALIBRATION (ICD 6.9) */
#define ACCEL_UNIT_MPS2 0.001f
#define GYRO_UNIT_RADPS 0.0001f

/* Last result and statistics -- watch them in the debugger. */
static CanNode_Calibration_t lastCalibration;
static uint32_t calibrationCount;
static uint32_t baroSamples[2];
static uint32_t imuSamples[2];

/* === Private helpers ====================================================== */

typedef struct {
  float sum_kPa;
  uint32_t count;
} BaroAccumulator_t;

typedef struct {
  SensorsI2c_ImuSample_t sum;
  uint32_t count;
} ImuAccumulator_t;

static void ExpectedGravity(float tiltDeg, float expected[3]) {
  const float a = tiltDeg * ((float)M_PI / 180.0f);
  const float horizontal = CALIBRATION_GRAVITY_MPS2 * sinf(a);

  expected[0] = 0.0f;
  expected[1] = 0.0f;
  expected[2] = CALIBRATION_GRAVITY_MPS2 * cosf(a);

#if CALIBRATION_TILT_TOWARD == CALIBRATION_TILT_TOWARD_POS_X
  expected[0] = -horizontal;
#elif CALIBRATION_TILT_TOWARD == CALIBRATION_TILT_TOWARD_NEG_X
  expected[0] = horizontal;
#elif CALIBRATION_TILT_TOWARD == CALIBRATION_TILT_TOWARD_POS_Y
  expected[1] = -horizontal;
#elif CALIBRATION_TILT_TOWARD == CALIBRATION_TILT_TOWARD_NEG_Y
  expected[1] = horizontal;
#else
#error "CALIBRATION_TILT_TOWARD must be one of CALIBRATION_TILT_TOWARD_*"
#endif
}

static void AccumulateImu(ImuAccumulator_t *acc,
                          const SensorsI2c_ImuSample_t *sample) {
  for (uint8_t axis = 0u; axis < 3u; axis++) {
    acc->sum.accel_mps2[axis] += sample->accel_mps2[axis];
    acc->sum.gyro_radps[axis] += sample->gyro_radps[axis];
  }
  acc->count++;
}

/* physical value -> int16 in `unit` steps; 0x7FFF if it does not fit */
static int16_t ToWire(float value, float unit) {
  const float steps = value / unit;
  if (isnan(steps) || (steps < -32768.0f) || (steps > 32766.0f)) {
    return CAN_NODE_CALIBRATION_INVALID_I16;
  }
  return (int16_t)lroundf(steps);
}

static void MarkInvalid(int16_t axes[3]) {
  axes[0] = CAN_NODE_CALIBRATION_INVALID_I16;
  axes[1] = CAN_NODE_CALIBRATION_INVALID_I16;
  axes[2] = CAN_NODE_CALIBRATION_INVALID_I16;
}

/* Mean of one IMU -> offsets. With no samples, nothing is applied and every
 * field is "not calibrated". */
static void FinishImu(uint8_t index, const ImuAccumulator_t *acc,
                      const float expected[3], int16_t accelWire[3],
                      int16_t gyroWire[3]) {
  if (acc->count == 0u) {
    MarkInvalid(accelWire);
    MarkInvalid(gyroWire);
    return;
  }

  SensorsI2c_ImuOffsets_t offsets;
  for (uint8_t axis = 0u; axis < 3u; axis++) {
    const float accelMean = acc->sum.accel_mps2[axis] / (float)acc->count;
    const float gyroMean = acc->sum.gyro_radps[axis] / (float)acc->count;

    offsets.accel_mps2[axis] = accelMean - expected[axis];
    offsets.gyro_radps[axis] = gyroMean;

    accelWire[axis] = ToWire(offsets.accel_mps2[axis], ACCEL_UNIT_MPS2);
    gyroWire[axis] = ToWire(offsets.gyro_radps[axis], GYRO_UNIT_RADPS);
  }

  SensorsI2c_SetImuOffsets(index, &offsets);
}

static float FinishBaro(const BaroAccumulator_t *acc) {
  return (acc->count == 0u) ? NAN : (acc->sum_kPa / (float)acc->count);
}

/* === API =================================================================== */

void Calibration_Run(float tiltDeg, CanNode_Calibration_t *result) {
  BaroAccumulator_t baro[2] = {0};
  ImuAccumulator_t imu[2] = {0};

  /* All sensors in the same loop: each pass reads whichever has a sample
   * ready. Barometers for the first CALIBRATION_BARO_TIME_MS, IMUs for the
   * whole CALIBRATION_IMU_TIME_MS. */
  const uint32_t start = HAL_GetTick();
  uint32_t elapsed;
  while ((elapsed = HAL_GetTick() - start) < CALIBRATION_IMU_TIME_MS) {
    Watchdog_Refresh(); /* this loop is longer than WATCHDOG_TIMEOUT_MS */
    if (elapsed < CALIBRATION_BARO_TIME_MS) {
      for (uint8_t i = 0u; i < 2u; i++) {
        Barometer_Sample_t sample;
        if (SensorsI2c_ReadBarometerRaw(i + 1u, &sample)) {
          baro[i].sum_kPa += sample.pressure_kPa;
          baro[i].count++;
        }
      }
    }

    for (uint8_t i = 0u; i < 2u; i++) {
      SensorsI2c_ImuSample_t sample;
      if (SensorsI2c_ReadImuRaw(i + 1u, &sample)) {
        AccumulateImu(&imu[i], &sample);
      }
    }
  }

  float expected[3];
  ExpectedGravity(tiltDeg, expected);

  FinishImu(1u, &imu[0], expected, result->imu1_accel, result->imu1_gyro);
  FinishImu(2u, &imu[1], expected, result->imu2_accel, result->imu2_gyro);
  result->baro1_kPa = FinishBaro(&baro[0]);
  result->baro2_kPa = FinishBaro(&baro[1]);

  baroSamples[0] = baro[0].count;
  baroSamples[1] = baro[1].count;
  imuSamples[0] = imu[0].count;
  imuSamples[1] = imu[1].count;
}

void Calibration_Update(void) {
  float tiltDeg = 0.0f;
  if (!CanNode_TakeCalibrationRequest(&tiltDeg)) {
    return;
  }

  Calibration_Run(tiltDeg, &lastCalibration);
  calibrationCount++;
  (void)CanNode_SendCalibration(&lastCalibration);
}
