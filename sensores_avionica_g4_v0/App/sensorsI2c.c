/**
 ******************************************************************************
 * @file    sensorsI2c.c
 * @brief   I2C sensors: read on data-ready and forwarded over CAN telemetry.
 *
 * All addresses below were confirmed with the i2c_scan PoC (probes every
 * 7-bit address and reports which ones ACK) rather than guessed from the
 * schematic: PADS1 and the WSEN-ISDS are both strapped SAO/SA0 = VCC, same
 * as PADS2, i.e. the *_I2C_1 address on both buses; the magnetometer's
 * address is fixed by its ordering code + A0/A1 strapping (see
 * magnetometer.h).
 ******************************************************************************
 */

#include "sensorsI2c.h"

#include "i2c.h"

#include "barometer.h"
#include "canTelemetry.h"
#include "imu.h"
#include "imu2.h"
#include "magnetometer.h"
#include "sensorIrq.h"
#include "timestamp.h"

/* How long an I2C sensor may go without a successful read before it counts
 * as failed. The slowest of them is the magnetometer at ~65 Hz (~15 ms per
 * conversion), so this is more than ten missed samples of slack -- while
 * still being well under one heartbeat blink period, so a sensor that dies
 * takes the LED with it almost immediately. */
#define I2C_SENSOR_TIMEOUT_MS 200u

/* The magnetometer is the one sensor the MCU has to trigger, and it is only
 * re-triggered from its own data-ready branch -- so a single lost or
 * rejected trigger would otherwise silence it for good. If no data-ready
 * shows up within this long of a trigger (a conversion takes ~15 ms), the
 * loop recovers it. Kept below I2C_SENSOR_TIMEOUT_MS so a one-off loss is
 * fixed before it costs the heartbeat. */
#define MAG_REARM_TIMEOUT_MS 100u

/* Lecturas fallidas seguidas tras las que se da un sensor por caido. Con
 * 20 se aguanta algun fallo suelto del bus sin apagar el sensor, y aun asi
 * se detecta en menos de 100 ms al ritmo al que se lee. */
#define SENSORS_I2C_FAIL_LIMIT 20u

/* Un sensor activo que no da ninguna lectura buena en este tiempo tambien se
 * da por caido: cubre el caso de que deje de pedir atencion (su data-ready
 * no vuelve a dispararse) en vez de fallar la lectura. */
#define SENSORS_I2C_DEAD_MS 1000u

/* Cada cuanto se reintenta inicializar un sensor caido, por si vuelve.
 * Cada intento cuesta ~50 ms (los *_Init() esperan a que el sensor arranque),
 * asi que no conviene bajarlo mucho. */
#define SENSORS_I2C_RETRY_PERIOD_MS 5000u

/* === Globals ===============================================================
 * Latest sample from each sensor -- inspect these from the debugger. Only
 * ever written from the main loop, so these don't need to be volatile. */
static Barometer_Sample_t baro1Sample;
static Barometer_Sample_t baro2Sample;
static Imu_Sample_t imuSample;
static Imu2_Sample_t imu2Sample;
static Magnetometer_Sample_t magSample;

static Barometer_t baro1;
static Barometer_t baro2;
static Imu_t imu;
static Imu2_t imu2;
static Magnetometer_t mag;

/* Estado de cada sensor. Solo se toca desde el bucle principal.
 *  - active: inicializado y respondiendo. Un sensor no activo no se lee (no
 *    gasta bus ni tiempo de bucle), solo se reintenta de vez en cuando.
 *  - lastOkTick: HAL tick de la ultima lectura buena (base de
 *    SensorsI2c_Healthy() y de la deteccion de sensores mudos). */
typedef struct {
  bool active;
  uint32_t lastOkTick;
  uint32_t consecutiveErrors;
  uint32_t errorCount;
  uint32_t retryCount;
  uint32_t lastRetryTick;
  I2C_HandleTypeDef *hi2c; /* bus al que esta conectado */
} SensorState_t;

static SensorState_t sensorState[SensorsI2c_Count];

/* Veces que se ha reinicializado un bus I2C por caerse todos sus sensores */
static uint32_t i2cBusResets;

/* Diagnostic: I2C sensors currently failing the health check, one bit
 * each -- bit 0 PADS1, 1 PADS2, 2 WSEN-ISDS, 3 LSM6DSV32X, 4 MLX90393.
 * 0 means all healthy (and TEST_LED blinking). Watch it in the debugger. */
static uint8_t i2cUnhealthyMask;

/* Magnetometer diagnostics -- watch these alongside mag.lastStatus (see
 * magnetometer.h for its bits) if the magnetometer misbehaves:
 *  - magInitOk: Magnetometer_Init() result (probe + filter config).
 *  - magArmOk: result of the latest Magnetometer_StartMeasurement().
 *  - magTimeoutCount: how many times no data-ready arrived in time. Slowly
 *    growing = occasional lost trigger, recovered. Growing every ~100 ms =
 *    data-ready never arrives at all. If magArmOk is true and
 *    mag.lastStatus shows SM_MODE (0x20) without ERROR (0x10), the sensor
 *    IS measuring and the problem is the MAG_INT line itself; if the data
 *    still comes in the magnetometer keeps working, just polled at ~10 Hz. */
static bool magInitOk;
static bool magArmOk;
static uint32_t magTimeoutCount;
static uint32_t magArmTick; /* HAL tick of the latest trigger */

/* Calibration offsets subtracted from every IMU sample before it is stored
 * and sent (set by App/calibration.c, 0 until the first calibration). Kept
 * in RAM only: a reset clears them. */
static SensorsI2c_ImuOffsets_t imu1Offsets;
static SensorsI2c_ImuOffsets_t imu2Offsets;

/* === Estado de los sensores =============================================== */

/* Inicializa un sensor y deja lista su forma de avisar: los que tienen
 * data-ready, con su interrupcion; el magnetometro, con su primera medida.
 * Devuelve false si el sensor no responde. */
static bool InitSensor(SensorsI2c_Id_t id) {
  switch (id) {
  case SensorsI2c_Barometer1:
    return Barometer_Init(&baro1) && Barometer_EnableDataReadyInterrupt(&baro1);
  case SensorsI2c_Barometer2:
    return Barometer_Init(&baro2) && Barometer_EnableDataReadyInterrupt(&baro2);
  case SensorsI2c_Imu1:
    return Imu_Init(&imu) && Imu_EnableDataReadyInterrupt(&imu);
  case SensorsI2c_Imu2:
    return Imu2_Init(&imu2) && Imu2_EnableDataReadyInterrupt(&imu2);
  case SensorsI2c_Magnetometer:
    magInitOk = Magnetometer_Init(&mag);
    if (!magInitOk) {
      return false;
    }
    magArmOk = Magnetometer_StartMeasurement(&mag);
    magArmTick = HAL_GetTick();
    return magArmOk;
  default:
    return false;
  }
}

/* Reinicia el periferico I2C: es lo que saca a un bus que se ha quedado
 * bloqueado, normalmente porque un sensor se reseteo a mitad de una
 * transferencia y dejo SDA en bajo. */
static void RecoverBus(I2C_HandleTypeDef *hi2c) {
  if (hi2c == NULL) {
    return;
  }
  i2cBusResets++;
  (void)HAL_I2C_DeInit(hi2c);
  (void)HAL_I2C_Init(hi2c);
}

/* Da un sensor por caido. Si con el se han caido todos los de su bus, el
 * problema es del bus y no de los sensores: se reinicia. */
static void Deactivate(SensorsI2c_Id_t id) {
  SensorState_t *state = &sensorState[id];
  if (!state->active) {
    return;
  }
  state->active = false;
  state->consecutiveErrors = 0u;
  /* Que el primer reintento no sea inmediato */
  state->lastRetryTick = HAL_GetTick();

  I2C_HandleTypeDef *hi2c = state->hi2c;
  for (uint32_t i = 0u; i < (uint32_t)SensorsI2c_Count; i++) {
    if ((sensorState[i].hi2c == hi2c) && sensorState[i].active) {
      return; /* al bus le queda algun sensor vivo */
    }
  }
  RecoverBus(hi2c);
}

static void MarkOk(SensorsI2c_Id_t id) {
  sensorState[id].lastOkTick = HAL_GetTick();
  sensorState[id].consecutiveErrors = 0u;
}

static void MarkError(SensorsI2c_Id_t id) {
  SensorState_t *state = &sensorState[id];
  state->errorCount++;
  state->consecutiveErrors++;
  if (state->consecutiveErrors >= SENSORS_I2C_FAIL_LIMIT) {
    Deactivate(id);
  }
}

/* Sensores activos que llevan demasiado tiempo sin dar una lectura buena:
 * no fallan, es que han dejado de avisar. Tambien se dan por caidos. */
static void DropSilentSensors(void) {
  const uint32_t now = HAL_GetTick();
  for (uint32_t i = 0u; i < (uint32_t)SensorsI2c_Count; i++) {
    if (sensorState[i].active &&
        ((now - sensorState[i].lastOkTick) >= SENSORS_I2C_DEAD_MS)) {
      Deactivate((SensorsI2c_Id_t)i);
    }
  }
}

/* Reintenta inicializar los sensores caidos, por si vuelven (un cable flojo,
 * un sensor que se reinicio...). Uno por vuelta como mucho: cada intento
 * bloquea ~50 ms y no conviene encadenarlos. */
static void RetryDownSensors(void) {
  const uint32_t now = HAL_GetTick();

  for (uint32_t i = 0u; i < (uint32_t)SensorsI2c_Count; i++) {
    SensorState_t *state = &sensorState[i];
    if (state->active ||
        ((now - state->lastRetryTick) < SENSORS_I2C_RETRY_PERIOD_MS)) {
      continue;
    }

    state->lastRetryTick = now;
    state->retryCount++;
    if (InitSensor((SensorsI2c_Id_t)i)) {
      state->active = true;
      state->consecutiveErrors = 0u;
      state->lastOkTick = HAL_GetTick();
    }
    return; /* solo un reintento por vuelta */
  }
}

/* === Private helpers ====================================================== */

static void ApplyOffsets(const SensorsI2c_ImuOffsets_t *offsets, float *ax,
                         float *ay, float *az, float *gx, float *gy, float *gz) {
  *ax -= offsets->accel_mps2[0];
  *ay -= offsets->accel_mps2[1];
  *az -= offsets->accel_mps2[2];
  *gx -= offsets->gyro_radps[0];
  *gy -= offsets->gyro_radps[1];
  *gz -= offsets->gyro_radps[2];
}

/* Which values each sensor sends is decided here, not inside canTelemetry.c,
 * so it is obvious at a glance which fields are transmitted -- note that only
 * the barometers send their temperature; the IMUs' and the magnetometer's is
 * deliberately left out. */

static void ServiceBarometer(Barometer_t *baro, Barometer_Sample_t *sample,
                             SensorIrq_Id_t irq, CanTelemetry_SensorId_t canId,
                             SensorsI2c_Id_t id) {
  if (!sensorState[id].active) {
    return;
  }

  uint32_t sampleUs = 0;
  if (!SensorIrq_Take(irq, &sampleUs)) {
    return; /* todavia no hay muestra: no es un error */
  }
  if (!Barometer_Read(baro, sample)) {
    MarkError(id);
    return;
  }

  MarkOk(id);
  const float values[] = {sample->pressure_kPa, sample->temperature_degC};
  CanTelemetry_Send(canId, sampleUs, values, 2);
}

static void ServiceImu1(void) {
  if (!sensorState[SensorsI2c_Imu1].active) {
    return;
  }

  uint32_t sampleUs = 0;
  if (!SensorIrq_Take(SensorIrq_Imu1, &sampleUs)) {
    return;
  }
  if (!Imu_Read(&imu, &imuSample)) {
    MarkError(SensorsI2c_Imu1);
    return;
  }
  MarkOk(SensorsI2c_Imu1);
  ApplyOffsets(&imu1Offsets, &imuSample.accelX_mps2, &imuSample.accelY_mps2,
               &imuSample.accelZ_mps2, &imuSample.gyroX_radps,
               &imuSample.gyroY_radps, &imuSample.gyroZ_radps);
  const float values[] = {imuSample.accelX_mps2, imuSample.accelY_mps2,
                          imuSample.accelZ_mps2, imuSample.gyroX_radps,
                          imuSample.gyroY_radps, imuSample.gyroZ_radps};
  CanTelemetry_Send(CanTelemetry_Imu1, sampleUs, values, 6);
}

static void ServiceImu2(void) {
  if (!sensorState[SensorsI2c_Imu2].active) {
    return;
  }

  uint32_t sampleUs = 0;
  if (!SensorIrq_Take(SensorIrq_Imu2, &sampleUs)) {
    return;
  }
  if (!Imu2_Read(&imu2, &imu2Sample)) {
    MarkError(SensorsI2c_Imu2);
    return;
  }
  MarkOk(SensorsI2c_Imu2);
  ApplyOffsets(&imu2Offsets, &imu2Sample.accelX_mps2, &imu2Sample.accelY_mps2,
               &imu2Sample.accelZ_mps2, &imu2Sample.gyroX_radps,
               &imu2Sample.gyroY_radps, &imu2Sample.gyroZ_radps);
  const float values[] = {imu2Sample.accelX_mps2, imu2Sample.accelY_mps2,
                          imu2Sample.accelZ_mps2, imu2Sample.gyroX_radps,
                          imu2Sample.gyroY_radps, imu2Sample.gyroZ_radps};
  CanTelemetry_Send(CanTelemetry_Imu2, sampleUs, values, 6);
}

/* Serviced on data-ready as usual, and ALSO when a trigger has gone
 * unanswered for MAG_REARM_TIMEOUT_MS -- see that constant. On a timeout the
 * reading is still attempted first: if the conversion did finish and only
 * the data-ready signal went missing, the sample is good. If there is
 * nothing to read, the sensor is taken back to IDLE (Exit Mode) to clear
 * whatever state swallowed the trigger. Either way it is then re-triggered,
 * so it can never stay silent. On a timeout there was no interrupt to take a
 * timestamp from, so the sample is stamped with the time it is read -- later
 * than the real end of conversion, by up to MAG_REARM_TIMEOUT_MS. */
static void ServiceMagnetometer(void) {
  if (!sensorState[SensorsI2c_Magnetometer].active) {
    return;
  }

  uint32_t sampleUs = 0;
  bool magPending = SensorIrq_Take(SensorIrq_Magnetometer, &sampleUs);
  bool magTimedOut =
      !magPending && ((HAL_GetTick() - magArmTick) >= MAG_REARM_TIMEOUT_MS);
  if (!magPending && !magTimedOut) {
    return;
  }

  if (magTimedOut) {
    SensorIrq_Clear(SensorIrq_Magnetometer);
    magTimeoutCount++;
    sampleUs = Timestamp_NowUs();
  }

  if (Magnetometer_ReadMeasurement(&mag, &magSample)) {
    MarkOk(SensorsI2c_Magnetometer);
    const float values[] = {magSample.fieldX_T, magSample.fieldY_T,
                            magSample.fieldZ_T};
    CanTelemetry_Send(CanTelemetry_Magnetometer, sampleUs, values, 3);
  } else if (magTimedOut) {
    MarkError(SensorsI2c_Magnetometer);
    Magnetometer_Exit(&mag);
    if (!sensorState[SensorsI2c_Magnetometer].active) {
      return; /* se acaba de dar por caido: no re-armar */
    }
  }

  magArmOk = Magnetometer_StartMeasurement(&mag); /* re-arm */
  magArmTick = HAL_GetTick();
}

/* === API =================================================================== */

/* Solo configura (direcciones y bus): no toca el hardware. La
 * inicializacion de verdad la hace SensorsI2c_Start(), que ya puede fallar
 * sensor a sensor sin parar a los demas. */
void SensorsI2c_Init(void) {
  Barometer_Config(&baro1, &hi2c1, PADS_ADDRESS_I2C_1);
  Barometer_Config(&baro2, &hi2c2, PADS_ADDRESS_I2C_1);
  Imu_Config(&imu, &hi2c1, ISDS_ADDRESS_I2C_1);
  Imu2_Config(&imu2, &hi2c2, LSM6DSV32X_ADDRESS_I2C_BOARD);
  Magnetometer_Config(&mag, &hi2c1, MLX90393_ADDRESS_I2C_BOARD);

  sensorState[SensorsI2c_Barometer1].hi2c = &hi2c1;
  sensorState[SensorsI2c_Barometer2].hi2c = &hi2c2;
  sensorState[SensorsI2c_Imu1].hi2c = &hi2c1;
  sensorState[SensorsI2c_Imu2].hi2c = &hi2c2;
  sensorState[SensorsI2c_Magnetometer].hi2c = &hi2c1;
}

/* Inicializa cada sensor y arma su data-ready. El que no responda se queda
 * marcado como caido y se reintenta luego (SENSORS_I2C_RETRY_PERIOD_MS); el
 * resto de la placa arranca igual. */
void SensorsI2c_Start(void) {
  const uint32_t now = HAL_GetTick();

  for (uint32_t i = 0u; i < (uint32_t)SensorsI2c_Count; i++) {
    sensorState[i].active = InitSensor((SensorsI2c_Id_t)i);
    sensorState[i].lastOkTick = now;
    sensorState[i].lastRetryTick = now;
  }
}

void SensorsI2c_Service(void) {
  ServiceBarometer(&baro1, &baro1Sample, SensorIrq_Barometer1,
                   CanTelemetry_Barometer1, SensorsI2c_Barometer1);
  ServiceBarometer(&baro2, &baro2Sample, SensorIrq_Barometer2,
                   CanTelemetry_Barometer2, SensorsI2c_Barometer2);
  ServiceImu1();
  ServiceImu2();
  ServiceMagnetometer();

  DropSilentSensors();
  RetryDownSensors();
}

/* Covers both failure modes: a read that errored out, and a sensor that
 * silently stopped being read (its timestamp simply goes stale). Also
 * refreshes i2cUnhealthyMask, so when the LED is dark the debugger shows
 * exactly which sensor is to blame.
 *
 * The subtractions are on purpose: unsigned arithmetic makes them behave
 * correctly across the HAL tick's ~49-day wraparound. */
bool SensorsI2c_Healthy(void) {
  const uint32_t now = HAL_GetTick();
  uint8_t mask = 0;

  for (uint32_t i = 0u; i < (uint32_t)SensorsI2c_Count; i++) {
    if (!sensorState[i].active ||
        ((now - sensorState[i].lastOkTick) >= I2C_SENSOR_TIMEOUT_MS)) {
      mask |= (uint8_t)(1u << i);
    }
  }

  i2cUnhealthyMask = mask;
  return mask == 0;
}

uint8_t SensorsI2c_ActiveMask(void) {
  uint8_t mask = 0;
  for (uint32_t i = 0u; i < (uint32_t)SensorsI2c_Count; i++) {
    if (sensorState[i].active) {
      mask |= (uint8_t)(1u << i);
    }
  }
  return mask;
}

bool SensorsI2c_IsActive(SensorsI2c_Id_t id) {
  return (id < SensorsI2c_Count) && sensorState[id].active;
}

uint32_t SensorsI2c_ErrorCount(SensorsI2c_Id_t id) {
  return (id < SensorsI2c_Count) ? sensorState[id].errorCount : 0u;
}

uint32_t SensorsI2c_RetryCount(SensorsI2c_Id_t id) {
  return (id < SensorsI2c_Count) ? sensorState[id].retryCount : 0u;
}

uint32_t SensorsI2c_BusResetCount(void) { return i2cBusResets; }

/* === Raw access for the calibration ===================================== */

/* Same data-ready path as the Service*() functions, but the sample is
 * returned raw (no offsets) and nothing is sent over CAN. */

bool SensorsI2c_ReadBarometerRaw(uint8_t index, Barometer_Sample_t *sample) {
  uint32_t sampleUs = 0;
  if (index == 1u) {
    return SensorIrq_Take(SensorIrq_Barometer1, &sampleUs) &&
           Barometer_Read(&baro1, sample);
  }
  if (index == 2u) {
    return SensorIrq_Take(SensorIrq_Barometer2, &sampleUs) &&
           Barometer_Read(&baro2, sample);
  }
  return false;
}

bool SensorsI2c_ReadImuRaw(uint8_t index, SensorsI2c_ImuSample_t *sample) {
  uint32_t sampleUs = 0;

  if (index == 1u) {
    Imu_Sample_t raw;
    if (!SensorIrq_Take(SensorIrq_Imu1, &sampleUs) || !Imu_Read(&imu, &raw)) {
      return false;
    }
    sample->accel_mps2[0] = raw.accelX_mps2;
    sample->accel_mps2[1] = raw.accelY_mps2;
    sample->accel_mps2[2] = raw.accelZ_mps2;
    sample->gyro_radps[0] = raw.gyroX_radps;
    sample->gyro_radps[1] = raw.gyroY_radps;
    sample->gyro_radps[2] = raw.gyroZ_radps;
    return true;
  }

  if (index == 2u) {
    Imu2_Sample_t raw;
    if (!SensorIrq_Take(SensorIrq_Imu2, &sampleUs) || !Imu2_Read(&imu2, &raw)) {
      return false;
    }
    sample->accel_mps2[0] = raw.accelX_mps2;
    sample->accel_mps2[1] = raw.accelY_mps2;
    sample->accel_mps2[2] = raw.accelZ_mps2;
    sample->gyro_radps[0] = raw.gyroX_radps;
    sample->gyro_radps[1] = raw.gyroY_radps;
    sample->gyro_radps[2] = raw.gyroZ_radps;
    return true;
  }

  return false;
}

void SensorsI2c_SetImuOffsets(uint8_t index,
                              const SensorsI2c_ImuOffsets_t *offsets) {
  if (index == 1u) {
    imu1Offsets = *offsets;
  } else if (index == 2u) {
    imu2Offsets = *offsets;
  }
}
