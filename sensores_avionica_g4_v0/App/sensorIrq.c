/**
 ******************************************************************************
 * @file    sensorIrq.c
 * @brief   Data-ready interrupts of the I2C sensors.
 ******************************************************************************
 */

#include "sensorIrq.h"

#include "main.h"
#include "timestamp.h"

typedef struct {
  GPIO_TypeDef *port;
  uint16_t pin;
} SensorIrq_Pin_t;

/* Data-ready pin of each sensor, in SensorIrq_Id_t order.
 *
 * IMU2_INT0_Pin (PA10) is the LSM6DSV32X's INT1 pad: the CubeMX labels for
 * the high-g IMU are off by one against the schematic (schematic
 * IMU2_INT1/INT2 = macros IMU2_INT0_Pin/IMU2_INT1_Pin). See imu2.h. */
static const SensorIrq_Pin_t pins[SensorIrq_Count] = {
    [SensorIrq_Barometer1] = {PADS1_INT1_GPIO_Port, PADS1_INT1_Pin},
    [SensorIrq_Barometer2] = {PADS2_INT1_GPIO_Port, PADS2_INT1_Pin},
    [SensorIrq_Imu1] = {IMU1_INT0_GPIO_Port, IMU1_INT0_Pin},
    [SensorIrq_Imu2] = {IMU2_INT0_GPIO_Port, IMU2_INT0_Pin},
    [SensorIrq_Magnetometer] = {MAG_INT_GPIO_Port, MAG_INT_Pin},
};

/* Set from HAL_GPIO_EXTI_Callback() (interrupt context), cleared from the
 * main loop once acted on. */
static volatile bool dataReady[SensorIrq_Count];

/* Timestamp (Timestamp_NowUs(), microseconds) of each sensor's most recent
 * data-ready edge, captured alongside the flag above -- this is what goes out
 * over CAN with the sample. Read only through SensorIrq_Take(), which picks
 * up flag and timestamp together. */
static volatile uint32_t edgeUs[SensorIrq_Count];

bool SensorIrq_Take(SensorIrq_Id_t id, uint32_t *sampleUs) {
  /* The flag and its timestamp are taken together with interrupts masked, so
   * an edge landing in between can't pair one edge's flag with another
   * edge's timestamp, nor be cleared without having been seen. */
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  bool edgeSeen = dataReady[id];
  uint32_t stampUs = edgeUs[id];
  dataReady[id] = false;
  __set_PRIMASK(primask);

  if (edgeSeen) {
    *sampleUs = stampUs;
    return true;
  }
  if (HAL_GPIO_ReadPin(pins[id].port, pins[id].pin) == GPIO_PIN_SET) {
    *sampleUs = Timestamp_NowUs();
    return true;
  }
  return false;
}

void SensorIrq_Clear(SensorIrq_Id_t id) { dataReady[id] = false; }

/* GPIO EXTI: fires for PADS1_INT1, PADS2_INT1, IMU1_INT0, IMU2_INT0 and
 * MAG_INT (see gpio.c) -- only the pins in `pins` are handled here. */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  /* Read before anything else, so it is as close to the data-ready edge as
   * it gets -- the only delay left is the EXTI dispatch itself (microseconds,
   * unless a higher-priority interrupt is running). */
  uint32_t nowUs = Timestamp_NowUs();

  for (uint32_t id = 0; id < (uint32_t)SensorIrq_Count; id++) {
    if (pins[id].pin == GPIO_Pin) {
      edgeUs[id] = nowUs;
      dataReady[id] = true;
      return;
    }
  }
}
