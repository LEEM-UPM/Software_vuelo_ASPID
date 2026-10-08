/******************************************************************************
 * WSEN-PADS simple polled reading - Proof of Concept
 *
 * Concept:
 *  - No FIFO, no interrupts
 *  - Poll the sensor at a fixed rate and print the reading
 *
 * Uses the Barometer_* library (Drivers/Sensors/barometer.h) which wraps the
 * low-level WSEN_PADS SDK.
 ******************************************************************************/

/* === Includes ============================================================ */
#include <stdio.h>

#include "gpio.h"
#include "i2c.h"
#include "usart.h"

#include "barometer.h"

/* === Configuration ======================================================= */
#define DEBUG_MODE true

/* === Globals ============================================================= */
static Barometer_t baro;

/* === Forward declarations ============================================== */
extern void SystemClock_Config(void);
static void mcu_init(void);

/* === Main ================================================================ */
int main(void) {
  mcu_init();

  Barometer_Config(&baro, &hi2c2, PADS_ADDRESS_I2C_1);
  Barometer_Init(&baro);

  printf("WSEN-PADS Ready.\r\n");

  while (1) {
    Barometer_Sample_t sample;
    if (Barometer_Read(&baro, &sample)) {
      printf("Pres: %.2f kPa | Temp: %.2f degC\r\n", sample.pressure_kPa,
             sample.temperature_degC);
    }
    HAL_Delay(100);
  }
}

static void mcu_init(void) {
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
}

int __io_putchar(int ch) {
#if DEBUG_MODE
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
#endif
  return ch;
}
