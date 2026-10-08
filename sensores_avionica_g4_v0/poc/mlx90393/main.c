/******************************************************************************
 * MLX90393 magnetometer simple polled reading - Proof of Concept
 *
 * Concept:
 *  - Single Measurement mode: trigger one on-demand conversion per read
 *  - Poll at a fixed rate and print the reading
 *
 * Uses the Magnetometer_* library (Drivers/Sensors/magnetometer.h), which talks
 * the sensor's I2C command set directly (there is no vendored SDK for this
 * chip, unlike the WSEN parts).
 *
 * Board wiring (see schematic): the MLX90393 is on I2C1 (PB7 SDA / PB8
 * SCL), sharing that bus with barometer PADS1 and the WSEN-ISDS main IMU.
 * Its address was confirmed with the i2c_scan PoC (0x18, matching the
 * ABA-014 option code the README lists for this board).
 ******************************************************************************/

/* === Includes ============================================================ */
#include <stdio.h>

#include "gpio.h"
#include "i2c.h"
#include "usart.h"

#include "magnetometer.h"

/* === Configuration ======================================================= */
#define DEBUG_MODE true

/* === Globals ============================================================= */
static Magnetometer_t mag;

/* === Forward declarations ============================================== */
extern void SystemClock_Config(void);
static void mcu_init(void);

/* === Main ================================================================ */
int main(void) {
  mcu_init();

  Magnetometer_Config(&mag, &hi2c1, MLX90393_ADDRESS_I2C_BOARD);
  Magnetometer_Init(&mag);

  printf("MLX90393 Ready.\r\n");

  while (1) {
    Magnetometer_Sample_t sample;
    if (Magnetometer_Read(&mag, &sample)) {
      printf("Bx: %.2f uT | By: %.2f uT | Bz: %.2f uT | Temp: %.2f degC\r\n",
             (double)(sample.fieldX_T * 1e6f),
             (double)(sample.fieldY_T * 1e6f),
             (double)(sample.fieldZ_T * 1e6f),
             (double)sample.temperature_degC);
    }
    HAL_Delay(100);
  }
}

static void mcu_init(void) {
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
}

int __io_putchar(int ch) {
#if DEBUG_MODE
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
#endif
  return ch;
}
