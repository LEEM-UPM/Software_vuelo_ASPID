/******************************************************************************
 * LSM6DSV32X high-g IMU simple polled reading - Proof of Concept
 *
 * Concept:
 *  - No FIFO, no interrupts
 *  - Poll the sensor at a fixed rate and print the reading
 *
 * Uses the Imu2_* library (Drivers/Sensors/imu2.h), which wraps ST's official
 * register-level driver (third_party/ST_LSM6DSV32X). Configured for
 * +-8 g / +-1000 dps at 480 Hz -- narrower than the part's +-32 g /
 * +-4000 dps maximum on purpose, see imu2.h.
 *
 * Board wiring (see schematic): the LSM6DSV32X (IMU 2) is on I2C2 (PA8 SDA
 * / PA9 SCL) -- NOT I2C1, where the main IMU lives -- sharing that bus with
 * barometer PADS2 and the WSEN-HIDS. Its address was confirmed with the
 * i2c_scan PoC (0x6B, i.e. SDO/SA0 strapped high).
 ******************************************************************************/

/* === Includes ============================================================ */
#include <stdio.h>

#include "gpio.h"
#include "i2c.h"
#include "usart.h"

#include "imu2.h"

/* === Configuration ======================================================= */
#define DEBUG_MODE true

/* === Globals ============================================================= */
static Imu2_t imu2;

/* === Forward declarations ============================================== */
extern void SystemClock_Config(void);
static void mcu_init(void);

/* === Main ================================================================ */
int main(void) {
  mcu_init();

  Imu2_Config(&imu2, &hi2c2, LSM6DSV32X_ADDRESS_I2C_BOARD);
  Imu2_Init(&imu2);

  printf("LSM6DSV32X Ready.\r\n");

  while (1) {
    Imu2_Sample_t sample;
    if (Imu2_Read(&imu2, &sample)) {
      printf("Acc: %.2f, %.2f, %.2f m/s^2 | Gyro: %.3f, %.3f, %.3f rad/s | "
             "Temp: %.2f degC\r\n",
             (double)sample.accelX_mps2, (double)sample.accelY_mps2,
             (double)sample.accelZ_mps2, (double)sample.gyroX_radps,
             (double)sample.gyroY_radps, (double)sample.gyroZ_radps,
             (double)sample.temperature_degC);
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
