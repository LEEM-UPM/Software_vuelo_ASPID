/******************************************************************************
 * WSEN-ISDS simple polled reading - Proof of Concept
 *
 * Concept:
 *  - No FIFO, no interrupts
 *  - Poll the sensor at a fixed rate and print the reading
 *
 * Uses the Imu_* library (Drivers/Sensors/imu.h) which wraps the low-level
 * WSEN_ISDS SDK.
 *
 * Board wiring (see schematic): the WSEN-ISDS (IMU 1, the main/low-g IMU,
 * IMU1_INT0/INT1 on PC14/PC15) is on I2C1 (PB7 SDA / PB8 SCL), sharing that
 * bus with barometer PADS1 and the MLX90393 magnetometer. Confirmed with
 * the i2c_scan PoC (probes every 7-bit address and reports which ones ACK):
 * it's strapped SA0 = VCC, i.e. ISDS_ADDRESS_I2C_1 (0x6B).
 ******************************************************************************/

/* === Includes ============================================================ */
#include <stdio.h>

#include "gpio.h"
#include "i2c.h"
#include "usart.h"

#include "imu.h"

/* === Configuration ======================================================= */
#define DEBUG_MODE true

/* === Globals ============================================================= */
static Imu_t imu;

/* === Forward declarations ============================================== */
extern void SystemClock_Config(void);
static void mcu_init(void);

/* === Main ================================================================ */
int main(void) {
  mcu_init();

  Imu_Config(&imu, &hi2c1, ISDS_ADDRESS_I2C_1);
  Imu_Init(&imu);

  printf("WSEN-ISDS Ready.\r\n");

  while (1) {
    Imu_Sample_t sample;
    if (Imu_Read(&imu, &sample)) {
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
}

int __io_putchar(int ch) {
#if DEBUG_MODE
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
#endif
  return ch;
}
