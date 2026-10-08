/******************************************************************************
 * I2C bus scanner - Proof of Concept
 *
 * Probes every 7-bit I2C address on both buses and records which ones ACK,
 * instead of guessing addresses from the schematic's SAO/SA0 strapping.
 *
 * Board wiring (see schematic):
 *  - I2C1 (PB7 SDA / PB8 SCL): PADS1 (barometer 1), WSEN-ISDS (main IMU),
 *    MLX90393 magnetometer.
 *  - I2C2 (PA8 SDA / PA9 SCL): PADS2 (barometer 2), LSM6DSV32X (high-g IMU),
 *    WSEN-HIDS (humidity).
 *
 * No UART needed (see App/main.c for why none is free on this
 * board anyway): results are kept in globals meant to be inspected from the
 * debugger (e.g. a Live Expressions / Watch view) once the scan is done --
 * this is a Debug build (see CMakeLists.txt), so the compiler won't
 * optimize these away even though only the debugger reads them.
 ******************************************************************************/

/* === Includes ============================================================ */
#include <stdint.h>

#include "gpio.h"
#include "i2c.h"

/* === Configuration ======================================================= */
#define I2C_SCAN_MIN_ADDR 0x08u /* 0x00-0x07 are reserved */
#define I2C_SCAN_MAX_ADDR 0x77u /* 0x78-0x7F are reserved */
#define I2C_SCAN_MAX_RESULTS 16u

/* === Globals ===============================================================
 * Inspect after a run: *Count is how many addresses ACKed on that bus,
 * *Addrs[0 .. *Count - 1] lists them (plain 7-bit values, not shifted). */
static uint8_t i2c1FoundAddrs[I2C_SCAN_MAX_RESULTS];
static uint8_t i2c1FoundCount;
static uint8_t i2c2FoundAddrs[I2C_SCAN_MAX_RESULTS];
static uint8_t i2c2FoundCount;

/* === Forward declarations ============================================== */
extern void SystemClock_Config(void);
static void mcu_init(void);
static void ScanBus(I2C_HandleTypeDef *hi2c, uint8_t *foundAddrs,
                     uint8_t *foundCount);

/* === Main ================================================================ */
int main(void) {
  mcu_init();

  ScanBus(&hi2c1, i2c1FoundAddrs, &i2c1FoundCount);
  ScanBus(&hi2c2, i2c2FoundAddrs, &i2c2FoundCount);

  while (1) {
    /* Blink out the total device count found across both buses, so there's
     * a visible sign of life even without a debugger attached. */
    uint8_t total = (uint8_t)(i2c1FoundCount + i2c2FoundCount);
    for (uint8_t i = 0; i < total; i++) {
      HAL_GPIO_WritePin(TEST_LED_GPIO_Port, TEST_LED_Pin, GPIO_PIN_SET);
      HAL_Delay(150);
      HAL_GPIO_WritePin(TEST_LED_GPIO_Port, TEST_LED_Pin, GPIO_PIN_RESET);
      HAL_Delay(150);
    }
    HAL_Delay(1000);
  }
}

/* Probes every address in [I2C_SCAN_MIN_ADDR, I2C_SCAN_MAX_ADDR] on `hi2c`,
 * appending the ones that ACK to `foundAddrs` (capped at
 * I2C_SCAN_MAX_RESULTS) and resetting/updating `*foundCount`. Short trials
 * and timeout: an unanswered address should not make the whole scan slow. */
static void ScanBus(I2C_HandleTypeDef *hi2c, uint8_t *foundAddrs,
                     uint8_t *foundCount) {
  *foundCount = 0;
  for (uint8_t addr = I2C_SCAN_MIN_ADDR; addr <= I2C_SCAN_MAX_ADDR; addr++) {
    if (HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(addr << 1), 2, 5) == HAL_OK) {
      if (*foundCount < I2C_SCAN_MAX_RESULTS) {
        foundAddrs[(*foundCount)++] = addr;
      }
    }
  }
}

static void mcu_init(void) {
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
}
