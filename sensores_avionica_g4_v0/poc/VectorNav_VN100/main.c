/******************************************************************************
 * VectorNav VN-100 simple polled reading - Proof of Concept
 *
 * Uses the VectorNav_* library (Drivers/Sensors/vectorNav.h): configures the
 * sensor's binary output once, then polls it in a loop.
 *
 * NOTE: on this board the VN-100 is the only device on USART2 (VNAV_RX/
 * VNAV_TX) and there is no separate debug UART broken out, so this PoC
 * cannot printf() its readings without corrupting the binary stream it is
 * trying to read. Instead it toggles the onboard TEST_LED on every
 * successfully parsed packet as a simple "alive" indicator -- inspect
 * `sample` in a debugger to see the actual values.
 ******************************************************************************/

/* === Includes ============================================================ */
#include "gpio.h"
#include "usart.h"

#include "vectorNav.h"

/* === Globals ============================================================= */
static VectorNav_t vn;

/* === Forward declarations ============================================== */
extern void SystemClock_Config(void);
static void mcu_init(void);

/* === Main ================================================================ */
int main(void) {
  mcu_init();

  VectorNav_Config(&vn, &huart2);
  VectorNav_Init(&vn);

  while (1) {
    VectorNav_Data_t sample;
    if (VectorNav_Read(&vn, &sample)) {
      HAL_GPIO_TogglePin(TEST_LED_GPIO_Port, TEST_LED_Pin);
    }
  }
}

static void mcu_init(void) {
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART2_UART_Init();
}
