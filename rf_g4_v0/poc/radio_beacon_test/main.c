/**
 * @file    main.c
 * @brief   Baliza de prueba sin CAN: comprueba el driver y el BSP tal cual
 *          quedan, sin ajustes por encima.
 *
 * Igual que la baliza del firmware de vuelo (12 bytes, 1 Hz), pero sin
 * esperar el wake up del core, para poder validar la radio sola.
 */
#include <stdio.h>
#include <string.h>

#include "board.h"
#include "bsp_sx1262.h"
#include "gpio.h"
#include "main.h"
#include "sx1262.h"
#include "usart.h"

#define TX_PERIOD_MS 1000U
#define TX_TEXT "hola"
#define TX_PAYLOAD_LEN (sizeof(TX_TEXT) - 1U)

volatile uint32_t diag_tx_ok;
volatile uint32_t diag_tx_fail;
volatile uint32_t diag_last_tx_ms;

int __io_putchar(int ch) {
  HAL_UART_Transmit(&hlpuart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

int main(void) {
  board_init();

  if (!sx1262_init(bsp_sx1262_get_config())) {
    Error_Handler();
  }
  if (sx1262_configure() != SX1262_OK) {
    printf("ERROR: paso %u\r\n", sx1262_get_failed_step());
    Error_Handler();
  }

  uint32_t seq = 0;
  uint32_t last_tx_ms = HAL_GetTick() - TX_PERIOD_MS;

  while (1) {
    if ((HAL_GetTick() - last_tx_ms) < TX_PERIOD_MS) {
      continue;
    }
    last_tx_ms += TX_PERIOD_MS;

    uint8_t payload[TX_PAYLOAD_LEN];
    memcpy(payload, TX_TEXT, TX_PAYLOAD_LEN);

    uint32_t tx_ms = 0;
    if (sx1262_transmit(payload, TX_PAYLOAD_LEN, &tx_ms) == SX1262_OK) {
      diag_tx_ok++;
      diag_last_tx_ms = tx_ms;
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    } else {
      diag_tx_fail++;
    }
    seq++;
  }
}
