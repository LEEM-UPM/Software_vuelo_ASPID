/**
 * @file    main.c
 * @brief   RF board firmware entry point.
 *
 * 1. Joins the common CAN1 bus and configures the SX1262.
 * 2. Waits for the core's CMD_WAKE_UP (services/can_node).
 * 3. Once awake: keeps answering the wake up, puts on the air whatever the
 *    core sends in a CMD_RF_TRANSMIT (ICD 6.10) and sends a test LoRa packet
 *    every RADIO_BEACON_PERIOD_MS (app/radio_beacon.c).
 *
 * LED (PB4): fast blink while waiting for the wake up; once awake it toggles
 * on every packet transmitted successfully (steady = radio not working).
 * Console: LPUART1, 115200 8N1 (TX PB11).
 */
#include <stdio.h>

#include "board.h"
#include "can_node.h"
#include "gpio.h"
#include "main.h"
#include "radio_beacon.h"
#include "usart.h"

#define LED_WAITING_HALF_PERIOD_MS 100U

int __io_putchar(int ch) {
  HAL_UART_Transmit(&hlpuart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

int main(void) {
  board_init();

  printf("\r\n=== Placa RF ===\r\n");
  printf("Arranque por: %s\r\n", board_reset_cause());

  if (!can_node_init()) {
    printf("ERROR: no se ha podido arrancar CAN1\r\n");
    Error_Handler();
  }

  /* A radio failure must not stop the board from answering the core: the
   * wake up still works and radio_beacon_update() simply does nothing. */
  (void)radio_beacon_init();

  printf("Esperando el wake up del core...\r\n");
  uint32_t last_toggle_ms = HAL_GetTick();
  while (!can_node_poll_wake_up()) {
    if ((HAL_GetTick() - last_toggle_ms) >= LED_WAITING_HALF_PERIOD_MS) {
      last_toggle_ms = HAL_GetTick();
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    }
  }
  printf("Wake up recibido, respondido con 0x240\r\n");
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  uint8_t radio_payload[CAN_NODE_RADIO_PAYLOAD_MAX];
  uint8_t radio_len = 0;

  /* Uplink: se escucha siempre que no se esté transmitiendo (ICD 6.12) */
  if (radio_listen()) {
    printf("Escuchando el enlace de subida\r\n");
  }

  /* Desde aquí, una vuelta del bucle de más de 1 s reinicia la placa
   * (board.h). Transmitir bloquea ~31 ms. */
  board_watchdog_start();
  uint32_t bus_off_dicho = 0;

  while (1) {
    board_watchdog_refresh();
    can_node_poll(); /* sigue respondiendo si el core se resetea; y saca
                        CAN1 de bus-off (bsp_can1.h) */
    if (can_node_bus_off_count() != bus_off_dicho) {
      bus_off_dicho = can_node_bus_off_count();
      printf("CAN1 sacado de bus-off (%lu veces)\r\n",
             (unsigned long)bus_off_dicho);
    }

    /*
     * Se mira la ESCUCHA ANTES que la transmisión a propósito. Transmitir
     * bloquea ~31 ms y deja la radio sorda ese rato; si un paquete acaba de
     * llegar, cogerlo primero evita que se quede esperando un turno entero.
     */
    uint8_t rx_payload[CAN_NODE_RX_PAYLOAD_MAX];
    uint8_t rx_len = 0;
    sx1262_pkt_status_t rx_status = {0};
    if (radio_receive_poll(rx_payload, sizeof(rx_payload), &rx_len,
                           &rx_status)) {
      /* Con cómo se ha oído: tierra lo ve en el acuse (ICD 6.12) */
      (void)can_node_send_received(rx_payload, rx_len, rx_status.rssi_dbm,
                                   rx_status.snr_db, rx_status.freq_error_hz);
    }

    /* CMD_RF_TRANSMIT: al aire tal cual, sin tocar nada */
    if (can_node_take_radio_payload(radio_payload, sizeof(radio_payload),
                                    &radio_len) &&
        radio_transmit(radio_payload, radio_len)) {
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    }

    /* Baliza desactivada: el aire es para la telemetria que manda el core
     * (CMD_RF_TRANSMIT). Para volver a activarla, descomentar. */
#if 0
    if (radio_beacon_update()) {
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    }
#endif
  }
}
