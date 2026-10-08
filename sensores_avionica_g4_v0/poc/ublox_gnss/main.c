/******************************************************************************
 * u-blox MAX-M10S GPS simple polled reading - Proof of Concept
 *
 * Uses the GPS_* library (Drivers/Sensors/gps.h), which parses the NMEA sentences
 * the module streams by default (GGA for position/altitude/fix, RMC for
 * speed/course/date).
 ******************************************************************************/

/* === Includes ============================================================ */
#include <stdio.h>

#include "gpio.h"
#include "usart.h"

#include "gps.h"

/* === Configuration ======================================================= */
#define DEBUG_MODE true

/* === Globals ============================================================= */
static GPS_t gps;

/* === Forward declarations ============================================== */
extern void SystemClock_Config(void);
static void mcu_init(void);

/* === Main ================================================================ */
int main(void) {
  mcu_init();

  GPS_Config(&gps, &hlpuart1, GPS_RST_GPIO_Port, GPS_RST_Pin);
  GPS_Init(&gps);

  printf("u-blox MAX-M10S Ready.\r\n");

  GPS_Data_t data = {0};

  while (1) {
    if (GPS_Read(&gps, &data) & GPS_PARSED_GGA) {
      if (data.gga.fixQuality > 0) {
        printf("Fix: %u | Sats: %u | Lat: %ld e-7 | Lon: %ld e-7 | Alt: %ld mm | "
               "Speed: %ld mm/s | Course: %u cdeg\r\n",
               data.gga.fixQuality, data.gga.satellites,
               (long)data.gga.latitude_e7, (long)data.gga.longitude_e7,
               (long)data.gga.altitudeMsl_mm, (long)data.rmc.speed_mmps,
               data.rmc.course_cdeg);
      } else {
        printf("No fix yet (sats: %u)\r\n", data.gga.satellites);
      }
    }
  }
}

static void mcu_init(void) {
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_LPUART1_UART_Init();
  MX_USART2_UART_Init();
}

int __io_putchar(int ch) {
#if DEBUG_MODE
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
#endif
  return ch;
}
