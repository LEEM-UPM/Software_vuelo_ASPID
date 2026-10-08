/**
 ******************************************************************************
 * @file    sensorsUart.h
 * @brief   UART sensors: u-blox GPS and VectorNav VN-100.
 *
 *  - GPS (LPUART1, PA2/PA3, NMEA text): interrupt-driven via idle-line
 *    detection (HAL_UARTEx_ReceiveToIdle_IT) -- a burst of NMEA sentences
 *    ends with a gap on the line, which the HAL turns into an
 *    HAL_UARTEx_RxEventCallback(). See gps.h's GPS_StartAsyncReceive().
 *  - VectorNav VN-100 (USART2, PB3/PB4, fixed-length binary packets):
 *    interrupt-driven via plain UART RX interrupts framed by byte count (sync
 *    byte, then a fixed-size read), NOT idle-line detection -- at 200 Hz /
 *    115200 baud a packet's transmit time is close to (and by one reasonable
 *    estimate, slightly over) its period, so there's no reliable idle gap to
 *    detect between packets. See vectorNav.h's VectorNav_StartReceive().
 *
 * Both libraries carry their own ready-flags (gps.rxReadyLen /
 * vn.rxPacketReady), since they need to track more state than a single pin
 * edge to know a full message is ready. The GPS is sent over CAN2 (0x106
 * GGA, 0x107 RMC, see canTelemetry.h); the VN-100 is not yet. The latest
 * data is kept in statics in sensorsUart.c for the debugger.
 ******************************************************************************
 */

#ifndef __SENSORSUART_H__
#define __SENSORSUART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

/* Configures and initialises both sensors. After the MX_*UART*_Init(). */
void SensorsUart_Init(void);

/* Starts interrupt-driven reception on both UARTs. */
void SensorsUart_Start(void);

/* Did each UART sensor start up? (for INIT_STATUS, ICD 6.11) */
bool SensorsUart_GpsActive(void);
bool SensorsUart_VectorNavActive(void);

/* Parses whatever has been fully received. Call on every pass of the main
 * loop. */
void SensorsUart_Service(void);

#ifdef __cplusplus
}
#endif

#endif /* __SENSORSUART_H__ */
