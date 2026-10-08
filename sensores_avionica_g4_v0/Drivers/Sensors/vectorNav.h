/**
 ******************************************************************************
 * @file    vectorNav.h
 * @brief   Board-level driver for the VectorNav VN-100 AHRS/IMU.
 *
 * Configures the VN-100 to stream its "Common Group" binary output (time,
 * yaw/pitch/roll, angular rate, acceleration, magnetic field, temperature
 * and pressure) over UART and parses that binary packet, validating its
 * CRC16.
 *
 * Reference: VectorNav VN-100 User Manual, "Binary Output Registers" and
 * "Binary Packets" sections (Appendix: CRC16-CCITT implementation).
 ******************************************************************************
 */

#ifndef __VECTORNAV_H__
#define __VECTORNAV_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "main.h"

/* One VN-100 Common Group reading. */
typedef struct {
  double time; /* Seconds since sensor startup */

  float yaw;   /* deg */
  float pitch; /* deg */
  float roll;  /* deg */

  float angRateX; /* rad/s */
  float angRateY; /* rad/s */
  float angRateZ; /* rad/s */

  float accelX; /* m/s^2 */
  float accelY; /* m/s^2 */
  float accelZ; /* m/s^2 */

  float magX; /* Gauss */
  float magY; /* Gauss */
  float magZ; /* Gauss */

  float temperature; /* degC */
  float pressure;    /* kPa */
  float altitude;    /* m, derived from pressure (ISA barometric formula) */
} VectorNav_Data_t;

/* Raw frame size (excluding the sync byte): group byte + Common Group
 * payload + 16-bit CRC. Must match the binary output configuration
 * VectorNav_Init() sends (field mask 0x0529) -- see vectorNav.c's
 * VN_PAYLOAD_* offsets if that ever changes. Exposed here (not just
 * privately in vectorNav.c) so VectorNav_t can size its interrupt-driven
 * receive buffers. */
#define VECTORNAV_RAW_PACKET_SIZE 67u

typedef enum {
  VectorNav_RxWaitSync,
  VectorNav_RxWaitPacket,
} VectorNav_RxState_t;

/* One VN-100 instance. */
typedef struct {
  UART_HandleTypeDef *huart;

  /* Interrupt-driven reception state -- see VectorNav_StartReceive() /
   * VectorNav_OnRxCplt() / VectorNav_ReadLatest(). Two packet buffers so
   * the reception re-armed in VectorNav_OnRxCplt() can't overwrite a
   * packet the main loop hasn't decoded yet. */
  VectorNav_RxState_t rxState;
  uint8_t rxSyncByte;
  uint8_t rxPacket[2][VECTORNAV_RAW_PACKET_SIZE];
  volatile uint8_t rxNextPacketBuf;  /* which rxPacket[] a full read fills */
  volatile uint8_t rxReadyPacketBuf; /* which rxPacket[] holds a completed one */
  volatile bool rxPacketReady;
} VectorNav_t;

/* === API ================================================================== */

/* Associates this instance with a UART. Does not touch the hardware. */
void VectorNav_Config(VectorNav_t *vn, UART_HandleTypeDef *huart);

/* Sends the binary output configuration command (register 75): Common
 * Group, output on the port it was received on, at 800 / 4 = 200 Hz.
 * Returns true if the command was sent successfully -- the sensor does not
 * reply to this command so this does not guarantee it was accepted. */
bool VectorNav_Init(VectorNav_t *vn);

/* Blocking read of a single binary packet: waits for the sync byte, reads
 * the packet, validates its CRC16 and decodes it into *data. Returns true
 * on success. */
bool VectorNav_Read(VectorNav_t *vn, VectorNav_Data_t *data);

/* Starts interrupt-driven reception: hunts for the sync byte one byte at a
 * time, then reads one full binary frame, marks it ready, and restarts the
 * hunt -- repeating indefinitely. Call once after VectorNav_Init().
 * Requires the application to forward HAL_UART_RxCpltCallback() to
 * VectorNav_OnRxCplt() for vn->huart.
 *
 * NOTE: unlike GPS_StartAsyncReceive(), this does NOT use idle-line
 * detection. At the VN-100's configured 200 Hz / 115200 baud, a frame is
 * 1 (sync) + VECTORNAV_RAW_PACKET_SIZE = 68 bytes, i.e. ~5.9 ms at 115200
 * 8N1 -- longer than the 5 ms packet period -- so the binary stream likely
 * leaves no reliable idle gap between packets. Framing is instead done the
 * same way the blocking VectorNav_Read() does it: sync byte, then a
 * fixed-length read. Returns true if the first reception was armed
 * successfully. */
bool VectorNav_StartReceive(VectorNav_t *vn);

/* Call from the application's HAL_UART_RxCpltCallback() when it fires for
 * vn->huart. */
void VectorNav_OnRxCplt(VectorNav_t *vn);

/* Non-blocking counterpart to VectorNav_Read(): decodes the packet most
 * recently completed by the interrupt-driven receive path (see
 * VectorNav_StartReceive()), if any -- check vn->rxPacketReady, or just
 * call this unconditionally, it's a no-op otherwise. Returns true if a
 * packet was ready and it decoded successfully (CRC valid). */
bool VectorNav_ReadLatest(VectorNav_t *vn, VectorNav_Data_t *data);

#ifdef __cplusplus
}
#endif

#endif /* __VECTORNAV_H__ */
