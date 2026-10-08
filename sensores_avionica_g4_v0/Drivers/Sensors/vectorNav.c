/**
 ******************************************************************************
 * @file    vectorNav.c
 * @brief   Board-level driver for the VectorNav VN-100 AHRS/IMU.
 ******************************************************************************
 */

#include "vectorNav.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

/* === Configuration ========================================================
 * Binary Output 1 register (75) command sent by VectorNav_Init():
 *   $VNWRG,75,1,4,1,0529*XX
 *     - AsyncMode   = 1    -> output on the serial port the command was
 *                             received on
 *     - RateDivisor = 4    -> 800 Hz / 4 = 200 Hz
 *     - Group 1     = Common Group
 *     - Field mask  = 0x0529 -> TimeStartup | Ypr | AngularRate | Accel |
 *                                MagPres (Mag + Temp + Pressure)
 * This exact field selection is what VN_PAYLOAD_* below decodes -- if the
 * field mask above is changed, the offsets/sizes must be updated to match.
 * ========================================================================== */
#define VN_CONFIG_CMD_BODY "VNWRG,75,1,4,1,0529"

#define VN_SYNC_BYTE 0xFAu
#define VN_GROUP_COMMON 0x01u

/* Common Group payload layout for field mask 0x0529 (offsets within the
 * payload, i.e. after the sync and group bytes). */
#define VN_PAYLOAD_TIME_OFFSET 0u     /* uint64, ns since startup */
#define VN_PAYLOAD_YPR_OFFSET 8u      /* float[3]: yaw, pitch, roll (deg) */
#define VN_PAYLOAD_ANGRATE_OFFSET 20u /* float[3]: x, y, z (rad/s) */
#define VN_PAYLOAD_ACCEL_OFFSET 32u   /* float[3]: x, y, z (m/s^2) */
#define VN_PAYLOAD_MAG_OFFSET 44u     /* float[3]: x, y, z (Gauss) */
#define VN_PAYLOAD_TEMP_OFFSET 56u    /* float: degC */
#define VN_PAYLOAD_PRES_OFFSET 60u    /* float: kPa */
#define VN_PAYLOAD_SIZE 64u

/* Bytes following the sync byte: group byte + payload + 16-bit CRC. Kept
 * in sync with the header's VECTORNAV_RAW_PACKET_SIZE (which VectorNav_t's
 * interrupt-driven receive buffers are sized with) via this #if. */
#define VN_PACKET_SIZE (1u + VN_PAYLOAD_SIZE + 2u)
#if VN_PACKET_SIZE != VECTORNAV_RAW_PACKET_SIZE
#error "VN_PACKET_SIZE and VECTORNAV_RAW_PACKET_SIZE have gone out of sync"
#endif

#define VN_BYTE_TIMEOUT_MS 20u
#define VN_MAX_SYNC_ATTEMPTS (VN_PACKET_SIZE * 4u)

#define VN_SEA_LEVEL_PRESSURE_KPA 101.325f

/* === Private helpers ======================================================
 */

/* CRC16-CCITT (poly 0x1021, init 0x0000), as specified by the VectorNav
 * binary packet protocol. */
static uint16_t VectorNav_CalculateCRC(const uint8_t *data, uint32_t length) {
  uint16_t crc = 0x0000u;
  for (uint32_t i = 0; i < length; i++) {
    crc = (uint16_t)((crc >> 8) | (crc << 8));
    crc ^= data[i];
    crc ^= (uint8_t)(crc & 0xFFu) >> 4;
    crc ^= (uint16_t)(crc << 12);
    crc ^= (uint16_t)((crc & 0x00FFu) << 5);
  }
  return crc;
}

/* 8-bit XOR checksum used by VectorNav/NMEA-style ASCII commands, computed
 * over the bytes between '$' and '*' (exclusive). */
static uint8_t VectorNav_CalculateChecksum(const uint8_t *data,
                                            uint32_t length) {
  uint8_t checksum = 0;
  for (uint32_t i = 0; i < length; i++) {
    checksum ^= data[i];
  }
  return checksum;
}

/* ISA barometric formula, valid below 11 km. */
static float VectorNav_PressureToAltitude(float pressure_kPa) {
  return 44330.0f *
         (1.0f - powf(pressure_kPa / VN_SEA_LEVEL_PRESSURE_KPA, 0.1902949572f));
}

/* === API ===================================================================
 */

void VectorNav_Config(VectorNav_t *vn, UART_HandleTypeDef *huart) {
  vn->huart = huart;
}

bool VectorNav_Init(VectorNav_t *vn) {
  char cmd[40];
  uint8_t checksum = VectorNav_CalculateChecksum(
      (const uint8_t *)VN_CONFIG_CMD_BODY, strlen(VN_CONFIG_CMD_BODY));

  int len = snprintf(cmd, sizeof(cmd), "$%s*%02X\r\n", VN_CONFIG_CMD_BODY,
                      checksum);
  if (len <= 0 || (size_t)len >= sizeof(cmd)) {
    return false;
  }

  return HAL_UART_Transmit(vn->huart, (uint8_t *)cmd, (uint16_t)len,
                            HAL_MAX_DELAY) == HAL_OK;
}

/* Validates (group byte + CRC16) and decodes one raw VN_PACKET_SIZE-byte
 * frame (group byte + payload + CRC, sync byte excluded/already consumed)
 * into *data. Shared by the blocking VectorNav_Read() and the
 * interrupt-driven VectorNav_ReadLatest(). Returns true on success. */
static bool VectorNav_DecodePacket(const uint8_t *packet,
                                    VectorNav_Data_t *data) {
  if (packet[0] != VN_GROUP_COMMON) {
    /* Unexpected group selection -- packet does not match VectorNav_Init()'s
     * configuration, drop it rather than misinterpret it. */
    return false;
  }

  uint16_t receivedCrc = (uint16_t)((packet[VN_PACKET_SIZE - 2] << 8) |
                                     packet[VN_PACKET_SIZE - 1]);
  uint16_t computedCrc = VectorNav_CalculateCRC(packet, VN_PACKET_SIZE - 2u);
  if (receivedCrc != computedCrc) {
    return false;
  }

  /* Decode the payload (little-endian, matches Cortex-M4 -- safe to
   * memcpy directly into typed locals). */
  const uint8_t *payload = &packet[1];

  uint64_t timeStartupNs;
  memcpy(&timeStartupNs, &payload[VN_PAYLOAD_TIME_OFFSET],
         sizeof(timeStartupNs));
  data->time = (double)timeStartupNs * 1e-9;

  float ypr[3];
  memcpy(ypr, &payload[VN_PAYLOAD_YPR_OFFSET], sizeof(ypr));
  data->yaw = ypr[0];
  data->pitch = ypr[1];
  data->roll = ypr[2];

  float angRate[3];
  memcpy(angRate, &payload[VN_PAYLOAD_ANGRATE_OFFSET], sizeof(angRate));
  data->angRateX = angRate[0];
  data->angRateY = angRate[1];
  data->angRateZ = angRate[2];

  float accel[3];
  memcpy(accel, &payload[VN_PAYLOAD_ACCEL_OFFSET], sizeof(accel));
  data->accelX = accel[0];
  data->accelY = accel[1];
  data->accelZ = accel[2];

  float mag[3];
  memcpy(mag, &payload[VN_PAYLOAD_MAG_OFFSET], sizeof(mag));
  data->magX = mag[0];
  data->magY = mag[1];
  data->magZ = mag[2];

  memcpy(&data->temperature, &payload[VN_PAYLOAD_TEMP_OFFSET], sizeof(float));
  memcpy(&data->pressure, &payload[VN_PAYLOAD_PRES_OFFSET], sizeof(float));

  data->altitude = VectorNav_PressureToAltitude(data->pressure);

  return true;
}

bool VectorNav_Read(VectorNav_t *vn, VectorNav_Data_t *data) {
  /* 1. Find the sync byte, bounded so a silent/disconnected sensor cannot
   *    hang the caller forever. */
  uint8_t rxByte = 0;
  uint32_t attempts = 0;
  do {
    if (HAL_UART_Receive(vn->huart, &rxByte, 1, VN_BYTE_TIMEOUT_MS) !=
        HAL_OK) {
      return false;
    }
    if (++attempts > VN_MAX_SYNC_ATTEMPTS) {
      return false;
    }
  } while (rxByte != VN_SYNC_BYTE);

  /* 2. Read the rest of the packet: group byte + payload + CRC. */
  uint8_t packet[VN_PACKET_SIZE];
  if (HAL_UART_Receive(vn->huart, packet, VN_PACKET_SIZE, VN_BYTE_TIMEOUT_MS) !=
      HAL_OK) {
    return false;
  }

  return VectorNav_DecodePacket(packet, data);
}

bool VectorNav_StartReceive(VectorNav_t *vn) {
  vn->rxState = VectorNav_RxWaitSync;
  vn->rxPacketReady = false;
  vn->rxNextPacketBuf = 0;
  return HAL_UART_Receive_IT(vn->huart, &vn->rxSyncByte, 1) == HAL_OK;
}

void VectorNav_OnRxCplt(VectorNav_t *vn) {
  if (vn->rxState == VectorNav_RxWaitSync) {
    if (vn->rxSyncByte == VN_SYNC_BYTE) {
      vn->rxState = VectorNav_RxWaitPacket;
      HAL_UART_Receive_IT(vn->huart, vn->rxPacket[vn->rxNextPacketBuf],
                           VN_PACKET_SIZE);
    } else {
      /* Not the sync byte -- keep hunting one byte at a time. */
      HAL_UART_Receive_IT(vn->huart, &vn->rxSyncByte, 1);
    }
    return;
  }

  /* VectorNav_RxWaitPacket: a full frame just arrived. */
  vn->rxReadyPacketBuf = vn->rxNextPacketBuf;
  vn->rxPacketReady = true;
  vn->rxNextPacketBuf = (uint8_t)(1u - vn->rxNextPacketBuf);
  vn->rxState = VectorNav_RxWaitSync;
  HAL_UART_Receive_IT(vn->huart, &vn->rxSyncByte, 1);
}

bool VectorNav_ReadLatest(VectorNav_t *vn, VectorNav_Data_t *data) {
  if (!vn->rxPacketReady) {
    return false;
  }
  vn->rxPacketReady = false;
  return VectorNav_DecodePacket(vn->rxPacket[vn->rxReadyPacketBuf], data);
}
