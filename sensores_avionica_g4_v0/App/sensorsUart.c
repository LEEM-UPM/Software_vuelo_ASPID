/**
 ******************************************************************************
 * @file    sensorsUart.c
 * @brief   UART sensors: u-blox GPS and VectorNav VN-100.
 ******************************************************************************
 */

#include "sensorsUart.h"

#include "usart.h"

#include "can_protocol/can_pack.h"
#include "canTelemetry.h"
#include "gps.h"
#include "timestamp.h"
#include "vectorNav.h"

/* Latest data from each sensor -- inspect these from the debugger. */
static VectorNav_Data_t vnSample;
static GPS_Data_t gpsData = {0}; /* see GPS_Data_t: zero-init, reuse */
static bool gpsConfigured;       /* GPS_Init() could set up the UART */
static bool vnConfigured;        /* VectorNav_Init() answered */
static bool vnReceiving;         /* its reception is armed */

static VectorNav_t vn;
static GPS_t gps;

/* === GPS -> CAN2 (0x106 GGA, 0x107 RMC, 0x108 NAV-PVT; canTelemetry.h) === */

static void SendGga(const GPS_Gga_t *g) {
  uint8_t b[CAN_TELEMETRY_GPS_GGA_BYTES];
  can_put_u32_le(&b[0], g->timeOfDay_ms);
  can_put_i32_le(&b[4], g->latitude_e7);
  can_put_i32_le(&b[8], g->longitude_e7);
  can_put_i32_le(&b[12], g->altitudeMsl_mm);
  can_put_u8(&b[16], g->fixQuality);
  can_put_u8(&b[17], g->satellites);
  can_put_u16_le(&b[18], g->hdop_x100);
  CanTelemetry_SendBytes(CanTelemetry_GpsGga, g->sampleUs, b, sizeof(b));
}

static void SendRmc(const GPS_Rmc_t *r) {
  uint8_t b[CAN_TELEMETRY_GPS_RMC_BYTES];
  can_put_u32_le(&b[0], r->timeOfDay_ms);
  can_put_i32_le(&b[4], r->latitude_e7);
  can_put_i32_le(&b[8], r->longitude_e7);
  can_put_i32_le(&b[12], r->speed_mmps);
  can_put_u16_le(&b[16], r->course_cdeg);
  can_put_u16_le(&b[18], r->year);
  can_put_u8(&b[20], r->month);
  can_put_u8(&b[21], r->day);
  can_put_u8(&b[22], r->valid ? 1u : 0u);
  CanTelemetry_SendBytes(CanTelemetry_GpsRmc, r->sampleUs, b, sizeof(b));
}

/*
 * The frame that makes vertical navigation possible: see canTelemetry.h and
 * GPS_Pvt_t. Position also comes in NAV-PVT and is better than GGA's, but it
 * is left out on purpose -- 0x106 already carries it and duplicating it
 * would put two sources of truth on the bus.
 */
static void SendPvt(const GPS_Pvt_t *p) {
  uint8_t b[CAN_TELEMETRY_GPS_PVT_BYTES];
  can_put_i32_le(&b[0], p->velN_mmps);
  can_put_i32_le(&b[4], p->velE_mmps);
  can_put_i32_le(&b[8], p->velD_mmps);
  can_put_u8(&b[12], p->fixType);
  can_put_u8(&b[13], p->satellites);
  can_put_u8(&b[14], p->valid ? 1u : 0u);
  CanTelemetry_SendBytes(CanTelemetry_GpsPvt, p->sampleUs, b, sizeof(b));
}

/* === API =================================================================== */

void SensorsUart_Init(void) {
  VectorNav_Config(&vn, &huart2);
  GPS_Config(&gps, &hlpuart1, GPS_RST_GPIO_Port, GPS_RST_Pin);

  vnConfigured = VectorNav_Init(&vn);

  /* Active antenna (J1): ACT_ANTENNA high switches VCC_RF onto the antenna
   * feed through Q3 (R24 + L1 bias tee). MX_GPIO_Init() leaves it low. */
  HAL_GPIO_WritePin(ACT_ANTENNA_GPIO_Port, ACT_ANTENNA_Pin, GPIO_PIN_SET);
  gpsConfigured = GPS_Init(&gps); /* reset, 10 Hz, GGA+RMC+NAV-PVT, 115200 */
}

void SensorsUart_Start(void) {
  vnReceiving = VectorNav_StartReceive(&vn);
  if (gpsConfigured) {
    gpsConfigured = GPS_StartAsyncReceive(&gps);
  }
}

bool SensorsUart_GpsActive(void) { return gpsConfigured; }

bool SensorsUart_VectorNavActive(void) { return vnConfigured && vnReceiving; }

void SensorsUart_Service(void) {
  if (vn.rxPacketReady) {
    VectorNav_ReadLatest(&vn, &vnSample);
  }

  if (gps.rxReadyLen != 0) {
    const uint8_t parsed = GPS_ProcessReceived(&gps, &gpsData);
    if (parsed & GPS_PARSED_GGA) {
      SendGga(&gpsData.gga);
    }
    if (parsed & GPS_PARSED_RMC) {
      SendRmc(&gpsData.rmc);
    }
    if (parsed & GPS_PARSED_PVT) {
      SendPvt(&gpsData.pvt);
    }
  }
}

/* === HAL interrupt callbacks ============================================= */

/* GPS's idle-line-detected reception (LPUART1). */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
  if (huart == &hlpuart1) {
    GPS_OnRxEvent(&gps, Size, Timestamp_NowUs());
  }
}

/* The HAL aborts a reception on overrun/noise/framing errors; re-arm it. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart == &hlpuart1) {
    GPS_OnRxError(&gps);
  }
}

/* VectorNav's byte-count-framed reception (USART2): fires once per
 * hunted-and-confirmed sync byte AND once per completed packet -- see
 * VectorNav_OnRxCplt(), which only marks vn.rxPacketReady on the latter. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == &huart2) {
    VectorNav_OnRxCplt(&vn);
  }
}
