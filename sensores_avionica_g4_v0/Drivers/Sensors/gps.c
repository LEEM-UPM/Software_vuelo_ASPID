/**
 ******************************************************************************
 * @file    gps.c
 * @brief   Board-level driver for the u-blox MAX-M10S GPS.
 ******************************************************************************
 */

#include "gps.h"

#include <string.h>

#define GPS_LINE_MAX_LEN 96u
#define GPS_LINE_TIMEOUT_MS 300u /* > 1 NMEA cycle at GPS_NAV_RATE_HZ */
#define GPS_BYTE_TIMEOUT_MS 100u
#define GPS_MAX_LINES_PER_READ 20u /* bound how long GPS_Read() can loop */

#define GPS_RESET_PULSE_MS 10u
#define GPS_BOOT_TIME_MS 1000u /* RESET_N released -> module accepts UBX */

/* Baud rates the module may be at when GPS_Init() runs: M10 factory default
 * (38400), older u-blox default (9600), and ours if the module kept its RAM
 * configuration across an MCU reset. */
static const uint32_t gpsCandidateBauds[] = {9600u, 38400u, GPS_UART_BAUD};

/* LPUART1 runs from PCLK1 (100 MHz): without a prescaler its BRR can't go
 * below ~24 kBd. /8 covers 9600 to 115200. */
#define GPS_UART_PRESCALER UART_PRESCALER_DIV8

/* === UBX configuration =================================================== */

/* u-blox M10 configuration keys (Interface Description, CFG-*) */
#define UBX_KEY_RATE_MEAS 0x30210001u      /* U2, ms */
#define UBX_KEY_RATE_NAV 0x30210002u       /* U2, measurements per solution */
#define UBX_KEY_UART1_BAUDRATE 0x40520001u /* U4 */
#define UBX_KEY_NMEA_GGA_UART1 0x209100bbu /* U1, output rate */
#define UBX_KEY_NMEA_RMC_UART1 0x209100acu
#define UBX_KEY_NMEA_GLL_UART1 0x209100cau
#define UBX_KEY_NMEA_GSA_UART1 0x209100c0u
#define UBX_KEY_NMEA_GSV_UART1 0x209100c5u
#define UBX_KEY_NMEA_VTG_UART1 0x209100b1u
#define UBX_KEY_UBX_NAV_PVT_UART1 0x20910007u /* U1, output rate */
/* E1, modelo dinamico del receptor. 8 = "Airborne <4g", el mas permisivo:
 * lo pide la entrega del GNC v1.2 (LEEME_INTEGRACION). Aun asi el fix se cae
 * durante el empuje (el cohete pasa de 4 g), y esta previsto; con el de
 * fabrica (0, Portable) se cae mucho antes y tarda mas en reengancharse. */
#define UBX_KEY_NAVSPG_DYNMODEL 0x20110021u
#define UBX_DYNMODEL_AIRBORNE_4G 8u

#define UBX_CLASS_CFG 0x06u
#define UBX_ID_CFG_VALSET 0x8Au
#define UBX_VALSET_LAYER_RAM 0x01u

#define UBX_MAX_FRAME_LEN 96u

typedef struct {
  uint8_t buf[UBX_MAX_FRAME_LEN];
  uint16_t len;
} UbxFrame_t;

static void Ubx_Put(UbxFrame_t *f, uint32_t value, uint8_t bytes) {
  for (uint8_t i = 0; i < bytes; i++) {
    f->buf[f->len++] = (uint8_t)(value >> (8u * i));
  }
}

static void Ubx_PutKey(UbxFrame_t *f, uint32_t key, uint32_t value,
                       uint8_t valueBytes) {
  Ubx_Put(f, key, 4u);
  Ubx_Put(f, value, valueBytes);
}

/* Builds the CFG-VALSET frame: B5 62 class id len payload ckA ckB */
static void Ubx_BuildConfig(UbxFrame_t *f) {
  f->len = 0;
  Ubx_Put(f, 0xB5u, 1u);
  Ubx_Put(f, 0x62u, 1u);
  Ubx_Put(f, UBX_CLASS_CFG, 1u);
  Ubx_Put(f, UBX_ID_CFG_VALSET, 1u);
  Ubx_Put(f, 0u, 2u); /* payload length, filled in below */

  const uint16_t payloadStart = f->len;
  Ubx_Put(f, 0u, 1u);                   /* version */
  Ubx_Put(f, UBX_VALSET_LAYER_RAM, 1u); /* layers */
  Ubx_Put(f, 0u, 2u);                   /* reserved */

  Ubx_PutKey(f, UBX_KEY_RATE_MEAS, 1000u / GPS_NAV_RATE_HZ, 2u);
  Ubx_PutKey(f, UBX_KEY_RATE_NAV, 1u, 2u);
  Ubx_PutKey(f, UBX_KEY_NMEA_GGA_UART1, 1u, 1u);
  Ubx_PutKey(f, UBX_KEY_NMEA_RMC_UART1, 1u, 1u);
  Ubx_PutKey(f, UBX_KEY_NMEA_GLL_UART1, 0u, 1u);
  Ubx_PutKey(f, UBX_KEY_NMEA_GSA_UART1, 0u, 1u);
  Ubx_PutKey(f, UBX_KEY_NMEA_GSV_UART1, 0u, 1u);
  Ubx_PutKey(f, UBX_KEY_NMEA_VTG_UART1, 0u, 1u);
  /* The one message that carries vertical velocity (see GPS_Pvt_t) */
  Ubx_PutKey(f, UBX_KEY_UBX_NAV_PVT_UART1, 1u, 1u);
  Ubx_PutKey(f, UBX_KEY_NAVSPG_DYNMODEL, UBX_DYNMODEL_AIRBORNE_4G, 1u);
  /* Last: the module switches baud rate once the message is applied */
  Ubx_PutKey(f, UBX_KEY_UART1_BAUDRATE, GPS_UART_BAUD, 4u);

  const uint16_t payloadLen = (uint16_t)(f->len - payloadStart);
  f->buf[4] = (uint8_t)payloadLen;
  f->buf[5] = (uint8_t)(payloadLen >> 8);

  /* 8-bit Fletcher checksum over class, id, length and payload */
  uint8_t ckA = 0;
  uint8_t ckB = 0;
  for (uint16_t i = 2u; i < f->len; i++) {
    ckA = (uint8_t)(ckA + f->buf[i]);
    ckB = (uint8_t)(ckB + ckA);
  }
  Ubx_Put(f, ckA, 1u);
  Ubx_Put(f, ckB, 1u);
}

static bool GPS_SetBaud(GPS_t *gps, uint32_t baud) {
  gps->huart->Init.BaudRate = baud;
  gps->huart->Init.ClockPrescaler = GPS_UART_PRESCALER;
  return HAL_UART_Init(gps->huart) == HAL_OK; /* disables, reprograms BRR */
}

/* === NMEA -> fixed point ================================================= */

/* Rounded value * mul / div, or GPS_INVALID_I32 if the field was empty */
static int32_t GPS_Scale(const struct minmea_float *f, int64_t mul,
                         int64_t div) {
  if (f->scale == 0) {
    return GPS_INVALID_I32;
  }
  const int64_t num = (int64_t)f->value * mul;
  const int64_t den = (int64_t)f->scale * div;
  const int64_t half = (num >= 0) ? (den / 2) : -(den / 2);
  return (int32_t)((num + half) / den);
}

/* NMEA ddmm.mmmm (negative for S/W) -> 1e-7 deg, exact integer arithmetic */
static int32_t GPS_CoordE7(const struct minmea_float *f) {
  if (f->scale == 0) {
    return GPS_INVALID_I32;
  }
  const int64_t scale = f->scale;
  const int64_t degrees = f->value / (scale * 100);
  const int64_t minutesScaled = f->value % (scale * 100); /* keeps the sign */
  const int64_t den = 60 * scale;
  const int64_t num = minutesScaled * 10000000LL;
  const int64_t half = (num >= 0) ? (den / 2) : -(den / 2);
  return (int32_t)((degrees * 10000000LL) + ((num + half) / den));
}

static uint16_t GPS_ToU16(int32_t v) {
  if (v == GPS_INVALID_I32 || v < 0) {
    return GPS_INVALID_U16;
  }
  return (v >= (int32_t)GPS_INVALID_U16) ? (uint16_t)(GPS_INVALID_U16 - 1u)
                                          : (uint16_t)v;
}

static uint32_t GPS_TimeOfDayMs(const struct minmea_time *t) {
  if (t->hours < 0) {
    return GPS_INVALID_TIME;
  }
  return ((uint32_t)t->hours * 3600000u) + ((uint32_t)t->minutes * 60000u) +
         ((uint32_t)t->seconds * 1000u) + ((uint32_t)t->microseconds / 1000u);
}

/* === UBX NAV-PVT ========================================================= */

#define UBX_SYNC1 0xB5u
#define UBX_SYNC2 0x62u
#define UBX_CLASS_NAV 0x01u
#define UBX_ID_NAV_PVT 0x07u
#define UBX_NAV_PVT_LEN 92u
#define UBX_HEADER_LEN 6u  /* sync1 sync2 class id len_lo len_hi */
#define UBX_OVERHEAD (UBX_HEADER_LEN + 2u) /* + the two checksum bytes */

static uint32_t Ubx_U32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

static int32_t Ubx_I32(const uint8_t *p) { return (int32_t)Ubx_U32(p); }

/* Field offsets within the NAV-PVT payload (u-blox interface description) */
static void GPS_ParsePvt(const uint8_t *payload, GPS_Data_t *data,
                         uint32_t sampleUs) {
  GPS_Pvt_t *p = &data->pvt;
  p->iTow_ms = Ubx_U32(&payload[0]);
  p->fixType = payload[20];
  p->valid = (payload[21] & 0x01u) != 0u; /* flags bit 0: gnssFixOK */
  p->satellites = payload[23];
  p->longitude_e7 = Ubx_I32(&payload[24]);
  p->latitude_e7 = Ubx_I32(&payload[28]);
  p->altitudeMsl_mm = Ubx_I32(&payload[36]);
  p->velN_mmps = Ubx_I32(&payload[48]);
  p->velE_mmps = Ubx_I32(&payload[52]);
  p->velD_mmps = Ubx_I32(&payload[56]);
  p->speed_mmps = Ubx_I32(&payload[60]);

  /* headMot comes in 1e-5 deg and can be negative; the ICD wants 0.01 deg
   * in 0..35999. */
  int32_t heading = Ubx_I32(&payload[64]) / 1000;
  heading %= 36000;
  if (heading < 0) {
    heading += 36000;
  }
  p->course_cdeg = (uint32_t)heading;

  p->sampleUs = sampleUs;
  data->pvtCount++;
}

/*
 * Consumes one UBX frame starting at buf[0] (already known to begin with the
 * two sync bytes). Returns how many bytes it used, or 0 if the frame is not
 * complete within len -- in that case the caller stops, because the rest
 * belongs to the next burst.
 */
static uint16_t GPS_ParseUbxFrame(const uint8_t *buf, uint16_t len,
                                  GPS_Data_t *data, uint32_t sampleUs,
                                  uint8_t *parsed) {
  if (len < UBX_OVERHEAD) {
    return 0u;
  }
  const uint16_t payloadLen = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);
  const uint32_t total = (uint32_t)payloadLen + UBX_OVERHEAD;
  if (total > len) {
    return 0u; /* truncated by the end of the burst */
  }

  /* 8-bit Fletcher over class, id, length and payload */
  uint8_t ckA = 0u;
  uint8_t ckB = 0u;
  for (uint16_t i = 2u; i < (UBX_HEADER_LEN + payloadLen); i++) {
    ckA = (uint8_t)(ckA + buf[i]);
    ckB = (uint8_t)(ckB + ckA);
  }
  if ((ckA != buf[UBX_HEADER_LEN + payloadLen]) ||
      (ckB != buf[UBX_HEADER_LEN + payloadLen + 1u])) {
    data->ubxBadChecksum++;
    return (uint16_t)total; /* se salta, pero sigue sincronizado */
  }

  if ((buf[2] == UBX_CLASS_NAV) && (buf[3] == UBX_ID_NAV_PVT) &&
      (payloadLen >= UBX_NAV_PVT_LEN)) {
    GPS_ParsePvt(&buf[UBX_HEADER_LEN], data, sampleUs);
    *parsed |= GPS_PARSED_PVT;
  }
  return (uint16_t)total;
}

/* Parses one NUL-terminated NMEA line. Returns the GPS_PARSED_* flag of the
 * sentence it updated, 0 if not recognized. */
static uint8_t GPS_ParseLine(const char *line, GPS_Data_t *data,
                             uint32_t sampleUs) {
  switch (minmea_sentence_id(line, false)) {
    case MINMEA_SENTENCE_GGA: {
      struct minmea_sentence_gga gga;
      if (!minmea_parse_gga(&gga, line)) {
        return 0u;
      }
      GPS_Gga_t *d = &data->gga;
      d->timeOfDay_ms = GPS_TimeOfDayMs(&gga.time);
      d->latitude_e7 = GPS_CoordE7(&gga.latitude);
      d->longitude_e7 = GPS_CoordE7(&gga.longitude);
      d->altitudeMsl_mm = GPS_Scale(&gga.altitude, 1000, 1);
      d->fixQuality = (uint8_t)gga.fix_quality;
      d->satellites = (uint8_t)gga.satellites_tracked;
      d->hdop_x100 = GPS_ToU16(GPS_Scale(&gga.hdop, 100, 1));
      d->sampleUs = sampleUs;
      data->ggaCount++;
      return GPS_PARSED_GGA;
    }

    case MINMEA_SENTENCE_RMC: {
      struct minmea_sentence_rmc rmc;
      if (!minmea_parse_rmc(&rmc, line)) {
        return 0u;
      }
      GPS_Rmc_t *d = &data->rmc;
      d->timeOfDay_ms = GPS_TimeOfDayMs(&rmc.time);
      d->latitude_e7 = GPS_CoordE7(&rmc.latitude);
      d->longitude_e7 = GPS_CoordE7(&rmc.longitude);
      /* knots -> mm/s: 1 kn = 1852 m / 3600 s */
      d->speed_mmps = GPS_Scale(&rmc.speed, 1852000, 3600);
      d->course_cdeg = GPS_ToU16(GPS_Scale(&rmc.course, 100, 1));
      if (rmc.date.year >= 0) {
        d->year = (uint16_t)(2000 + rmc.date.year); /* 2-digit year */
        d->month = (uint8_t)rmc.date.month;
        d->day = (uint8_t)rmc.date.day;
      } else {
        d->year = 0u;
        d->month = 0u;
        d->day = 0u;
      }
      d->valid = rmc.valid;
      d->sampleUs = sampleUs;
      data->rmcCount++;
      return GPS_PARSED_RMC;
    }

    default:
      return 0u; /* other sentence types are not decoded */
  }
}

/* Reads one '\n'-terminated line (without the line ending) into `line`,
 * dropping '\r' and silently truncating lines longer than maxLen - 1.
 * Returns false if no full line arrived within GPS_LINE_TIMEOUT_MS. */
static bool GPS_ReadLine(GPS_t *gps, char *line, size_t maxLen) {
  size_t idx = 0;
  uint32_t start = HAL_GetTick();

  while ((HAL_GetTick() - start) < GPS_LINE_TIMEOUT_MS) {
    uint8_t rxByte = 0;
    if (HAL_UART_Receive(gps->huart, &rxByte, 1, GPS_BYTE_TIMEOUT_MS) !=
        HAL_OK) {
      continue;
    }
    if (rxByte == '\r') {
      continue;
    }
    if (rxByte == '\n') {
      if (idx == 0) {
        continue; /* skip stray blank lines */
      }
      line[idx] = '\0';
      return true;
    }
    if (idx < (maxLen - 1u)) {
      line[idx++] = (char)rxByte;
    }
  }
  return false;
}

/* === API =================================================================== */

void GPS_Config(GPS_t *gps, UART_HandleTypeDef *huart, GPIO_TypeDef *resetPort,
                uint16_t resetPin) {
  gps->huart = huart;
  gps->resetPort = resetPort;
  gps->resetPin = resetPin;
}

bool GPS_Init(GPS_t *gps) {
  /* Clean reset: the module starts from its factory configuration */
  if (gps->resetPort != NULL) {
    HAL_GPIO_WritePin(gps->resetPort, gps->resetPin, GPIO_PIN_RESET);
    HAL_Delay(GPS_RESET_PULSE_MS);
    HAL_GPIO_WritePin(gps->resetPort, gps->resetPin, GPIO_PIN_SET);
    HAL_Delay(GPS_BOOT_TIME_MS);
  }

  UbxFrame_t frame;
  Ubx_BuildConfig(&frame);

  for (uint32_t i = 0; i < (sizeof(gpsCandidateBauds) / sizeof(gpsCandidateBauds[0]));
       i++) {
    if (!GPS_SetBaud(gps, gpsCandidateBauds[i])) {
      return false;
    }
    (void)HAL_UART_Transmit(gps->huart, frame.buf, frame.len, 200u);
    /* Let the last byte leave and the module apply the new configuration */
    HAL_Delay(100u);
  }

  return GPS_SetBaud(gps, GPS_UART_BAUD);
}

uint8_t GPS_Read(GPS_t *gps, GPS_Data_t *data) {
  uint8_t parsed = 0u;

  for (uint32_t lineCount = 0; lineCount < GPS_MAX_LINES_PER_READ;
       lineCount++) {
    char line[GPS_LINE_MAX_LEN];
    if (!GPS_ReadLine(gps, line, sizeof(line))) {
      break;
    }
    parsed |= GPS_ParseLine(line, data, 0u);
  }
  return parsed;
}

bool GPS_StartAsyncReceive(GPS_t *gps) {
  gps->rxNextBuf = 0;
  gps->rxReadyLen = 0;
  return HAL_UARTEx_ReceiveToIdle_IT(gps->huart, gps->rxBuf[0],
                                      GPS_ASYNC_RX_BUF_LEN) == HAL_OK;
}

void GPS_OnRxEvent(GPS_t *gps, uint16_t size, uint32_t timestampUs) {
  uint8_t completedBuf = gps->rxNextBuf;
  gps->rxNextBuf = (uint8_t)(1u - completedBuf);

  /* Re-arm immediately so no bytes are lost while the main loop is still
   * catching up on the chunk that just completed. */
  HAL_UARTEx_ReceiveToIdle_IT(gps->huart, gps->rxBuf[gps->rxNextBuf],
                               GPS_ASYNC_RX_BUF_LEN);

  gps->rxReadyBuf = completedBuf;
  gps->rxReadyUs = timestampUs;
  gps->rxReadyLen = size;
}

uint8_t GPS_ProcessReceived(GPS_t *gps, GPS_Data_t *data) {
  uint16_t len = gps->rxReadyLen;
  if (len == 0) {
    return 0u;
  }
  gps->rxReadyLen = 0; /* consumed */

  const uint8_t *buf = gps->rxBuf[gps->rxReadyBuf];
  const uint32_t sampleUs = gps->rxReadyUs;
  uint8_t parsed = 0u;
  /*
   * A burst now mixes two formats: NMEA text lines and binary UBX frames.
   * They cannot be told apart by scanning for '\n', because UBX payload
   * bytes take any value, 0x0A included -- so each item is identified by how
   * it starts and skipped by its own length.
   */
  uint16_t i = 0;
  while (i < len) {
    if ((buf[i] == UBX_SYNC1) && ((i + 1u) < len) &&
        (buf[i + 1u] == UBX_SYNC2)) {
      const uint16_t used = GPS_ParseUbxFrame(&buf[i], (uint16_t)(len - i),
                                              data, sampleUs, &parsed);
      if (used == 0u) {
        break; /* frame cut short by the end of the burst: drop the rest */
      }
      i = (uint16_t)(i + used);
      continue;
    }

    if (buf[i] == '$') {
      uint16_t end = i;
      while ((end < len) && (buf[end] != '\n')) {
        end++;
      }
      if (end >= len) {
        break; /* partial line, same as before: dropped, not carried over */
      }

      uint16_t lineLen = (uint16_t)(end - i);
      if ((lineLen > 0u) && (buf[i + lineLen - 1u] == '\r')) {
        lineLen--;
      }
      if ((lineLen > 0u) && (lineLen < GPS_LINE_MAX_LEN)) {
        char line[GPS_LINE_MAX_LEN];
        memcpy(line, &buf[i], lineLen);
        line[lineLen] = '\0';
        parsed |= GPS_ParseLine(line, data, sampleUs);
      }
      /* else: blank or too-long line, silently skipped. */

      i = (uint16_t)(end + 1u);
      continue;
    }

    i++; /* junk between items (or the tail of a dropped frame) */
  }
  return parsed;
}
