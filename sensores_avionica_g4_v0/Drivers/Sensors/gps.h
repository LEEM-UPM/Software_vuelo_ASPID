/**
 ******************************************************************************
 * @file    gps.h
 * @brief   Board-level driver for the u-blox MAX-M10S GPS.
 *
 * GPS_Init() configures the module over UBX (CFG-VALSET, RAM layer only, so
 * it is redone on every boot): GPS_NAV_RATE_HZ navigation rate, the GGA and
 * RMC NMEA sentences plus the binary UBX NAV-PVT message, and the UART at
 * GPS_UART_BAUD. NAV-PVT is what carries the vertical velocity (see
 * GPS_Pvt_t); the reception path handles the two formats interleaved. The NMEA
 * sentences are parsed with the vendored minmea library
 * (third_party/minmea) into fixed-point integers, so no precision is lost
 * to floats:
 *   - GGA: UTC time, latitude/longitude, altitude, fix quality, satellites,
 *     HDOP.
 *   - RMC: UTC time and date, validity, latitude/longitude, ground speed,
 *     course.
 * Fields that the sentence left empty (e.g. no fix yet) are set to the
 * GPS_INVALID_* values.
 ******************************************************************************
 */

#ifndef __GPS_H__
#define __GPS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "main.h"
#include "minmea.h"

/* === Configuration ======================================================= */

#define GPS_NAV_RATE_HZ 10u
#define GPS_UART_BAUD 115200u

/* === Invalid markers ===================================================== */

#define GPS_INVALID_I32 INT32_MIN   /* 0x80000000 */
#define GPS_INVALID_U16 0xFFFFu
#define GPS_INVALID_TIME 0xFFFFFFFFu

/* === Data ================================================================ */

typedef struct {
  uint32_t timeOfDay_ms;  /* UTC, ms since 00:00:00 */
  int32_t latitude_e7;    /* 1e-7 deg, +N/-S */
  int32_t longitude_e7;   /* 1e-7 deg, +E/-W */
  int32_t altitudeMsl_mm; /* above mean sea level */
  uint8_t fixQuality;     /* 0 = no fix, 1 = GPS, 2 = DGPS, ... */
  uint8_t satellites;     /* satellites used */
  uint16_t hdop_x100;     /* HDOP * 100 */
  uint32_t sampleUs;      /* Timestamp_NowUs() when the burst was received */
} GPS_Gga_t;

typedef struct {
  uint32_t timeOfDay_ms;  /* UTC */
  int32_t latitude_e7;
  int32_t longitude_e7;
  int32_t speed_mmps;     /* ground speed */
  uint16_t course_cdeg;   /* true course, 0.01 deg */
  uint16_t year;          /* e.g. 2026, 0 if unknown */
  uint8_t month;          /* 1..12, 0 if unknown */
  uint8_t day;            /* 1..31, 0 if unknown */
  bool valid;             /* RMC status 'A' */
  uint32_t sampleUs;
} GPS_Rmc_t;

/* Latest value of each sentence. Zero-initialize once and keep reusing the
 * same instance: each call only overwrites the sentence(s) it parsed. */
/*
 * UBX NAV-PVT. The NMEA sentences we enable cannot carry vertical velocity:
 * GGA has no velocity at all and RMC only gives speed over ground and course,
 * which is purely horizontal. Without it the flight computer's EKF never sees
 * the rocket climb -- it calls apogee as soon as the motor burns out. NAV-PVT
 * is the receiver's native solution and includes the full NED velocity, which
 * it computes anyway from Doppler.
 */
typedef struct {
  uint32_t iTow_ms;       /* ms into the GPS week (not UTC time of day) */
  int32_t latitude_e7;    /* 1e-7 deg */
  int32_t longitude_e7;
  int32_t altitudeMsl_mm;
  int32_t velN_mmps;      /* NED velocity, the reason this message exists */
  int32_t velE_mmps;
  int32_t velD_mmps;      /* positive = descending */
  int32_t speed_mmps;     /* ground speed */
  uint32_t course_cdeg;   /* heading of motion, 0.01 deg */
  uint8_t fixType;        /* 0 none, 2 = 2D, 3 = 3D */
  uint8_t satellites;
  bool valid;             /* gnssFixOK */
  uint32_t sampleUs;
} GPS_Pvt_t;

typedef struct {
  GPS_Gga_t gga;
  GPS_Rmc_t rmc;
  GPS_Pvt_t pvt;
  uint32_t ggaCount; /* sentences parsed since boot */
  uint32_t rmcCount;
  uint32_t pvtCount;
  uint32_t ubxBadChecksum; /* UBX frames dropped */
} GPS_Data_t;

/* Return value of the parsing functions: which sentences were updated */
#define GPS_PARSED_GGA 0x01u
#define GPS_PARSED_RMC 0x02u
#define GPS_PARSED_PVT 0x04u

/* Big enough to hold a full burst of NMEA sentences between idle gaps --
 * see GPS_StartAsyncReceive(). With only GGA + RMC enabled a burst is
 * ~170 bytes of NMEA plus the 100-byte NAV-PVT frame. If a burst is larger than this, HAL_UARTEx_ReceiveToIdle_IT()
 * simply completes early (buffer full) instead of on idle, splitting it
 * across two receive events; the partial trailing line in the first one is
 * dropped (see GPS_ProcessReceived()) rather than carried over. */
#define GPS_ASYNC_RX_BUF_LEN 512u

/* One GPS instance. */
typedef struct {
  UART_HandleTypeDef *huart;

  /* MCU output wired to the module's RESET_N (active low), or NULL. */
  GPIO_TypeDef *resetPort;
  uint16_t resetPin;

  /* Interrupt-driven (idle-line) reception state -- see
   * GPS_StartAsyncReceive() / GPS_OnRxEvent() / GPS_ProcessReceived().
   * Two buffers so the reception re-armed immediately in GPS_OnRxEvent()
   * can't overwrite data the main loop hasn't parsed yet. */
  uint8_t rxBuf[2][GPS_ASYNC_RX_BUF_LEN];
  volatile uint8_t rxNextBuf;   /* which rxBuf[] the next reception fills */
  volatile uint8_t rxReadyBuf;  /* which rxBuf[] holds a completed one */
  volatile uint16_t rxReadyLen; /* its length; 0 = nothing waiting */
  volatile uint32_t rxReadyUs;  /* timestamp passed to GPS_OnRxEvent() */
  volatile uint32_t rxErrors;   /* UART errors (overrun, noise...) recovered */
} GPS_t;

/* === API ================================================================== */

/* Associates this instance with a UART and the RESET_N pin (resetPort may be
 * NULL if the pin isn't wired). Does not touch the hardware. */
void GPS_Config(GPS_t *gps, UART_HandleTypeDef *huart, GPIO_TypeDef *resetPort,
                uint16_t resetPin);

/* Resets the module through RESET_N and waits for it to boot (CubeMX leaves
 * GPS_RST as an output driven low, i.e. the module held in reset, until
 * this runs). Then configures it (rate, sentences, baud rate) and leaves the UART at
 * GPS_UART_BAUD. The module's current baud rate is unknown (factory default,
 * or GPS_UART_BAUD if it kept power across an MCU reset), so the same
 * CFG-VALSET is sent at every candidate baud rate; the module ignores the
 * copies that arrive at the wrong speed. Blocking, ~1.5 s. Returns false if
 * the UART could not be reconfigured. */
bool GPS_Init(GPS_t *gps);

/* Blocking read: parses NMEA sentences from the UART until a line times out
 * or GPS_MAX_LINES_PER_READ lines have been read. Returns GPS_PARSED_* flags
 * of what was updated (0 if nothing). For PoCs; the flight firmware uses the
 * interrupt-driven functions below. */
uint8_t GPS_Read(GPS_t *gps, GPS_Data_t *data);

/* Starts interrupt-driven reception: the UART receives into an internal
 * buffer until the line goes idle (HAL_UARTEx_ReceiveToIdle_IT()), which is
 * taken as "one burst of NMEA sentences finished". Call once after
 * GPS_Init(). Requires the application to forward
 * HAL_UARTEx_RxEventCallback() to GPS_OnRxEvent() for gps->huart, and
 * LPUART1's NVIC interrupt to be enabled (see Core/Src/usart.c's
 * HAL_UART_MspInit()). Returns true if the first reception was armed. */
bool GPS_StartAsyncReceive(GPS_t *gps);

/* Call from HAL_UARTEx_RxEventCallback() when it fires for gps->huart, with
 * the current timestamp (it becomes the sampleUs of what is parsed from this
 * burst). Re-arms reception into the other internal buffer. */
void GPS_OnRxEvent(GPS_t *gps, uint16_t size, uint32_t timestampUs);

/* Call from HAL_UART_ErrorCallback() when it fires for gps->huart. An overrun
 * (or noise/framing error) makes the HAL abort the reception, and without
 * this it would never be re-armed. Re-arms it and counts gps->rxErrors. */
void GPS_OnRxError(GPS_t *gps);

/* Non-blocking: parses whatever GPS_OnRxEvent() most recently marked ready
 * (no-op otherwise) into *data. Returns GPS_PARSED_* flags. */
uint8_t GPS_ProcessReceived(GPS_t *gps, GPS_Data_t *data);

#ifdef __cplusplus
}
#endif

#endif /* __GPS_H__ */
