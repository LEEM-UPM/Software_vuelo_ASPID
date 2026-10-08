/******************************************************************************
 * Avionics board - flight firmware
 *
 * Interrupt-driven: every sensor with a data-ready signal sets a flag from
 * an interrupt context, and the main loop only ever does I/O when that
 * flag says there's something to read -- no unconditional polling.
 *
 * Where things are:
 *  - sensorsI2c.c: barometers, IMUs and magnetometer -> CAN2 telemetry.
 *  - sensorIrq.c: their data-ready interrupts (HAL_GPIO_EXTI_Callback).
 *  - sensorsUart.c: GPS and VectorNav (UART callbacks).
 *  - Drivers/Can_driver: CAN1 common bus (canNode), CAN2 telemetry and the
 *    H7's servo command (canTelemetry, canCommand).
 *
 * NOTE on output: the board only has two UARTs, and both are spoken for --
 * LPUART1 is the GPS's NMEA stream and USART2 is wired straight to the VN-100
 * (VNAV_RX/VNAV_TX pins, see vectorNav's PoC for why that rules out sharing
 * it for printf). So there is no UART left free for a debug console here.
 * Instead, every sensor's latest sample is kept in a plain static meant to
 * be inspected live from the debugger (e.g. a Live Expressions / Watch view)
 * rather than printed -- this is a Debug build (see CMakeLists.txt), so the
 * compiler won't optimize these away even though they're read only by the
 * debugger. `loopCount` is there as a cheap "is it still alive" signal.
 *
 * TEST_LED is the I2C health indicator: it blinks at 1 Hz for as long as
 * every I2C sensor is answering, and goes dark otherwise (see
 * SensorsI2c_Healthy()). The UART sensors (GPS, VN-100) are deliberately
 * not part of this.
 ******************************************************************************/

/* === Includes ============================================================ */
#include <stdbool.h>

#include "fdcan.h"
#include "gpio.h"
#include "i2c.h"
#include "usart.h"

#include "calibration.h"
#include "canBusOff.h"
#include "canCommand.h"
#include "canNode.h"
#include "canTelemetry.h"
#include "sensorsI2c.h"
#include "sensorsUart.h"
#include "servo.h"
#include "timestamp.h"
#include "watchdog.h"

/* === Configuration ========================================================
 * TEST_LED heartbeat: toggling every 500 ms gives one blink per second. */
#define HEARTBEAT_HALF_PERIOD_MS 500u

/* === Globals ============================================================= */
static uint32_t loopCount;

/* Last time TEST_LED was toggled. */
static uint32_t lastBlinkTick;

/* Servo on PB10, commanded by the H7 over CAN (frame 0x200, see
 * canCommand.h) -- watch these in the debugger:
 *  - servoCommand: last value (0..255) applied to the servo.
 *  - servoPulseUs: resulting high time, 500..2500 us. 0 until the first
 *    frame arrives: until then no pulses are output at all.
 *  - canCommandLinkAlive: a frame arrived in the last 100 ms. If the link
 *    drops, the servo simply holds the last commanded position. */
static uint8_t servoCommand;
static uint32_t servoPulseUs;
static bool canCommandLinkAlive;

/* Último INIT_STATUS enviado al core (bits en canNode.h) */
static uint32_t initStatusOk;

/* === Forward declarations ============================================== */
extern void SystemClock_Config(void);
static void mcu_init(void);
static void heartbeat(void);
static void SendInitStatus(void);

/* === Main ================================================================ */
int main(void) {
  mcu_init();

  SensorsI2c_Init();
  SensorsUart_Init();

  CanTelemetry_Init(&hfdcan2);
  CanCommand_Init(&hfdcan2); /* after CanTelemetry_Init() -- see canCommand.h */
  CanNode_Init(&hfdcan1); /* CAN1, common bus between boards -- see canNode.h */

  /* Arm every sensor's data-ready interrupt / interrupt-driven reception. */
  SensorsI2c_Start();
  SensorsUart_Start();

  while (!CanNode_PollWakeUp()) {
    CanBusOff_Poll(); /* in bus-off the wake up would never arrive */
  }
  SendInitStatus();

  /* From here on, a loop pass longer than 1 s resets the board (watchdog.h).
   * Calibration_Update() blocks ~2 s, so it refreshes it itself. */
  Watchdog_Start();

  while (1) {
    Watchdog_Refresh();
    CanBusOff_Poll(); /* CAN1 and CAN2: a bus-off never clears by itself */

    /* Un wake up nuevo significa que el core ha vuelto a arrancar: se le
     * repite con qué componentes cuenta esta placa */
    if (CanNode_PollWakeUp()) { /* llama a CanNode_Poll() por dentro */
      SendInitStatus();
    }

    Calibration_Update();   /* CMD_AVIONICS_CALIBRATE: blocks ~2 s */

    SensorsI2c_Service();
    SensorsUart_Service();

    if (CanCommand_TakeLatest(&servoCommand)) {
      servoPulseUs = Servo_SetCommand(servoCommand);
    }
    canCommandLinkAlive = CanCommand_IsLinkAlive();

    loopCount++;
    heartbeat();
  }
}

/* INIT_STATUS (ICD 6.11): le dice al core qué componentes han arrancado. El
 * mapa de bits de los sensores I2C es el mismo que SensorsI2c_ActiveMask()
 * (ver el orden en sensorsI2c.h); el GPS y la VN-100 se añaden encima. */
static void SendInitStatus(void) {
  uint32_t ok = SensorsI2c_ActiveMask();

  if (SensorsUart_GpsActive()) {
    ok |= CAN_NODE_INIT_GPS;
  }
  if (SensorsUart_VectorNavActive()) {
    ok |= CAN_NODE_INIT_VECTORNAV;
  }

  initStatusOk = ok; /* visible desde el depurador */
  (void)CanNode_SendInitStatus(ok, CAN_NODE_INIT_SUPPORTED);
}

/* Heartbeat: 1 Hz while every I2C sensor is answering, dark otherwise. */
static void heartbeat(void) {
  if (SensorsI2c_Healthy()) {
    if ((HAL_GetTick() - lastBlinkTick) >= HEARTBEAT_HALF_PERIOD_MS) {
      lastBlinkTick = HAL_GetTick();
      HAL_GPIO_TogglePin(TEST_LED_GPIO_Port, TEST_LED_Pin);
    }
  } else {
    HAL_GPIO_WritePin(TEST_LED_GPIO_Port, TEST_LED_Pin, GPIO_PIN_RESET);
  }
}

static void mcu_init(void) {
  HAL_Init();
  SystemClock_Config();
  /* Before MX_GPIO_Init(): that is what enables the data-ready EXTIs, and
   * their callback reads the timestamp. */
  Timestamp_Init();

  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_LPUART1_UART_Init();
  MX_USART2_UART_Init();
  MX_FDCAN1_Init(); /* FDCAN1 first: it owns the clock divider both share */
  MX_FDCAN2_Init();

  /* After MX_GPIO_Init(), which configures PB10 as a plain output -- see
   * servo.h. Starts with no pulses. */
  Servo_Init();
}
