/**
 ******************************************************************************
 * @file    servo.h
 * @brief   Hobby-servo PWM output on PB10 (AIRBRAKE).
 *
 * Standard 50 Hz servo signal: 20 ms period, high time from 0.5 ms (2.5 %
 * duty) to 2.5 ms (12.5 % duty). The position is commanded as a uint8_t,
 * mapped linearly: 0 -> 0.5 ms, 255 -> 2.5 ms (~7.8 us per step).
 *
 * PB10 can only reach one timer, TIM2_CH3 (AF1) -- the other functions on
 * that pin are LPUART1_RX, USART3_TX, QUADSPI1_CLK, SAI1_SCK_A, TIM1_BKIN,
 * HRTIM1_FLT3. TIM2 runs with a 1 us tick and ARR = 19999, so the
 * compare value is the high time in microseconds directly.
 *
 * Like timestamp.h, this is set up through the registers directly, NOT
 * through CubeMX: the .ioc still has PB10 as a plain GPIO output
 * (AIRBRAKE), and MX_GPIO_Init() still configures it that way. Servo_Init()
 * has to run after MX_GPIO_Init() so that it can take the pin over. If TIM2
 * is ever set up in CubeMX instead, drop this module's timer code.
 ******************************************************************************
 */

#ifndef __SERVO_H__
#define __SERVO_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "main.h"

/* Starts the 50 Hz PWM on PB10 with NO pulses (output held low) -- the
 * servo stays unpowered until the first Servo_SetCommand(), rather than
 * jumping to some arbitrary position at boot. Must be called after
 * SystemClock_Config() and MX_GPIO_Init(). */
void Servo_Init(void);

/* Sets the servo position: 0 -> 0.5 ms (2.5 % duty), 255 -> 2.5 ms (12.5 %).
 * Takes effect at the start of the next 20 ms period (preloaded compare), so
 * a pulse is never cut short or stretched mid-period. Returns the resulting
 * high time in microseconds (diagnostic). */
uint32_t Servo_SetCommand(uint8_t command);

#ifdef __cplusplus
}
#endif

#endif /* __SERVO_H__ */
