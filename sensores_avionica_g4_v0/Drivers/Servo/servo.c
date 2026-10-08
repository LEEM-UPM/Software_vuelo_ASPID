/**
 ******************************************************************************
 * @file    servo.c
 * @brief   Hobby-servo PWM output on PB10 (AIRBRAKE).
 ******************************************************************************
 */

#include "servo.h"

#define SERVO_PERIOD_US 20000u    /* 50 Hz */
#define SERVO_MIN_PULSE_US 500u   /* 2.5 % of 20 ms, command 0 */
#define SERVO_MAX_PULSE_US 2500u  /* 12.5 % of 20 ms, command 255 */

void Servo_Init(void) {
  __HAL_RCC_TIM2_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* TIM2 sits on APB1. Its input clock is PCLK1 when the APB1 prescaler is
   * 1, and twice PCLK1 otherwise (reference manual, RCC clock tree). */
  uint32_t timerClockHz = HAL_RCC_GetPCLK1Freq();
  if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) {
    timerClockHz *= 2u;
  }

  TIM2->CR1 = 0;
  TIM2->PSC = (timerClockHz / 1000000u) - 1u; /* 1 tick = 1 us */
  TIM2->ARR = SERVO_PERIOD_US - 1u;
  TIM2->CCR3 = 0; /* no pulse until the first Servo_SetCommand() */

  /* Channel 3: PWM mode 1 (high while CNT < CCR3), compare preload on so a
   * new CCR3 only takes effect at the next period. */
  TIM2->CCMR2 = (TIM2->CCMR2 & ~(TIM_CCMR2_CC3S | TIM_CCMR2_OC3M)) |
                (6u << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;
  TIM2->CCER |= TIM_CCER_CC3E; /* active high */

  /* Load PSC/ARR/CCR3 now (update event), clear the flag that leaves. */
  TIM2->EGR = TIM_EGR_UG;
  TIM2->SR = 0;

  TIM2->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;

  /* Only now hand the pin to the timer, so it goes straight from the low
   * level MX_GPIO_Init() left it at to a (still pulse-free) PWM output. */
  GPIO_InitTypeDef gpio = {0};
  gpio.Pin = AIRBRAKE_Pin;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(AIRBRAKE_GPIO_Port, &gpio);
}

uint32_t Servo_SetCommand(uint8_t command) {
  /* Linear map, rounded to the nearest microsecond. */
  uint32_t span = SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US;
  uint32_t pulseUs = SERVO_MIN_PULSE_US + ((command * span) + 127u) / 255u;

  TIM2->CCR3 = pulseUs;
  return pulseUs;
}
