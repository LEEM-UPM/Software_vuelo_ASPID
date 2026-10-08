/**
 ******************************************************************************
 * @file    timestamp.c
 * @brief   Free-running microsecond timestamp, for tagging sensor samples.
 ******************************************************************************
 */

#include "timestamp.h"

void Timestamp_Init(void) {
  __HAL_RCC_TIM5_CLK_ENABLE();

  /* TIM5 sits on APB1. Its input clock is PCLK1 when the APB1 prescaler is
   * 1, and twice PCLK1 otherwise (reference manual, RCC clock tree). */
  uint32_t timerClockHz = HAL_RCC_GetPCLK1Freq();
  if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) {
    timerClockHz *= 2u;
  }

  TIM5->CR1 = 0;
  TIM5->PSC = (timerClockHz / 1000000u) - 1u; /* 1 tick = 1 us */
  TIM5->ARR = 0xFFFFFFFFu;                    /* free-running, full 32 bits */
  TIM5->CNT = 0;

  /* PSC is only loaded on an update event: force one, then clear the flag it
   * leaves behind (nothing is listening for it, but keep the state clean). */
  TIM5->EGR = TIM_EGR_UG;
  TIM5->SR = 0;

  TIM5->CR1 = TIM_CR1_CEN;
}
