/**
 ******************************************************************************
 * @file    timestamp.h
 * @brief   Free-running microsecond timestamp, for tagging sensor samples.
 *
 * TIM5 (32-bit on the STM32G4) is set up as a plain up-counter ticking once
 * per microsecond, so reading the time is a single register read -- cheap
 * enough to do first thing inside an interrupt handler. No interrupt, no
 * HAL_TIM module: TIM5 is not configured in CubeMX (sensores_avionica_g4_v0
 * .ioc), so if it is ever enabled there for something else, this has to move.
 * TIM2, the other 32-bit timer, is taken: it drives the servo PWM on PB10,
 * the only timer that pin can be routed to (see servo.h).
 *
 * The count wraps every 2^32 us (~71.6 minutes). Differences between two
 * timestamps computed with unsigned arithmetic stay correct across a wrap,
 * as long as the two are less than that apart.
 ******************************************************************************
 */

#ifndef __TIMESTAMP_H__
#define __TIMESTAMP_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "main.h"

/* Starts the microsecond counter from 0. Must be called after
 * SystemClock_Config() -- the prescaler is derived from the clock tree as
 * configured at that point. */
void Timestamp_Init(void);

/* Microseconds since Timestamp_Init(), wrapping at 2^32. Safe to call from
 * any context, including interrupt handlers. */
static inline uint32_t Timestamp_NowUs(void) { return TIM5->CNT; }

#ifdef __cplusplus
}
#endif

#endif /* __TIMESTAMP_H__ */
