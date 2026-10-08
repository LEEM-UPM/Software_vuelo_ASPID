#include "watchdog.h"

#include "main.h"

/* LSI 32 kHz / 32 = 1 kHz: cada cuenta es 1 ms (maximo 4095) */
#define WATCHDOG_PRESCALER_32 3u
#define WATCHDOG_RELOAD (WATCHDOG_TIMEOUT_MS - 1u)

static uint32_t resetFlags; /* RCC_CSR al arrancar */

void Watchdog_Start(void) {
  resetFlags = RCC->CSR;
  RCC->CSR |= RCC_CSR_RMVF; /* el proximo arranque dira lo suyo */

  DBGMCU->APB1FZR1 |= DBGMCU_APB1FZR1_DBG_IWDG_STOP; /* parado con el depurador */
  IWDG->KR = 0xCCCCu; /* en marcha (enciende el LSI) */
  IWDG->KR = 0x5555u; /* deja escribir PR y RLR */
  IWDG->PR = WATCHDOG_PRESCALER_32;
  IWDG->RLR = WATCHDOG_RELOAD;
  const uint32_t t0 = HAL_GetTick();
  while ((IWDG->SR != 0u) && ((HAL_GetTick() - t0) < 100u)) {
  }
  IWDG->KR = 0xAAAAu;
}

void Watchdog_Refresh(void) { IWDG->KR = 0xAAAAu; }

bool Watchdog_LastResetWasWatchdog(void) {
  return (resetFlags & RCC_CSR_IWDGRSTF) != 0u;
}

uint32_t Watchdog_ResetFlags(void) { return resetFlags; }
