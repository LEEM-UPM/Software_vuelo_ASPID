#include "watchdog.h"

#include "main.h"

/* LSI 32 kHz / 64 = 500 Hz: cada cuenta son 2 ms */
#define PRESCALER_64 4U
#define CUENTAS ((WATCHDOG_MS / 2U) - 1U)

void watchdog_iniciar(void) {
  DBGMCU->APB4FZ1 |= DBGMCU_APB4FZ1_DBG_IWDG1; /* parado con el depurador */
  IWDG1->KR = 0xCCCCU;                         /* en marcha (enciende el LSI) */
  IWDG1->KR = 0x5555U;                         /* deja escribir PR y RLR */
  IWDG1->PR = PRESCALER_64;
  IWDG1->RLR = CUENTAS;
  const uint32_t t0 = HAL_GetTick();
  while ((IWDG1->SR != 0U) && ((HAL_GetTick() - t0) < 100U)) {
  }
  IWDG1->KR = 0xAAAAU;
}

void watchdog_refrescar(void) { IWDG1->KR = 0xAAAAU; }

/* LSI 32 kHz / 256 = 125 Hz, 4096 cuentas: ~32 s */
#define PRESCALER_256 6U
#define CUENTAS_MAX 0xFFFU

void watchdog_alargar(void) {
  IWDG1->KR = 0xAAAAU; /* el plazo viejo, lleno, mientras se cambia */
  IWDG1->KR = 0x5555U;
  IWDG1->PR = PRESCALER_256;
  IWDG1->RLR = CUENTAS_MAX;
  const uint32_t t0 = HAL_GetTick();
  while ((IWDG1->SR != 0U) && ((HAL_GetTick() - t0) < 100U)) {
  }
  IWDG1->KR = 0xAAAAU;
}
