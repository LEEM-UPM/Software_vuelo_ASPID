#include "canBusOff.h"

#include "main.h"

/* [0] FDCAN1, [1] FDCAN2. Para el depurador. */
static uint32_t busOffCount[2];

/* Quitado INIT, BO sigue a 1 hasta que termina la recuperacion: mirando
 * tambien INIT se cuenta una vez por caida y no una por vuelta. Un FDCAN sin
 * arrancar tiene INIT a 1 pero BO a 0: no se toca. */
static void Recover(FDCAN_GlobalTypeDef *fdcan, uint32_t *count) {
  if (((fdcan->PSR & FDCAN_PSR_BO) != 0u) &&
      ((fdcan->CCCR & FDCAN_CCCR_INIT) != 0u)) {
    CLEAR_BIT(fdcan->CCCR, FDCAN_CCCR_INIT);
    (*count)++;
  }
}

void CanBusOff_Poll(void) {
  Recover(FDCAN1, &busOffCount[0]);
  Recover(FDCAN2, &busOffCount[1]);
}

uint32_t CanBusOff_Count(uint32_t bus) {
  return ((bus >= 1u) && (bus <= 2u)) ? busOffCount[bus - 1u] : 0u;
}
