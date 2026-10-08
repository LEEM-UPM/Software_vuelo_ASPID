/*
 * Watchdog independiente (IWDG): si el bucle principal deja de refrescarlo
 * durante WATCHDOG_TIMEOUT_MS, la placa se reinicia sola.
 *
 * Por que. Una placa colgada en vuelo (un bucle sin salida, una espera a un
 * periferico que no contesta) se queda asi hasta que se apaga el cohete. Con
 * el watchdog se reinicia en WATCHDOG_TIMEOUT_MS, vuelve a esperar el wake up
 * del core (que lo repite cada 2 s como latido) y sigue con lo suyo.
 *
 * Cuenta con el LSI (32 kHz), que no depende del reloj principal: sigue
 * aunque el HSE o el PLL fallen. Una vez en marcha NO SE PUEDE PARAR (solo
 * lo para un reinicio). Con el depurador parado en un punto de ruptura se
 * congela, para poder depurar.
 *
 * Arrancarlo justo antes del bucle principal y refrescarlo en cada vuelta.
 * Cualquier espera de mas de WATCHDOG_TIMEOUT_MS dentro del bucle tiene que
 * refrescarlo ella misma.
 */
#ifndef WATCHDOG_H
#define WATCHDOG_H

#include <stdbool.h>
#include <stdint.h>

/* La vuelta mas larga del bucle principal hoy no llega a 200 ms */
#define WATCHDOG_TIMEOUT_MS 1000u

void Watchdog_Start(void);
void Watchdog_Refresh(void);

/* El ultimo arranque fue por el watchdog. Lo mira Watchdog_Start() en
 * RCC_CSR, que deja a cero para el siguiente; Watchdog_ResetFlags() es el
 * registro entero (IWDGRSTF b29, SFTRSTF b28, PINRSTF b26, BORRSTF b27),
 * para el depurador. */
bool Watchdog_LastResetWasWatchdog(void);
uint32_t Watchdog_ResetFlags(void);

#endif /* WATCHDOG_H */
