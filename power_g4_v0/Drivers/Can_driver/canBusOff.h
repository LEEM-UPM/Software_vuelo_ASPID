/*
 * Salida de BUS-OFF de los dos FDCAN de esta placa.
 *
 * Cada nodo CAN cuenta sus errores al transmitir (+8 por trama fallida, -1
 * por trama buena). Si pasa de 255 (unas 32 tramas fallidas seguidas, que
 * con los reintentos automaticos son pocos milisegundos: un mal contacto,
 * CANH y CANL en corto, ruido) el nodo se desconecta solo del bus. El FDCAN
 * del STM32 se queda entonces en INIT y NO VUELVE POR SU CUENTA: hay que
 * quitar CCCR.INIT, y entonces espera 128 ratos de bus libre (~1,4 ms a
 * 1 Mbit/s) y se reincorpora solo.
 *
 * Llamar CanBusOff_Poll() en cada vuelta de los bucles, TAMBIEN mientras se
 * espera el wake up: con la placa en bus-off no llegaria nunca. Es barato:
 * dos lecturas de registro por FDCAN.
 */
#ifndef CAN_BUS_OFF_H
#define CAN_BUS_OFF_H

#include <stdint.h>

void CanBusOff_Poll(void);

/* Veces que se ha sacado de bus-off: bus 1 = FDCAN1, 2 = FDCAN2 */
uint32_t CanBusOff_Count(uint32_t bus);

#endif /* CAN_BUS_OFF_H */
