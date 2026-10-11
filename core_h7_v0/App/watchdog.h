/*
 * Watchdog independiente (IWDG1, reloj LSI de 32 kHz): si el bucle principal
 * se cuelga más de WATCHDOG_MS, la core se reinicia. Con la memoria de vuelo
 * (emergencia.h) un cuelgue en el aire pasa a ser un reinicio del que se
 * vuelve en menos de un segundo, en vez de un cohete sin nadie al mando.
 *
 * Se arranca justo antes del bucle (el arranque normal tiene esperas largas:
 * la melodía, las placas, la SD) y una vez arrancado NO SE PUEDE PARAR hasta
 * el siguiente reset. Con el depurador parado en un punto de ruptura se
 * congela (DBGMCU), así que depurar no lo dispara.
 *
 * Lo que puede bloquear el bucle más de 2 s hoy: una SD que muere en mitad
 * de una operación de FileX (el HAL espera hasta ~14 s, ver PENDIENTES).
 * Con el watchdog eso es un reinicio en vez de 14 s parado.
 */
#ifndef WATCHDOG_H
#define WATCHDOG_H

#include <stdint.h>

#define WATCHDOG_MS 2000U

void watchdog_iniciar(void);
void watchdog_refrescar(void);

/* Lleva el plazo al máximo (~32 s) y refresca: para algo que bloquea más de
 * WATCHDOG_MS a propósito y termina en un reset (boton_memoria.c). No se puede
 * volver a acortar sin un watchdog_iniciar(). El plazo sobrevive al reset por
 * software: el IWDG solo se para con un encendido. */
void watchdog_alargar(void);

#endif /* WATCHDOG_H */
