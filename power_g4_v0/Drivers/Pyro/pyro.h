#ifndef PYRO_H
#define PYRO_H

/*
 * Canales pirotécnicos: alimentación de 7 V de cada par (Pyro1/Pyro2), MOSFET
 * de cada canal (1A, 1B, 2A, 2B) y lectura de continuidad.
 * Documentación de cada función en README_UTILIDADES.txt.
 */

#include <stdbool.h>

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

void Pyro1_ON(void);
void Pyro1_OFF(void);
void Pyro2_ON(void);
void Pyro2_OFF(void);

void Pyro1A_ON(void);
void Pyro1A_OFF(void);
void Pyro1B_ON(void);
void Pyro1B_OFF(void);
void Pyro2A_ON(void);
void Pyro2A_OFF(void);
void Pyro2B_ON(void);
void Pyro2B_OFF(void);

bool Pyro1A_Continuity(void);
bool Pyro1B_Continuity(void);
bool Pyro2A_Continuity(void);
bool Pyro2B_Continuity(void);

/*
 * Prueba de los cuatro canales al arrancar (AllPyroCheck desde main.c).
 *
 * TIENE QUE ESTAR A 0 SIEMPRE QUE HAYA UNA CARGA CONECTADA. Enciende 1A, 1B,
 * 2A y 2B en cascada, un segundo cada uno, y acaba con los cuatro encendidos
 * a la vez: con cargas puestas las quema todas al encender el cohete. Y como
 * se ejecuta con el primer wake up, y el core lo manda para siempre, un
 * reinicio de esta placa en vuelo (una caida de tension) las quemaria todas
 * en el aire. Ademas son 5 s de HAL_Delay sin atender el CAN.
 *
 * Hasta el 2026-10-02 se ejecutaba en cada arranque. Ponerlo a 1 solo en el
 * banco, sin cargas, para ver encenderse los canales.
 */
#ifndef PYRO_PRUEBA_ARRANQUE
#define PYRO_PRUEBA_ARRANQUE 0
#endif

/* Ver PYRO_PRUEBA_ARRANQUE: solo en el banco y sin cargas */
void AllPyroCheck(void);

/*
 * PRUEBA SIN CAN (2026-10-09). Nada más arrancar, sin esperar el wake up ni
 * nada del CAN, enciende la alimentación de los dos pares (Pyro1, Pyro2) y
 * los cuatro canales (1A, 1B, 2A, 2B) A LA VEZ, y los deja ENCENDIDOS hasta
 * que se quite la corriente: sin apagado automático. Es para ver en el banco
 * que cada etapa responde (LED, polímetro, osciloscopio). La placa no hace
 * nada más: ni CAN, ni continuidad, ni watchdog.
 *
 * SOLO EN EL BANCO Y SIN CARGAS: con un iniciador conectado lo dispara al
 * encender. Por eso aquí se queda a 0 y la prueba se compila aparte, en su
 * propia carpeta, sin tocar este archivo:
 *
 *   cmake --preset Debug -B build/PruebaPiros -DCMAKE_C_FLAGS=-DPYRO_PRUEBA_SIN_CAN=1
 *   cmake --build build/PruebaPiros
 *
 * (avisa al compilar). Para volver al firmware de vuelo, grabar el de
 * build/Debug.
 */
#ifndef PYRO_PRUEBA_SIN_CAN
#define PYRO_PRUEBA_SIN_CAN 0
#endif

/* Ver PYRO_PRUEBA_SIN_CAN. No vuelve. */
void PyroPruebaSinCan(void);

#ifdef __cplusplus
}
#endif

#endif /* PYRO_H */
