/*
 * La altura del aerofreno: por debajo de ella (AGL) el guiado no abre el
 * aerofreno. Lo manda tierra por radio (UPLINK_CMD_ALTURA_FRENO) y aquí se
 * guarda; quien se la pasa al GNC es quien la cambia (uplink.c) y
 * flight_software.c al arrancar, con gnc_set_altura_freno().
 *
 * Igual que la rampa (lanzamiento.h): SE GUARDA EN LAS DOS SD (FRENO.TXT) y
 * se lee al arrancar, para que un reinicio en la rampa no vuelva al valor de
 * fábrica sin que nadie se entere. Sólo se escribe si el valor CAMBIA: tierra
 * reintenta el comando hasta ver la confirmación.
 *
 * Unidades: metros enteros, como viaja por radio. Qué valores valen lo dice
 * el GNC (gnc_altura_freno_valida): mayor que 0 y menor que el apogeo
 * objetivo.
 */

#ifndef FRENO_ALTURA_H
#define FRENO_ALTURA_H

#include <stdbool.h>
#include <stdint.h>

#define FRENO_ALTURA_ARCHIVO "FRENO.TXT"

/* Lee el último guardado de la SD1 (o de la SD2 si la 1 no lo tiene).
 * Llamar después de storage_init(). Sin archivo: "sin definir". */
void freno_altura_init(void);

/* Guarda una altura nueva. Devuelve false si el GNC no la acepta (no cambia
 * nada). Que el cohete esté en tierra lo decide quien llama (uplink.c). */
bool freno_altura_definir(uint16_t metros);

/* La guardada; sin definir, la de fábrica del GNC */
uint16_t freno_altura_m(void);

/* true si el valor salió de un comando o de la SD; false si es el de fábrica */
bool freno_altura_definida(void);

/* true si la última escritura en la SD salió bien en al menos una tarjeta */
bool freno_altura_guardada(void);

/* De dónde salió el valor actual, para la consola */
const char *freno_altura_origen(void);

#endif /* FRENO_ALTURA_H */
