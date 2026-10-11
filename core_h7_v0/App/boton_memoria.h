/*
 * Botón BUTTON1 (PG4): borra la memoria de vuelo (memoria_vuelo.h) y
 * reinicia la core, que arranca como si nunca hubiera volado.
 *
 * PG4 es entrada con pull-down: el botón lo pone en alto mientras se pulsa.
 * Hay que MANTENERLO BOTON_MEMORIA_MS, para que un rebote o una vibración no
 * borre nada.
 *
 * SOLO EN TIERRA: con un vuelo en marcha (o en el modo de emergencia) borrar
 * dejaría a la core sin saber dónde estaba si se reinicia, y además el chip
 * se queda parado 1-2 s. Quién decide si se puede es flight_software.c
 * (_boton_permitido). Si no se puede, se dice por la consola y no se hace
 * nada.
 *
 * El reinicio es por software: la causa del siguiente arranque es
 * "software" y, sin nada guardado, arranca normal (INICIO). El número de
 * vuelo vuelve a empezar.
 */
#ifndef BOTON_MEMORIA_H
#define BOTON_MEMORIA_H

#include <stdbool.h>

#define BOTON_MEMORIA_MS 1000U

/* En cada vuelta del bucle. permitido = en tierra, sin vuelo en marcha. */
void boton_memoria_update(bool permitido);

#endif /* BOTON_MEMORIA_H */
