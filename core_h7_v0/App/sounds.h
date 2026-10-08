/**
 * @file sounds.h
 * @brief Melodías y avisos sonoros con el zumbador (Drivers/Buzzer).
 */

#ifndef SOUNDS_H
#define SOUNDS_H

#include <stdbool.h>

/* Poner a 0 para arrancar sin melodía (tarda unos 13 s) */
#define SOUNDS_STARTUP_ENABLE 1

/* Bella Ciao al arrancar (bloqueante) */
void sounds_startup(void);

/*
 * PITIDO DE VUELO: un pitido cada segundo mientras la core esté en cualquier
 * estado de vuelo que no sea la espera en tierra (quién decide eso:
 * flight_software.c). Dice de oído que la core cree estar volando y, sobre
 * todo, ayuda a encontrar el cohete: tras el aterrizaje sigue pitando hasta
 * que se apague.
 *
 * NO BLOQUEA: llamar en cada vuelta del bucle con 'activo'; enciende y apaga
 * el zumbador cuando toca. Empieza a pitar en cuanto 'activo' pasa a true.
 *
 * La frecuencia: el zumbador suena más fuerte cerca de su resonancia, que en
 * los típicos está entre 2 y 4 kHz (y el oído también es más sensible ahí).
 * Si suena flojo, probar otras.
 */
#define SOUNDS_PITIDO_HZ 2700U
#define SOUNDS_PITIDO_MS 200U          /* lo que suena cada vez */
#define SOUNDS_PITIDO_PERIODO_MS 1000U /* uno por segundo */

void sounds_pitido_vuelo(bool activo);
bool sounds_pitido_activo(void);

#endif /* SOUNDS_H */
