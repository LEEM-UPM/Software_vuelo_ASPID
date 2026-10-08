/**
 * @file led_show.h
 * @brief Animaciones en los 6 LEDs RGB de la placa (Drivers/Leds).
 */

#ifndef LED_SHOW_H
#define LED_SHOW_H

#include <stdbool.h>

/* Poner a 0 para arrancar sin animación (dura unos 4 s) */
#define LED_SHOW_STARTUP_ENABLE 1

/* Brillo máximo de las animaciones, 0-255. A 255 deslumbran bastante. */
#define LED_SHOW_BRIGHTNESS 150U

/* Secuencia de arranque, antes de la melodía (bloqueante). */
void led_show_startup(void);

/*
 * Los LEDs mientras vuela el programa. Llamar en cada vuelta del bucle con
 * el estado del modo test y de las cámaras. NO bloquea: con el modo test
 * refresca la tira cada LED_SHOW_TEST_STEP_MS (~0,25 ms cada refresco); sin
 * él, solo cuando cambia algo. Con todo apagado deja la tira apagada.
 *
 * MODO TEST: los LEDs "respiran" en rojo, subiendo y bajando de forma
 * gradual, un ciclo completo por segundo. Por que un aviso en la propia
 * placa: el modo test es el estado en el que el cohete acepta encender
 * pirotecnicos por radio. Quien este al lado del cohete tiene que poder
 * saberlo mirandolo, sin pantalla.
 *
 * CAMARAS ENCENDIDAS (camaras.h): el PRIMER LED, fijo en violeta. Si a la
 * vez hay modo test, el rojo respira en los otros cinco.
 */
void led_show_update(bool modo_test, bool camaras);

/* Color del primer LED con las cámaras encendidas (antes del brillo
 * LED_SHOW_BRIGHTNESS): violeta */
#define LED_SHOW_CAMARAS_R 110U
#define LED_SHOW_CAMARAS_G 0U
#define LED_SHOW_CAMARAS_B 255U

/* Ciclo completo de la respiracion y paso de refresco */
#define LED_SHOW_TEST_PERIOD_MS 1000U
#define LED_SHOW_TEST_STEP_MS 20U

#endif /* LED_SHOW_H */
