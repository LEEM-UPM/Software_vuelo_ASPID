/*
 * Despliegue de paracaídas: convierte las banderas del GNC en órdenes a los
 * pirotécnicos.
 *
 *   drogue_deploy_flag  ->  canales 1A y 1B, A LA VEZ (redundancia: los dos
 *                           iniciadores del drogue en la misma vuelta)
 *   main_deploy_flag    ->  canal 2A
 *
 * Cada canal lleva su propio estado (reloj de repeticion, ordenes): 1A y 1B
 * se repiten y se reintentan por separado, asi que si power rechaza uno el
 * otro no se entera.
 *
 * ES EL FLUJO DE VUELO, Y LA SIMULACIÓN PASA POR ÉL TAL CUAL. No hay ninguna
 * rama "si se está simulando": la simulación de ruta (gnc_playback.c) sólo
 * cambia de dónde salen los datos del GNC, y todo lo que cuelga de él se
 * ejecuta igual. Es a propósito, para validar en el banco exactamente el
 * código que va a volar. Consecuencia: UNA SIMULACIÓN EN EL BANCO ENCIENDE
 * DE VERDAD 1A, 1B Y 2A. Simular con cargas de prueba, no con cargas reales.
 *
 * Cómo da las banderas el GNC (GuidanceControl.c): cada una vale 1 durante el
 * PRIMER SEGUNDO de su fase (DROGUE y MAIN) y luego vuelve a 0. Mientras vale
 * 1 aquí se repite la orden cada DESPLIEGUE_REPETIR_MS: así una trama de CAN
 * perdida no cuesta un paracaídas. Repetir no hace daño: power mantiene el
 * canal 1 s y cada orden sólo reinicia ese segundo (ICD 6.3), y un iniciador
 * ya quemado no se puede volver a quemar.
 *
 * Sólo se obedece a las banderas con la máquina de estados en EN VUELO. Una
 * bandera espuria en tierra no enciende nada. La simulación pasa por EN VUELO
 * igual que un vuelo de verdad, así que esto no le quita nada a la validación.
 *
 * Lo que se ha mandado se olvida en cada gnc_init() (vuelo o simulación
 * nueva), mirando gnc_init_count().
 */

#ifndef DESPLIEGUE_H
#define DESPLIEGUE_H

#include <stdbool.h>
#include <stdint.h>

#include "sys_utilities.h"

/* Mientras la bandera esté a 1, una orden cada esto: 5 en el segundo */
#define DESPLIEGUE_REPETIR_MS 200U

/* Si power no acepta la orden (no despierta, cola de CAN llena), se reintenta
 * tras esto en vez de en la siguiente vuelta del bucle, que es cuestión de
 * microsegundos y sólo serviría para saturar el bus. */
#define DESPLIEGUE_REINTENTO_MS 20U

/* Llamar en cada vuelta del bucle principal, DESPUÉS del GNC. */
void despliegue_update(const sys_state *state);

/* Para la consola y el paquete de estado */
uint32_t despliegue_ordenes_1a(void); /* órdenes aceptadas por power */
uint32_t despliegue_ordenes_1b(void);
uint32_t despliegue_ordenes_2a(void);
bool despliegue_drogue_mandado(void); /* 1A o 1B aceptada alguna vez */
bool despliegue_main_mandado(void);   /* 2A aceptada alguna vez */
uint32_t despliegue_fallos(void);     /* órdenes que power no aceptó */
uint32_t despliegue_drogue_ms(void);  /* HAL_GetTick() de la 1.ª; 0 = no */
uint32_t despliegue_main_ms(void);

#endif /* DESPLIEGUE_H */
