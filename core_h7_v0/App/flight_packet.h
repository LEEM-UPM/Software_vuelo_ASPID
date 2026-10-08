/*
 * Paquete de telemetría de vuelo: 24 bytes que el core manda a la RF por CAN1
 * (CMD_RF_TRANSMIT) y esta pone en el aire tal cual.
 *
 * POR QUÉ 24 Y NO MENOS. LoRa no cobra el aire byte a byte: cuantiza el
 * payload en bloques. A SF7 con CR 4/5, cualquier tamaño entre 23 y 26 bytes
 * cuesta EXACTAMENTE lo mismo, 30,9 ms. Bajar a 22 ahorraría 2,6 ms; subir de
 * 26 salta al siguiente bloque. Así que 24 es gratis frente a 23 y deja sitio
 * para los indicadores de validez, que es lo que más falta hace.
 *
 * Este módulo solo serializa: no transmite, no mira el reloj y no tiene
 * estado más allá del contador de paquete. La cadencia la decide quien lo
 * llama.
 *
 * FORMATO (little endian, como el resto del proyecto)
 *
 *  byte  campo                     tipo    codificación
 *   0    contador                  uint8   +1 por paquete, envuelve en 256
 *   1    bits 0-3 continuidad de los 4 pirotécnicos (1A, 1B, 2A, 2B)
 *        bits 4-6 fase de vuelo (0 IDLE ... 7 EN TIERRA)
 *        bit  7   navegación válida
 *   2    bits 0-2 estado del cohete (0 POWER_ON, 1 MANTENIMIENTO, 2 EN VUELO,
 *        3 ERROR; 4 EMERGENCIA: reiniciado en vuelo, decide con el
 *        barómetro, ver emergencia.h)
 *        bits 3-7 código de error (satura en 31)
 *   3-4  altura sobre el origen    int16   0,25 m      +/-8191 m
 *   5-6  posición norte            int16   1 m         +/-32767 m
 *   7-8  posición este             int16   1 m
 *   9-10 velocidad vertical        int16   0,05 m/s    + = subiendo
 *  11    velocidad norte           int8    1 m/s
 *  12    velocidad este            int8    1 m/s
 *  13    aceleración X (cuerpo)    int8    0,25 g      +/-31,75 g
 *  14    aceleración Y (cuerpo)    int8    0,25 g
 *  15    aceleración Z (cuerpo)    int8    0,25 g
 *  16-19 cuaternión w, x, y, z     4xint8  1/127
 *  20    apogeo previsto           uint8   25 m        0 a 6375 m
 *  21    aerofreno                 uint8   0,4 %       0 a 100 %
 *  22    tensión de bateria        uint8   80 mV       0 a 20,4 V
 *  23    temperatura del barómetro int8    1 grado C
 *
 * DECISIONES DE ESCALA, porque no son las obvias:
 *
 * - Todo lo que puede ser negativo VA CON SIGNO. La velocidad vertical sobre
 *   todo: el cohete pasa la mayor parte del vuelo BAJANDO, y sin signo esa
 *   parte no se puede representar.
 *
 * - Las resoluciones son deliberadamente más gruesas de lo que permitirían
 *   los bits. No tiene sentido mandar la altura en centímetros cuando el EKF
 *   deriva metros sin GPS (medido: 10 cm en reposo, y mucho más en vuelo).
 *   Esos bits se gastan mejor en rango, que es lo que evita que un vuelo
 *   inesperadamente alto sature el campo.
 *
 * - La batería va de 0 a 20,4 V en pasos de 80 mV en vez del rango ajustado
 *   de una 3S. Un campo que satura no informa de nada. (Los 17,05 V que se
 *   leían en el banco resultaron ser el ADC de la power SATURADO por tener el
 *   pin VREF+ flotando; con el arreglo de la power se leen ~13,7 V. Ver
 *   PENDIENTES.) 80 mV sobra para seguir una descarga.
 *
 * - La ACTITUD VA EN CUATERNIÓN, no en Euler. A 90 grados de cabeceo hay
 *   bloqueo de cardán y alabeo y guiñada dejan de ser independientes, y 90
 *   grados es exactamente la actitud del cohete durante toda la cuenta atrás.
 *   Con 8 bits por componente la resolución es de ~0,9 grados.
 *
 * - Van nav_valid y el código de error porque SIN ELLOS NO SE PUEDE
 *   DISTINGUIR un dato bueno de uno inventado. Pasó en el banco: la consola
 *   mostraba una actitud de 84,5 grados perfectamente creíble que eran los
 *   valores de fábrica del modelo, no una medida. Desde tierra, sin ese bit,
 *   no hay forma de saberlo.
 */

#ifndef FLIGHT_PACKET_H
#define FLIGHT_PACKET_H

#include <stdint.h>

#include "sys_utilities.h"

#define FLIGHT_PACKET_BYTES 24U

/*
 * Rellena out[] con el estado actual del sistema y avanza el contador de
 * paquete. Lee del GNC (gnc.h), de la placa de sensores (sensor_board.h) y
 * del estado del sistema que se le pasa.
 */
void flight_packet_build(const sys_state *state,
                         uint8_t out[FLIGHT_PACKET_BYTES]);

/* Valor que llevará el PRÓXIMO paquete construido. */
uint8_t flight_packet_next_counter(void);

/* Pone el contador a cero. Para arrancar una cuenta limpia en cada vuelo. */
void flight_packet_reset_counter(void);

#endif /* FLIGHT_PACKET_H */
