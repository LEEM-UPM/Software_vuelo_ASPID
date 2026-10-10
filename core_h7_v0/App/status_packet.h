/*
 * Paquete de estado: 32 bytes con la salud del cohete, no con su vuelo.
 *
 * El de vuelo (flight_packet.h) contesta "¿dónde está y qué hace?" quince
 * veces por segundo. Este contesta "¿está todo entero?" y se manda muy de
 * tarde en tarde: unas veces al arrancar y luego sólo cuando tierra lo pide
 * (uplink.h). Son preguntas distintas con ritmos distintos, y por eso son dos
 * paquetes y no uno más gordo: meter esto en el de vuelo costaría aire 15
 * veces por segundo para repetir cosas que no cambian nunca.
 *
 * CÓMO SE DISTINGUE DEL DE VUELO, que es la parte delicada. El de vuelo mide
 * 24 bytes y su byte 0 es un CONTADOR, así que una vez cada 256 paquetes vale
 * 0xA5 por pura casualidad. Si se identificara por el primer byte, uno de
 * cada 256 paquetes de vuelo se leería como si fuera de estado. Por eso:
 *
 *     el de vuelo se reconoce por MEDIR 24 BYTES
 *     el de estado, por medir 32 Y ADEMÁS empezar por 0xA5
 *
 * Las dos condiciones a la vez. Cualquier paquete nuevo tendrá que llevar
 * también su marca y una longitud que no choque.
 *
 *
 * FORMATO (little endian)
 *
 *  byte  campo
 *   0    0xA5, marca de paquete de estado
 *   1    versión del formato (STATUS_PACKET_VERSION)
 *   2    contador, +1 por paquete
 *   3    placas y validez
 *          b0 power respondió al wake up
 *          b1 aviónica respondió
 *          b2 rf respondió
 *          b3 INIT_STATUS de aviónica recibido
 *          b4 INIT_STATUS de power recibido
 *          b5 INIT_STATUS de rf recibido
 *          b6 calibración de aviónica hecha
 *          b7 navegación válida
 *   4    sensores de aviónica ARRANCADOS
 *          b0 baro1  b1 baro2  b2 imu1  b3 imu2  b4 mag  b5 gps  b6 vn100
 *   5    sensores que la placa de aviónica TIENE (supported)
 *   6    tarjetas
 *          b0-1 estado SD1   b2-3 estado SD2   (0 sin iniciar, 1 no responde,
 *                                               2 no monta, 3 montada)
 *          b4 autotest SD1 OK   b5 autotest SD2 OK
 *          b6 pin CD SD1        b7 pin CD SD2
 *   7    arranques de SD1 (uint8, satura en 255)
 *   8    arranques de SD2
 *   9-10 MB libres en SD1 / 16 (uint16, hasta 1 TB)
 *  11-12 MB libres en SD2 / 16
 *  13-14 tramas por segundo en CAN1 (uint16)
 *  15-16 tramas por segundo en CAN2 (uint16)
 *  17    errores de CAN1 (desbordes + perdidas + errores tx, uint8 saturado)
 *  18    errores de CAN2
 *  19-20 segundos desde el arranque (uint16, satura a las 18 h)
 *  21    wake ups reenviados (uint8 saturado)
 *  22-23 paquetes de radio que no se pudieron encolar (uint16 saturado)
 *  24    b0-2 fase de vuelo, b3-5 estado del cohete
 *          b6 GNC alineado en la rampa   b7 0 AGL del barómetro enclavado
 *             (nav_init_flags del GNC v1.6, versión 7)
 *  25    modo test (uplink.h)
 *          b0 activo
 *          b1-3 ultimo pirotecnico aceptado (0 ninguno, 1=1A 2=1B 3=2A 4=2B)
 *          b4 SIMULANDO: el GNC vuela VUELO.BIN, los datos NO son reales
 *          b5 POSTVUELO: reiniciado tras un vuelo (emergencia.h): no toca
 *             la SD ni enciende las cámaras ni manda pirotécnicos de vuelo
 *             (versión 6)
 *          b6 origen GNSS enclavado   b7 navegando SIN haber alineado
 *             (degradado). Con b6 y b7 del byte 24 y b6 de este, el GNC
 *             está LISTO PARA VOLAR (nav_ready): lo que hay que ver antes de
 *             armar el iniciador (versión 7)
 *  26    segundos que le quedan al modo test (uint8 saturado)
 *  27    pirotecnicos aceptados desde el arranque (uint8 saturado)
 *  28    comandos de subida denegados (uint8 saturado): bien formados
 *        pero no permitidos, p. ej. un pirotecnico fuera del modo test
 *  29    avance de la simulacion, 0..100 %
 *  30    motivo de la ULTIMA denegacion (uplink_denegado_t), para que tierra
 *        pueda decir por que no se ha hecho lo que pidio
 *  31    despliegue de paracaidas (despliegue.h), en este vuelo
 *          b0 se ha mandado 1A por la bandera de drogue
 *          b1 se ha mandado 1B por la bandera de main
 *          b2 power rechazo alguna orden de despliegue
 *          b3 CAMARAS encendidas (camaras.h; version 5)
 *          b4 EMERGENCIA: reiniciado en vuelo, decide con el barómetro
 *          b5 emergencia: APOGEO detectado por el barómetro
 *          b6 emergencia: alcanzada la ALTURA DEL MAIN (400 m)
 *          b7 emergencia: barómetro NO fiable ahora (descartando medidas)
 *          (b4-b7 versión 6; b0 y b1 cuentan también lo mandado antes
 *          del reinicio)
 *
 * VERSION 6 (2026-10-06) completa el byte 31 con el modo de emergencia
 * (b4-b7) y usa el b5 del byte 25 para POSTVUELO.
 * VERSION 4 (2026-10-01) usa el byte 31, el ultimo libre: el paquete ya no
 * tiene sitio, lo siguiente obliga a pensar en otro paquete o en mas bytes.
 * VERSION 5 (2026-10-06) usa el b3 del byte 31 para las camaras; quedan
 * libres b4-b7. Una estacion anterior lo ignora (y las ve como apagadas).
 * VERSION 3 (2026-10-01) añade el bit de simulacion y el byte 29.
 * VERSION 2 (2026-10-01) añade los bytes 25 a 28. Eran reservados a cero,
 * asi que una estacion de la version 1 los lee como "modo test inactivo"
 * y no se rompe nada.
 *
 * POR QUE 32 Y NO 26, que es lo que ocupan los campos. CAN FD no admite
 * cualquier longitud: solo 0-8, 12, 16, 20, 24, 32, 48 y 64. Al pedir 26 el
 * driver rellena con ceros hasta 32 y la placa de RF pone en el aire esos 32
 * bytes, asi que el paquete que se recibe mide 32 se quiera o no. Mas vale
 * decirlo aqui y quedarse los 6 bytes de sobra para lo que venga, que fingir
 * que mide 26 y que tierra tenga que adivinar el relleno.
 * Cuesta 36 ms de aire en vez de los 30,9 del de vuelo; como se manda tres
 * veces al arrancar y luego solo cuando se pide, da igual.
 *
 * POR QUÉ VAN 'ok' Y 'supported' DE LOS SENSORES, y no sólo 'ok'. Un bit a 0
 * puede significar dos cosas muy distintas: que el sensor falló, o que esa
 * placa no lo lleva montado. Sin el mapa de lo que la placa tiene, desde
 * tierra no hay forma de saber cuál de las dos es, y se acaba persiguiendo un
 * fallo que no existe.
 *
 * POR QUÉ LAS TRAMAS/S Y NO EL CONTADOR ACUMULADO. CAN2 pasa del millón de
 * tramas en media hora: no cabe en dos bytes y en cuatro son ocho bytes entre
 * los dos buses. Y lo que se quiere saber es si el bus está VIVO AHORA, que
 * es un ritmo, no un total. Los errores sí van acumulados, porque ahí lo que
 * importa es si ha pasado alguna vez.
 */

#ifndef STATUS_PACKET_H
#define STATUS_PACKET_H

#include <stdint.h>

#include "sys_utilities.h"

#define STATUS_PACKET_BYTES 32U
#define STATUS_PACKET_MARCA 0xA5U
#define STATUS_PACKET_VERSION 7U

/*
 * Muestrea la actividad de los buses. Llamar en cada vuelta del bucle
 * principal.
 *
 * Hace falta porque el driver de CAN sólo lleva contadores acumulados y aquí
 * se manda un ritmo. Se calcula sobre una ventana de un segundo, no sobre el
 * hueco entre dos paquetes de estado: entre uno y otro pueden pasar horas, y
 * una media de horas no dice si el bus está vivo ahora mismo.
 */
void status_packet_update(void);

/* Rellena out[] con el estado actual y avanza el contador. */
void status_packet_build(const sys_state *state,
                         uint8_t out[STATUS_PACKET_BYTES]);

/* Tramas por segundo medidas en la última ventana, para la consola. */
uint32_t status_packet_can_hz(uint32_t port); /* 0 = CAN1, 1 = CAN2 */

/*
 * false hasta que se ha completado la primera ventana de medida.
 *
 * Hace falta porque un bus SIN MEDIR y un bus MUERTO dan el mismo cero, y son
 * cosas muy distintas. Paso en el banco: el primer paquete de estado salia
 * tan pronto tras el arranque que aun no habia medida, y decia 0 tramas/s en
 * los dos buses con CAN2 moviendo 800. Quien manda el paquete espera a esto.
 */
bool status_packet_medido(void);

uint8_t status_packet_next_counter(void);

#endif /* STATUS_PACKET_H */
