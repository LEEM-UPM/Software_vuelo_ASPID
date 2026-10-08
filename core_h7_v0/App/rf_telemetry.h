/*
 * Telemetría por radio: construye el paquete de vuelo (flight_packet.h) y se
 * lo manda a la placa de RF por CAN1 (CMD_RF_TRANSMIT), que lo pone en el
 * aire tal cual.
 *
 * LA CADENCIA LA MARCA EL CORE. La RF no decide nada: transmite un paquete
 * LoRa por cada trama de CAN que recibe. Así que el ritmo de la telemetría es
 * exactamente la frecuencia con la que se llama aquí.
 *
 * CICLO DE TRABAJO. Un paquete de 24 bytes a SF7/BW250 ocupa el aire 30,9 ms.
 * La banda de 869,4 a 869,65 MHz permite un 10 % medido sobre UNA HORA (360 s
 * de emisión por hora). Por eso la cadencia depende de si se ha despegado:
 *
 *   en tierra, hasta ver el lanzamiento    3 Hz   9,3 %: se puede sostener
 *                                                 todo el tiempo en la rampa
 *   desde el lanzamiento                  15 Hz   46 %: el presupuesto se
 *                                                 agota en 13 minutos
 *
 * LANZAMIENTO: el GNC pasa a EN VUELO, o una IMU mide más de 3 g durante
 * 100 ms (el respaldo de camaras.c, por si el GNC no lo ve), o la core
 * arranca en modo de emergencia (se reinició en el aire). En un vuelo de
 * verdad se queda en 15 Hz hasta el siguiente arranque. SIMULAR RUTA sube a
 * 15 Hz mientras el GNC está EN VUELO y vuelve a 3 al acabar: una simulación
 * no es un lanzamiento.
 *
 * Bajar otra vez tras el aterrizaje (o tras unos minutos de vuelo), para que
 * la recuperación no se coma el presupuesto, está en PENDIENTES.
 *
 * Además, con la radio menos tiempo transmitiendo, la RF escucha más: en
 * tierra, a 3 Hz, la subida de comandos tiene mucho más hueco que a 15.
 */

#ifndef RF_TELEMETRY_H
#define RF_TELEMETRY_H

#include "flight_packet.h"
#include "status_packet.h"
#include "sys_utilities.h"

/* Paquetes por segundo. Ver el aviso de ciclo de trabajo de arriba. */
#define RF_TELEMETRY_HZ_TIERRA 3U
#define RF_TELEMETRY_HZ_VUELO 15U

/*
 * Si el bucle principal se bloquea más que esto (la calibración de aviónica,
 * una escritura larga en la SD), NO se recupera el retraso soltando una
 * ráfaga: se resincroniza. Una ráfaga se comería el ciclo de trabajo de golpe
 * y llenaría la cola de la RF.
 */
#define RF_TELEMETRY_RESYNC_MS 500U

/*
 * Paquetes de estado (status_packet.h) que se mandan al arrancar. Varios
 * porque el camino de bajada no tiene acuse: si el unico se pierde, tierra se
 * queda sin estado hasta que pida uno. Tres cuestan 93 ms de aire.
 */
#define RF_TELEMETRY_ESTADO_ARRANQUE 3U

/* Espaciado entre esos tres. */
#define RF_TELEMETRY_ESTADO_REPETICION_MS 2000U

/*
 * Cuanto se espera, como mucho, a que las tres placas contesten al wake up
 * antes de mandar el primer paquete de estado. Si se mandara antes saldria
 * diciendo que no hay nada montado, que es peor que no mandarlo; pero si una
 * placa NO va a contestar nunca, ese es justo el caso en el que mas falta
 * hace que tierra se entere.
 */
#define RF_TELEMETRY_ESTADO_ESPERA_MS 5000U

/* Llamar en cada vuelta del bucle principal; transmite cuando toca. */
void rf_telemetry_update(sys_state *state);

/*
 * Pide que el proximo hueco de transmision lleve un paquete de estado en vez
 * de uno de vuelo. Lo llama uplink.c cuando tierra lo pide. Es idempotente:
 * pedirlo dos veces antes de que salga manda uno solo.
 */
void rf_telemetry_pedir_estado(void);

/*
 * Lo mismo con el paquete de cámaras (camaras.h): ocupa un hueco detrás del
 * de estado. Lo pide camaras.c (cada 5 s con ellas encendidas y en cada
 * cambio) y uplink.c cuando tierra pide el estado. Idempotente.
 */
void rf_telemetry_pedir_camaras(void);
uint32_t rf_telemetry_camaras_enviados(void);

/*
 * Pone un acuse (uplink.h) en el proximo hueco de transmision, por delante
 * del paquete de estado y del de vuelo. Si llega otro antes de que salga, el
 * nuevo pisa al viejo: los acuses llevan el valor ACTUAL, asi que el ultimo
 * siempre es el bueno. Como mucho 8 bytes.
 */
void rf_telemetry_mandar_acuse(const uint8_t *datos, uint8_t len);
uint32_t rf_telemetry_acuses_enviados(void);

uint32_t rf_telemetry_sent(void);
uint32_t rf_telemetry_failed(void);

/* Último paquete construido: FLIGHT_PACKET_BYTES bytes, exactamente lo que se
 * mandó (o se intentó mandar) por radio. Nunca es NULL. */
const uint8_t *rf_telemetry_last_packet(void);

/* Cadencia real medida sobre el último segundo, para contrastarla en la
 * consola con la que toca (rf_telemetry_cadencia_hz). */
uint32_t rf_telemetry_measured_hz(void);

/* La que toca ahora: RF_TELEMETRY_HZ_TIERRA o RF_TELEMETRY_HZ_VUELO */
uint32_t rf_telemetry_cadencia_hz(void);

/* Lanzamiento visto en un vuelo de verdad (no simulando): ya no se baja */
bool rf_telemetry_lanzamiento_visto(void);

uint32_t rf_telemetry_estado_enviados(void);

/* Ultimo paquete de estado construido: STATUS_PACKET_BYTES bytes. Todo a
 * cero mientras no se haya mandado ninguno. */
const uint8_t *rf_telemetry_ultimo_estado(void);

#endif /* RF_TELEMETRY_H */
