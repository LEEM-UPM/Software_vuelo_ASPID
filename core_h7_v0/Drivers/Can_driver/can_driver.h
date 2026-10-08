/*
 * Driver de los dos buses CAN del core (FDCAN1 y FDCAN2).
 *
 * - CAN_PORT_1 (FDCAN1): bus común entre placas. IDs y codificación según
 *   el ICD de third_party/can_protocol.
 * - CAN_PORT_2 (FDCAN2): bus dedicado a la placa de sensores (sensor_board.h).
 *
 * Recepción: la ISR copia cada trama a un ring buffer por puerto y el bucle
 * principal las saca con can_driver_receive(). Nunca se procesa nada en la ISR.
 *
 * Transmisión: can_driver_send() encola en la TX FIFO del periférico. Solo
 * debe llamarse desde el bucle principal (no es reentrante con otra llamada
 * al mismo puerto).
 *
 * Uso:
 *   MX_FDCAN1_Init();
 *   MX_FDCAN2_Init();
 *   can_driver_init();
 *
 *   can_rx_frame_t frame;
 *   while (can_driver_receive(CAN_PORT_1, &frame)) { ... }
 *
 *   can_driver_send(CAN_PORT_1, id, data, len);
 */

#ifndef CAN_DRIVER_H
#define CAN_DRIVER_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  CAN_PORT_1 = 0, /* FDCAN1: bus común entre placas */
  CAN_PORT_2,     /* FDCAN2: placa de sensores */
  CAN_PORT_COUNT
} can_port_t;

typedef struct {
  uint32_t id;
  uint32_t len;        /* bytes de datos según el DLC (incluye relleno) */
  uint32_t rx_tick_ms; /* HAL_GetTick() al recibir la trama */
  /*
   * 64 bytes aunque la RX FIFO sea de 32: HAL copia tantos bytes como indique
   * el DLC, así que una trama de 48/64 bytes desbordaría un buffer de 32.
   * En ese caso solo los primeros 32 bytes son válidos.
   */
  uint8_t data[64];
} can_rx_frame_t;

typedef struct {
  uint32_t rx_frames;     /* tramas guardadas en el ring buffer */
  uint32_t rx_overflows;  /* ring buffer lleno: trama descartada */
  uint32_t fifo_msg_lost; /* FIFO HW desbordada antes de atenderla */
  uint32_t tx_errors;     /* longitud no válida o cola TX llena */
  uint32_t bus_off;       /* veces que se ha sacado de bus-off */
} can_driver_stats_t;

/*
 * Saca los transceptores de standby, configura filtros e interrupciones y
 * arranca ambos periféricos. Llamar después de MX_FDCAN1_Init() y
 * MX_FDCAN2_Init(). Si algo falla llama a Error_Handler().
 */
void can_driver_init(void);

/*
 * Encola una trama de datos con ID estándar en formato CAN FD sin BRS.
 * len: bytes útiles. Si no es una longitud CAN FD válida se rellena con ceros
 * hasta la siguiente (p. ej. 10 -> 12), como pide el ICD.
 * Máximo: 32 bytes en CAN_PORT_1, 8 en CAN_PORT_2 (tamaño de su TX FIFO).
 * Devuelve false si len supera el máximo o la cola TX está llena.
 */
bool can_driver_send(can_port_t port, uint32_t id, const uint8_t *data,
                     uint32_t len);

/* Saca la trama más antigua del puerto. Devuelve false si no hay ninguna. */
bool can_driver_receive(can_port_t port, can_rx_frame_t *frame);

/* Copia los contadores de diagnóstico del puerto. */
void can_driver_get_stats(can_port_t port, can_driver_stats_t *stats);

/*
 * BUS-OFF. Cada nodo CAN cuenta sus errores al transmitir (+8 por trama
 * fallida, -1 por trama buena). Si pasa de 255 (unas 32 tramas fallidas
 * seguidas, que con los reintentos automáticos son pocos milisegundos: un
 * mal contacto, CANH y CANL en corto, ruido), el nodo se desconecta solo del
 * bus. El FDCAN del STM32 se queda entonces en INIT y NO VUELVE POR SU
 * CUENTA: hay que quitar CCCR.INIT, y entonces espera 128 ratos de bus libre
 * (~1,4 ms a 1 Mbit/s) y se reincorpora.
 *
 * Esto lo hace en los dos puertos y cuenta cada caída (stats.bus_off). Llamar
 * en cada vuelta del bucle principal. Es barato: dos lecturas de registro.
 */
void can_driver_poll_bus_off(void);

/*
 * PRUEBA, a 1 solo en el banco: a los 40 s del arranque tira el CAN1 a
 * bus-off y a los 50 s el CAN2 (flight_software.c), para ver que
 * can_driver_poll_bus_off() los recupera y todo sigue: la telemetria, las
 * placas del CAN1 y los sensores del GNC. Lo hace sin tocar cables: pone el
 * transceptor en standby (STB) mientras transmite, asi que el FDCAN manda y
 * no se oye, cuenta un error de bit por intento y en ~32 llega a bus-off,
 * lo mismo que con CANH y CANL en corto.
 */
#ifndef CAN_DRIVER_PRUEBA_BUS_OFF
#define CAN_DRIVER_PRUEBA_BUS_OFF 0
#endif
#if CAN_DRIVER_PRUEBA_BUS_OFF
/* Devuelve los ms que tardo en llegar a bus-off, o -1 si no llego */
int32_t can_driver_prueba_bus_off(can_port_t port);
#endif

#endif /* CAN_DRIVER_H */
