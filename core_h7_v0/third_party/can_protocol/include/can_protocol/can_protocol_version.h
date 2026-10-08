/*
 * Versión del protocolo CAN1 compartido entre placas.
 *
 * MAJOR: sube cuando un cambio rompe la compatibilidad. Una placa con la
 *        versión anterior interpretaría mal los datos. Ejemplos: mover o
 *        quitar un campo, cambiar un tipo o una unidad, reutilizar un ID.
 *        Al subir MAJOR, MINOR vuelve a 0.
 * MINOR: sube con cambios compatibles, que una placa antigua puede ignorar.
 *        Ejemplo: añadir un mensaje nuevo o usar un byte que antes era relleno.
 *
 * Todo cambio de versión va en el mismo PR que el cambio del protocolo y se
 * anota en el historial del ICD.md.
 */

#ifndef CAN_PROTOCOL_VERSION_H
#define CAN_PROTOCOL_VERSION_H

#include <stdint.h>

#define CAN_PROTOCOL_VERSION_MAJOR 0U  /* cambios incompatibles */
#define CAN_PROTOCOL_VERSION_MINOR 2U  /* cambios compatibles (mensaje nuevo...) */

#define CAN_PROTOCOL_VERSION \
  ((uint16_t)((CAN_PROTOCOL_VERSION_MAJOR << 8) | CAN_PROTOCOL_VERSION_MINOR))

#define CAN_PROTOCOL_VERSION_MAJOR_OF(v) ((uint8_t)((v) >> 8))

#define CAN_PROTOCOL_IS_COMPATIBLE(v) \
  (CAN_PROTOCOL_VERSION_MAJOR_OF(v) == CAN_PROTOCOL_VERSION_MAJOR)

#endif