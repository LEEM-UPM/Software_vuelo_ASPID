/*
 * Nodos e identificadores del bus CAN1 (ver ICD.md, secciones 4 y 5).
 *
 * Ninguna placa escribe IDs a mano: todos se construyen con CAN_ID().
 *
 * Estructura del ID estándar de 11 bits:
 *
 *    bit  10  9  8 | 7  6  5  4 | 3  2  1  0
 *        [ clase  ] [   nodo   ] [ mensaje  ]
 *         3 bits      4 bits       4 bits
 *
 * - clase:   prioridad. En CAN gana el ID más bajo, así que la clase manda
 *            sobre todo lo demás en el arbitraje.
 * - nodo:    placa ORIGEN del mensaje, salvo en los comandos, donde es la
 *            placa DESTINO (el origen de los comandos es siempre el core).
 * - mensaje: número de mensaje dentro de esa clase y ese nodo (0–15).
 *
 * Con esta estructura cada ID tiene un único transmisor: el nodo del campo
 * "nodo", o el core en los comandos.
 */

#ifndef CAN_IDS_H
#define CAN_IDS_H

/* ------------------------------------------------------------------------
 * Nodos (4 bits)
 * ------------------------------------------------------------------------ */
#define CAN_NODE_BROADCAST 0x0U /* solo como destino de comandos: todos */
#define CAN_NODE_CORE 0x1U      /* core H7: ordenador de vuelo */
#define CAN_NODE_POWER 0x2U     /* placa de potencia */
#define CAN_NODE_AVIONICS 0x3U  /* placa de aviónica */
#define CAN_NODE_RF 0x4U        /* placa de radiofrecuencia */
/* 0x5–0xF libres */

/* ------------------------------------------------------------------------
 * Clases (3 bits), de más a menos prioridad
 * ------------------------------------------------------------------------ */
#define CAN_CLASS_CRITICAL 0x0U  /* emergencias y abort */
#define CAN_CLASS_COMMAND 0x1U   /* core -> nodo (nodo = destino) */
#define CAN_CLASS_RESPONSE 0x2U  /* nodo -> core, respuesta a un comando */
#define CAN_CLASS_TELEMETRY 0x3U /* datos periódicos de cada nodo */
/* 0x4–0x5 reservadas */
#define CAN_CLASS_HEARTBEAT 0x6U /* estado y versión de cada nodo */
#define CAN_CLASS_DEBUG 0x7U     /* diagnóstico, no necesario en vuelo */

/* ------------------------------------------------------------------------
 * Construcción y lectura de IDs
 * ------------------------------------------------------------------------ */
#define CAN_ID_CLASS_SHIFT 8U
#define CAN_ID_NODE_SHIFT 4U

#define CAN_ID_CLASS_MASK 0x700U
#define CAN_ID_NODE_MASK 0x0F0U
#define CAN_ID_MSG_MASK 0x00FU

#define CAN_ID(cls, node, msg)                                                 \
  ((((cls) & 0x7U) << CAN_ID_CLASS_SHIFT) |                                    \
   (((node) & 0xFU) << CAN_ID_NODE_SHIFT) | ((msg) & 0xFU))

#define CAN_ID_GET_CLASS(id) (((id) & CAN_ID_CLASS_MASK) >> CAN_ID_CLASS_SHIFT)
#define CAN_ID_GET_NODE(id) (((id) & CAN_ID_NODE_MASK) >> CAN_ID_NODE_SHIFT)
#define CAN_ID_GET_MSG(id) ((id) & CAN_ID_MSG_MASK)

/*
 * Máscaras para filtros por hardware (filtro "ID + máscara"):
 * - Todos los mensajes de una clase:  ID = CAN_ID(cls, 0, 0), máscara = CAN_ID_CLASS_MASK
 * - Todo lo que envía un nodo:        ID = CAN_ID(0, node, 0), máscara = CAN_ID_NODE_MASK
 * - Una clase de un nodo concreto:    ID = CAN_ID(cls, node, 0),
 *                                     máscara = CAN_ID_CLASS_MASK | CAN_ID_NODE_MASK
 */

/* ------------------------------------------------------------------------
 * Mensajes
 * Se definen aquí según se añadan al ICD, por ejemplo:
 *   #define CAN_ID_POWER_HEARTBEAT CAN_ID(CAN_CLASS_HEARTBEAT, CAN_NODE_POWER, 0x0U)
 * ------------------------------------------------------------------------ */

/* 6.1 CMD_WAKE_UP: core -> todas las placas (0x100) */
#define CAN_ID_CMD_WAKE_UP CAN_ID(CAN_CLASS_COMMAND, CAN_NODE_BROADCAST, 0x0U)

/* 6.2 WAKE_UP_ACK: cada placa -> core (power 0x220, avionics 0x230, rf 0x240) */
#define CAN_MSG_WAKE_UP_ACK 0x0U
#define CAN_ID_WAKE_UP_ACK(node) CAN_ID(CAN_CLASS_RESPONSE, (node), CAN_MSG_WAKE_UP_ACK)

/* 6.3 CMD_PYRO_FIRE: core -> power (0x120), 1 byte: canal */
#define CAN_ID_CMD_PYRO_FIRE CAN_ID(CAN_CLASS_COMMAND, CAN_NODE_POWER, 0x0U)

/* 6.4 CMD_PYRO_CONTINUITY: core -> power (0x121), sin datos */
#define CAN_ID_CMD_PYRO_CONTINUITY CAN_ID(CAN_CLASS_COMMAND, CAN_NODE_POWER, 0x1U)

/* 6.5 PYRO_CONTINUITY: power -> core (0x221), 1 byte: bit por canal */
#define CAN_ID_PYRO_CONTINUITY CAN_ID(CAN_CLASS_RESPONSE, CAN_NODE_POWER, 0x1U)

/* 6.6 CMD_POWER_RAILS: core -> power (0x122), sin datos */
#define CAN_ID_CMD_POWER_RAILS CAN_ID(CAN_CLASS_COMMAND, CAN_NODE_POWER, 0x2U)

/* 6.7 POWER_RAILS: power -> core (0x222), 12 bytes: 3 corrientes [mA] +
 * 3 tensiones [mV], uint16; 0xFFFF = lectura no válida */
#define CAN_ID_POWER_RAILS CAN_ID(CAN_CLASS_RESPONSE, CAN_NODE_POWER, 0x2U)

/* 6.8 CMD_AVIONICS_CALIBRATE: core -> avionics (0x131), 4 bytes: float32
 * ángulo de inclinación del cohete [grados] */
#define CAN_ID_CMD_AVIONICS_CALIBRATE CAN_ID(CAN_CLASS_COMMAND, CAN_NODE_AVIONICS, 0x1U)

/* 6.9 AVIONICS_CALIBRATION: avionics -> core (0x231), 32 bytes: 12 x int16
 * (offsets de las 2 IMU) + 2 x float32 (presión base de los 2 barómetros,
 * kPa); 0x7FFF / NaN = no calibrado */
#define CAN_ID_AVIONICS_CALIBRATION CAN_ID(CAN_CLASS_RESPONSE, CAN_NODE_AVIONICS, 0x1U)

/* 6.10 CMD_RF_TRANSMIT: core -> rf (0x140), 1-32 bytes que la rf transmite
 * tal cual en un paquete LoRa */
#define CAN_ID_CMD_RF_TRANSMIT CAN_ID(CAN_CLASS_COMMAND, CAN_NODE_RF, 0x0U)

/* 6.12 RF_RECEIVED: rf -> core (0x241), 1-32 bytes recibidos por radio y
 * pasados tal cual, sin interpretar. Es el camino de subida tierra -> cohete */
#define CAN_ID_RF_RECEIVED CAN_ID(CAN_CLASS_RESPONSE, CAN_NODE_RF, 0x1U)

/* 6.11 INIT_STATUS: cada placa -> core (power 0x22F, avionics 0x23F,
 * rf 0x24F), 8 bytes: uint32 componentes que han arrancado + uint32
 * componentes que tiene esa placa */
#define CAN_MSG_INIT_STATUS 0xFU
#define CAN_ID_INIT_STATUS(node) CAN_ID(CAN_CLASS_RESPONSE, (node), CAN_MSG_INIT_STATUS)

#endif /* CAN_IDS_H */
