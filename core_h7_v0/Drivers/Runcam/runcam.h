/*
 * Cámaras RunCam (Split 4) por UART: el "RunCam Device Protocol", el que
 * usan Betaflight (src/main/io/rcdevice.c) y ArduPilot (AP_RunCam.cpp).
 *
 * 115200 8N1. Cada paquete es
 *     0xCC | comando | parámetros | CRC-8 DVB-S2
 * con el CRC (polinomio 0xD5, empieza en 0) de todo lo anterior. Comprobado
 * con el ejemplo publicado: empezar a grabar = CC 01 03 98.
 *
 * Comandos:
 *   0x00 GET_DEVICE_INFO, sin parámetros. Contesta 5 bytes:
 *        CC | versión del protocolo | funciones (uint16 LE) | CRC
 *   0x01 CAMERA_CONTROL + operación (RUNCAM_OP_*). NO contesta.
 *   0x10 GET_SETTINGS + carpeta + trozo: los ajustes que cuelgan de una
 *        carpeta del árbol (la raíz es la 0). Contesta
 *        CC | trozos que faltan | largo | [id | nombre\0 | valor\0]... | CRC
 *   0x11 READ_SETTING_DETAIL + ajuste + trozo: tipo, límites, opciones.
 *        Contesta con el mismo marco (CC | faltan | largo | datos | CRC).
 *   0x13 WRITE_SETTING + ajuste + valor. La respuesta según Betaflight es
 *        CC | resultado (0 bien) | hay que releer el menú | CRC; por si acaso
 *        se acepta también el marco de los dos anteriores. Lo seguro es
 *        volver a leer el ajuste.
 *   Los de ajustes solo si la cámara tiene RUNCAM_F_AJUSTES.
 *
 * Mandar: el paquete entero va a un anillo y la interrupción del UART lo
 * vacía en la FIFO (escribir la hora son 22 bytes, más que la FIFO).
 * Recibir: la interrupción copia cada byte a otro anillo y runcam_update()
 * lo interpreta en el bucle (las respuestas de ajustes llegan a 64 bytes).
 * Nunca se espera.
 */
#ifndef RUNCAM_H
#define RUNCAM_H

#include <stdbool.h>
#include <stdint.h>

#include "main.h"

/* Operaciones de CAMERA_CONTROL */
#define RUNCAM_OP_BOTON_WIFI 0x00U
#define RUNCAM_OP_BOTON_ENCENDIDO 0x01U /* en la Split 4: grabar / parar */
#define RUNCAM_OP_CAMBIAR_MODO 0x02U
#define RUNCAM_OP_EMPEZAR_GRABAR 0x03U  /* la Split 4 lo anuncia y no hace nada */
#define RUNCAM_OP_PARAR_GRABAR 0x04U

/* Funciones que dice tener la cámara (GET_DEVICE_INFO) */
#define RUNCAM_F_BOTON_ENCENDIDO (1U << 0)
#define RUNCAM_F_BOTON_WIFI (1U << 1)
#define RUNCAM_F_CAMBIAR_MODO (1U << 2)
#define RUNCAM_F_OSD_5_TECLAS (1U << 3)
#define RUNCAM_F_AJUSTES (1U << 4)     /* GET_SETTINGS y compañía */
#define RUNCAM_F_PANTALLA (1U << 5)    /* escribir en su OSD */
#define RUNCAM_F_EMPEZAR_GRABAR (1U << 6)
#define RUNCAM_F_PARAR_GRABAR (1U << 7)
#define RUNCAM_F_MENU (1U << 8)
#define RUNCAM_F_ACTITUD (1U << 9)

/* Comandos con respuesta */
#define RUNCAM_CMD_INFO 0x00U
#define RUNCAM_CMD_AJUSTES 0x10U
#define RUNCAM_CMD_DETALLE 0x11U
#define RUNCAM_CMD_ESCRIBIR 0x13U

#define RUNCAM_RESP_MAX 72U
#define RUNCAM_ANILLO 256U
#define RUNCAM_TX 64U

/* Lo que se espera una respuesta antes de darla por perdida */
#define RUNCAM_PLAZO_MS 1000U

typedef struct {
  UART_HandleTypeDef *huart;
  bool iniciado;

  /* Recepción: lo llena la interrupción, lo vacía runcam_update() */
  volatile uint8_t anillo[RUNCAM_ANILLO];
  volatile uint32_t cab;
  uint32_t cola;
  volatile uint32_t errores_uart; /* desbordes, ruido, trama */

  /* Envío: lo llena _enviar(), lo vacía la interrupción */
  volatile uint8_t tx[RUNCAM_TX];
  volatile uint32_t tx_cab;
  volatile uint32_t tx_cola;

  /* La respuesta que se espera (de qué comando) y la que se va formando */
  uint8_t esperando; /* 0xFF = nada */
  uint32_t pedido_ms; /* HAL_GetTick() de la última petición */
  uint8_t resp[RUNCAM_RESP_MAX];
  uint32_t resp_n;
  bool resp_lista;   /* respuesta completa y con el CRC bien */

  uint8_t ultimos[8]; /* los últimos bytes que han llegado, para depurar */
  uint8_t n_ultimos;

  /* Lo que ha dicho la cámara */
  bool info_ok;
  uint8_t version;    /* 0 = RCSplit antiguo, 1 = protocolo 1.0 */
  uint16_t funciones; /* RUNCAM_F_* */
  uint32_t info_ms;   /* HAL_GetTick() de la última respuesta de INFO */

  /* Contadores */
  uint32_t enviados;
  uint32_t no_cabia;  /* comandos que no cupieron en el anillo de salida */
  uint32_t bytes_rx;
  uint32_t respuestas;
  uint32_t errores_crc;
  uint32_t ignorados; /* bytes que llegaron sin esperar nada */
} runcam_t;

/* uart ya inicializado (MX_UARTx_Init). Activa la FIFO y la interrupción de
 * recepción (el IRQHandler del UART tiene que llamar a runcam_irq). */
bool runcam_init(runcam_t *c, UART_HandleTypeDef *huart, IRQn_Type irq);
void runcam_irq(runcam_t *c);

/* La cámara se ha quedado sin corriente: olvidar lo que se esperaba y lo que
 * dijo (al volver hay que preguntarle otra vez) */
void runcam_olvidar(runcam_t *c);

/* Piden algo a la cámara sin esperar. false si no cupo. */
bool runcam_pedir_info(runcam_t *c);
bool runcam_control(runcam_t *c, uint8_t operacion);
bool runcam_pedir_ajustes(runcam_t *c, uint8_t carpeta, uint8_t trozo);
bool runcam_pedir_detalle(runcam_t *c, uint8_t ajuste, uint8_t trozo);
/* n bytes de valor (un texto, con su \0 al final) */
bool runcam_escribir(runcam_t *c, uint8_t ajuste, const uint8_t *valor,
                     uint32_t n);

/* En cada vuelta del bucle: interpreta lo que haya llegado */
void runcam_update(runcam_t *c);

/* Hay una petición esperando respuesta (menos de RUNCAM_PLAZO_MS) o una
 * respuesta sin recoger: no mandar otra, que la pisaría */
bool runcam_ocupada(const runcam_t *c);

/* Recoge la respuesta de AJUSTES, DETALLE o ESCRIBIR terminada (CRC bien):
 * sus bytes, desde la cabecera hasta el CRC. NULL si no hay. Vale hasta la
 * siguiente petición. */
const uint8_t *runcam_respuesta(runcam_t *c, uint32_t *n);

#endif /* RUNCAM_H */
