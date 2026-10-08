/*
 * Una cámara RunCam (Split 4) por UART (Drivers/Runcam): saber si graba,
 * hacer que grabe o pare, leer con qué graba y ponerle la hora. Hay dos
 * iguales (camaras.c), con la misma alimentación (PA2) y cada una en su UART.
 *
 * CÓMO SE SABE SI GRABA. El protocolo no tiene comando de estado, pero la
 * cámara dice el espacio libre de su tarjeta (ajuste SDCARD_CAPACITY,
 * "libre/total" en MB) y lo actualiza en directo: grabando 1080p60 baja
 * ~5,7 MB/s (46 Mbit/s). Se leen todos sus ajustes (GET_SETTINGS) cada
 * CAMARA_LECTURA_MS y:
 *   GRABANDO  si el espacio libre ha bajado en los últimos CAMARA_GRABA_MS
 *   PARADA    si lleva CAMARA_GRABA_MS sin bajar
 * Con la microSD vieja (4 GB) no bajaba nunca: no grababa y nada más lo
 * decía. Medido el 2026-10-06.
 *
 * CÓMO SE LE HACE GRABAR O PARAR. Graba sola al darle corriente. Por UART
 * solo funciona el botón de encendido (RUNCAM_OP_BOTON_ENCENDIDO), que es un
 * interruptor: parada la pone a grabar (~2 s) y grabando la para (~1 s).
 * EMPEZAR A GRABAR lo anuncia pero no hace nada. Un interruptor a ciegas
 * puede parar justo lo que se quería grabar, así que SOLO SE PULSA CUANDO EL
 * ESPACIO LIBRE HA CONFIRMADO EL ESTADO CONTRARIO al que se quiere:
 *   quiere grabar: si a los CAMARA_ESPERA_GRABAR_MS de darle corriente está
 *     PARADA, se pulsa UNA vez. Si a los CAMARA_EFECTO_MS sigue parada, se
 *     rinde y avisa: NO GRABA (tarjeta llena, rota o demasiado lenta).
 *   quiere parar: si está GRABANDO, se pulsa; si a los CAMARA_EFECTO_MS
 *     sigue grabando, otra vez, hasta CAMARA_PULSOS_PARAR.
 *
 * LA HORA. Su reloj se pierde sin corriente (vuelve a 2021-09-07). Si
 * camara_update() recibe la hora ("20261006T143000.0") se escribe en su
 * ajuste CAMERA_TIME (WRITE_SETTING) y se comprueba en la siguiente lectura;
 * hasta CAMARA_RELOJ_INTENTOS veces. OJO: la Split 4 le SUMA SU HUSO a lo que
 * se escribe (medido 2026-10-06: escrito 21:07:12, leído 05:07:15 del día
 * siguiente, +8 h, la hora de China). En vez de suponerlo, se aprende: si
 * tras escribir se lee la hora desplazada un número entero de cuartos de
 * hora (hasta 14 h), ese es su huso, se guarda en desfase_s y se vuelve a
 * escribir restándolo. El primer vídeo ya ha empezado con la hora mala
 * (graba desde el arranque); su fecha de modificación y los siguientes
 * salen bien.
 *
 * Todo sin esperar: una petición cada vez y su respuesta en otra vuelta.
 */
#ifndef CAMARA_H
#define CAMARA_H

#include <stdbool.h>
#include <stdint.h>

#include "runcam.h"

#define CAMARA_ESPERA_MS 3000U        /* arranque: no preguntar antes */
#define CAMARA_REINTENTO_MS 1000U     /* sin contestar, preguntar cada... */
#define CAMARA_SIN_RESPUESTA_MS 6000U /* con corriente y sin contestar: NO
                                         CONTESTA (desconectada o rota) */
#define CAMARA_PERDIDA_MS 5000U       /* contestaba y deja de hacerlo */
#define CAMARA_LECTURA_MS 1000U       /* leer los ajustes cada... */
#define CAMARA_LATIDO_MS 5000U        /* si no tiene ajustes: GET_DEVICE_INFO */
#define CAMARA_GRABA_MS 3000U
#define CAMARA_ESPERA_GRABAR_MS 10000U
#define CAMARA_EFECTO_MS 6000U
#define CAMARA_PULSOS_PARAR 2U
#define CAMARA_RELOJ_INTENTOS 3U
#define CAMARA_RELOJ_REINTENTO_MS 3000U
#define CAMARA_RELOJ_TOLERANCIA_S 5

/* El estado, tal cual va a tierra (camaras.h) */
typedef enum {
  CAMARA_SIN_CORRIENTE = 0,
  CAMARA_ARRANCANDO,  /* con corriente, todavía sin contestar */
  CAMARA_NO_CONTESTA, /* CAMARA_SIN_RESPUESTA_MS con corriente y nada */
  CAMARA_COMPROBANDO, /* contesta; aún no se sabe si graba */
  CAMARA_GRABANDO,
  CAMARA_PARADA,
  CAMARA_PARANDO,     /* pulsado para parar, todavía grabando */
} camara_estado_t;

#define CAMARA_PAQUETE_BYTES 14U

typedef struct {
  runcam_t rc;
  uint8_t num;          /* 1 o 2, para los mensajes */

  bool corriente;
  uint32_t t_corriente; /* HAL_GetTick() al darle corriente */
  bool contesta;
  uint32_t t_respuesta; /* la última respuesta buena */

  uint8_t op;           /* qué se le ha pedido (camara.c) */
  uint32_t t_pedido;
  uint32_t t_lectura;   /* la última lectura de ajustes empezada */
  uint8_t trozo;
  uint8_t datos[256];   /* los trozos de GET_SETTINGS, juntos */
  uint32_t n_datos;

  /* Lo que dice: se queda lo último aunque se apague (para la consola) */
  bool sd_leida;        /* SDCARD_CAPACITY leído desde que tiene corriente */
  uint32_t libre_mb;
  uint32_t total_mb;    /* 0 = sin tarjeta */
  uint32_t queda_s;     /* REMAINING_TIME */
  bool ajustes_leidos;  /* resolución y TV leídas desde que tiene corriente */
  char resolucion[16];  /* "1080P60FPS" */
  uint16_t lineas;
  uint8_t fps;
  bool pal;
  char hora_camara[24]; /* CAMERA_TIME */
  uint8_t id_hora;      /* el ajuste CAMERA_TIME; 0xFF sin leer */

  /* Grabación, por el espacio libre */
  uint32_t lecturas;    /* desde que tiene corriente */
  uint32_t t_primera;
  uint32_t libre_antes;
  bool bajado;          /* ha bajado alguna vez desde que tiene corriente */
  uint32_t t_bajada;    /* la última vez que bajó */
  uint32_t t_empezo;    /* cuando se vio empezar a bajar */
  camara_estado_t estado;

  /* Lo que se quiere y los pulsos del botón */
  bool quiere_grabar;
  uint8_t pulsos;       /* por lo que se quiere ahora */
  uint32_t t_pulso;
  bool no_graba;        /* quería grabar, pulsó y sigue parada */
  uint32_t pulsaciones; /* desde el arranque */

  /* Reloj */
  bool reloj_ok;
  bool reloj_por_comprobar; /* escrita y aún sin releer */
  uint8_t reloj_intentos;
  uint32_t t_reloj;
  int64_t reloj_objetivo;   /* la hora que se quiere, en s desde 1970 */
  int32_t desfase_s;        /* su huso: lo que suma a lo que se escribe */
  char reloj_escrito[20];
  uint32_t escrituras_rechazadas;

  /* Contadores */
  uint32_t lecturas_total;
  uint32_t sin_respuesta; /* peticiones que caducaron */
  uint32_t perdidas;      /* veces que dejó de contestar */
} camara_t;

/* uart ya inicializado. El IRQHandler del UART llama a camara_irq(). */
void camara_init(camara_t *c, uint8_t num, UART_HandleTypeDef *huart,
                 IRQn_Type irq);
void camara_irq(camara_t *c);

/* La alimentación (PA2) se ha encendido o apagado. Al encender, quiere
 * grabar (lo hace sola). */
void camara_corriente(camara_t *c, bool on);

/* Grabar o parar (ver arriba cómo) */
void camara_quiere(camara_t *c, bool grabar);

/* En cada vuelta. hora: la hora que ponerle ("20261006T143000.0"), o NULL
 * si no se sabe. */
void camara_update(camara_t *c, uint32_t ahora, const char *hora);

camara_estado_t camara_estado(const camara_t *c);
const char *camara_estado_nombre(camara_estado_t e);
uint32_t camara_segundos_grabando(const camara_t *c, uint32_t ahora);

/* Se le puede cortar la corriente sin estropear un vídeo: no está grabando
 * (o no contesta y no se puede hacer nada) */
bool camara_lista_para_cortar(const camara_t *c);

/* Sus CAMARA_PAQUETE_BYTES del paquete de cámaras (camaras.h) */
void camara_paquete(const camara_t *c, uint32_t ahora, uint8_t *out);

#endif /* CAMARA_H */
