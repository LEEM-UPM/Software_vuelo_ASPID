/*
 * Las dos cámaras (RunCam Split 4, iguales): una alimentación para las dos,
 * PA2 (CAM_EN, en alto encendidas), y cada una por su UART (camara.h):
 *   cámara 1: UART8, PE1 = CAM1_TX, PE0 = CAM1_RX
 *   cámara 2: UART9, PD15 = CAM2_TX, PD14 = CAM2_RX
 * Las dos graban y paran en los mismos momentos. Una que no esté conectada
 * sale como NO CONTESTA y no estorba a la otra.
 *
 * Arrancan APAGADAS (gpio.c deja el pin a 0) y tierra lo sabe desde el primer
 * paquete de estado del arranque (byte 31, b3; status_packet.h). Encendidas,
 * el primer LED RGB se pone violeta (led_show.h).
 *
 * SE ENCIENDEN SOLAS AL DETECTAR EL DESPEGUE, igual que el registro de la SD:
 * el GNC pasa a EN VUELO, o una de las IMU mide más de 3 g durante 100 ms
 * (no depender solo del GNC). Nunca por una simulación. Graban solas al
 * darles corriente, pero tardan ~3 s en arrancar: si se quiere el despegue
 * en vídeo hay que encenderlas antes desde tierra.
 *
 * Y SE APAGAN SOLAS AL TERMINAR EL VUELO, con las mismas condiciones que
 * cierran el registro de la SD (aspid_data.h), pero calculadas aquí: si la
 * SD falla, las cámaras tienen que apagarse igual. Tras el despegue, lo
 * primero de:
 *   - bajar de 50 m después del apogeo (con la navegación válida),
 *   - el GNC en EN TIERRA,
 *   - 20 minutos desde el despegue (contados desde que el GNC lo confirma).
 * Si en ese "vuelo" no se pasó de 100 m no era un vuelo (un golpe en la
 * rampa): se apagan y se vuelven a armar para el despegue de verdad.
 *
 * APAGAR ES PARAR PRIMERO. Cortar la corriente a una cámara que graba puede
 * dejar el vídeo sin cerrar. Así que apagar (al terminar el vuelo o desde
 * tierra) es: pedir a cada cámara que pare (camara.h: el botón, solo si el
 * espacio libre confirma que graba), esperar a que las dos estén PARADAS (o
 * no contesten) y CAMARAS_CIERRE_MS más para que cierren el archivo, y
 * entonces cortar PA2. Pase lo que pase, se corta a los
 * CAMARAS_APAGADO_MAX_MS. Mientras tanto siguen encendidas y "apagando".
 *
 * Desde tierra, UPLINK_CMD_CAMARAS (uplink.h): encender, siempre (si estaban
 * apagándose, vuelven a grabar); apagar, solo en tierra. Contesta con un
 * acuse con el estado de verdad, y la estación reintenta hasta verlo.
 *
 * LA HORA. Con la hora del GPS (trama RMC de la aviónica) se pone el reloj
 * de cada cámara al encenderla, en UTC como el registro de la SD (o en
 * CAMARAS_HUSO_H).
 *
 * PAQUETE DE CÁMARAS (radio, 32 bytes, little endian). Cada
 * CAMARAS_PAQUETE_MS con ellas encendidas, en cada cambio (como mucho uno por
 * CAMARAS_PAQUETE_MIN_MS), al arrancar y cuando tierra pide el estado:
 *
 *   0     0xCA, marca (mide 32 como el de estado; se distinguen por la marca)
 *   1     versión (CAMARAS_PAQUETE_VERSION)
 *   2     contador, +1 por paquete
 *   3     b0 alimentación (PA2)   b1 apagando (parando para cortar)
 *         b2-3 quién las encendió o apagó (camaras_origen_t)
 *         b4 en vuelo (despegue visto)   b5 hay hora del GPS
 *         b6 el último apagado se cortó por plazo, sin confirmar paradas
 *   4-17  cámara 1 (camara_paquete)
 *  18-31  cámara 2
 * Cada cámara, 14 bytes:
 *   +0    estado (camara_estado_t)
 *   +1    b0 quiere grabar  b1 NO GRABA (pulsó y sigue parada)
 *         b2 reloj puesto   b3 SD leída en este encendido
 *         b4 PAL (si no, NTSC)   b5 resolución leída   b6 sin tarjeta
 *   +2-3  líneas de la resolución (1080)   +4 fps (60)
 *   +5-6  MB libres / 16   +7-8 MB totales / 16
 *   +9-10 segundos de grabación que caben (REMAINING_TIME)
 *   +11-12 segundos que lleva grabando
 *   +13   pulsaciones del botón desde el arranque (uint8 saturado)
 */
#ifndef CAMARAS_H
#define CAMARAS_H

#include <stdbool.h>
#include <stdint.h>

#include "camara.h"

#define CAMARAS_N 2U

#define CAMARAS_DESPEGUE_G 3.0f
#define CAMARAS_DESPEGUE_MS 100U

#define CAMARAS_CIERRE_MS 1500U       /* paradas: esto más antes de cortar */
#define CAMARAS_APAGADO_MAX_MS 12000U /* y se corta seguro a los... */

#define CAMARAS_HUSO_H 0 /* horas que sumar a la UTC del GPS para su reloj */

#define CAMARAS_PAQUETE_BYTES 32U
#define CAMARAS_PAQUETE_MARCA 0xCAU
#define CAMARAS_PAQUETE_VERSION 1U
#define CAMARAS_PAQUETE_MS 5000U
#define CAMARAS_PAQUETE_MIN_MS 1000U

/* A 1, prueba en la mesa: con la hora de la compilación como si fuera la del
 * GPS, a los 25 s del arranque las enciende, a los +40 s las apaga (parando
 * primero), a los +60 s otra vez encender y a los +100 s apagar. Cada cambio
 * sale por la consola USB. DEJAR A 0 PARA VOLAR. */
#ifndef CAMARAS_PRUEBA_UART
#define CAMARAS_PRUEBA_UART 0
#endif

typedef enum {
  CAMARAS_ORIGEN_NINGUNO = 0, /* apagadas desde el arranque */
  CAMARAS_ORIGEN_DESPEGUE,    /* encendidas al detectar el despegue */
  CAMARAS_ORIGEN_TIERRA,      /* comando de subida */
  CAMARAS_ORIGEN_FIN_VUELO,   /* apagadas al terminar el vuelo (50 m...) */
} camaras_origen_t;

/* Una vez, tras MX_GPIO_Init(): arranca los UART de las dos */
void camaras_init(void);

/* En cada vuelta del bucle, después del GNC: las cámaras, el despegue, el
 * fin del vuelo y el apagado */
void camaras_update(void);

/* Desde UART8_IRQHandler y UART9_IRQHandler (stm32h7xx_it.c) */
void camaras_uart8_irq(void);
void camaras_uart9_irq(void);

/* Desde tierra (uplink.c). Apagar empieza el apagado (ver arriba). */
void camaras_encender(camaras_origen_t origen);
void camaras_apagar(void);

bool camaras_encendidas(void);         /* PA2: también mientras se apagan */
bool camaras_apagando(void);
camaras_origen_t camaras_origen(void); /* quién las encendió o apagó */
uint32_t camaras_cambio_ms(void);      /* HAL_GetTick() del último cambio */
bool camaras_en_vuelo(void);           /* despegue visto y vuelo sin terminar */
bool camaras_hora_gps(void);           /* se sabe la hora para su reloj */
uint32_t camaras_cortes_por_plazo(void); /* apagados sin confirmar paradas */

const camara_t *camaras_camara(uint32_t i); /* 0 o 1 */

/* El paquete de cámaras (arriba). Lo llama rf_telemetry.c. */
void camaras_paquete(uint8_t out[CAMARAS_PAQUETE_BYTES]);

#endif /* CAMARAS_H */
