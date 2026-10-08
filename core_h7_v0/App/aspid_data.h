/*
 * Registro de cada sesión en la SD1: telemetría desde el arranque, datos de
 * vuelo a 100 Hz desde el despegue hasta los 50 m de bajada, y las IMU a
 * 400 Hz durante el motor (la curva de empuje).
 *
 * Cada arranque crea en la raíz de la SD1:
 *
 *   aspid_data_2026-10-05_14-30-12/     fecha y hora UTC del arranque
 *     sesion.txt                        configuración al arrancar, sesgos del
 *                                       EKF, eventos importantes y dónde cayó
 *     eventos/eventos.csv               TODOS los eventos, uno por línea:
 *                                       fases, despliegues, continuidad, modo
 *                                       test, placas, errores...
 *     salud/salud.csv                   1 Hz desde el arranque: bucle, GNC,
 *                                       sensores, buses, radio, power y SD
 *     telemetria/telemetria.csv         cada paquete que sale por radio (tipo
 *                                       y bytes) y cada uno que llega (bytes,
 *                                       cómo se oyó y qué se hizo con él)
 *     vuelo/vuelo_01.csv                100 Hz, despegue -> 50 m bajando
 *     trust_curve/trust_curve_01.csv    las dos IMU a 400 Hz, del medio
 *                                       segundo antes del despegue a 5 s
 *                                       después de apagarse el motor
 *     simulaciones/sim_01.csv           lo mismo durante un SIMULAR RUTA
 *     simulaciones/sim_01_trust_curve.csv
 *
 * LA FECHA. La core no tiene reloj en hora: la fecha sale del GPS (trama RMC
 * con fix). Hasta que llega, la carpeta se llama aspid_data_sin_fecha_NNNN
 * (NNNN = arranque de la SD1) y las filas solo llevan t_ms (ms desde el
 * arranque de la core). En cuanto el GPS da fecha la carpeta se renombra con
 * la hora UTC del arranque, y desde ahí las filas llevan también la hora UTC
 * (las del buffer de antes del despegue también). Solo se renombra en tierra.
 * La relación t_ms <-> UTC queda en sesion.txt.
 *
 * EL VUELO. Empieza al detectar el despegue: el GNC pasa a EN VUELO, o una
 * de las IMU mide más de 3 g durante 100 ms (el registro no puede depender
 * solo del GNC). Incluye el segundo de antes (medio en trust_curve), que se
 * guarda en buffers circulares, porque el GNC tarda 60 ms en confirmar el
 * despegue y sin él se perdería el encendido del motor. Termina al bajar de
 * 50 m después del apogeo, con el GNC EN TIERRA o a los 20 minutos. Si pasó
 * de 100 m se cierra TODO, telemetría incluida, y la SD no se vuelve a tocar
 * en este arranque. Si no (un golpe en la rampa), se cierra ese archivo y se
 * deja otro preparado.
 *
 * SIMULACIONES. Un SIMULAR RUTA pasa por las mismas fases, así que se graba
 * igual pero en simulaciones/, y nunca sella la SD. OJO: en esas filas las
 * columnas del GNC son de la simulación, pero las de los sensores (y toda la
 * trust curve) son las de la placa de verdad en la mesa.
 *
 * NADA BLOQUEA, NI CON LA TARJETA MUERTA. En tierra los archivos se crean y
 * se RESERVAN con FileX (vuelo 64 MB, trust curve 8 MB, telemetría por tramos
 * de 8 MB). Después los datos van a esos sectores directamente por DMA, sin
 * FileX y sin esperar (sd_card_write_start / sd_card_write_poll): el bucle
 * lanza un bloque de 4 KB y sigue, y mira si ha acabado en las vueltas
 * siguientes. Cada segundo baja también lo que quede a medias y actualiza el
 * tamaño de cada archivo escribiendo, igual por DMA, el sector de su carpeta:
 * un corte de corriente pierde como mucho el último segundo. Si la tarjeta
 * falla o no contesta en lo que da la norma, se da por perdida y no se vuelve
 * a tocar. Mientras tanto los datos esperan en RAM; si se llena, se pierden y
 * se cuentan. FileX (bloqueante) solo se usa en tierra o al terminar.
 *
 * Lo reservado que no se usa se libera al cerrar. Si se corta la corriente
 * antes (en la rampa, lo normal), la lista de lo reservado queda en
 * aspid_reservas.txt y el arranque siguiente lo libera.
 */
#ifndef ASPID_DATA_H
#define ASPID_DATA_H

#include <stdbool.h>
#include <stdint.h>

#include "sys_utilities.h"

/* A 1, una prueba sin GPS ni lanzamiento: fecha falsa a los 15 s, despegue
 * forzado a los 30 s, apogeo a los 45 s y fin con sellado. Para probar en la
 * mesa. DEJAR A 0 PARA VOLAR. */
#ifndef ASPID_DATA_PRUEBA
#define ASPID_DATA_PRUEBA 0
#endif

/* A 1, cada vez que se cierran los archivos de un vuelo o de una simulación
 * los relee de la tarjeta y saca por USB cuántas líneas y bytes tienen, la
 * cabecera y la primera y última fila (bloquea el bucle ~1 s). Para comprobar
 * en la mesa. DEJAR A 0 PARA VOLAR. */
#ifndef ASPID_DATA_RELEER
#define ASPID_DATA_RELEER 0
#endif

/* Con ASPID_DATA_PRUEBA: a los 38 s, en mitad del vuelo falso, desconecta
 * los pines de la SD1 como si se hubiera soltado la tarjeta. Para comprobar
 * que el bucle (y el GNC) siguen sin pararse. DEJAR A 0. */
#ifndef ASPID_DATA_PRUEBA_MUERTE
#define ASPID_DATA_PRUEBA_MUERTE 0
#endif

#define ASPID_DATA_HZ 100U                /* filas de vuelo por segundo */
#define ASPID_DATA_ANTES_MS 1000U         /* lo que se guarda de antes del despegue */
#define ASPID_DATA_FIN_ALTURA_M 50.0f     /* bajando, por debajo de esto se cierra */
#define ASPID_DATA_VUELO_REAL_M 100.0f    /* por encima, era un vuelo: se sella */
#define ASPID_DATA_DESPEGUE_G 3.0f        /* IMU en bruto, como el GNC */
#define ASPID_DATA_DESPEGUE_MS 100U       /* durante cuánto */
#define ASPID_DATA_MAX_VUELO_MS (20U * 60U * 1000U)
#define ASPID_DATA_SYNC_MS 1000U          /* tamaños al directorio */

/* Trust curve: una fila por tick del GNC (400 Hz) */
#define ASPID_DATA_EMPUJE_ANTES_MS 500U
#define ASPID_DATA_EMPUJE_DESPUES_MS 5000U /* tras salir de BOOST */
#define ASPID_DATA_EMPUJE_MAX_MS 30000U    /* por si el GNC no lo dice */

typedef enum {
  ASPID_TX_VUELO = 0,
  ASPID_TX_ESTADO,
  ASPID_TX_ACUSE,
  ASPID_TX_CAMARAS,
} aspid_tx_tipo_t;

typedef enum {
  ASPID_RX_EJECUTADO = 0,
  ASPID_RX_DENEGADO,
  ASPID_RX_DESCONOCIDO, /* bien formado, comando que no existe */
  ASPID_RX_MAL_FORMADO, /* ni marca ni XOR: no es un comando nuestro */
} aspid_rx_resultado_t;

typedef enum {
  ASPID_SIN_SD = 0,   /* la SD1 no está montada, falló o se dio por perdida */
  ASPID_ESPERANDO,    /* telemetría grabando, vuelo esperando el despegue */
  ASPID_GRABANDO,     /* grabando un vuelo o una simulación */
  ASPID_SELLADO,      /* hubo vuelo: todo cerrado, la SD ya no se toca */
} aspid_estado_t;

typedef struct {
  aspid_estado_t estado;
  char carpeta[48];
  char archivo[48];       /* el de vuelo que graba, o el último que grabó */
  bool hora_conocida;
  bool grabando_simulacion;
  bool empuje_grabando;   /* la trust curve está grabando ahora */
  bool tarjeta_perdida;   /* falló una escritura por DMA: no se toca más */
  uint32_t vuelos;        /* archivos de vuelo usados en esta sesión */
  uint32_t simulaciones;
  uint32_t filas;         /* filas del archivo de vuelo/simulación actual */
  uint32_t filas_total;
  uint32_t filas_empuje;  /* filas de la trust curve actual */
  uint32_t filas_perdidas; /* muestras que no cupieron en los buffers */
  uint32_t lineas_telemetria;
  uint32_t eventos;        /* líneas de eventos.csv */
  uint32_t filas_salud;
  uint32_t bytes_perdidos; /* texto que no cupo en RAM esperando a la SD */
  uint32_t bytes_vuelo;   /* ya escritos en la tarjeta */
  uint32_t bytes_empuje;
  uint32_t bytes_telemetria;
  uint32_t escrituras;     /* bloques por DMA */
  uint32_t escritura_max_us; /* la más lenta (no bloquea: es lo que tardó) */
  uint32_t filex_max_us;     /* la operación de FileX (bloqueante) más larga */
  uint32_t formato_max_us;   /* formatear una fila de vuelo */
  uint32_t errores;          /* operaciones fallidas */
  uint32_t ultimo_error;     /* FX_* o HAL_SD_ERROR_* */
  uint32_t reservas_liberadas; /* de la sesión anterior, al arrancar */
  uint32_t despegue_ms;      /* HAL_GetTick() del último despegue, 0 = no */
  float altura_max_m;
} aspid_data_info_t;

/* Tras storage_init(), lanzamiento_init() y gnc_init(). Crea la carpeta,
 * reserva los archivos y apunta la configuración (bloquea unas decenas de
 * ms). */
void aspid_data_init(const sys_state *state);

/* En cada vuelta del bucle, después del GNC, el despliegue y el aerofreno */
void aspid_data_update(const sys_state *state);

/* Lo llama rf_telemetry.c por cada paquete que pasa a la RF. ok = false si
 * no se pudo encolar en el CAN. */
void aspid_data_tx(aspid_tx_tipo_t tipo, const uint8_t *datos, uint8_t len,
                   bool ok);

/* Lo llama uplink.c por cada paquete que llega de tierra. motivo es el
 * uplink_denegado_t si se denegó. */
void aspid_data_rx(const uint8_t *datos, uint8_t len,
                   const sys_rf_rx_meta_t *meta,
                   aspid_rx_resultado_t resultado, uint8_t motivo);

const aspid_data_info_t *aspid_data_info(void);

#endif /* ASPID_DATA_H */
