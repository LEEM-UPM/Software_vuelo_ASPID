/*
 * REINICIO EN VUELO: la core recuerda en qué punto del vuelo estaba (memoria
 * de vuelo en la flash, memoria_vuelo.h) y, si se reinicia en el aire, sigue
 * el vuelo SOLO CON LOS BARÓMETROS (baro_alt.h). El GNC no se puede volver a
 * alinear en vuelo: para eso necesita el cohete quieto.
 *
 * QUÉ SE GUARDA (emergencia_vigilar, en cada vuelta, también en un vuelo
 * normal): al despegar, la presión del suelo de cada barómetro (de ~2 s
 * antes) y el número de vuelo; luego cada cambio de estado o fase, el apogeo,
 * el drogue y el main mandados, el final del vuelo, y la altura máxima de
 * cada barómetro (cada 10 m, como mucho una vez por segundo). NUNCA durante
 * una simulación.
 *
 * AL ARRANCAR, ANTES DE LA MÚSICA Y LAS LUCES (emergencia_arranque y
 * emergencia_confirmar):
 *   - sin vuelo guardado (o de una simulación)  -> INICIO: arranque normal.
 *   - vuelo TERMINADO -> POSTVUELO: no toca la SD (queda como se selló), las
 *     cámaras no se encienden solas y no se manda ningún pirotécnico de
 *     vuelo. Si el reset es de ENCENDIDO (apagar y encender a propósito), se
 *     olvida y arranca normal: es otro día.
 *   - vuelo SIN TERMINAR -> se mira el aire durante como mucho
 *     EMERGENCIA_COMPROBAR_MS: barómetro fiable a más de EMERGENCIA_AIRE_M,
 *     barómetro en bruto a más de EMERGENCIA_AIRE_BRUTO_M (aunque haga
 *     ruido) o aceleración lejos de 1 g -> EN VUELO, modo de emergencia.
 *     Quieto en el suelo -> POSTVUELO (o INICIO si fue un encendido). Si no
 *     hay datos para decidir, EN VUELO: no dispara nada sin barómetro fiable.
 * Fuera de INICIO no hay melodía ni secuencia de LEDs, no se espera 1 s a las
 * otras placas (salvo tras un encendido, que también arrancan ellas), ni se
 * monta la SD, ni se inicia el GNC: decide en menos de un segundo.
 *
 * EL MODO DE EMERGENCIA (emergencia_update, en el sitio del GNC):
 *   - Rellena un gnc_data_t como el del GNC (gnc_get_data() devuelve este):
 *     EN VUELO, la fase, la altura y la velocidad del barómetro, nav_valid =
 *     barómetro fiable, y las banderas de drogue y main. Así despliegue.c,
 *     airbrake.c, camaras.c y uplink.c funcionan sin cambios.
 *   - SUBIENDO: el aerofreno se extiende al máximo (como pide tierra), pero
 *     no con el motor encendido (aceleración axial > EMERGENCIA_MOTOR_G),
 *     igual que el GNC, que solo lo abre en CONTROL. Se recoge al apogeo.
 *   - APOGEO: un barómetro FIABLE ha bajado EMERGENCIA_APOGEO_BAJADA_M de SU
 *     altura máxima durante EMERGENCIA_CONFIRMAR_MS, sin que el otro (si es
 *     fiable) siga subiendo a más de EMERGENCIA_SUBIENDO_MPS. Cada uno
 *     respecto a sí mismo: si uno tiene un error de escala, no importa.
 *     -> drogue (1A y 1B).
 *   - MAIN: después del apogeo, la media de los barómetros fiables por
 *     debajo de EMERGENCIA_MAIN_M durante EMERGENCIA_CONFIRMAR_MS -> 2A.
 *   - Nada se dispara por debajo de EMERGENCIA_MIN_ALTURA_M ni sin
 *     barómetro fiable. Lo que la memoria dice que ya se mandó no se repite.
 *   - Si el reset fue ya pasado el apogeo (fase guardada APOGEO o DROGUE) y
 *     el drogue no consta como mandado, se manda en cuanto el barómetro es
 *     fiable; lo mismo con el main en fase MAIN.
 *   - EN TIERRA: por debajo de 30 m, quieto, 5 s.
 * Por radio, el paquete de vuelo lleva el estado EMERGENCIA (4) y el de
 * estado (versión 6) dice si se ha detectado el apogeo, si se ha llegado a
 * la altura del main y si el barómetro es fiable.
 */
#ifndef EMERGENCIA_H
#define EMERGENCIA_H

#include <stdbool.h>
#include <stdint.h>

#include "gnc.h"
#include "memoria_vuelo.h"
#include "sys_utilities.h"

#define EMERGENCIA_APOGEO_BAJADA_M 10.0f
#define EMERGENCIA_MAIN_M 400.0f
#define EMERGENCIA_CONFIRMAR_MS 100U
#define EMERGENCIA_SUBIENDO_MPS 10.0f
#define EMERGENCIA_MIN_ALTURA_M 20.0f
#define EMERGENCIA_TIERRA_M 30.0f
#define EMERGENCIA_TIERRA_VZ_MPS 5.0f /* el ruido en reposo da hasta 4 */
#define EMERGENCIA_TIERRA_MS 5000U
#define EMERGENCIA_BANDERA_MS 1000U /* banderas de drogue y main, como el GNC */
#define EMERGENCIA_MOTOR_G 1.5f
#define EMERGENCIA_MOTOR_APAGADO_MS 300U
#define EMERGENCIA_FRENO_PASO 0.0615f /* por paso de 20 ms, como el GNC */

/* Decidir al arrancar */
#define EMERGENCIA_COMPROBAR_MS 1500U
#define EMERGENCIA_AIRE_M 20.0f
#define EMERGENCIA_AIRE_BRUTO_M 100.0f
#define EMERGENCIA_AIRE_BRUTO_MUESTRAS 10U
#define EMERGENCIA_ACEL_RARA_MS 200U
#define EMERGENCIA_QUIETO_MS 300U

/* Estado del cohete en el paquete de vuelo durante la emergencia */
#define EMERGENCIA_ESTADO_PAQUETE 4U

/* Pruebas en la mesa (NO VOLAR con ninguna):
 * 1: en un arranque normal, a los 15 s escribe un vuelo falso en CONTROL y
 *    se reinicia; el siguiente arranque entra en emergencia con un vuelo
 *    inventado en los barómetros (con ruido transónico al principio):
 *    apogeo a ~2250 m, main a 400 m, suelo. Por el camino: a los 5 s cuelga
 *    el bucle (el watchdog reinicia), a los 8 s un fallo de memoria (MPU) y
 *    bajando con el drogue un reset (no se repite el drogue). Los
 *    pirotécnicos NO se mandan (solo se dice por la consola). Al acabar,
 *    POSTVUELO, y luego deja la memoria limpia.
 * 2: ruido de los barómetros de verdad en reposo, 2 minutos.
 * 3: llena la memoria de vuelo y reinicia: el siguiente arranque tiene que
 *    borrarla quedándose con el último registro. */
#ifndef EMERGENCIA_PRUEBA
#define EMERGENCIA_PRUEBA 0
#endif

typedef enum {
  EMERGENCIA_INICIO = 0, /* arranque normal */
  EMERGENCIA_EN_VUELO,   /* reiniciado en el aire: modo de emergencia */
  EMERGENCIA_POSTVUELO,  /* reiniciado tras un vuelo: no tocar nada */
  EMERGENCIA_POR_VER,    /* vuelo sin terminar: falta mirar el aire */
} emergencia_modo_t;

/* Justo tras DebugOut_Init(): lo que dice la memoria. INICIO, POSTVUELO o
 * POR_VER. */
emergencia_modo_t emergencia_arranque(void);

/* sys_init rápido: las otras placas ya estaban despiertas */
bool emergencia_arranque_rapido(void);

/* Tras sys_init(): si estaba POR_VER, mira el aire (hasta
 * EMERGENCIA_COMPROBAR_MS) y decide. Devuelve el modo definitivo. */
emergencia_modo_t emergencia_confirmar(sys_state *state);

/* En cada vuelta, en el sitio del GNC, solo en EN_VUELO */
void emergencia_update(void);

/* En cada vuelta, en todos los modos: guarda lo que cambie del vuelo */
void emergencia_vigilar(void);

emergencia_modo_t emergencia_modo(void);
const char *emergencia_modo_nombre(emergencia_modo_t m);
const char *emergencia_motivo(void); /* por qué ese modo, para la consola */
bool emergencia_activa(void);        /* EN_VUELO */
bool emergencia_postvuelo(void);
const gnc_data_t *emergencia_gnc(void);

bool emergencia_apogeo_detectado(void); /* por el barómetro, en emergencia */
bool emergencia_main_alcanzado(void);   /* 400 m, en emergencia */
bool emergencia_drogue_hecho(void);     /* según la memoria */
bool emergencia_main_hecho(void);
float emergencia_freno(void);           /* lo que pide al aerofreno */
bool emergencia_motor(void);            /* aceleración de motor */
const memoria_vuelo_t *emergencia_memoria(void);

#if EMERGENCIA_PRUEBA
/* Desde baro_alt.c: cambia la presión de verdad por la del vuelo inventado */
void emergencia_prueba_presion(uint32_t k, float p0_pa, float *p_pa);
/* Desde despliegue.c: true = no mandar de verdad */
bool emergencia_prueba_sin_piros(void);
#endif

#endif /* EMERGENCIA_H */
