/*
 * Altura por barómetro, sin GNC: lo que usa el modo de emergencia
 * (emergencia.h) para decidir el apogeo y el main tras un reset en vuelo, y
 * lo que se guarda en la memoria de vuelo durante un vuelo normal (la
 * referencia del suelo y la altura máxima de cada barómetro).
 *
 * CADA BARÓMETRO POR SU CUENTA. Los dos WSEN-PADS no marcan lo mismo (en la
 * mesa, 93,78 frente a 95,36 kPa: 1,6 kPa, unos 140 m), así que cada uno
 * lleva su propia presión de referencia en el suelo y su propia altura sobre
 * ella: h = 44330,77 * (1 - (p / p0)^0,190263) (atmósfera estándar).
 *
 * CUÁNDO SE FÍA DE UNO. Cerca de Mach 1 la presión estática da saltos que no
 * son altura: la onda de choque pasando por la toma. Para descartarlos, cada
 * muestra nueva (~200 Hz) pasa por:
 *   - presión dentro de BARO_P_MIN..MAX_PA,
 *   - salto con la anterior menor que BARO_SALTO_MAX_M (1000 m/s a 200 Hz),
 *   - ruido: RMS de la SEGUNDA DIFERENCIA de la altura en las últimas
 *     BARO_VENTANA muestras. Una trayectoria de verdad casi no tiene (a 30 g
 *     son 7 mm entre muestras); el ruido normal del sensor, unos cm; los
 *     saltos transónicos, decenas o cientos de metros.
 * Si algo falla queda "sucio", y solo vuelve a ser FIABLE tras
 * BARO_LIMPIO_MS seguidos limpio. Las alturas de un barómetro no fiable no
 * cuentan para nada (ni para la máxima). Sin muestras nuevas en
 * BARO_VIEJO_MS, o con la ventana idéntica (congelado), tampoco es fiable.
 */
#ifndef BARO_ALT_H
#define BARO_ALT_H

#include <stdbool.h>
#include <stdint.h>

#define BARO_N 2U
#define BARO_VENTANA 40U        /* 0,2 s a 200 Hz */
#define BARO_RUIDO_MAX_M 2.0f   /* RMS de la segunda diferencia */
#define BARO_SALTO_MAX_M 5.0f
#define BARO_LIMPIO_MS 500U
#define BARO_VIEJO_MS 100U
#define BARO_P_MIN_PA 20000.0f  /* ~11 km */
#define BARO_P_MAX_PA 110000.0f

/* Referencia del suelo: medias de 1 s; la que se da es la de hace
 * BARO_SUELO_ATRAS bloques, para que no la estropee el propio despegue */
#define BARO_SUELO_BLOQUE_MS 1000U
#define BARO_SUELO_ATRAS 2U

typedef struct {
  float p0_pa;       /* referencia del suelo; 0 = sin referencia */
  float p_pa;        /* última presión */
  float h_m;         /* última altura sobre p0 */
  float vz_mps;      /* velocidad vertical en la ventana */
  float ruido_m;     /* RMS de la segunda diferencia en la ventana */
  float max_m;       /* la mayor altura FIABLE */
  bool fiable;
  bool limpio;       /* la última muestra pasó las pruebas */
  uint32_t t_ultima; /* HAL_GetTick() de la última muestra */
  uint32_t t_limpio; /* limpio sin interrupción desde aquí */
  uint32_t muestras;
  uint32_t sucias;   /* muestras que no pasaron las pruebas */
  /* internos */
  uint32_t visto;    /* count del sensor ya procesado */
  float h[BARO_VENTANA];
  uint32_t t[BARO_VENTANA];
  float d2[BARO_VENTANA];
  uint32_t n;        /* muestras en la ventana (hasta BARO_VENTANA) */
  uint32_t i;        /* posición de la siguiente */
  /* suelo */
  float suelo_suma;
  uint32_t suelo_n;
  uint32_t suelo_t0;
  float suelo_bloques[BARO_SUELO_ATRAS + 1U];
  uint32_t suelo_bloques_n;
} baro_canal_t;

/* Referencia de cada barómetro (Pa). Olvida las ventanas y las máximas que
 * no se den (max_m = NULL: 0). */
void baro_alt_referencia(const float p0_pa[BARO_N], const float max_m[BARO_N]);

/* En cada vuelta: procesa las muestras nuevas de sensor_board. Devuelve
 * cuántas ha procesado (0, 1 o 2). */
uint32_t baro_alt_update(uint32_t ahora);

/* En tierra: va guardando las medias de 1 s para la referencia del suelo */
void baro_alt_suelo_update(uint32_t ahora);
/* La referencia del suelo de cada uno, de hace ~2 s. false si aún no hay. */
bool baro_alt_suelo(float p0_pa[BARO_N]);

const baro_canal_t *baro_alt_canal(uint32_t i);
bool baro_alt_fiable(void);   /* alguno fiable */
float baro_alt_altura(void);  /* media de los fiables (la última si ninguno) */
float baro_alt_vz(void);      /* igual */

#endif /* BARO_ALT_H */
