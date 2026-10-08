#include "baro_alt.h"

#include <math.h>
#include <string.h>

#include "emergencia.h"
#include "sensor_board.h"

static baro_canal_t s_c[BARO_N];
static float s_altura;
static float s_vz;

static float _altura(float p_pa, float p0_pa) {
  return 44330.77f * (1.0f - powf(p_pa / p0_pa, 0.190263f));
}

void baro_alt_referencia(const float p0_pa[BARO_N], const float max_m[BARO_N]) {
  for (uint32_t k = 0U; k < BARO_N; k++) {
    baro_canal_t *c = &s_c[k];
    c->p0_pa = p0_pa[k];
    c->max_m = (max_m != NULL) ? max_m[k] : 0.0f;
    c->n = 0U;
    c->i = 0U;
    c->fiable = false;
    c->limpio = false;
  }
  /* Hasta que alguno sea fiable, la altura es la mejor que se sabe: la
   * máxima guardada (tras un reinicio en vuelo), no 0 */
  s_altura = 0.0f;
  s_vz = 0.0f;
  if (max_m != NULL) {
    s_altura = (max_m[0] > max_m[1]) ? max_m[0] : max_m[1];
  }
}

/* La presión de la muestra: la de verdad o, en la prueba de la mesa, la de
 * un vuelo inventado (emergencia.h) */
static bool _presion(uint32_t k, float *p_pa) {
  const baro_data_t *b = &sensor_board_get_data()->baro[k];
  if (b->count == s_c[k].visto) {
    return false;
  }
  s_c[k].visto = b->count;
  *p_pa = b->pressure_kpa * 1000.0f;
#if EMERGENCIA_PRUEBA
  emergencia_prueba_presion(k, s_c[k].p0_pa, p_pa);
#endif
  return true;
}

static void _muestra(baro_canal_t *c, float p, uint32_t ahora) {
  c->muestras++;
  c->p_pa = p;
  c->t_ultima = ahora;
  bool limpio = (p > BARO_P_MIN_PA) && (p < BARO_P_MAX_PA) && (c->p0_pa > 0.0f);
  const float h = limpio ? _altura(p, c->p0_pa) : c->h_m;
  if (!isfinite(h)) {
    limpio = false;
  }
  /* Ventana: altura, instante y segunda diferencia */
  const uint32_t a1 = (c->i + BARO_VENTANA - 1U) % BARO_VENTANA;
  const uint32_t a2 = (c->i + BARO_VENTANA - 2U) % BARO_VENTANA;
  float d2 = 0.0f;
  if (limpio && (c->n >= 1U) && (fabsf(h - c->h[a1]) > BARO_SALTO_MAX_M)) {
    limpio = false; /* salto imposible */
  }
  if (c->n >= 2U) {
    d2 = h - (2.0f * c->h[a1]) + c->h[a2];
  }
  c->h[c->i] = h;
  c->t[c->i] = ahora;
  c->d2[c->i] = d2;
  c->i = (c->i + 1U) % BARO_VENTANA;
  if (c->n < BARO_VENTANA) {
    c->n++;
  }
  c->h_m = h;

  /* Ruido en la ventana, y si está congelada (todo idéntico) */
  float s = 0.0f;
  bool cambia = false;
  const uint32_t nd = (c->n > 2U) ? (c->n - 2U) : 0U;
  for (uint32_t j = 0U; j < nd; j++) {
    const float v = c->d2[(c->i + BARO_VENTANA - 1U - j) % BARO_VENTANA];
    s += v * v;
    cambia = cambia || (v != 0.0f);
  }
  c->ruido_m = (nd > 0U) ? sqrtf(s / (float)nd) : 0.0f;
  if ((c->ruido_m > BARO_RUIDO_MAX_M) || ((c->n == BARO_VENTANA) && !cambia)) {
    limpio = false;
  }

  /* Velocidad: pendiente por mínimos cuadrados de toda la ventana. Con la
   * diferencia entre la primera y la última, el ruido del sensor en reposo
   * daba hasta 8,5 m/s (medido); así, unas 4 veces menos. */
  float tm = 0.0f;
  float hm = 0.0f;
  for (uint32_t j = 0U; j < c->n; j++) {
    tm += (float)(int32_t)(c->t[j] - ahora);
    hm += c->h[j];
  }
  tm /= (float)c->n;
  hm /= (float)c->n;
  float stt = 0.0f;
  float sth = 0.0f;
  for (uint32_t j = 0U; j < c->n; j++) {
    const float dt = (float)(int32_t)(c->t[j] - ahora) - tm;
    stt += dt * dt;
    sth += dt * (c->h[j] - hm);
  }
  c->vz_mps = (stt > 0.0f) ? ((sth / stt) * 1000.0f) : 0.0f;

  if (limpio && !c->limpio) {
    c->t_limpio = ahora;
  }
  c->limpio = limpio;
  if (!limpio) {
    c->sucias++;
  }
  c->fiable = limpio && (c->n == BARO_VENTANA) &&
              ((ahora - c->t_limpio) >= BARO_LIMPIO_MS);
  if (c->fiable && (h > c->max_m)) {
    c->max_m = h;
  }
}

uint32_t baro_alt_update(uint32_t ahora) {
  uint32_t nuevas = 0U;
  for (uint32_t k = 0U; k < BARO_N; k++) {
    float p;
    if (_presion(k, &p)) {
      _muestra(&s_c[k], p, ahora);
      nuevas++;
    } else if (s_c[k].fiable && ((ahora - s_c[k].t_ultima) > BARO_VIEJO_MS)) {
      s_c[k].fiable = false; /* callado: ya no vale */
      s_c[k].limpio = false;
    }
  }
  /* La de todos: media de los fiables */
  float h = 0.0f;
  float v = 0.0f;
  uint32_t n = 0U;
  for (uint32_t k = 0U; k < BARO_N; k++) {
    if (s_c[k].fiable) {
      h += s_c[k].h_m;
      v += s_c[k].vz_mps;
      n++;
    }
  }
  if (n > 0U) {
    s_altura = h / (float)n;
    s_vz = v / (float)n;
  }
  return nuevas;
}

void baro_alt_suelo_update(uint32_t ahora) {
  for (uint32_t k = 0U; k < BARO_N; k++) {
    baro_canal_t *c = &s_c[k];
    const baro_data_t *b = &sensor_board_get_data()->baro[k];
    if (b->count == c->visto) {
      continue;
    }
    c->visto = b->count;
    const float p = b->pressure_kpa * 1000.0f;
    c->p_pa = p;
    c->t_ultima = ahora;
    if ((p > BARO_P_MIN_PA) && (p < BARO_P_MAX_PA)) {
      if (c->suelo_n == 0U) {
        c->suelo_t0 = ahora;
      }
      c->suelo_suma += p;
      c->suelo_n++;
    }
    if ((c->suelo_n > 0U) && ((ahora - c->suelo_t0) >= BARO_SUELO_BLOQUE_MS)) {
      memmove(&c->suelo_bloques[1], &c->suelo_bloques[0],
              BARO_SUELO_ATRAS * sizeof(float));
      c->suelo_bloques[0] = c->suelo_suma / (float)c->suelo_n;
      if (c->suelo_bloques_n <= BARO_SUELO_ATRAS) {
        c->suelo_bloques_n++;
      }
      c->suelo_suma = 0.0f;
      c->suelo_n = 0U;
    }
  }
}

bool baro_alt_suelo(float p0_pa[BARO_N]) {
  bool ok = true;
  for (uint32_t k = 0U; k < BARO_N; k++) {
    const baro_canal_t *c = &s_c[k];
    if (c->suelo_bloques_n == 0U) {
      p0_pa[k] = 0.0f;
      ok = false;
      continue;
    }
    /* La más vieja que haya, hasta BARO_SUELO_ATRAS bloques atrás */
    p0_pa[k] = c->suelo_bloques[c->suelo_bloques_n - 1U];
  }
  return ok;
}

const baro_canal_t *baro_alt_canal(uint32_t i) {
  return &s_c[(i < BARO_N) ? i : 0U];
}

bool baro_alt_fiable(void) {
  for (uint32_t k = 0U; k < BARO_N; k++) {
    if (s_c[k].fiable) {
      return true;
    }
  }
  return false;
}

float baro_alt_altura(void) { return s_altura; }
float baro_alt_vz(void) { return s_vz; }
