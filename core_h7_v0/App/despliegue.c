#include "despliegue.h"

#include "debug_out.h"
#include "emergencia.h"
#include "gnc.h"
#include "main.h"

typedef struct {
  bool previa;          /* bandera en la vuelta anterior */
  uint32_t proximo_ms;  /* cuándo toca la siguiente orden */
  uint32_t ordenes;     /* aceptadas por power en este vuelo */
  uint32_t primera_ms;  /* HAL_GetTick() de la primera; 0 = ninguna */
} canal_t;

/* Un canal_t POR CANAL PIROTECNICO, aunque dos cuelguen de la misma
 * bandera: cada uno lleva su propio reloj de repeticion. Si 1A y 1B
 * compartieran uno, la orden a 1A lo adelantaria 200 ms y la de 1B, en la
 * misma vuelta, lo encontraria en el futuro: 1B no se encenderia nunca. */
static canal_t s_1a; /* drogue */
static canal_t s_1b; /* drogue */
static canal_t s_2a; /* main */
static uint32_t s_fallos;
static uint32_t s_inits_vistos;

static void _olvidar(void) {
  s_1a = (canal_t){0};
  s_1b = (canal_t){0};
  s_2a = (canal_t){0};
  s_fallos = 0U;
}

static void _canal(const sys_state *state, canal_t *c, bool bandera,
                   bool (*disparar)(const sys_state *), uint32_t ahora) {
  if (!bandera) {
    c->previa = false;
    return;
  }
  if (!c->previa) {
    c->previa = true;
    c->proximo_ms = ahora; /* flanco de subida: la primera, ya */
  }
  if ((int32_t)(ahora - c->proximo_ms) < 0) {
    return;
  }

  if (disparar(state)) {
    c->ordenes++;
    if (c->primera_ms == 0U) {
      c->primera_ms = (ahora != 0U) ? ahora : 1U;
    }
    c->proximo_ms = ahora + DESPLIEGUE_REPETIR_MS;
  } else {
    s_fallos++;
    c->proximo_ms = ahora + DESPLIEGUE_REINTENTO_MS;
  }
}

#if EMERGENCIA_PRUEBA
/* Prueba de emergencia en la mesa: se cuenta como mandado pero NO sale */
static bool _falso(const char *canal) {
  DebugOut_Printf("PRUEBA EMERGENCIA: %s (NO se manda)\r\n", canal);
  return true;
}
static bool _falso_1a(const sys_state *s) { (void)s; return _falso("1A"); }
static bool _falso_1b(const sys_state *s) { (void)s; return _falso("1B"); }
static bool _falso_2a(const sys_state *s) { (void)s; return _falso("2A"); }
#endif

void despliegue_update(const sys_state *state) {
  /* Reiniciado tras un vuelo: nada de pirotecnia de vuelo */
  if (emergencia_postvuelo()) {
    return;
  }
  /* gnc_init() nuevo = vuelo nuevo: lo de antes ya no cuenta */
  const uint32_t inits = gnc_init_count();
  if (inits != s_inits_vistos) {
    s_inits_vistos = inits;
    _olvidar();
  }

  const gnc_data_t *g = gnc_get_data();
  const bool en_vuelo = (g->rocket_state == GNC_ESTADO_EN_VUELO);
  const uint32_t ahora = HAL_GetTick();

  const bool drogue = en_vuelo && g->drogue_flag;
  const bool main_ = en_vuelo && g->main_flag;

  bool (*f1a)(const sys_state *) = sys_pyro_fire_1a;
  bool (*f1b)(const sys_state *) = sys_pyro_fire_1b;
  bool (*f2a)(const sys_state *) = sys_pyro_fire_2a;
#if EMERGENCIA_PRUEBA
  if (emergencia_prueba_sin_piros()) {
    f1a = _falso_1a;
    f1b = _falso_1b;
    f2a = _falso_2a;
  }
#endif
  /* Drogue: 1A y 1B a la vez, en la misma vuelta */
  _canal(state, &s_1a, drogue, f1a, ahora);
  _canal(state, &s_1b, drogue, f1b, ahora);
  /* Main: 2A */
  _canal(state, &s_2a, main_, f2a, ahora);
}

uint32_t despliegue_ordenes_1a(void) { return s_1a.ordenes; }
uint32_t despliegue_ordenes_1b(void) { return s_1b.ordenes; }
uint32_t despliegue_ordenes_2a(void) { return s_2a.ordenes; }

bool despliegue_drogue_mandado(void) {
  return (s_1a.ordenes > 0U) || (s_1b.ordenes > 0U);
}
bool despliegue_main_mandado(void) { return s_2a.ordenes > 0U; }

uint32_t despliegue_fallos(void) { return s_fallos; }

/* La primera de las dos: la que de verdad marca el momento del drogue */
uint32_t despliegue_drogue_ms(void) {
  if (s_1a.primera_ms == 0U) {
    return s_1b.primera_ms;
  }
  if (s_1b.primera_ms == 0U) {
    return s_1a.primera_ms;
  }
  return ((int32_t)(s_1a.primera_ms - s_1b.primera_ms) <= 0) ? s_1a.primera_ms
                                                             : s_1b.primera_ms;
}
uint32_t despliegue_main_ms(void) { return s_2a.primera_ms; }
