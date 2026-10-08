#include "airbrake.h"

#include <math.h>

#include "can_driver.h"
#include "gnc.h"
#include "main.h"

#define GRADOS_POR_RAD 57.295779513f

static float s_last_cmd;
static float s_last_grados;
static uint8_t s_last_byte;
static bool s_last_ok;
static uint32_t s_sent;
static uint32_t s_failed;

/* Para mandar una trama por paso del GNC, ni más ni menos */
static bool s_alguna;
static uint32_t s_paso_visto;
static uint32_t s_inits_vistos;

/* Pruebas de tierra */
typedef enum { PRUEBA_NINGUNA = 0, PRUEBA_SECUENCIA, PRUEBA_FIJA } prueba_t;
static prueba_t s_prueba;
static uint32_t s_prueba_t0_ms;
static float s_prueba_fija;

/* === Cinemática ========================================================== */

/* Ángulo de la manivela respecto a la guía para una extensión x (ley del
 * coseno en el triángulo manivela-biela-guía, ecs. 3 y 4 del PDF) */
static float _psi_grados(float x_mm) {
  const float r = AIRBRAKE_MANIVELA_MM;
  const float l = AIRBRAKE_BIELA_MM;
  float c = ((x_mm * x_mm) + (r * r) - (l * l)) / (2.0f * x_mm * r);
  if (c > 1.0f) {
    c = 1.0f;
  } else if (c < -1.0f) {
    c = -1.0f;
  }
  return acosf(c) * GRADOS_POR_RAD;
}

float airbrake_extension_a_grados(float extension) {
  if (!(extension > 0.0f)) {
    extension = 0.0f; /* también atrapa NaN */
  } else if (extension > 1.0f) {
    extension = 1.0f;
  }
  /* El 0 % empieza AIRBRAKE_CERO_MM fuera del tope de dentro; el 100 % es el
   * tope de fuera (airbrake.h) */
  const float x0 = AIRBRAKE_X_MIN_MM + AIRBRAKE_CERO_MM;
  const float x = x0 + (extension * (AIRBRAKE_X_MAX_MM - x0));
  /* theta = theta0 - psi (ec. 2), con theta0 = psi en el tope de dentro, que
   * es donde se midió el servo (AIRBRAKE_SERVO_GRADOS_RECOGIDO) */
  const float grados = _psi_grados(AIRBRAKE_X_MIN_MM) - _psi_grados(x);
  return (grados > 0.0f) ? grados : 0.0f;
}

float airbrake_manivela_a_servo(float grados_manivela) {
  float f = grados_manivela / AIRBRAKE_MANIVELA_MAX_GRADOS;
  if (!(f > 0.0f)) {
    f = 0.0f;
  } else if (f > 1.0f) {
    f = 1.0f;
  }
  return AIRBRAKE_SERVO_GRADOS_RECOGIDO +
         (f * (AIRBRAKE_SERVO_GRADOS_FUERA - AIRBRAKE_SERVO_GRADOS_RECOGIDO));
}

uint8_t airbrake_grados_a_byte(float grados_servo) {
  const float b = (grados_servo * 255.0f) / AIRBRAKE_SERVO_GRADOS;
  if (!(b > 0.0f)) {
    return 0U;
  }
  if (b > 255.0f) {
    return 255U;
  }
  return (uint8_t)(b + 0.5f); /* al más cercano, no truncado */
}

/* === Envío =============================================================== */

static void _mandar(float extension) {
  if (!(extension > 0.0f)) {
    extension = 0.0f; /* también atrapa NaN */
  } else if (extension > 1.0f) {
    extension = 1.0f;
  }

  const float grados =
      airbrake_manivela_a_servo(airbrake_extension_a_grados(extension));
  const uint8_t value = airbrake_grados_a_byte(grados);

  s_last_cmd = extension;
  s_last_grados = grados;
  s_last_byte = value;
  s_last_ok = can_driver_send(CAN_PORT_2, AIRBRAKE_CAN_ID, &value, 1U);
  if (s_last_ok) {
    s_sent++;
  } else {
    s_failed++;
  }
}

/*
 * La secuencia de prueba en función del tiempo desde que empezó:
 *   AIRBRAKE_PRUEBA_CICLOS veces todo fuera FUERA_MS y recogido DENTRO_MS,
 *   luego de 0 a 1 en RAMPA_MS y de 1 a 0 en otros RAMPA_MS.
 * Devuelve false al acabar.
 */
static bool _secuencia(uint32_t t_ms, float *extension) {
  const uint32_t ciclo = AIRBRAKE_PRUEBA_FUERA_MS + AIRBRAKE_PRUEBA_DENTRO_MS;
  const uint32_t ciclos_ms = AIRBRAKE_PRUEBA_CICLOS * ciclo;

  if (t_ms < ciclos_ms) {
    *extension = ((t_ms % ciclo) < AIRBRAKE_PRUEBA_FUERA_MS) ? 1.0f : 0.0f;
    return true;
  }
  t_ms -= ciclos_ms;
  if (t_ms < AIRBRAKE_PRUEBA_RAMPA_MS) {
    *extension = (float)t_ms / (float)AIRBRAKE_PRUEBA_RAMPA_MS;
    return true;
  }
  t_ms -= AIRBRAKE_PRUEBA_RAMPA_MS;
  if (t_ms < AIRBRAKE_PRUEBA_RAMPA_MS) {
    *extension = 1.0f - ((float)t_ms / (float)AIRBRAKE_PRUEBA_RAMPA_MS);
    return true;
  }
  *extension = 0.0f;
  return false;
}

void airbrake_update(void) {
  const gnc_data_t *g = gnc_get_data();

  /* Paso nuevo del GUIADO (50 Hz: es quien da el mando, y el servo no
   * aprovecha más de 50 Hz), o GNC reiniciado (que puede dejar el contador
   * donde estaba o en un valor ya visto) */
  const bool reiniciado = (gnc_init_count() != s_inits_vistos);
  if (s_alguna && (g->gc_steps == s_paso_visto) && !reiniciado) {
    return;
  }
  s_alguna = true;
  s_paso_visto = g->gc_steps;
  s_inits_vistos = gnc_init_count();

  const bool en_vuelo = (g->rocket_state == GNC_ESTADO_EN_VUELO);

  /* Una prueba de tierra nunca sobrevive a un despegue ni a un reinicio del
   * GNC (empezar o acabar una simulación) */
  if (en_vuelo || reiniciado) {
    s_prueba = PRUEBA_NINGUNA;
  }

  float extension = en_vuelo ? g->actuator_cmd : 0.0f;

  if (s_prueba == PRUEBA_SECUENCIA) {
    if (!_secuencia(HAL_GetTick() - s_prueba_t0_ms, &extension)) {
      s_prueba = PRUEBA_NINGUNA;
      extension = 0.0f;
    }
  } else if (s_prueba == PRUEBA_FIJA) {
    extension = s_prueba_fija;
  }

  _mandar(extension);
}

/* === Pruebas de tierra =================================================== */

void airbrake_prueba_secuencia(void) {
  s_prueba = PRUEBA_SECUENCIA;
  s_prueba_t0_ms = HAL_GetTick();
}

void airbrake_prueba_extension(float extension) {
  if (!(extension > 0.0f)) {
    extension = 0.0f;
  } else if (extension > 1.0f) {
    extension = 1.0f;
  }
  s_prueba = PRUEBA_FIJA;
  s_prueba_fija = extension;
}

void airbrake_prueba_parar(void) { s_prueba = PRUEBA_NINGUNA; }

bool airbrake_prueba_activa(void) { return s_prueba != PRUEBA_NINGUNA; }

bool airbrake_prueba_secuencia_activa(void) {
  return s_prueba == PRUEBA_SECUENCIA;
}

/* === Lecturas ============================================================ */

float airbrake_last_command(void) { return s_last_cmd; }

float airbrake_last_grados(void) { return s_last_grados; }

uint8_t airbrake_last_byte(void) { return s_last_byte; }

bool airbrake_last_ok(void) { return s_last_ok; }

uint32_t airbrake_sent(void) { return s_sent; }

uint32_t airbrake_failed(void) { return s_failed; }
