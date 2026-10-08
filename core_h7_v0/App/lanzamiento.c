#include "lanzamiento.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "storage.h"

static uint16_t s_centesimas;
static bool s_definido;
static uint16_t s_rumbo;
static bool s_rumbo_definido;
static bool s_guardado;
static const char *s_origen = "por defecto (vertical)";

/*
 * El archivo es texto legible a proposito:
 *   ANGULO_VERTICAL_CENTESIMAS=525
 *   RUMBO_CENTESIMAS=9000
 * Si alguna vez hay que comprobar con que rampa arranco el cohete, basta con
 * abrir la tarjeta en un PC. Un archivo viejo sin la linea del rumbo vale:
 * el rumbo queda sin definir.
 */
#define CLAVE "ANGULO_VERTICAL_CENTESIMAS="
#define CLAVE_RUMBO "RUMBO_CENTESIMAS="

/* El numero que sigue a 'clave', si esta y cabe en 0..maximo */
static bool _campo(const char *buf, const char *clave, long maximo,
                   uint16_t *out) {
  const char *p = strstr(buf, clave);
  if (p == NULL) {
    return false;
  }
  char *fin = NULL;
  const long v = strtol(p + strlen(clave), &fin, 10);
  if ((fin == p + strlen(clave)) || (v < 0) || (v > maximo)) {
    return false; /* roto: mejor el valor por defecto que basura */
  }
  *out = (uint16_t)v;
  return true;
}

/* Lee el archivo de una tarjeta. Devuelve si estaba el angulo; el rumbo
 * puede faltar (archivos de antes del 2026-10-04). */
static bool _leer(sd_card_id_t sd, uint16_t *angulo, uint16_t *rumbo,
                  bool *hay_rumbo) {
  char buf[96];
  uint32_t n = 0U;
  if (!storage_read_file(sd, LANZAMIENTO_ARCHIVO, (uint8_t *)buf,
                         sizeof(buf) - 1U, &n)) {
    return false;
  }
  buf[n] = '\0';
  if (!_campo(buf, CLAVE, (long)LANZAMIENTO_MAX_CENTESIMAS, angulo)) {
    return false;
  }
  *hay_rumbo =
      _campo(buf, CLAVE_RUMBO, (long)LANZAMIENTO_RUMBO_MAX_CENTESIMAS, rumbo);
  return true;
}

static bool _escribir(uint16_t centesimas, uint16_t rumbo) {
  char buf[96];
  const int n = snprintf(buf, sizeof(buf), CLAVE "%u\r\n" CLAVE_RUMBO "%u\r\n",
                         (unsigned)centesimas, (unsigned)rumbo);
  if (n <= 0) {
    return false;
  }
  /* En las dos: con que una salga bien, el angulo sobrevive a un reinicio */
  const bool ok1 = storage_write_file(SD_CARD_1, LANZAMIENTO_ARCHIVO,
                                      (const uint8_t *)buf, (uint32_t)n, false);
  const bool ok2 = storage_write_file(SD_CARD_2, LANZAMIENTO_ARCHIVO,
                                      (const uint8_t *)buf, (uint32_t)n, false);
  return ok1 || ok2;
}

void lanzamiento_init(void) {
  uint16_t v = 0U;
  uint16_t r = 0U;
  bool hay_rumbo = false;
  if (_leer(SD_CARD_1, &v, &r, &hay_rumbo)) {
    s_origen = "SD1";
  } else if (_leer(SD_CARD_2, &v, &r, &hay_rumbo)) {
    s_origen = "SD2";
  } else {
    return; /* se queda en 0, vertical, "sin definir" */
  }
  s_centesimas = v;
  s_definido = true;
  s_rumbo = hay_rumbo ? r : 0U;
  s_rumbo_definido = hay_rumbo;
  s_guardado = true;
}

bool lanzamiento_definir(uint16_t centesimas) {
  if (centesimas > LANZAMIENTO_MAX_CENTESIMAS) {
    return false;
  }
  if (s_definido && (centesimas == s_centesimas)) {
    return true; /* repeticion del mismo comando: nada que escribir */
  }
  s_centesimas = centesimas;
  s_definido = true;
  s_origen = "radio";
  s_guardado = _escribir(s_centesimas, s_rumbo);
  return true;
}

bool lanzamiento_definir_rumbo(uint16_t centesimas) {
  if (centesimas > LANZAMIENTO_RUMBO_MAX_CENTESIMAS) {
    return false;
  }
  if (s_rumbo_definido && (centesimas == s_rumbo)) {
    return true; /* repeticion del mismo comando: nada que escribir */
  }
  s_rumbo = centesimas;
  s_rumbo_definido = true;
  s_origen = "radio";
  s_guardado = _escribir(s_centesimas, s_rumbo);
  return true;
}

uint16_t lanzamiento_rumbo_centesimas(void) { return s_rumbo; }

float lanzamiento_rumbo_grados(void) { return (float)s_rumbo * 0.01f; }

bool lanzamiento_rumbo_definido(void) { return s_rumbo_definido; }

uint16_t lanzamiento_centesimas(void) { return s_centesimas; }

float lanzamiento_grados(void) { return (float)s_centesimas * 0.01f; }

bool lanzamiento_definido(void) { return s_definido; }

bool lanzamiento_guardado(void) { return s_guardado; }

const char *lanzamiento_origen(void) { return s_origen; }
