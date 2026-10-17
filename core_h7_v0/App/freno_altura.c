#include "freno_altura.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "gnc.h"
#include "storage.h"

static uint16_t s_metros;
static bool s_definida;
static bool s_guardada;
static const char *s_origen = "de fabrica";

/*
 * El archivo es texto legible a proposito, como ANGULO.TXT:
 *   ALTURA_FRENO_M=1500
 */
#define CLAVE "ALTURA_FRENO_M="

/* Lee el archivo de una tarjeta. false si no esta o lo que hay no vale. */
static bool _leer(sd_card_id_t sd, uint16_t *metros) {
  char buf[64];
  uint32_t n = 0U;
  if (!storage_read_file(sd, FRENO_ALTURA_ARCHIVO, (uint8_t *)buf,
                         sizeof(buf) - 1U, &n)) {
    return false;
  }
  buf[n] = '\0';
  const char *p = strstr(buf, CLAVE);
  if (p == NULL) {
    return false;
  }
  char *fin = NULL;
  const long v = strtol(p + strlen(CLAVE), &fin, 10);
  if ((fin == p + strlen(CLAVE)) || (v <= 0L) || (v > 65534L) ||
      !gnc_altura_freno_valida((float)v)) {
    return false; /* roto: mejor el valor de fabrica que basura */
  }
  *metros = (uint16_t)v;
  return true;
}

static bool _escribir(uint16_t metros) {
  char buf[64];
  const int n = snprintf(buf, sizeof(buf), CLAVE "%u\r\n", (unsigned)metros);
  if (n <= 0) {
    return false;
  }
  /* En las dos: con que una salga bien, sobrevive a un reinicio */
  const bool ok1 = storage_write_file(SD_CARD_1, FRENO_ALTURA_ARCHIVO,
                                      (const uint8_t *)buf, (uint32_t)n, false);
  const bool ok2 = storage_write_file(SD_CARD_2, FRENO_ALTURA_ARCHIVO,
                                      (const uint8_t *)buf, (uint32_t)n, false);
  return ok1 || ok2;
}

void freno_altura_init(void) {
  uint16_t v = 0U;
  if (_leer(SD_CARD_1, &v)) {
    s_origen = "SD1";
  } else if (_leer(SD_CARD_2, &v)) {
    s_origen = "SD2";
  } else {
    return; /* se queda en la de fabrica, "sin definir" */
  }
  s_metros = v;
  s_definida = true;
  s_guardada = true;
}

bool freno_altura_definir(uint16_t metros) {
  if (!gnc_altura_freno_valida((float)metros)) {
    return false;
  }
  if (s_definida && (metros == s_metros)) {
    return true; /* repeticion del mismo comando: nada que escribir */
  }
  s_metros = metros;
  s_definida = true;
  s_origen = "radio";
  s_guardada = _escribir(s_metros);
  return true;
}

uint16_t freno_altura_m(void) {
  if (s_definida) {
    return s_metros;
  }
  return (uint16_t)(gnc_altura_freno_fabrica_m() + 0.5f);
}

bool freno_altura_definida(void) { return s_definida; }

bool freno_altura_guardada(void) { return s_guardada; }

const char *freno_altura_origen(void) { return s_origen; }
