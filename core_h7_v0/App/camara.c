#include "camara.h"

#include <string.h>

/* Qué se le ha pedido */
enum {
  OP_NADA = 0,
  OP_INFO,
  OP_AJUSTES,
  OP_ESCRIBIR,
};

#define SIN_ID 0xFFU
#define TROZOS_MAX 15U

/* === Números en texto ==================================================== */

static uint32_t _numero(const char *s, const char **fin) {
  uint32_t v = 0U;
  while ((*s >= '0') && (*s <= '9')) {
    v = (v * 10U) + (uint32_t)(*s - '0');
    s++;
  }
  *fin = s;
  return v;
}

/* Días desde 1970-01-01 y vuelta (calendario civil, H. Hinnant) */
static int64_t _dias(int32_t y, int32_t m, int32_t d) {
  y -= (m <= 2) ? 1 : 0;
  const int32_t era = ((y >= 0) ? y : (y - 399)) / 400;
  const int32_t yoe = y - (era * 400);
  const int32_t doy = ((((153 * (m + ((m > 2) ? -3 : 9))) + 2) / 5) + d) - 1;
  const int32_t doe = ((yoe * 365) + (yoe / 4) - (yoe / 100)) + doy;
  return ((int64_t)era * 146097) + doe - 719468;
}

static void _civil(int64_t z, int32_t *y, int32_t *m, int32_t *d) {
  z += 719468;
  const int64_t era = ((z >= 0) ? z : (z - 146096)) / 146097;
  const int32_t doe = (int32_t)(z - (era * 146097));
  const int32_t yoe =
      (doe - (doe / 1460) + (doe / 36524) - (doe / 146096)) / 365;
  const int32_t doy = doe - ((365 * yoe) + (yoe / 4) - (yoe / 100));
  const int32_t mp = ((5 * doy) + 2) / 153;
  *d = doy - (((153 * mp) + 2) / 5) + 1;
  *m = (mp < 10) ? (mp + 3) : (mp - 9);
  *y = (int32_t)(yoe + (era * 400)) + ((*m <= 2) ? 1 : 0);
}

static int32_t _2(const char *s) { return ((s[0] - '0') * 10) + (s[1] - '0'); }

/* "20261006T143000.0" -> segundos desde 1970. false si no tiene esa forma */
static bool _hora(const char *s, int64_t *abs) {
  for (uint32_t i = 0U; i < 15U; i++) {
    const bool digito = (s[i] >= '0') && (s[i] <= '9');
    if ((i == 8U) ? (s[i] != 'T') : !digito) {
      return false;
    }
  }
  const int32_t y = (_2(&s[0]) * 100) + _2(&s[2]);
  *abs = (_dias(y, _2(&s[4]), _2(&s[6])) * 86400) +
         ((int64_t)_2(&s[9]) * 3600) + ((int64_t)_2(&s[11]) * 60) +
         _2(&s[13]);
  return true;
}

static char *_2d(char *p, int32_t v) {
  *p++ = (char)('0' + ((v / 10) % 10));
  *p++ = (char)('0' + (v % 10));
  return p;
}

/* Y al revés, para escribirla */
static void _texto(int64_t abs, char *out) {
  int64_t dias = abs / 86400;
  int64_t seg = abs % 86400;
  if (seg < 0) {
    seg += 86400;
    dias--;
  }
  int32_t y, m, d;
  _civil(dias, &y, &m, &d);
  char *p = out;
  p = _2d(p, y / 100);
  p = _2d(p, y % 100);
  p = _2d(p, m);
  p = _2d(p, d);
  *p++ = 'T';
  p = _2d(p, (int32_t)(seg / 3600));
  p = _2d(p, (int32_t)((seg / 60) % 60));
  p = _2d(p, (int32_t)(seg % 60));
  *p++ = '.';
  *p++ = '0';
  *p = '\0';
}

static void _copiar(char *dst, uint32_t tam, const char *src) {
  uint32_t i = 0U;
  for (; (i + 1U < tam) && (src[i] != '\0'); i++) {
    dst[i] = src[i];
  }
  dst[i] = '\0';
}

/* === Lo que dice la cámara =============================================== */

static bool _graba(const camara_t *c, uint32_t ahora) {
  return c->bajado && ((ahora - c->t_bajada) < CAMARA_GRABA_MS);
}

/* Una lectura del espacio libre: si ha bajado, está grabando */
static void _lectura(camara_t *c, uint32_t ahora) {
  if (c->lecturas == 0U) {
    c->t_primera = ahora;
  } else if (c->libre_mb < c->libre_antes) {
    if (!_graba(c, ahora)) {
      c->t_empezo = ahora;
    }
    c->bajado = true;
    c->t_bajada = ahora;
  }
  c->libre_antes = c->libre_mb;
  c->lecturas++;
}

/* La primera lectura tras escribir la hora: ¿marca la que se quería? Si
 * marca otra desplazada un número entero de cuartos de hora, es su huso: se
 * aprende y la siguiente escritura lo descuenta (ver camara.h) */
static void _comprobar_reloj(camara_t *c, uint32_t ahora) {
  if (c->reloj_ok || !c->reloj_por_comprobar) {
    return;
  }
  int64_t leida;
  if (!_hora(c->hora_camara, &leida)) {
    return;
  }
  c->reloj_por_comprobar = false;
  const int64_t esperada =
      c->reloj_objetivo + (int64_t)((ahora - c->t_reloj) / 1000U);
  const int64_t dif = leida - esperada;
  const int64_t tol = CAMARA_RELOJ_TOLERANCIA_S;
  if ((dif >= -tol) && (dif <= tol)) {
    c->reloj_ok = true;
    return;
  }
  if ((dif >= -(14 * 3600) - tol) && (dif <= (14 * 3600) + tol)) {
    const int64_t huso = ((dif >= 0) ? (dif + 450) : (dif - 450)) / 900 * 900;
    if (((dif - huso) >= -tol) && ((dif - huso) <= tol)) {
      c->desfase_s += (int32_t)huso;
    }
  }
}

/* La lista de GET_SETTINGS entera: [id | nombre\0 | valor\0]... */
static void _lista(camara_t *c, uint32_t ahora) {
  bool sd = false;
  uint32_t i = 0U;
  while (i < c->n_datos) {
    const uint8_t id = c->datos[i++];
    const char *nombre = (const char *)&c->datos[i];
    const uint32_t ln = (uint32_t)strnlen(nombre, c->n_datos - i);
    i += ln + 1U;
    if (i >= c->n_datos) {
      break; /* cortada: lo leído hasta aquí vale */
    }
    const char *valor = (const char *)&c->datos[i];
    const uint32_t lv = (uint32_t)strnlen(valor, c->n_datos - i);
    i += lv + 1U;
    if (i > c->n_datos) {
      break;
    }
    const char *f;
    if (strcmp(nombre, "SDCARD_CAPACITY") == 0) {
      c->libre_mb = _numero(valor, &f);
      c->total_mb = (*f == '/') ? _numero(f + 1, &f) : 0U;
      sd = true;
    } else if (strcmp(nombre, "REMAINING_TIME") == 0) {
      c->queda_s = _numero(valor, &f);
    } else if (strcmp(nombre, "RESOLUTION") == 0) {
      /* "1080P60FPS", "720P60FPS"; "4K30FPS" por si acaso */
      _copiar(c->resolucion, sizeof(c->resolucion), valor);
      const uint32_t n = _numero(valor, &f);
      c->lineas = (uint16_t)((*f == 'K') ? (n * 540U) : n);
      c->fps = (uint8_t)(((*f == 'P') || (*f == 'K')) ? _numero(f + 1, &f)
                                                      : 0U);
      c->ajustes_leidos = true;
    } else if (strcmp(nombre, "TV_MODE") == 0) {
      c->pal = (strcmp(valor, "PAL") == 0);
    } else if (strcmp(nombre, "CAMERA_TIME") == 0) {
      c->id_hora = id;
      _copiar(c->hora_camara, sizeof(c->hora_camara), valor);
      _comprobar_reloj(c, ahora);
    }
  }
  c->lecturas_total++;
  if (sd) {
    c->sd_leida = true;
    _lectura(c, ahora);
  }
}

/* === Respuestas ========================================================== */

static void _respuestas(camara_t *c, uint32_t ahora) {
  if (c->op == OP_NADA) {
    return;
  }
  if (c->op == OP_INFO) {
    if (runcam_ocupada(&c->rc)) {
      return;
    }
    /* Contestada si la respuesta es de después de pedirla */
    if (c->rc.info_ok && ((int32_t)(c->rc.info_ms - c->t_pedido) >= 0)) {
      c->contesta = true;
      c->t_respuesta = ahora;
    } else {
      c->sin_respuesta++;
    }
    c->op = OP_NADA;
    return;
  }
  uint32_t n;
  const uint8_t *r = runcam_respuesta(&c->rc, &n);
  if (r == NULL) {
    if (!runcam_ocupada(&c->rc)) {
      c->sin_respuesta++; /* caducó */
      c->op = OP_NADA;
    }
    return;
  }
  c->t_respuesta = ahora;
  if (c->op == OP_ESCRIBIR) {
    /* Forma corta: CC | resultado | releer | CRC. Se comprueba de todos
     * modos releyendo la hora. */
    if ((n == 4U) && (r[1] != 0U)) {
      c->escrituras_rechazadas++;
    }
    c->op = OP_NADA;
    return;
  }
  /* OP_AJUSTES: CC | faltan | largo | datos | CRC */
  const uint8_t faltan = r[1];
  const uint32_t largo = r[2];
  for (uint32_t i = 0U; (i < largo) && (c->n_datos < sizeof(c->datos)); i++) {
    c->datos[c->n_datos++] = r[3U + i];
  }
  if ((faltan > 0U) && (c->trozo < TROZOS_MAX)) {
    c->trozo++;
    c->t_pedido = ahora;
    (void)runcam_pedir_ajustes(&c->rc, 0U, c->trozo);
    return;
  }
  c->op = OP_NADA;
  _lista(c, ahora);
}

/* Contestaba y ya no: volver a preguntarle quién es */
static void _perdida(camara_t *c, uint32_t ahora) {
  if (c->contesta && ((ahora - c->t_respuesta) > CAMARA_PERDIDA_MS)) {
    c->contesta = false;
    c->perdidas++;
    runcam_olvidar(&c->rc);
    c->op = OP_NADA;
    c->sd_leida = false;
    c->lecturas = 0U;
    c->bajado = false;
  }
}

/* === Estado ============================================================== */

static void _estado(camara_t *c, uint32_t ahora) {
  camara_estado_t e;
  if (!c->contesta) {
    e = ((ahora - c->t_corriente) < CAMARA_SIN_RESPUESTA_MS)
            ? CAMARA_ARRANCANDO
            : CAMARA_NO_CONTESTA;
  } else if ((c->rc.funciones & RUNCAM_F_AJUSTES) == 0U) {
    e = CAMARA_COMPROBANDO; /* contesta pero no hay forma de saberlo */
  } else if (!c->sd_leida) {
    e = CAMARA_COMPROBANDO;
  } else if (c->total_mb == 0U) {
    e = CAMARA_PARADA; /* sin tarjeta */
  } else if (_graba(c, ahora)) {
    e = (!c->quiere_grabar && (c->pulsos > 0U)) ? CAMARA_PARANDO
                                                : CAMARA_GRABANDO;
  } else {
    const uint32_t desde = c->bajado ? c->t_bajada : c->t_primera;
    e = ((c->lecturas >= 2U) && ((ahora - desde) >= CAMARA_GRABA_MS))
            ? CAMARA_PARADA
            : CAMARA_COMPROBANDO;
  }
  c->estado = e;
}

static void _pulsar(camara_t *c, uint32_t ahora) {
  if (runcam_control(&c->rc, RUNCAM_OP_BOTON_ENCENDIDO)) {
    c->pulsos++;
    c->pulsaciones++;
    c->t_pulso = ahora;
  }
}

/* El botón, solo si el espacio libre ha confirmado lo contrario de lo que se
 * quiere (ver camara.h) */
static void _objetivo(camara_t *c, uint32_t ahora) {
  if (c->quiere_grabar) {
    if (c->estado == CAMARA_GRABANDO) {
      c->no_graba = false;
      return;
    }
    if (c->estado != CAMARA_PARADA) {
      return;
    }
    if (c->total_mb == 0U) {
      c->no_graba = true; /* sin tarjeta: el botón no lo arregla */
      return;
    }
    if ((ahora - c->t_corriente) < CAMARA_ESPERA_GRABAR_MS) {
      return;
    }
    if (c->pulsos > 0U) {
      if ((ahora - c->t_pulso) >= CAMARA_EFECTO_MS) {
        c->no_graba = true; /* pulsado y sigue parada: se rinde */
      }
      return;
    }
  } else {
    if ((c->estado != CAMARA_GRABANDO) && (c->estado != CAMARA_PARANDO)) {
      return;
    }
    if ((c->pulsos > 0U) && ((ahora - c->t_pulso) < CAMARA_EFECTO_MS)) {
      return;
    }
    if (c->pulsos >= CAMARA_PULSOS_PARAR) {
      return;
    }
  }
  /* El botón no tiene respuesta, pero pisaría una que se esté esperando */
  if ((c->op != OP_NADA) || runcam_ocupada(&c->rc)) {
    return;
  }
  _pulsar(c, ahora);
}

/* === Peticiones ========================================================== */

static void _pedir(camara_t *c, uint32_t ahora, const char *hora) {
  if ((c->op != OP_NADA) || runcam_ocupada(&c->rc) ||
      ((ahora - c->t_corriente) < CAMARA_ESPERA_MS)) {
    return;
  }
  if (!c->contesta || ((c->rc.funciones & RUNCAM_F_AJUSTES) == 0U)) {
    /* Quién es: hasta que conteste y, si no tiene ajustes, de latido */
    const uint32_t cada = c->contesta ? CAMARA_LATIDO_MS : CAMARA_REINTENTO_MS;
    if ((ahora - c->t_pedido) >= cada) {
      c->op = OP_INFO;
      c->t_pedido = ahora;
      (void)runcam_pedir_info(&c->rc);
    }
    return;
  }
  int64_t objetivo;
  if ((hora != NULL) && (c->id_hora != SIN_ID) && !c->reloj_ok &&
      !c->reloj_por_comprobar &&
      (c->reloj_intentos < CAMARA_RELOJ_INTENTOS) &&
      ((c->reloj_intentos == 0U) ||
       ((ahora - c->t_reloj) >= CAMARA_RELOJ_REINTENTO_MS)) &&
      _hora(hora, &objetivo)) {
    /* Lo que se quiere que marque, menos lo que ella le suma */
    c->reloj_objetivo = objetivo;
    _texto(objetivo - c->desfase_s, c->reloj_escrito);
    c->reloj_por_comprobar = true;
    c->reloj_intentos++;
    c->t_reloj = ahora;
    c->op = OP_ESCRIBIR;
    c->t_pedido = ahora;
    (void)runcam_escribir(&c->rc, c->id_hora, (const uint8_t *)c->reloj_escrito,
                          (uint32_t)strlen(c->reloj_escrito) + 1U);
    return;
  }
  if ((ahora - c->t_lectura) >= CAMARA_LECTURA_MS) {
    c->t_lectura = ahora;
    c->trozo = 0U;
    c->n_datos = 0U;
    c->op = OP_AJUSTES;
    c->t_pedido = ahora;
    (void)runcam_pedir_ajustes(&c->rc, 0U, 0U);
  }
}

/* === API ================================================================= */

void camara_init(camara_t *c, uint8_t num, UART_HandleTypeDef *huart,
                 IRQn_Type irq) {
  memset((void *)c, 0, sizeof(*c));
  c->num = num;
  c->id_hora = SIN_ID;
  c->quiere_grabar = true;
  (void)runcam_init(&c->rc, huart, irq);
}

void camara_irq(camara_t *c) { runcam_irq(&c->rc); }

void camara_corriente(camara_t *c, bool on) {
  if (on == c->corriente) {
    return;
  }
  c->corriente = on;
  c->t_corriente = HAL_GetTick();
  runcam_olvidar(&c->rc);
  c->op = OP_NADA;
  c->contesta = false;
  c->sd_leida = false;
  c->ajustes_leidos = false;
  c->lecturas = 0U;
  c->bajado = false;
  c->quiere_grabar = true; /* al encenderse graba sola */
  c->pulsos = 0U;
  c->no_graba = false;
  c->reloj_ok = false;
  c->reloj_por_comprobar = false;
  c->reloj_intentos = 0U; /* desfase_s se queda: es de la cámara */
  c->estado = on ? CAMARA_ARRANCANDO : CAMARA_SIN_CORRIENTE;
}

void camara_quiere(camara_t *c, bool grabar) {
  if (grabar == c->quiere_grabar) {
    return;
  }
  c->quiere_grabar = grabar;
  c->pulsos = 0U;
  c->no_graba = false;
}

void camara_update(camara_t *c, uint32_t ahora, const char *hora) {
  runcam_update(&c->rc);
  if (!c->corriente) {
    c->estado = CAMARA_SIN_CORRIENTE;
    return;
  }
  _respuestas(c, ahora);
  _perdida(c, ahora);
  _estado(c, ahora);
  _objetivo(c, ahora);
  _pedir(c, ahora, hora);
}

camara_estado_t camara_estado(const camara_t *c) { return c->estado; }

const char *camara_estado_nombre(camara_estado_t e) {
  switch (e) {
  case CAMARA_SIN_CORRIENTE:
    return "sin corriente";
  case CAMARA_ARRANCANDO:
    return "arrancando";
  case CAMARA_NO_CONTESTA:
    return "NO CONTESTA";
  case CAMARA_COMPROBANDO:
    return "comprobando";
  case CAMARA_GRABANDO:
    return "GRABANDO";
  case CAMARA_PARADA:
    return "parada";
  case CAMARA_PARANDO:
    return "parando";
  default:
    return "?";
  }
}

uint32_t camara_segundos_grabando(const camara_t *c, uint32_t ahora) {
  if ((c->estado != CAMARA_GRABANDO) && (c->estado != CAMARA_PARANDO)) {
    return 0U;
  }
  return (ahora - c->t_empezo) / 1000U;
}

bool camara_lista_para_cortar(const camara_t *c) {
  /* Si contesta pero no da ajustes no hay forma de saber si graba ni de
   * pararla con seguridad: tampoco se le hace esperar */
  const bool sin_ajustes =
      c->contesta && ((c->rc.funciones & RUNCAM_F_AJUSTES) == 0U);
  return (c->estado == CAMARA_SIN_CORRIENTE) ||
         (c->estado == CAMARA_NO_CONTESTA) || (c->estado == CAMARA_PARADA) ||
         sin_ajustes;
}

static void _u16(uint8_t *p, uint32_t v) {
  if (v > 0xFFFFU) {
    v = 0xFFFFU;
  }
  p[0] = (uint8_t)(v & 0xFFU);
  p[1] = (uint8_t)(v >> 8);
}

void camara_paquete(const camara_t *c, uint32_t ahora, uint8_t *out) {
  memset(out, 0, CAMARA_PAQUETE_BYTES);
  out[0] = (uint8_t)c->estado;
  out[1] = (uint8_t)((c->quiere_grabar ? 0x01U : 0U) |
                     (c->no_graba ? 0x02U : 0U) |
                     (c->reloj_ok ? 0x04U : 0U) |
                     (c->sd_leida ? 0x08U : 0U) |
                     (c->pal ? 0x10U : 0U) |
                     (c->ajustes_leidos ? 0x20U : 0U) |
                     ((c->sd_leida && (c->total_mb == 0U)) ? 0x40U : 0U));
  _u16(&out[2], c->lineas);
  out[4] = c->fps;
  _u16(&out[5], c->libre_mb / 16U);
  _u16(&out[7], c->total_mb / 16U);
  _u16(&out[9], c->queda_s);
  _u16(&out[11], camara_segundos_grabando(c, ahora));
  out[13] = (uint8_t)((c->pulsaciones > 255U) ? 255U : c->pulsaciones);
}
