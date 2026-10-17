#include "uplink.h"

#include "airbrake.h"
#include "aspid_data.h"
#include "camaras.h"
#include "emergencia.h"
#include "freno_altura.h"
#include "gnc.h"
#include "gnc_playback.h"
#include "lanzamiento.h"
#include "main.h"
#include "rf_telemetry.h"

static uint32_t s_recibidos;
static uint32_t s_validos;
static uint32_t s_rechazados;
static uint32_t s_denegados;
static uint32_t s_desconocidos;
static uplink_denegado_t s_ultimo_denegado;
static uint8_t s_ultimo_cmd;
static uint32_t s_ultimo_ms;
static sys_rf_rx_meta_t s_rx_meta; /* cómo se oyó el último paquete */

/* Modo test */
static bool s_test;
static uint32_t s_test_hasta_ms;
static uint32_t s_test_proximo_estado_ms;
static uint8_t s_ultimo_piro;
static uint32_t s_piros_aceptados;
static uint8_t s_ultima_prueba_aerofreno; /* número; 0 = ninguna todavía */
static uint8_t s_ultimo_realineado;       /* número; 0 = ninguno todavía */

/* Simulación: para avisar a tierra al empezar, durante y al acabar */
static bool s_simulando_previo;
static uint32_t s_sim_proximo_estado_ms;

/*
 * ¿Está el cohete en tierra y en un estado en el que se puede probar?
 *
 * Mira las DOS cosas, estado y fase, porque cada una sola deja huecos: la
 * fase vuelve a IDLE en algunos reinicios del modelo, y el estado ERROR puede
 * darse en vuelo. Sólo si las dos dicen "tierra" se permite.
 */
static bool _en_tierra(void) {
  const gnc_data_t *g = gnc_get_data();
  if (g == NULL) {
    return false;
  }
  const bool estado_ok = (g->rocket_state == GNC_ESTADO_POWER_ON) ||
                         (g->rocket_state == GNC_ESTADO_MANTENIMIENTO) ||
                         (g->rocket_state == GNC_ESTADO_ERROR);
  const bool fase_ok = (g->flight_phase == GNC_FASE_IDLE) ||
                       (g->flight_phase == GNC_FASE_EN_TIERRA);
  return estado_ok && fase_ok;
}

/*
 * Una denegacion se le cuenta a tierra EN EL MOMENTO, con un paquete de
 * estado. Sin esto tierra no se entera nunca: pide modo test, el cohete lo
 * deniega (paso con el estado READY de la v1.1), y la estacion se queda
 * "esperando confirmacion" para siempre sin saber por que.
 * rf_telemetry_pedir_estado() solo levanta una bandera, asi que varias
 * denegaciones seguidas dan un unico paquete.
 */
static void _denegar(uplink_denegado_t motivo) {
  s_denegados++;
  s_ultimo_denegado = motivo;
  rf_telemetry_pedir_estado();
}

/* Todo lo que se hace en modo test se acaba con él */
static void _cerrar_test(void) {
  s_test = false;
  airbrake_prueba_parar();
}

static void _salir_test(void) {
  if (s_test) {
    _cerrar_test();
    /* Un último paquete para que tierra vea la salida sin esperar */
    rf_telemetry_pedir_estado();
  }
}

static bool _disparar(const sys_state *state, uint8_t canal) {
  switch (canal) {
  case 1U:
    return sys_pyro_fire_1a(state);
  case 2U:
    return sys_pyro_fire_1b(state);
  case 3U:
    return sys_pyro_fire_2a(state);
  case 4U:
    return sys_pyro_fire_2b(state);
  default:
    return false;
  }
}

/*
 * ¿Se puede cambiar el ángulo de lanzamiento ahora? En vuelo no: es la
 * actitud con la que se alinea el GNC, y cambiarla con el cohete en el aire
 * no tiene ningún sentido. Tampoco simulando, por lo mismo. En la rampa sí
 * (POWER ON), que es justo cuando hace falta: con la v1.5 el GNC se realinea
 * en el acto con el ángulo nuevo (gnc_set_rampa).
 */
static bool _angulo_cambiable(void) {
  if (gnc_playback_active()) {
    return false;
  }
  const gnc_data_t *g = gnc_get_data();
  if (g == NULL) {
    return false;
  }
  return (g->rocket_state != GNC_ESTADO_EN_VUELO) &&
         ((g->flight_phase == GNC_FASE_IDLE) ||
          (g->flight_phase == GNC_FASE_EN_TIERRA));
}

/* Lo mismo que pide un pirotécnico manual: modo test, en tierra y sin simular */
static uplink_denegado_t _aerofreno_permitido(void) {
  if (gnc_playback_active()) {
    return UPLINK_DENEGADO_SIMULANDO;
  }
  if (!s_test) {
    return UPLINK_DENEGADO_SIN_MODO_TEST;
  }
  if (!_en_tierra()) {
    return UPLINK_DENEGADO_FUERA_DE_TIERRA;
  }
  return UPLINK_DENEGADO_NADA;
}

static uint8_t s_acuses;

/* Contesta a un comando con el valor que el cohete tiene AHORA */
static void _acuse(uint8_t cmd, uint8_t resultado, uint16_t valor) {
  bool definido;
  bool guardado;
  if (cmd == UPLINK_CMD_ALTURA_FRENO) {
    definido = freno_altura_definida();
    guardado = freno_altura_guardada();
  } else {
    definido = (cmd == UPLINK_CMD_RUMBO) ? lanzamiento_rumbo_definido()
                                         : lanzamiento_definido();
    guardado = lanzamiento_guardado();
  }
  uint8_t a[UPLINK_ACUSE_BYTES] = {0};
  a[0] = UPLINK_ACUSE_MARCA;
  a[1] = cmd;
  a[2] = resultado;
  a[3] = (uint8_t)(valor & 0xFFU);
  a[4] = (uint8_t)(valor >> 8);
  a[5] = s_acuses++;
  a[6] = (uint8_t)((definido ? 0x01U : 0U) | (guardado ? 0x02U : 0U));
  /* Cómo oyó el cohete ESTE comando: tierra ve el enlace de subida desde
   * este lado (RSSI, SNR y error de frecuencia) */
  if (s_rx_meta.valida) {
    a[7] = 0x01U;
    a[8] = (uint8_t)s_rx_meta.rssi_dbm;
    a[9] = (uint8_t)s_rx_meta.snr_db;
    a[10] = (uint8_t)((uint16_t)s_rx_meta.df_hz & 0xFFU);
    a[11] = (uint8_t)((uint16_t)s_rx_meta.df_hz >> 8);
  }
  rf_telemetry_mandar_acuse(a, (uint8_t)sizeof(a));
}

static void _ejecutar(const sys_state *state, uint8_t cmd, uint16_t parametro) {
  const uint32_t ahora = HAL_GetTick();

  switch (cmd) {
  case UPLINK_CMD_PEDIR_ESTADO:
    rf_telemetry_pedir_estado();
    rf_telemetry_pedir_camaras();
    break;

  case UPLINK_CMD_PING:
    /* Ida y vuelta de verdad: tierra sabe que el cohete la oye Y que ella
     * oye al cohete, y cuanto tarda. Siempre permitido. */
    _acuse(cmd, (uint8_t)UPLINK_DENEGADO_NADA, parametro);
    break;

  case UPLINK_CMD_MODO_TEST:
    if (parametro == 0U) {
      /* Salir siempre está permitido */
      _salir_test();
      break;
    }
    if (gnc_playback_active()) {
      _denegar(UPLINK_DENEGADO_SIMULANDO);
      return;
    }
    if (!_en_tierra()) {
      _denegar(UPLINK_DENEGADO_FUERA_DE_TIERRA);
      return;
    }
    if (!s_test) {
      s_test_proximo_estado_ms = ahora; /* confirmación inmediata */
    }
    s_test = true;
    s_test_hasta_ms = ahora + UPLINK_TEST_TIMEOUT_MS;
    break;

  case UPLINK_CMD_PIRO:
    /* Redundante con lo de abajo (al simular se cierra el modo test), pero
     * esto es pirotecnia: dos cerrojos mejor que uno. */
    if (gnc_playback_active()) {
      _denegar(UPLINK_DENEGADO_SIMULANDO);
      return;
    }
    if (!s_test) {
      _denegar(UPLINK_DENEGADO_SIN_MODO_TEST);
      return;
    }
    if ((parametro < 1U) || (parametro > 4U)) {
      _denegar(UPLINK_DENEGADO_CANAL);
      return;
    }
    if (!_disparar(state, parametro)) {
      _denegar(UPLINK_DENEGADO_POWER);
      return;
    }
    s_piros_aceptados++;
    s_ultimo_piro = parametro;
    /* Que tierra lo vea en el próximo hueco, no dentro de un segundo */
    rf_telemetry_pedir_estado();
    break;

  case UPLINK_CMD_SIMULACION:
    if (parametro == 0U) {
      gnc_playback_stop(); /* parar siempre está permitido */
      break;
    }
    if (gnc_playback_active()) {
      break; /* ya en marcha: repetirlo no hace nada (idempotente) */
    }
    if (!s_test) {
      _denegar(UPLINK_DENEGADO_SIN_MODO_TEST);
      return;
    }
    if (!_en_tierra()) {
      _denegar(UPLINK_DENEGADO_FUERA_DE_TIERRA);
      return;
    }
    /* El modo test se cierra ANTES de arrancar: desde el primer paso de la
     * simulación no puede quedar nada que acepte un pirotécnico. */
    _cerrar_test();
    if (!gnc_playback_start()) {
      _denegar(UPLINK_DENEGADO_SIN_ARCHIVO);
      return;
    }
    s_sim_proximo_estado_ms = ahora; /* confirmación inmediata */
    break;

  case UPLINK_CMD_ANGULO: {
    if (parametro == UPLINK_ANGULO_CONSULTA) {
      /* Solo lectura: siempre permitida, no toca nada */
      _acuse(cmd, (uint8_t)UPLINK_DENEGADO_NADA, lanzamiento_centesimas());
      break;
    }
    uplink_denegado_t motivo = UPLINK_DENEGADO_NADA;
    if (!_angulo_cambiable()) {
      motivo = gnc_playback_active() ? UPLINK_DENEGADO_SIMULANDO
                                     : UPLINK_DENEGADO_FUERA_DE_TIERRA;
    } else if (!lanzamiento_definir(parametro)) {
      motivo = UPLINK_DENEGADO_FUERA_DE_RANGO;
    } else {
      /* Al GNC: en tierra se realinea en el acto con él (gnc_set_rampa) */
      gnc_set_rampa(lanzamiento_grados(), lanzamiento_rumbo_grados(),
                    GNC_ALABEO_MEDIR);
    }
    /* El acuse va SIEMPRE, también si se deniega: con el valor que de
     * verdad hay, para que tierra vea que no ha cambiado y por qué. */
    _acuse(cmd, (uint8_t)motivo, lanzamiento_centesimas());
    if (motivo != UPLINK_DENEGADO_NADA) {
      s_denegados++;
      s_ultimo_denegado = motivo;
      return;
    }
    break;
  }

  case UPLINK_CMD_RUMBO: {
    if (parametro == UPLINK_ANGULO_CONSULTA) {
      _acuse(cmd, (uint8_t)UPLINK_DENEGADO_NADA,
             lanzamiento_rumbo_centesimas());
      break;
    }
    uplink_denegado_t motivo = UPLINK_DENEGADO_NADA;
    if (!_angulo_cambiable()) {
      motivo = gnc_playback_active() ? UPLINK_DENEGADO_SIMULANDO
                                     : UPLINK_DENEGADO_FUERA_DE_TIERRA;
    } else if (!lanzamiento_definir_rumbo(parametro)) {
      motivo = UPLINK_DENEGADO_FUERA_DE_RANGO;
    } else {
      gnc_set_rampa(lanzamiento_grados(), lanzamiento_rumbo_grados(),
                    GNC_ALABEO_MEDIR);
    }
    _acuse(cmd, (uint8_t)motivo, lanzamiento_rumbo_centesimas());
    if (motivo != UPLINK_DENEGADO_NADA) {
      s_denegados++;
      s_ultimo_denegado = motivo;
      return;
    }
    break;
  }

  case UPLINK_CMD_ALTURA_FRENO: {
    if (parametro == UPLINK_ANGULO_CONSULTA) {
      _acuse(cmd, (uint8_t)UPLINK_DENEGADO_NADA,
             (uint16_t)(gnc_altura_freno_m() + 0.5f));
      break;
    }
    /* En vuelo no: el guiado estaría persiguiendo con una altura distinta
     * de la del ciclo anterior. En tierra vale para el siguiente paso. */
    uplink_denegado_t motivo = UPLINK_DENEGADO_NADA;
    if (!_angulo_cambiable()) {
      motivo = gnc_playback_active() ? UPLINK_DENEGADO_SIMULANDO
                                     : UPLINK_DENEGADO_FUERA_DE_TIERRA;
    } else if (!freno_altura_definir(parametro)) {
      motivo = UPLINK_DENEGADO_FUERA_DE_RANGO;
    } else {
      (void)gnc_set_altura_freno((float)freno_altura_m());
    }
    /* Con la que usa DE VERDAD el GNC, no con la pedida */
    _acuse(cmd, (uint8_t)motivo, (uint16_t)(gnc_altura_freno_m() + 0.5f));
    if (motivo != UPLINK_DENEGADO_NADA) {
      s_denegados++;
      s_ultimo_denegado = motivo;
      return;
    }
    break;
  }

  case UPLINK_CMD_REALINEAR: {
    uplink_denegado_t motivo = UPLINK_DENEGADO_NADA;
    if (parametro == 0U) {
      motivo = UPLINK_DENEGADO_FUERA_DE_RANGO;
    } else if (parametro == s_ultimo_realineado) {
      /* Reintento de tierra de una petición ya hecha: solo se contesta */
    } else if (gnc_playback_active()) {
      motivo = UPLINK_DENEGADO_SIMULANDO;
    } else if ((emergencia_modo() != EMERGENCIA_INICIO) || !_en_tierra() ||
               !gnc_realinear()) {
      /* En el modo de emergencia (o tras un vuelo) el GNC no está en marcha,
       * y gnc_get_data() es el de la emergencia */
      motivo = UPLINK_DENEGADO_FUERA_DE_TIERRA;
    } else {
      s_ultimo_realineado = (uint8_t)parametro;
    }
    _acuse(cmd, (uint8_t)motivo, parametro);
    if (motivo != UPLINK_DENEGADO_NADA) {
      s_denegados++;
      s_ultimo_denegado = motivo;
      return;
    }
    break;
  }

  case UPLINK_CMD_PRUEBA_AEROFRENO: {
    uplink_denegado_t motivo = UPLINK_DENEGADO_NADA;
    if (parametro == 0U) {
      airbrake_prueba_parar(); /* parar siempre está permitido */
    } else {
      motivo = _aerofreno_permitido();
      /* El mismo número otra vez es un reintento: solo se contesta */
      if ((motivo == UPLINK_DENEGADO_NADA) &&
          (parametro != s_ultima_prueba_aerofreno)) {
        s_ultima_prueba_aerofreno = (uint8_t)parametro;
        airbrake_prueba_secuencia();
      }
    }
    _acuse(cmd, (uint8_t)motivo, parametro);
    if (motivo != UPLINK_DENEGADO_NADA) {
      s_denegados++;
      s_ultimo_denegado = motivo;
      return;
    }
    break;
  }

  case UPLINK_CMD_EXTENSION_AEROFRENO: {
    uplink_denegado_t motivo = (parametro > 100U)
                                   ? UPLINK_DENEGADO_FUERA_DE_RANGO
                                   : _aerofreno_permitido();
    if (motivo == UPLINK_DENEGADO_NADA) {
      airbrake_prueba_extension((float)parametro / 100.0f);
    }
    /* Con el valor que se va a mandar al servo: el de la prueba si se ha
     * aceptado, el que había si no */
    const uint16_t pct =
        (motivo == UPLINK_DENEGADO_NADA)
            ? parametro
            : (uint16_t)((airbrake_last_command() * 100.0f) + 0.5f);
    _acuse(cmd, (uint8_t)motivo, pct);
    if (motivo != UPLINK_DENEGADO_NADA) {
      s_denegados++;
      s_ultimo_denegado = motivo;
      return;
    }
    break;
  }

  case UPLINK_CMD_CAMARAS: {
    uplink_denegado_t motivo = UPLINK_DENEGADO_NADA;
    if (parametro == 1U) {
      camaras_encender(CAMARAS_ORIGEN_TIERRA);
    } else if (parametro != 0U) {
      motivo = UPLINK_DENEGADO_FUERA_DE_RANGO;
    } else if (!_en_tierra()) {
      motivo = UPLINK_DENEGADO_FUERA_DE_TIERRA; /* en vuelo no se apagan */
    } else {
      camaras_apagar();
    }
    _acuse(cmd, (uint8_t)motivo,
           (uint16_t)((camaras_encendidas() ? 0x01U : 0U) |
                      (camaras_apagando() ? 0x02U : 0U)));
    if (motivo != UPLINK_DENEGADO_NADA) {
      s_denegados++;
      s_ultimo_denegado = motivo;
      return;
    }
    break;
  }

  default:
    s_desconocidos++;
    return;
  }

  s_validos++;
  s_ultimo_cmd = cmd;
  s_ultimo_ms = ahora;
}

/*
 * Vigilancia del modo test, en CADA vuelta, haya llegado algo o no. Lo de la
 * salida de tierra no puede esperar a que llegue un comando: si el cohete
 * despega en modo test, el modo test tiene que caer en ese mismo paso.
 */
static void _vigilar_test(void) {
  if (!s_test) {
    return;
  }
  const uint32_t ahora = HAL_GetTick();

  if (!_en_tierra()) {
    _salir_test();
    return;
  }
  if ((int32_t)(ahora - s_test_hasta_ms) >= 0) {
    _salir_test();
    return;
  }
  if ((int32_t)(ahora - s_test_proximo_estado_ms) >= 0) {
    rf_telemetry_pedir_estado();
    s_test_proximo_estado_ms = ahora + UPLINK_TEST_ESTADO_PERIODO_MS;
  }
}

/*
 * Paquete de estado por segundo mientras se simula, y uno más al terminar:
 * la simulación puede acabar sola (fin del archivo) y tierra tiene que dejar
 * de pintar los datos como simulados en cuanto pase.
 */
static void _vigilar_simulacion(void) {
  const bool simulando = gnc_playback_active();
  const uint32_t ahora = HAL_GetTick();

  if (simulando &&
      ((int32_t)(ahora - s_sim_proximo_estado_ms) >= 0)) {
    rf_telemetry_pedir_estado();
    s_sim_proximo_estado_ms = ahora + UPLINK_TEST_ESTADO_PERIODO_MS;
  }
  if (s_simulando_previo && !simulando) {
    rf_telemetry_pedir_estado();
  }
  s_simulando_previo = simulando;
}

void uplink_update(sys_state *state) {
  uint8_t buf[32];
  uint8_t len = 0U;

  if (sys_take_rf_received(state, buf, sizeof(buf), &len, &s_rx_meta)) {
    s_recibidos++;
    /* Para el registro de la SD: qué se hizo con él, por los contadores */
    const uint32_t denegados = s_denegados;
    const uint32_t desconocidos = s_desconocidos;
    aspid_rx_resultado_t res = ASPID_RX_EJECUTADO;

    if ((len == UPLINK_CMD_BYTES) && (buf[0] == UPLINK_MARCA) &&
        (buf[3] == (uint8_t)(buf[0] ^ buf[1] ^ buf[2]))) {
      _ejecutar(state, buf[1], buf[2]);
    } else if ((len == UPLINK_CMD_BYTES_16) && (buf[0] == UPLINK_MARCA) &&
               (buf[4] == (uint8_t)(buf[0] ^ buf[1] ^ buf[2] ^ buf[3]))) {
      _ejecutar(state, buf[1], (uint16_t)(buf[2] | ((uint16_t)buf[3] << 8)));
    } else {
      s_rechazados++;
      res = ASPID_RX_MAL_FORMADO;
    }
    if (s_desconocidos != desconocidos) {
      res = ASPID_RX_DESCONOCIDO;
    } else if (s_denegados != denegados) {
      res = ASPID_RX_DENEGADO;
    }
    aspid_data_rx(buf, len, &s_rx_meta, res, (uint8_t)s_ultimo_denegado);
  }

  _vigilar_test();
  _vigilar_simulacion();
}

bool uplink_modo_test(void) { return s_test; }

uint32_t uplink_test_restante_s(void) {
  if (!s_test) {
    return 0U;
  }
  const int32_t resta = (int32_t)(s_test_hasta_ms - HAL_GetTick());
  return (resta > 0) ? ((uint32_t)resta + 999U) / 1000U : 0U;
}

uint8_t uplink_ultimo_piro(void) { return s_ultimo_piro; }
uint8_t uplink_ultima_prueba_aerofreno(void) {
  return s_ultima_prueba_aerofreno;
}
uint32_t uplink_piros_aceptados(void) { return s_piros_aceptados; }

uint32_t uplink_recibidos(void) { return s_recibidos; }
uint32_t uplink_validos(void) { return s_validos; }
uint32_t uplink_rechazados(void) { return s_rechazados; }
uint32_t uplink_denegados(void) { return s_denegados; }
uint32_t uplink_desconocidos(void) { return s_desconocidos; }
uplink_denegado_t uplink_ultimo_denegado(void) { return s_ultimo_denegado; }
uint8_t uplink_ultimo_cmd(void) { return s_ultimo_cmd; }
uint32_t uplink_ultimo_ms(void) { return s_ultimo_ms; }
sys_rf_rx_meta_t uplink_ultima_recepcion(void) { return s_rx_meta; }
