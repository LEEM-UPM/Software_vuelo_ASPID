#include "debug_console.h"

#include <math.h>
#include <stdbool.h>

#include "can_driver.h"
#include "can_protocol/can_protocol_version.h"
#include "debug_out.h"
#include "fx_api.h"
#include "airbrake.h"
#include "aspid_data.h"
#include "camaras.h"
#include "gnc.h"
#include "rf_telemetry.h"
#include "status_packet.h"
#include "uplink.h"
#include "baro_alt.h"
#include "despliegue.h"
#include "emergencia.h"
#include "memoria_vuelo.h"
#include "lanzamiento.h"
#include "gnc_playback.h"
#include "main.h"
#include "sensor_board.h"
#include "sounds.h"
#include "storage.h"

#if DEBUG_CONSOLE_USE_COLOR
#define C_RESET "\033[0m"
#define C_TITLE "\033[1;36m" /* cian negrita */
#define C_OK    "\033[32m"   /* verde */
#define C_BAD   "\033[31m"   /* rojo */
#define C_WARN  "\033[33m"   /* amarillo */
#define C_DIM   "\033[90m"   /* gris */
#else
#define C_RESET ""
#define C_TITLE ""
#define C_OK    ""
#define C_BAD   ""
#define C_WARN  ""
#define C_DIM   ""
#endif

#define LINE_THICK "=================================================================\r\n"
#define LINE_THIN  "-----------------------------------------------------------------\r\n"

#define P DebugOut_Printf

static uint32_t _last_report_ms;
static uint32_t _report_count;

/* ===========================================================================
 * Formato de valores
 * ======================================================================== */

/* "OK" en verde / "--" en rojo */
static const char *_flag(bool ok) { return ok ? C_OK "OK" C_RESET : C_BAD "--" C_RESET; }

/* Antigüedad de un dato: "hace 0.4 s" o "sin datos" */
static void _age(bool valid, uint32_t rx_ms) {
  if (!valid) {
    P(C_DIM "(sin datos)" C_RESET "\r\n");
    return;
  }
  uint32_t age_ms = HAL_GetTick() - rx_ms;
  P("%s(hace %lu.%lu s)" C_RESET "\r\n", (age_ms > 3000U) ? C_WARN : C_DIM,
    (unsigned long)(age_ms / 1000U), (unsigned long)((age_ms % 1000U) / 100U));
}

/* Campo uint16 de POWER_RAILS: número o "----" si es 0xFFFF */
static void _rail_field(uint16_t value) {
  if (value == SYS_POWER_RAILS_INVALID) {
    P(C_BAD "  ----" C_RESET);
  } else {
    P("%6u", value);
  }
}

/* Terna X Y Z en float; NaN -> "---" */
static void _xyz(const float v[3], int decimals) {
  for (int i = 0; i < 3; i++) {
    if (isnan(v[i])) {
      P(C_BAD "    ---   " C_RESET);
    } else {
      P(" %+9.*f", decimals, (double)v[i]);
    }
  }
}

/* ===========================================================================
 * Secciones del informe
 * ======================================================================== */

static void _section_boards(const sys_state *s) {
  P(" " C_TITLE "PLACAS" C_RESET "    power %s   avionica %s   rf %s\r\n",
    _flag(s->power), _flag(s->avionics), _flag(s->rf));
}

/* Componentes de cada placa (INIT_STATUS, ICD 6.11) */
typedef struct {
  uint32_t bit;
  const char *name;
} _component_t;

static const _component_t _avionics_components[] = {
    {SYS_AVIONICS_INIT_BARO1, "baro1"},
    {SYS_AVIONICS_INIT_BARO2, "baro2"},
    {SYS_AVIONICS_INIT_IMU1, "imu1"},
    {SYS_AVIONICS_INIT_IMU2, "imu2"},
    {SYS_AVIONICS_INIT_MAG, "mag"},
    {SYS_AVIONICS_INIT_GPS, "gps"},
    {SYS_AVIONICS_INIT_VECTORNAV, "vn100"},
};

/* Una línea con cada componente y si ha arrancado. Los bits que la placa no
 * tiene (fuera de supported) no se enseñan. */
static void _components_line(const char *board, const sys_init_status *st,
                             const _component_t *table, uint32_t count) {
  P("   %-9s", board);

  if (!st->valid) {
    P(C_DIM "(sin INIT_STATUS)" C_RESET "\r\n");
    return;
  }

  for (uint32_t i = 0U; i < count; i++) {
    if ((st->supported & table[i].bit) == 0U) {
      continue;
    }
    const bool ok = (st->ok & table[i].bit) != 0U;
    P(" %s%s%s" C_RESET, ok ? C_OK : C_BAD, ok ? "" : "!", table[i].name);
  }

  P("\r\n");
}

static void _section_components(const sys_state *s) {
  P(LINE_THIN);
  P(" " C_TITLE "COMPONENTES INICIADOS" C_RESET "\r\n");
  _components_line("avionica", &s->avionics_init, _avionics_components,
                   sizeof(_avionics_components) /
                       sizeof(_avionics_components[0]));

  /* power y rf todavía no mandan INIT_STATUS: se enseña el mapa en crudo */
  if (s->power_init.valid) {
    P("   %-9s ok 0x%08lX / tiene 0x%08lX\r\n", "power",
      (unsigned long)s->power_init.ok, (unsigned long)s->power_init.supported);
  }
  if (s->rf_init.valid) {
    P("   %-9s ok 0x%08lX / tiene 0x%08lX\r\n", "rf",
      (unsigned long)s->rf_init.ok, (unsigned long)s->rf_init.supported);
  }
}

static void _section_power(const sys_state *s) {
  P(LINE_THIN);
  P(" " C_TITLE "POWER" C_RESET "  railes ");
  _age(s->power_rails_valid, s->power_rails_rx_ms);

  const sys_power_rails *r = &s->power_rails;
  P("          tension [mV]  corriente [mA]\r\n");
  P("   3V3  ");
  _rail_field(r->voltage_3v3_mv);
  P("        ");
  _rail_field(r->current_3v3_ma);
  P("\r\n   5V   ");
  _rail_field(r->voltage_5v_mv);
  P("        ");
  _rail_field(r->current_5v_ma);
  P("\r\n   11V  ");
  _rail_field(r->voltage_11v_mv);
  P("        ");
  _rail_field(r->current_11v_ma);
  P("\r\n");

  P("   Pirotecnicos (continuidad) ");
  if (s->pyro_continuity_valid) {
    P(" 1A %s  1B %s  2A %s  2B %s  ", _flag(s->pyro_continuity & SYS_PYRO_1A),
      _flag(s->pyro_continuity & SYS_PYRO_1B),
      _flag(s->pyro_continuity & SYS_PYRO_2A),
      _flag(s->pyro_continuity & SYS_PYRO_2B));
  }
  _age(s->pyro_continuity_valid, s->pyro_continuity_rx_ms);
}

static void _section_calibration(const sys_state *s) {
  static const char *const names[] = {
      [SYS_CALIBRATION_IDLE] = C_DIM "no se pide (los sesgos los estima el GNC)" C_RESET,
      [SYS_CALIBRATION_WAITING] = C_WARN "en curso..." C_RESET,
      [SYS_CALIBRATION_DONE] = C_OK "hecha" C_RESET,
      [SYS_CALIBRATION_TIMEOUT] = C_BAD "sin respuesta (timeout)" C_RESET,
  };

  P(LINE_THIN);
  P(" " C_TITLE "CALIBRACION AVIONICA" C_RESET "  %s\r\n",
    names[s->avionics_calibration_status]);

  if (s->avionics_calibration_status != SYS_CALIBRATION_DONE) {
    return;
  }

  const sys_avionics_calibration *c = &s->avionics_calibration;
  P("                     X          Y          Z\r\n");
  P("   IMU1 acc [m/s2] ");
  _xyz(c->imu1_accel_mps2, 3);
  P("\r\n   IMU1 gyr [rad/s]");
  _xyz(c->imu1_gyro_radps, 4);
  P("\r\n   IMU2 acc [m/s2] ");
  _xyz(c->imu2_accel_mps2, 3);
  P("\r\n   IMU2 gyr [rad/s]");
  _xyz(c->imu2_gyro_radps, 4);
  P("\r\n   Presion base [kPa]   baro1 %.3f   baro2 %.3f\r\n",
    (double)c->baro1_kpa, (double)c->baro2_kpa);
}

/* Coordenada en 1e-7 grados: "+40.4167890" */
static void _deg_e7(int32_t v) {
  if (v == SENSOR_BOARD_GPS_INVALID_I32) {
    P(C_BAD "     ---    " C_RESET);
    return;
  }
  const uint32_t a = (v < 0) ? (uint32_t)(-(int64_t)v) : (uint32_t)v;
  P("%c%3lu.%07lu", (v < 0) ? '-' : '+', (unsigned long)(a / 10000000UL),
    (unsigned long)(a % 10000000UL));
}

/* Milímetros con signo: "650.123" */
static void _mm(int32_t v) {
  if (v == SENSOR_BOARD_GPS_INVALID_I32) {
    P(C_BAD "---" C_RESET);
    return;
  }
  const uint32_t a = (v < 0) ? (uint32_t)(-(int64_t)v) : (uint32_t)v;
  P("%s%lu.%03lu", (v < 0) ? "-" : "", (unsigned long)(a / 1000UL),
    (unsigned long)(a % 1000UL));
}

static void _utc(uint32_t ms) {
  if (ms == SENSOR_BOARD_GPS_INVALID_TIME) {
    P(C_BAD "--:--:--.-" C_RESET);
    return;
  }
  P("%02lu:%02lu:%02lu.%lu", (unsigned long)(ms / 3600000UL),
    (unsigned long)((ms / 60000UL) % 60UL), (unsigned long)((ms / 1000UL) % 60UL),
    (unsigned long)((ms % 1000UL) / 100UL));
}

/* Tramas recibidas desde el informe anterior (un informe por segundo = Hz) */
static uint32_t _rate(uint32_t count, uint32_t *last) {
  const uint32_t r = count - *last;
  *last = count;
  return r;
}

static void _section_gps(const sensor_board_data_t *d) {
  static uint32_t last_gga;
  static uint32_t last_rmc;
  static uint32_t last_pvt;
  const gps_gga_data_t *g = &d->gps_gga;
  const gps_rmc_data_t *r = &d->gps_rmc;
  const gps_pvt_data_t *v = &d->gps_pvt;

  if ((g->count == 0U) && (r->count == 0U) && (v->count == 0U)) {
    P("   GPS      " C_DIM "(sin datos)" C_RESET "\r\n");
    return;
  }

  P("   GPS GGA  fix %s%u" C_RESET "  sat %2u  hdop ", (g->fix_quality > 0U) ? C_OK : C_BAD,
    g->fix_quality, g->satellites);
  if (g->hdop_x100 == SENSOR_BOARD_GPS_INVALID_U16) {
    P(C_BAD "----" C_RESET);
  } else {
    P("%u.%02u", g->hdop_x100 / 100U, g->hdop_x100 % 100U);
  }
  P("  lat ");
  _deg_e7(g->latitude_e7);
  P("  lon ");
  _deg_e7(g->longitude_e7);
  P("  alt ");
  _mm(g->altitude_mm);
  P(" m  ");
  _utc(g->time_ms);
  P(" UTC  (%lu, %lu Hz)\r\n", (unsigned long)g->count,
    (unsigned long)_rate(g->count, &last_gga));

  P("   GPS RMC  %s" C_RESET "  %02u/%02u/%04u  vel ", r->valid ? C_OK "valido" : C_BAD "no valido",
    r->day, r->month, r->year);
  _mm(r->speed_mmps);
  P(" m/s  rumbo ");
  if (r->course_cdeg == SENSOR_BOARD_GPS_INVALID_U16) {
    P(C_BAD "---" C_RESET);
  } else {
    P("%u.%02u", r->course_cdeg / 100U, r->course_cdeg % 100U);
  }
  P("  ");
  _utc(r->time_ms);
  P(" UTC  (%lu, %lu Hz)\r\n", (unsigned long)r->count,
    (unsigned long)_rate(r->count, &last_rmc));
  /* La unica fuente de velocidad vertical: sin ella el EKF no ve subir el
   * cohete (ver sensor_board.h, 0x108). */
  P("   GPS vel  N %+7.2f  E %+7.2f  D %+7.2f m/s   fix %uD  sat %2u  %s" C_RESET,
    (double)v->vel_n_mmps * 1e-3, (double)v->vel_e_mmps * 1e-3,
    (double)v->vel_d_mmps * 1e-3, (unsigned)v->fix_type,
    (unsigned)v->satellites,
    v->valid ? C_OK "valido" : C_BAD "no valido");
  if (v->count == 0U) {
    P(C_BAD "   SIN NAV-PVT" C_RESET);
  }
  P("   (%lu, %lu Hz)\r\n", (unsigned long)v->count,
    (unsigned long)_rate(v->count, &last_pvt));
}

static void _section_sensor_board(void) {
  const sensor_board_data_t *d = sensor_board_get_data();
  const uint32_t errors =
      d->err_unknown_id + d->err_short_frame + d->err_id_mismatch;

  P(LINE_THIN);
  P(" " C_TITLE "PLACA DE SENSORES (CAN2)" C_RESET "  resets %lu  errores %s%lu" C_RESET
    "\r\n",
    (unsigned long)d->board_resets, (errors > 0U) ? C_BAD : "",
    (unsigned long)errors);

  for (int i = 0; i < 2; i++) {
    P("   Baro%d  %9.3f kPa  %6.2f C          (%lu tramas)\r\n", i + 1,
      (double)d->baro[i].pressure_kpa, (double)d->baro[i].temperature_c,
      (unsigned long)d->baro[i].count);
  }
  for (int i = 0; i < 2; i++) {
    P("   IMU%d acc [m/s2] ", i + 1);
    _xyz(d->imu[i].accel_mps2, 3);
    P("  (%lu)\r\n   IMU%d gyr [rad/s]", (unsigned long)d->imu[i].count, i + 1);
    _xyz(d->imu[i].gyro_rps, 4);
    P("\r\n");
  }
  P("   Mag  [uT]       ");
  const float mag_ut[3] = {d->mag.field_t[0] * 1e6f, d->mag.field_t[1] * 1e6f,
                           d->mag.field_t[2] * 1e6f};
  _xyz(mag_ut, 2);
  P("  (%lu)\r\n", (unsigned long)d->mag.count);
  _section_gps(d);
}

/* Errores de FileX más habituales */
static const char *_fx_error_name(uint32_t e) {
  switch (e) {
  case FX_BOOT_ERROR:      return "boot record no valido";
  case FX_MEDIA_INVALID:   return "formato no soportado (exFAT? usar FAT32)";
  case FX_FAT_READ_ERROR:  return "error leyendo la FAT";
  case FX_NOT_FOUND:       return "archivo no encontrado";
  case FX_NO_MORE_SPACE:   return "tarjeta llena";
  case FX_INVALID_NAME:    return "nombre no valido";
  case FX_IO_ERROR:        return "error de E/S (SDMMC)";
  default:                 return "";
  }
}

static void _section_storage(void) {
  P(LINE_THIN);
  P(" " C_TITLE "TARJETAS SD" C_RESET "\r\n");

  for (int i = 0; i < (int)SD_CARD_COUNT; i++) {
    const storage_sd_info_t *sd = storage_get_info((sd_card_id_t)i);
    /* Reloj del bus: 20 MHz (PLL1Q) / (2 * div), div 0 = sin dividir */
    const uint32_t div = sd_card_get_clock_div((sd_card_id_t)i);
    const uint32_t bus_khz = (div == 0U) ? 20000U : (10000U / div);
    P("   SD%d  CD=%d  ", i + 1, sd->cd_pin_high ? 1 : 0);

    switch (sd->state) {
    case STORAGE_SD_MOUNTED:
      P(C_OK "montada" C_RESET "  %lu MB, libres %lu MB   arranques %lu   test %s"
        C_RESET " (%lu ms)   bus %lu kHz\r\n",
        (unsigned long)sd->capacity_mb, (unsigned long)sd->free_mb,
        (unsigned long)sd->boot_count,
        (sd->self_test == STORAGE_TEST_OK) ? C_OK "OK" : C_BAD "FALLO",
        (unsigned long)sd->self_test_ms, (unsigned long)bus_khz);
      break;
    case STORAGE_SD_INIT_ERROR:
      P(C_BAD "no responde" C_RESET " (sin tarjeta?)\r\n");
      break;
    case STORAGE_SD_MOUNT_ERROR:
      P(C_BAD "no se puede montar" C_RESET "\r\n");
      break;
    default:
      P(C_DIM "sin iniciar" C_RESET "\r\n");
      break;
    }

    if ((sd->self_test == STORAGE_TEST_FAILED) && (sd->fx_error == 0U)) {
      P("        " C_WARN "lo releido no coincide con lo escrito: la tarjeta acepta"
        " escrituras pero no las guarda (gastada / solo lectura?)" C_RESET "\r\n");
    }
    if ((sd->fx_error != 0U) || (sd->hal_error != 0U)) {
      P("        " C_WARN "ultimo error: FileX 0x%02lX %s  HAL SD 0x%08lX" C_RESET "\r\n",
        (unsigned long)sd->fx_error, _fx_error_name(sd->fx_error),
        (unsigned long)sd->hal_error);
    }
  }
}

/* Nombres de los estados y fases del GNC (gnc.h; iguales en la v1.2 y la
 * v1.5) */
static const char *_rocket_state_name(int32_t state) {
  switch (state) {
  case GNC_ESTADO_POWER_ON:      return "POWER_ON";
  case GNC_ESTADO_MANTENIMIENTO: return "MANTENIMIENTO";
  case GNC_ESTADO_EN_VUELO:      return "EN VUELO";
  case GNC_ESTADO_ERROR:         return "ERROR";
  default:                       return "?";
  }
}

static const char *_flight_phase_name(int32_t phase) {
  switch (phase) {
  case GNC_FASE_IDLE:      return "IDLE";
  case GNC_FASE_BOOST:     return "BOOST";
  case GNC_FASE_COAST:     return "COAST (sin control)";
  case GNC_FASE_CONTROL:   return "COAST (con control)";
  case GNC_FASE_APOGEO:    return "APOGEO";
  case GNC_FASE_DROGUE:    return "DROGUE";
  case GNC_FASE_MAIN:      return "MAIN";
  case GNC_FASE_EN_TIERRA: return "EN TIERRA";
  default:                 return "?";
  }
}

/* Registro de la sesión en la SD1 (aspid_data.c) */
static void _section_aspid(void) {
  const aspid_data_info_t *a = aspid_data_info();
  P(LINE_THIN);
  P(" " C_TITLE "REGISTRO SD1" C_RESET "  %s   %s\r\n",
    (a->carpeta[0] != '\0') ? a->carpeta : "(sin carpeta)",
    a->hora_conocida ? C_OK "hora UTC del GPS" C_RESET
                     : C_WARN "sin hora (esperando al GPS)" C_RESET);
  switch (a->estado) {
  case ASPID_ESPERANDO:
    P("   telemetria grabando, vuelo " C_OK "esperando el despegue" C_RESET);
    break;
  case ASPID_GRABANDO:
    P("   " C_WARN "GRABANDO %s" C_RESET " %s  %lu filas (%.1f s), altura max "
      "%.0f m",
      a->grabando_simulacion ? "SIMULACION" : "VUELO", a->archivo,
      (unsigned long)a->filas, (double)a->filas / (double)ASPID_DATA_HZ,
      (double)a->altura_max_m);
    break;
  case ASPID_SELLADO:
    P("   " C_OK "SELLADA" C_RESET ": hubo vuelo, todo cerrado, la SD no se "
      "toca hasta el proximo arranque");
    break;
  case ASPID_SIN_SD:
  default:
    if (a->tarjeta_perdida) {
      P("   " C_BAD "TARJETA PERDIDA" C_RESET ": fallo una escritura por DMA,"
        " no se vuelve a tocar");
    } else {
      P("   " C_BAD "NO SE GRABA" C_RESET " (SD1 sin montar o %lu fallos)",
        (unsigned long)a->errores);
    }
    break;
  }
  P("   vuelos %lu, simulaciones %lu\r\n", (unsigned long)a->vuelos,
    (unsigned long)a->simulaciones);
  P("   telemetria %lu lineas (%lu KB)   vuelo %lu filas (%lu KB)   trust "
    "curve %lu filas (%lu KB)%s\r\n",
    (unsigned long)a->lineas_telemetria,
    (unsigned long)(a->bytes_telemetria / 1024U),
    (unsigned long)a->filas_total, (unsigned long)(a->bytes_vuelo / 1024U),
    (unsigned long)a->filas_empuje, (unsigned long)(a->bytes_empuje / 1024U),
    a->empuje_grabando ? C_WARN " GRABANDO" C_RESET : "");
  P("   eventos %lu   salud %lu filas\r\n", (unsigned long)a->eventos,
    (unsigned long)a->filas_salud);
  P("   DMA: %lu escrituras, la mas lenta %.1f ms (no bloquea)   FileX max "
    "%.1f ms   fila %lu us   perdido: %lu filas, %lu B",
    (unsigned long)a->escrituras, (double)a->escritura_max_us / 1000.0,
    (double)a->filex_max_us / 1000.0, (unsigned long)a->formato_max_us,
    (unsigned long)a->filas_perdidas, (unsigned long)a->bytes_perdidos);
  if (a->errores > 0U) {
    P("   " C_BAD "errores %lu (ultimo 0x%02lX)" C_RESET,
      (unsigned long)a->errores, (unsigned long)a->ultimo_error);
  }
  {
    const sd_card_tiempos_t t = sd_card_tiempos(SD_CARD_1);
    P("\r\n   HAL max: lanzar %lu us, estado %lu us, IRQ %lu us",
      (unsigned long)t.inicio_max_us, (unsigned long)t.estado_max_us,
      (unsigned long)t.irq_max_us);
  }
  if (a->reservas_liberadas > 0U) {
    P("   liberadas %lu reservas de la sesion anterior",
      (unsigned long)a->reservas_liberadas);
  }
  P("\r\n");
}

static void _section_gnc(void) {
  const gnc_data_t *g = gnc_get_data();

  P(LINE_THIN);
  P(" " C_TITLE "GNC" C_RESET "  %s", _rocket_state_name(g->rocket_state));
  if (g->error_code != 0U) {
    P(C_BAD "  error 0x%02X" C_RESET, g->error_code);
  }
  P("   fase %s   nav %s   altura %.1f m AGL%s%s\r\n",
    _flight_phase_name(g->flight_phase), _flag(g->nav_valid),
    (double)g->alt_agl_m, g->pyro_armed ? C_WARN "   PIRO ARMADA" C_RESET : "",
    g->gnss_only ? "   solo GNSS" : "");

  /* v1.6: listo para volar = las tres referencias de la rampa. Es lo que
   * hay que mirar antes de armar el iniciador (nav_valid NO vale: es 1
   * tambien navegando degradado). */
  {
    const uint8_t f = g->nav_init_flags;
    P("   LISTO PARA VOLAR %s   alineado %s  0 AGL %s  origen GNSS %s%s"
      "   palabra 0x%08lX\r\n",
      g->nav_ready ? C_OK "SI" C_RESET : C_WARN "NO" C_RESET,
      _flag((f & GNC_NAV_INIT_ALINEADO) != 0U),
      _flag((f & GNC_NAV_INIT_BARO) != 0U),
      _flag((f & GNC_NAV_INIT_GNSS) != 0U),
      ((f & GNC_NAV_INIT_GNSS) == 0U) ? C_DIM " (sin fix)" C_RESET : "",
      (unsigned long)g->status_word);
    if ((f & GNC_NAV_INIT_DEGRADADO) != 0U) {
      P(C_BAD "   NAVEGANDO SIN HABER ALINEADO (degradado)" C_RESET "\r\n");
    }
  }

  P("   Posicion NED [m]  ");
  _xyz(g->pos_ned_m, 2);
  P("\r\n   Velocidad  [m/s]  ");
  _xyz(g->vel_ned_mps, 2);
  P("\r\n   Actitud    [deg]  ");
  _xyz(g->euler_rpy_deg, 2);
  P("   (roll, pitch, yaw)\r\n");

  /* La rampa con la que se alinea (o se alineó): ver gnc_set_rampa() */
  P("   Rampa      %5.2f grados desde la vertical, rumbo %6.2f, alabeo %s"
    "%7.2f%s",
    (double)g->rampa_vertical_deg, (double)g->rampa_rumbo_deg,
    g->rampa_alabeo_medido ? "medido " : "dado ", (double)g->rampa_alabeo_deg,
    g->nav_valid ? "   (alineado con esta)" : "");
  if (g->realineados > 0U) {
    P("   Realineado %lu veces (comando 10 de la v1.5, al cambiar la rampa)\r\n",
      (unsigned long)g->realineados);
  }
  if (g->rampa_cambiada) {
    P(C_WARN "   LA RAMPA HA CAMBIADO: %.2f / %.2f se usara al realinear"
      " (en tierra lo hace solo; si no, al arrancar)" C_RESET,
      (double)lanzamiento_grados(), (double)lanzamiento_rumbo_grados());
  }
  P("\r\n");

  /* Mientras el EKF no se alinea todo lo de arriba son los valores de
   * fabrica del modelo, asi que se dice que falta para conseguirlo. */
  if (!g->nav_valid) {
    const float accel_err = g->accel_norm_mps2 - 9.80665f;
    const bool accel_ok =
        ((accel_err < 0.0f ? -accel_err : accel_err) <= GNC_ALIGN_ACCEL_TOL_MPS2);
    const bool gyro_ok = (g->gyro_norm_dps <= GNC_ALIGN_GYRO_MAX_DPS);

    P("   Alineando  %3lu/%lu pasos   |a| %6.2f m/s2 %s   |w| %6.2f deg/s %s\r\n",
      (unsigned long)g->align_count, (unsigned long)GNC_ALIGN_STEPS,
      (double)g->accel_norm_mps2, accel_ok ? "ok " : "NO ",
      (double)g->gyro_norm_dps, gyro_ok ? "ok" : "NO");
    if (!accel_ok || !gyro_ok) {
      P(C_DIM "   (el cohete tiene que estar quieto 0,5 s seguidos: |a| a menos"
        " de %.1f de 9,81 y |w| por debajo de %.0f)" C_RESET "\r\n",
        (double)GNC_ALIGN_ACCEL_TOL_MPS2, (double)GNC_ALIGN_GYRO_MAX_DPS);
    } else {
      /* Quieto: lo que puede faltar es que la rampa cuadre con la gravedad.
       * La diferencia es entre VECTORES (como en nav.c), no solo entre
       * inclinaciones: una inclinación pequeña hacia otro lado también
       * cuenta. */
      const float dif = g->rampa_error_deg;
      P("   %sInclinacion medida %5.2f grados, rampa %5.2f: la gravedad esta"
        " a %5.2f grados de la esperada (max %.0f)%s" C_RESET "\r\n",
        (dif > GNC_ALIGN_ATT_MAX_DEG) ? C_BAD : C_DIM,
        (double)g->inclinacion_medida_deg, (double)g->rampa_vertical_deg,
        (double)dif, (double)GNC_ALIGN_ATT_MAX_DEG,
        (dif > GNC_ALIGN_ATT_MAX_DEG)
            ? "  NO SE PUEDE ALINEAR: corregir el angulo de rampa"
            : "");
    }
  }

  P("   Apogeo previsto %8.1f m   error %+8.1f m   GNC pide %.0f %%\r\n",
    (double)g->apogee_pred_m, (double)g->apogee_error_m,
    (double)(g->actuator_cmd * 100.0f));

  P("   %lu ticks a %u Hz (%lu de guiado), ultimo %lu us, max %lu us"
    " (%lu con guiado)%s\r\n",
    (unsigned long)g->steps, (unsigned)GNC_RATE_HZ,
    (unsigned long)g->gc_steps,
    (unsigned long)g->last_step_us, (unsigned long)g->max_step_us,
    (unsigned long)g->max_gc_step_us,
    (g->max_step_us >= GNC_PERIOD_US) ? C_BAD "  NO CABE EN UN TICK" C_RESET
                                      : "");
  P("   %sticks perdidos %lu%s, recuperados %lu   IMU1 sin dato nuevo en"
    " %lu%% de los ticks   %sIMU caida: %lu + %lu ticks%s   baro %lu\r\n",
    (g->overruns > 0U) ? C_WARN : "", (unsigned long)g->overruns, C_RESET,
    (unsigned long)g->ticks_recuperados,
    (unsigned long)((g->steps > 0U) ? (100U * g->imu1_sin_dato) / g->steps
                                    : 0U),
    ((g->imu1_caida + g->imu2_caida) > 0U) ? C_WARN : "",
    (unsigned long)g->imu1_caida, (unsigned long)g->imu2_caida, C_RESET,
    (unsigned long)g->baro_updates);
  P("   IMU1 %lu + IMU2 %lu muestras/tick  GPS %lu  %sfix %uD%s\r\n",
    (unsigned long)g->imu1_samples,
    (unsigned long)g->imu2_samples, (unsigned long)g->gps_updates,
    (g->gps_fix_type >= 3U) ? C_OK : C_DIM, (unsigned)g->gps_fix_type,
    (g->gps_fix_type >= 3U) ? " (el EKF lo usa)" C_RESET
                            : " (el EKF NO usa el GPS)" C_RESET);

  P("   Aerofreno %5.1f %%  (servo %3u/255)   %lu ordenes", 
    (double)(airbrake_last_command() * 100.0f),
    (unsigned)airbrake_last_byte(), (unsigned long)airbrake_sent());
  P(C_DIM "  %5.1f grados de servo" C_RESET, (double)airbrake_last_grados());
  if (airbrake_prueba_secuencia_activa()) {
    P(C_WARN "   PRUEBA DE TIERRA en curso" C_RESET);
  } else if (airbrake_prueba_activa()) {
    P(C_WARN "   EXTENSION FIJADA desde tierra" C_RESET);
  } else if ((g->rocket_state != GNC_ESTADO_EN_VUELO) &&
             !(airbrake_last_command() > 0.0f)) {
    P(C_DIM "   recogido: en el 0 %%, 4 mm fuera del tope (solo sale mas"
            " EN VUELO)" C_RESET);
  }
  if (airbrake_failed() > 0U) {
    P(C_WARN "   %lu sin enviar" C_RESET, (unsigned long)airbrake_failed());
  }
  P("\r\n");

  /*
   * Paquete de vuelo: se decodifica lo que REALMENTE ha salido al aire, no
   * los datos del GNC de arriba. Es lo unico que detecta un fallo de
   * codificacion (una escala mal puesta, un campo que satura), porque compara
   * el origen con el resultado en la misma pantalla. El volcado en hexadecimal
   * es para cotejarlo byte a byte con lo que reciba la estacion de tierra.
   */
  const uint8_t *pk = rf_telemetry_last_packet();
  P("   Radio: %lu enviados", (unsigned long)rf_telemetry_sent());
  if (rf_telemetry_failed() > 0U) {
    P(C_WARN "   %lu sin enviar" C_RESET, (unsigned long)rf_telemetry_failed());
  }
  {
    const bool vuelo = (rf_telemetry_cadencia_hz() == RF_TELEMETRY_HZ_VUELO);
    P("   %lu Hz (tocan %lu: %s)   paquete #%u\r\n",
      (unsigned long)rf_telemetry_measured_hz(),
      (unsigned long)rf_telemetry_cadencia_hz(),
      !vuelo ? "en tierra hasta el lanzamiento"
             : (rf_telemetry_lanzamiento_visto() ? "lanzamiento visto"
                                                 : "simulando"),
      (unsigned)pk[0]);
  }

  {
    const int16_t alt = (int16_t)((uint16_t)pk[3] | ((uint16_t)pk[4] << 8));
    const int16_t vv = (int16_t)((uint16_t)pk[9] | ((uint16_t)pk[10] << 8));
    P("   decodificado: alt %+8.2f m  vv %+7.2f m/s  fase %u  estado %u"
      "  err %u  nav %s\r\n",
      (double)alt * 0.25, (double)vv * 0.05, (unsigned)((pk[1] >> 4) & 0x07u),
      (unsigned)(pk[2] & 0x07u), (unsigned)(pk[2] >> 3),
      ((pk[1] & 0x80u) != 0u) ? "OK" : "--");
    P("   aerofreno %3u %%  apogeo %5u m  bateria %5.2f V  baro %+d C"
      "  pirotecnia %c%c%c%c\r\n",
      (unsigned)((pk[21] * 100u) / 250u), (unsigned)pk[20] * 25u,
      (double)pk[22] * 0.08, (int)(int8_t)pk[23],
      (pk[1] & 0x01u) ? '1' : '-', (pk[1] & 0x02u) ? '2' : '-',
      (pk[1] & 0x04u) ? '3' : '-', (pk[1] & 0x08u) ? '4' : '-');
    P("   en el aire:  ");
    for (uint32_t i = 0U; i < FLIGHT_PACKET_BYTES; i++) {
      P("%02X ", (unsigned)pk[i]);
    }
    P(" (%u B)\r\n", (unsigned)FLIGHT_PACKET_BYTES);
  }

  if (g->overruns > 0U) {
    P("   " C_WARN "%lu ventanas de 10 ms perdidas" C_RESET "\r\n",
      (unsigned long)g->overruns);
  }
}

static void _section_status(void) {
  P(LINE_THIN);
  P(" " C_TITLE "PAQUETE DE ESTADO Y ENLACE DE SUBIDA" C_RESET "\r\n");

  P("   Buses  CAN1 %4lu tramas/s   CAN2 %4lu tramas/s   %s\r\n",
    (unsigned long)status_packet_can_hz(0U),
    (unsigned long)status_packet_can_hz(1U),
    status_packet_medido() ? "" C_DIM "(medido)" C_RESET
                           : C_WARN "(sin medir todavia)" C_RESET);

  P("   Estado mandados %lu   proximo contador %u   paquetes de camaras "
    "mandados %lu\r\n",
    (unsigned long)rf_telemetry_estado_enviados(),
    (unsigned)status_packet_next_counter(),
    (unsigned long)rf_telemetry_camaras_enviados());

  const uint8_t *st = rf_telemetry_ultimo_estado();
  if (rf_telemetry_estado_enviados() > 0U) {
    P("   ultimo:  ");
    for (uint32_t i = 0U; i < STATUS_PACKET_BYTES; i++) {
      P("%02X ", (unsigned)st[i]);
    }
    P(" (%u B)\r\n", (unsigned)STATUS_PACKET_BYTES);
  } else {
    P("   " C_DIM "todavia no se ha mandado ninguno" C_RESET "\r\n");
  }

  /* Subida: tierra -> RF -> CAN1 (ICD 6.12) -> uplink.c */
  P("   Subida recibidos %lu   validos %lu   %srechazados %lu" C_RESET
    "   %sdenegados %lu" C_RESET "   desconocidos %lu\r\n",
    (unsigned long)uplink_recibidos(), (unsigned long)uplink_validos(),
    (uplink_rechazados() > 0U) ? C_WARN : "",
    (unsigned long)uplink_rechazados(),
    (uplink_denegados() > 0U) ? C_WARN : "",
    (unsigned long)uplink_denegados(), (unsigned long)uplink_desconocidos());

  if (uplink_ultimo_ms() > 0U) {
    P("   ultimo comando 0x%02X hace %.1f s", (unsigned)uplink_ultimo_cmd(),
      (double)(HAL_GetTick() - uplink_ultimo_ms()) / 1000.0);
    const sys_rf_rx_meta_t m = uplink_ultima_recepcion();
    if (m.valida) {
      P("   oido con RSSI %d dBm, SNR %d dB, error de frecuencia %+d Hz\r\n",
        (int)m.rssi_dbm, (int)m.snr_db, (int)m.df_hz);
    } else {
      P(C_DIM "   (la RF no manda como lo oyo: firmware anterior)" C_RESET
        "\r\n");
    }
  }

  static const char *const motivos[] = {
      "", "fuera de tierra", "sin modo test", "canal invalido",
      "power no responde", "simulando", "sin VUELO.BIN en la SD1",
      "angulo fuera de 0..90"};
  const uplink_denegado_t den = uplink_ultimo_denegado();
  if (den != UPLINK_DENEGADO_NADA) {
    P("   " C_WARN "ultimo denegado: %s" C_RESET "\r\n",
      ((uint32_t)den < (sizeof(motivos) / sizeof(motivos[0])))
          ? motivos[den] : "?");
  }

  if (gnc_playback_active()) {
    P("   " C_BAD ">>> SIMULANDO RUTA (VUELO.BIN), DATOS NO REALES <<<" C_RESET
      "   %u %%\r\n", (unsigned)gnc_playback_progress_pct());
  }

  P("   Rampa de lanzamiento %5.2f grados desde la vertical, rumbo %6.2f%s"
    "   %s%s\r\n",
    (double)lanzamiento_grados(), (double)lanzamiento_rumbo_grados(),
    lanzamiento_rumbo_definido() ? "" : " (sin definir)",
    lanzamiento_definido() ? "origen " : "",
    lanzamiento_definido() ? lanzamiento_origen() : "sin definir, vertical");

  {
    static const char *const origen[] = {"desde el arranque", "por el despegue",
                                         "desde tierra",
                                         "al terminar el vuelo"};
    const uint32_t ahora = HAL_GetTick();
    P("   Pitido de vuelo  %s\r\n",
      sounds_pitido_activo() ? C_WARN "PITANDO (1 por segundo: estado de vuelo)"
                               C_RESET
                             : "no (esperando en tierra)");
    P("   Camaras (PA2)  %s%s  %s, hace %lu s   hora para su reloj: %s   "
      "cortes por plazo %lu\r\n",
      camaras_encendidas() ? C_OK "ENCENDIDAS" C_RESET : "apagadas",
      camaras_apagando() ? C_WARN " APAGANDO (parando)" C_RESET : "",
      origen[camaras_origen()],
      (unsigned long)((ahora - camaras_cambio_ms()) / 1000U),
      camaras_hora_gps() ? "si" : "no",
      (unsigned long)camaras_cortes_por_plazo());
    for (uint32_t i = 0U; i < CAMARAS_N; i++) {
      const camara_t *c = camaras_camara(i);
      const camara_estado_t e = camara_estado(c);
      const char *col = (e == CAMARA_GRABANDO) ? C_OK
                        : ((e == CAMARA_NO_CONTESTA) || c->no_graba) ? C_BAD
                        : (e == CAMARA_SIN_CORRIENTE) ? ""
                                                      : C_WARN;
      const uint32_t g = camara_segundos_grabando(c, ahora);
      P("   Camara %u (UART%u) %s%s%s" C_RESET, (unsigned)c->num,
        (c->num == 1U) ? 8U : 9U, col, camara_estado_nombre(e),
        c->no_graba ? " - NO GRABA" : "");
      if ((e == CAMARA_GRABANDO) || (e == CAMARA_PARANDO)) {
        P(" %lu:%02lu", (unsigned long)(g / 60U), (unsigned long)(g % 60U));
      }
      if (c->ajustes_leidos || c->sd_leida) {
        P("  %s %s  SD %lu/%lu MB, quedan %lu min", c->resolucion,
          c->pal ? "PAL" : "NTSC", (unsigned long)c->libre_mb,
          (unsigned long)c->total_mb, (unsigned long)(c->queda_s / 60U));
      }
      if (c->id_hora != 0xFFU) {
        P("  reloj %s%s", c->hora_camara, c->reloj_ok ? " (puesto)" : "");
      }
      P("  boton %lu\r\n", (unsigned long)c->pulsaciones);
      const runcam_t *r = &c->rc;
      P("     (protocolo %u, funciones 0x%04X, mandados %lu, no cabian %lu, "
        "rx %lu B, respuestas %lu, sin respuesta %lu, perdidas %lu, crc %lu, "
        "uart %lu, sin pedir %lu, escrituras rechazadas %lu",
        (unsigned)r->version, (unsigned)r->funciones,
        (unsigned long)r->enviados, (unsigned long)r->no_cabia,
        (unsigned long)r->bytes_rx, (unsigned long)r->respuestas,
        (unsigned long)c->sin_respuesta, (unsigned long)c->perdidas,
        (unsigned long)r->errores_crc, (unsigned long)r->errores_uart,
        (unsigned long)r->ignorados,
        (unsigned long)c->escrituras_rechazadas);
      if (r->n_ultimos > 0U) {
        P(", ultimos:");
        for (uint32_t k = 0U; k < r->n_ultimos; k++) {
          P(" %02X", (unsigned)r->ultimos[k]);
        }
      }
      P(")\r\n");
    }
  }

  /* Despliegue de paracaidas, si ha pasado algo en este vuelo */
  if (despliegue_drogue_mandado() || despliegue_main_mandado() ||
      (despliegue_fallos() > 0U)) {
    P("   " C_BAD "DESPLIEGUE" C_RESET "  drogue 1A: %lu  1B: %lu ordenes   "
      "main 2A: %lu ordenes   %srechazadas %lu" C_RESET "\r\n",
      (unsigned long)despliegue_ordenes_1a(),
      (unsigned long)despliegue_ordenes_1b(),
      (unsigned long)despliegue_ordenes_2a(),
      (despliegue_fallos() > 0U) ? C_WARN : "",
      (unsigned long)despliegue_fallos());
  }

  /* Modo test: lo primero que tiene que saltar a la vista */
  if (uplink_modo_test()) {
    P("   " C_BAD ">>> MODO TEST ACTIVO <<<" C_RESET
      "   quedan %lu s   pirotecnicos aceptados %lu   ultimo %s\r\n",
      (unsigned long)uplink_test_restante_s(),
      (unsigned long)uplink_piros_aceptados(),
      (uplink_ultimo_piro() == 1U) ? "1A" :
      (uplink_ultimo_piro() == 2U) ? "1B" :
      (uplink_ultimo_piro() == 3U) ? "2A" :
      (uplink_ultimo_piro() == 4U) ? "2B" : "ninguno");
  } else if (uplink_piros_aceptados() > 0U) {
    P("   modo test inactivo   pirotecnicos aceptados %lu\r\n",
      (unsigned long)uplink_piros_aceptados());
  }
}

static void _section_can(void) {
  P(LINE_THIN);
  for (int port = 0; port < (int)CAN_PORT_COUNT; port++) {
    can_driver_stats_t st;
    can_driver_get_stats((can_port_t)port, &st);
    const bool bad = (st.rx_overflows + st.fifo_msg_lost + st.tx_errors +
                      st.bus_off) > 0U;
    P(" " C_TITLE "CAN%d" C_RESET "  rx %-9lu %sdesbordes %lu  perdidas %lu  errores tx %lu"
      "  bus-off %lu" C_RESET "\r\n",
      port + 1, (unsigned long)st.rx_frames, bad ? C_WARN : "",
      (unsigned long)st.rx_overflows, (unsigned long)st.fifo_msg_lost,
      (unsigned long)st.tx_errors, (unsigned long)st.bus_off);
  }
  const uint32_t dropped = DebugOut_DroppedBytes();
  if (dropped > 0U) {
    P(" " C_WARN "Debug USB: %lu bytes descartados (sin PC o buffer lleno)" C_RESET "\r\n",
      (unsigned long)dropped);
  }
}

/* ===========================================================================
 * API
 * ======================================================================== */

static const char *_fase_nombre(int32_t f) {
  static const char *const n[] = {"IDLE",   "BOOST",  "COAST", "CONTROL",
                                  "APOGEO", "DROGUE", "MAIN",  "EN TIERRA"};
  return ((f >= 0) && (f <= 7)) ? n[f] : "?";
}

/* Memoria de vuelo y, si está, el modo de emergencia */
static void _section_vuelo(void) {
  const emergencia_modo_t modo = emergencia_modo();
  const memoria_vuelo_t *m = emergencia_memoria();
  P(LINE_THIN);
  P(" " C_TITLE "MEMORIA DE VUELO" C_RESET "  %s%s" C_RESET
    "   (reset: %s)   %s\r\n",
    (modo == EMERGENCIA_EN_VUELO) ? C_BAD
    : (modo == EMERGENCIA_POSTVUELO) ? C_WARN
                                     : C_OK,
    emergencia_modo_nombre(modo),
    memoria_vuelo_causa_nombre(memoria_vuelo_causa()), emergencia_motivo());
  P("   vuelo #%u  %s  fase %s  %s%s%s%s%s  t %.1f s  reinicios en vuelo %u\r\n",
    (unsigned)m->vuelo, ((m->flags & MV_DESPEGUE) != 0U) ? "DESPEGADO" : "en tierra",
    _fase_nombre(m->fase), ((m->flags & MV_APOGEO) != 0U) ? "apogeo " : "",
    ((m->flags & MV_DROGUE) != 0U) ? "drogue " : "",
    ((m->flags & MV_MAIN) != 0U) ? "main " : "",
    ((m->flags & MV_TERMINADO) != 0U) ? "TERMINADO " : "",
    ((m->flags & MV_EMERGENCIA) != 0U) ? "(paso por emergencia)" : "",
    (double)m->t_vuelo_ms / 1000.0, (unsigned)m->reinicios);
  P("   suelo baro1 %.0f Pa  baro2 %.0f Pa   maxima %.1f / %.1f m   flash: "
    "%lu de %lu, %lu escritos (max %lu us), %lu fallos, %lu malos, %lu "
    "borrados%s\r\n",
    (double)m->p0_pa[0], (double)m->p0_pa[1], (double)m->max_m[0],
    (double)m->max_m[1], (unsigned long)memoria_vuelo_usados(),
    (unsigned long)MEMORIA_VUELO_REGISTROS,
    (unsigned long)memoria_vuelo_escritos(),
    (unsigned long)memoria_vuelo_escritura_max_us(),
    (unsigned long)memoria_vuelo_fallos(), (unsigned long)memoria_vuelo_malos(),
    (unsigned long)memoria_vuelo_borrados(),
    memoria_vuelo_disponible() ? "" : C_BAD "  NO DISPONIBLE" C_RESET);
  if (memoria_vuelo_borrados() > 0U) {
    P("   (sector borrado al arrancar en %lu ms, todo parado mientras)\r\n",
      (unsigned long)memoria_vuelo_borrado_ms());
  }
  if ((modo != EMERGENCIA_EN_VUELO) && ((m->flags & MV_DESPEGUE) == 0U)) {
    return;
  }
  for (uint32_t k = 0U; k < BARO_N; k++) {
    const baro_canal_t *c = baro_alt_canal(k);
    P("   baro%lu %s%-9s" C_RESET " h %8.1f m  vz %+7.1f m/s  max %8.1f m  "
      "ruido %6.2f m  sucias %lu de %lu\r\n",
      (unsigned long)(k + 1U), c->fiable ? C_OK : C_WARN,
      c->fiable ? "FIABLE" : "no fiable", (double)c->h_m, (double)c->vz_mps,
      (double)c->max_m, (double)c->ruido_m, (unsigned long)c->sucias,
      (unsigned long)c->muestras);
  }
  if (modo == EMERGENCIA_EN_VUELO) {
    const gnc_data_t *g = emergencia_gnc();
    P("   " C_BAD "EMERGENCIA" C_RESET "  fase %s  altura %.1f m  apogeo %s  "
      "main %s  drogue %s  main mandado %s  aerofreno %.0f %%%s\r\n",
      _fase_nombre(g->flight_phase), (double)g->alt_agl_m,
      emergencia_apogeo_detectado() ? C_BAD "DETECTADO" C_RESET : "no",
      emergencia_main_alcanzado() ? C_BAD "400 m" C_RESET : "no",
      emergencia_drogue_hecho() ? "si" : "no",
      emergencia_main_hecho() ? "si" : "no",
      (double)(emergencia_freno() * 100.0f),
      emergencia_motor() ? "  (MOTOR: no se abre)" : "");
  }
}

void DebugConsole_Banner(void) {
  P("\r\n\r\n" C_TITLE LINE_THICK);
  P("   LEEM  -  CORE H7  (ordenador de vuelo)\r\n");
  P(LINE_THICK C_RESET);
  P("   Compilado      " __DATE__ "  " __TIME__ "\r\n");
  P("   Protocolo CAN1 v%u.%u   (1 Mbit/s, CAN FD sin BRS)\r\n",
    (unsigned)CAN_PROTOCOL_VERSION_MAJOR, (unsigned)CAN_PROTOCOL_VERSION_MINOR);
  P("   Consola        USB (puerto COM virtual), informe cada %lu ms\r\n",
    (unsigned long)DEBUG_CONSOLE_PERIOD_MS);
  P(LINE_THIN);
#if SOUNDS_STARTUP_ENABLE
  P("   Melodia de arranque en el zumbador (PE4, TIM15)...\r\n");
#endif
}

void DebugConsole_WakeUpStart(void) {
  P("   Enviando wake up al resto de placas...\r\n");
}

void DebugConsole_Startup(const sys_state *state) {
  P("   Wake up terminado en %lu ms:  power %s   avionica %s   rf %s\r\n",
    (unsigned long)HAL_GetTick(), _flag(state->power), _flag(state->avionics),
    _flag(state->rf));
  P("   Arranque: %s%s" C_RESET " (reset: %s) - %s\r\n",
    (emergencia_modo() == EMERGENCIA_INICIO) ? C_OK : C_BAD,
    emergencia_modo_nombre(emergencia_modo()),
    memoria_vuelo_causa_nombre(memoria_vuelo_causa()), emergencia_motivo());
  _section_components(state);
  P(C_TITLE LINE_THICK C_RESET "\r\n");
  _last_report_ms = HAL_GetTick();
}

void DebugConsole_Update(const sys_state *state) {
  if ((HAL_GetTick() - _last_report_ms) < DEBUG_CONSOLE_PERIOD_MS) {
    return;
  }
  _last_report_ms += DEBUG_CONSOLE_PERIOD_MS;
  _report_count++;

  const uint32_t t_s = HAL_GetTick() / 1000U;
  P(C_TITLE LINE_THICK C_RESET);
  P(" " C_TITLE "CORE H7" C_RESET "   t = %02lu:%02lu:%02lu   informe #%lu\r\n",
    (unsigned long)(t_s / 3600U), (unsigned long)((t_s / 60U) % 60U),
    (unsigned long)(t_s % 60U), (unsigned long)_report_count);
  P(C_TITLE LINE_THICK C_RESET);

  _section_boards(state);
  _section_vuelo();
  _section_components(state);
  _section_power(state);
  _section_calibration(state);
  _section_sensor_board();
  _section_gnc();
  _section_storage();
  _section_aspid();
  _section_status();
  _section_can();

  P(C_TITLE LINE_THICK C_RESET "\r\n");
}
