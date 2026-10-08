#include "aspid_data.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "airbrake.h"
#include "camaras.h"
#include "can_driver.h"
#include "can_protocol/can_protocol_version.h"
#include "debug_out.h"
#include "despliegue.h"
#include "fx_api.h"
#include "gnc.h"
#include "gnc_playback.h"
#include "lanzamiento.h"
#include "main.h"
#include "rf_telemetry.h"
#include "sd_card.h"
#include "sensor_board.h"
#include "status_packet.h"
#include "storage.h"
#include "uplink.h"

/* De storage.c, como en storage_stream.c */
void *storage_media_handle(sd_card_id_t sd);
FX_MEDIA *storage_media_fx(void *handle);

#define ASPID_SD SD_CARD_1
#define G0 9.80665f
#define MB (1024U * 1024U)
#define SECTOR 512U

/* Lo que se lanza por DMA de una vez: ~1,4 ms a 20 MHz (App/sd_bench.c) */
#define BLOQUE 4096U

/* Texto en RAM esperando a la tarjeta (múltiplos de BLOQUE). Es lo que
 * aguanta una tarjeta lenta sin perder nada: ~0,4 s de cada uno. */
#define ANILLO_VUELO (24U * 1024U)  /* ~50 KB/s */
#define ANILLO_EMPUJE (32U * 1024U) /* ~90 KB/s */
#define ANILLO_TELEM (8U * 1024U)   /* ~2 KB/s */
#define ANILLO_SESION (8U * 1024U)
#define ANILLO_EVENTOS (8U * 1024U)
#define ANILLO_SALUD (8U * 1024U) /* ~0,25 KB/s */

/* Cada anillo tiene que ser AL MENOS DOS bloques: un bloque solo sale cuando
 * está entero, así que con un anillo de un bloque justo, cuando quedan unos
 * pocos bytes para completarlo la línea siguiente ya no cabe, el bloque nunca
 * se completa y desde ahí se pierde todo (pasó con salud.csv, 2026-10-06) */
_Static_assert(ANILLO_VUELO >= (2U * BLOQUE), "anillo de vuelo");
_Static_assert(ANILLO_EMPUJE >= (2U * BLOQUE), "anillo de trust curve");
_Static_assert(ANILLO_TELEM >= (2U * BLOQUE), "anillo de telemetria");
_Static_assert(ANILLO_SESION >= (2U * BLOQUE), "anillo de sesion");
_Static_assert(ANILLO_EVENTOS >= (2U * BLOQUE), "anillo de eventos");
_Static_assert(ANILLO_SALUD >= (2U * BLOQUE), "anillo de salud");

/* Lo que se reserva en la tarjeta para cada archivo */
#define RESERVA_VUELO (64U * MB)  /* 20 min a ~50 KB/s */
#define RESERVA_EMPUJE (8U * MB)  /* 30 s a ~90 KB/s, con holgura */
#define RESERVA_TELEM (8U * MB)   /* ~1 h; en tierra se añaden tramos */
#define RESERVA_TELEM_MIN (4U * MB)
#define RESERVA_SESION (1U * MB)
#define RESERVA_EVENTOS (1U * MB)
#define RESERVA_SALUD (4U * MB)   /* ~2 h; en tierra se añaden tramos */
#define RESERVA_SALUD_MIN (1U * MB)
#define RESERVA_SIM (8U * MB)
#define RESERVA_SIM_EMPUJE (4U * MB)
#define MAX_TRAMOS 16U

#define FILA_MAX 1536U
#define LINEA_MAX 224U
#define EVENTO_MAX 320U
#define MSG_MAX 240U

/* Muestras de vuelo a 100 Hz: el segundo de antes del despegue y holgura */
#define MUESTRAS_N 128U
#define ANTES_N ((ASPID_DATA_ANTES_MS * ASPID_DATA_HZ) / 1000U)
/* IMU a 400 Hz (un tick del GNC): medio segundo de antes y holgura */
#define IMU_N 256U
#define IMU_ANTES_N ((ASPID_DATA_EMPUJE_ANTES_MS * GNC_RATE_HZ) / 1000U)
#define FORMATO_VUELO_VUELTA 4U
#define FORMATO_IMU_VUELTA 8U

#define POLL_US 100U    /* cada cuánto se pregunta a la tarjeta (CMD13) */
#define ERRORES_MAX 3U  /* de FileX, en tierra */
#define DIR_HUECOS 8U
#define RESERVAS "aspid_reservas.txt"

/* === Tipos =============================================================== */

typedef struct {
  uint32_t t_ms;
  uint32_t tick;
  uint8_t estado;
  uint8_t fase;
  uint8_t error;
  uint8_t flags; /* b0 nav, b1 solo GNSS, b2 piro armada, b3 drogue,
                    b4 main, b5 simulando */
  float alt;
  float pos[3];
  float vel[3];
  float quat[4];
  float euler[3];
  float acc_b[3];
  float gyr_b[3];
  float sesgo_a[3];
  float sesgo_g[3];
  float apogeo;
  float err_apogeo;
  float t_apogeo;
  float aero_gnc;
  float aero_mando;
  uint8_t servo;
  uint8_t continuidad; /* SYS_PYRO_*, 0xFF = sin dato */
  uint8_t gps_calidad;
  uint8_t gps_sats;
  float imu1_a[3];
  float imu1_g[3];
  float imu2_a[3];
  float imu2_g[3];
  uint64_t imu1_t_us;
  uint64_t imu2_t_us;
  float baro1_kpa;
  float baro1_c;
  float baro2_kpa;
  float baro2_c;
  float mag_ut[3];
  int32_t lat_e7;
  int32_t lon_e7;
  int32_t alt_mm;
  uint16_t hdop;
  uint8_t pvt_fix;
  int32_t vn_mmps;
  int32_t ve_mmps;
  int32_t vd_mmps;
  uint16_t v11_mv; /* 0 = sin dato */
  uint16_t i11_ma;
} muestra_t;

/* Una fila de la trust curve: lo último de cada IMU en un tick del GNC */
typedef struct {
  uint32_t t_ms;
  uint32_t tick;
  uint8_t fase;
  uint8_t n1; /* muestras nuevas de cada IMU desde la fila anterior: hoy
                 llegan a ~200 Hz y la mitad de las filas repiten (n = 0) */
  uint8_t n2;
  float a1[3];
  float g1[3];
  float a2[3];
  float g2[3];
  uint64_t t1_us;
  uint64_t t2_us;
} imu_t;

/*
 * Un archivo que se escribe por DMA. Los bytes se ponen en un anillo en RAM;
 * la posición en el anillo es la del archivo módulo su tamaño, así que un
 * bloque de 4 KB que empiece en múltiplo de 4 KB nunca da la vuelta y sale
 * entero, alineado, al DMA.
 */
typedef struct {
  FX_FILE file;
  bool abierto;
  bool cerrando;      /* no admite más datos: se acaba de escribir y se cierra */
  uint8_t *anillo;
  uint32_t tam;
  uint32_t cab;       /* bytes puestos = tamaño final del archivo */
  uint32_t escrito;   /* bytes ya en la tarjeta (múltiplo de BLOQUE) */
  uint32_t sync_cab;  /* hasta dónde llegó el último trozo a medias */
  uint32_t en_dir;    /* tamaño que dice ya el directorio */
  uint32_t reservado;
  uint32_t n_tramos;  /* tramos contiguos (fx_file_allocate los da así) */
  uint32_t tramo_lba[MAX_TRAMOS];
  uint32_t tramo_bytes[MAX_TRAMOS];
  uint8_t ronda;      /* volcado de cada segundo: 1 trozo, 2 tamaño, 0 ya */
  int32_t dir_hueco;  /* sector de su carpeta en s_dir, -1 = no localizado */
  uint32_t dir_off;
  uint32_t *bytes;    /* contador de s_info */
  uint32_t ampliar;   /* en tierra, otro tramo de esto si queda menos de */
  uint32_t ampliar_min; /* esto reservado y sin usar (0 = nunca) */
  char ruta[48];      /* dentro de la carpeta de la sesión */
} flujo_t;

/* Sectores de carpeta con entradas de archivos abiertos: una copia en RAM
 * para actualizar el tamaño sin FileX. Dos archivos en la misma carpeta
 * (los de una simulación) comparten copia, si no se pisarían. */
typedef struct {
  uint8_t buf[SECTOR] __attribute__((aligned(32)));
  uint32_t lba;
  bool usado;
} dir_t;

typedef enum {
  T_NADA = 0,
  T_DATOS,   /* un bloque entero */
  T_PARCIAL, /* lo que haya a medias (sin dar por escrito) */
  T_DIR,     /* el sector de la carpeta con el tamaño nuevo */
} trabajo_t;

typedef enum {
  EMP_ESPERA = 0, /* guardando el medio segundo de antes */
  EMP_GRABANDO,
  EMP_DRENANDO,   /* se acabó la ventana: formatear lo que queda */
  EMP_HECHO,
} empuje_t;

typedef enum {
  CIERRE_NADA = 0,
  CIERRE_DRENAR,    /* formatear lo que quede en los anillos de muestras */
  CIERRE_DATOS,     /* esperar a que todo esté en la tarjeta */
  CIERRE_FILEX,     /* cerrar con FileX, uno por vuelta */
  CIERRE_FLUSH,
  CIERRE_SIGUIENTE,
} cierre_t;

static const char CABECERA_VUELO[] =
    "t_ms,hora_utc,tick_gnc,estado,fase,error,nav_valida,solo_gnss,"
    "piro_armada,drogue_mandado,main_mandado,simulando,"
    "alt_agl_m,pos_n_m,pos_e_m,pos_d_m,vel_n_mps,vel_e_mps,vel_d_mps,"
    "q_w,q_x,q_y,q_z,alabeo_deg,cabeceo_deg,guinada_deg,"
    "acc_cuerpo_x_mps2,acc_cuerpo_y_mps2,acc_cuerpo_z_mps2,"
    "gyr_cuerpo_x_rps,gyr_cuerpo_y_rps,gyr_cuerpo_z_rps,"
    "sesgo_acc_x,sesgo_acc_y,sesgo_acc_z,sesgo_gyr_x,sesgo_gyr_y,"
    "sesgo_gyr_z,"
    "apogeo_pred_m,error_apogeo_m,t_apogeo_s,aerofreno_gnc,"
    "aerofreno_mandado,servo_byte,"
    "imu1_ax,imu1_ay,imu1_az,imu1_gx,imu1_gy,imu1_gz,imu1_t_us,"
    "imu2_ax,imu2_ay,imu2_az,imu2_gx,imu2_gy,imu2_gz,imu2_t_us,"
    "baro1_kpa,baro1_c,baro2_kpa,baro2_c,mag_x_ut,mag_y_ut,mag_z_ut,"
    "gps_lat_deg,gps_lon_deg,gps_alt_m,gps_calidad,gps_sats,gps_hdop,"
    "gps_vn_mps,gps_ve_mps,gps_vd_mps,gps_fix_pvt,"
    "continuidad_1a,continuidad_1b,continuidad_2a,continuidad_2b,"
    "bateria_v,bateria_a\r\n";

static const char CABECERA_EMPUJE[] =
    "t_ms,hora_utc,tick_gnc,fase,"
    "imu1_ax,imu1_ay,imu1_az,imu1_gx,imu1_gy,imu1_gz,imu1_t_us,imu1_nuevas,"
    "imu2_ax,imu2_ay,imu2_az,imu2_gx,imu2_gy,imu2_gz,imu2_t_us,imu2_nuevas,"
    "imu1_a_g,imu2_a_g\r\n";

static const char CABECERA_EVENTOS[] = "t_ms,hora_utc,tipo,valor,detalle\r\n";

static const char CABECERA_SALUD[] =
    "t_ms,hora_utc,registro,bucle_vueltas,bucle_max_ms,"
    "gnc_ticks,gnc_tick_max_us,gnc_perdidos,gnc_recuperados,gnc_estado,"
    "gnc_fase,gnc_error,nav_valida,alineando,"
    "imu1_hz,imu2_hz,baro1_hz,baro2_hz,mag_hz,gps_hz,avionica_resets,"
    "can1_hz,can2_hz,can1_desbordes,can1_perdidas,can1_err_tx,"
    "can2_desbordes,can2_perdidas,can2_err_tx,"
    "radio_enviados,radio_fallidos,radio_estados,radio_acuses,"
    "subida_recibidos,subida_validos,subida_rechazados,subida_denegados,"
    "power,avionica,rf,wakeup_reintentos,"
    "v3v3,i3v3_a,v5,i5_a,v11,i11_a,"
    "continuidad_1a,continuidad_1b,continuidad_2a,continuidad_2b,"
    "sd_escrituras,sd_escritura_max_ms,sd_bytes_perdidos,sd_filas_perdidas,"
    "sd_errores,usb_descartados,camaras\r\n";

static const char CABECERA_TELEM[] =
    "t_ms,hora_utc,sentido,tipo,largo,rssi_dbm,snr_db,error_frec_hz,"
    "resultado,bytes\r\n";

/* === Estado ============================================================== */

static aspid_data_info_t s_info;
static FX_MEDIA *s_m;
static uint32_t s_cluster_bytes;

static uint8_t s_mem_vuelo[ANILLO_VUELO] __attribute__((aligned(32)));
static uint8_t s_mem_empuje[ANILLO_EMPUJE] __attribute__((aligned(32)));
static uint8_t s_mem_telem[ANILLO_TELEM] __attribute__((aligned(32)));
static uint8_t s_mem_sesion[ANILLO_SESION] __attribute__((aligned(32)));
static uint8_t s_mem_eventos[ANILLO_EVENTOS] __attribute__((aligned(32)));
static uint8_t s_mem_salud[ANILLO_SALUD] __attribute__((aligned(32)));

/* Los de vuelo de verdad están abiertos (vacíos y reservados) desde el
 * arranque: en el despegue no se toca FileX. Los de una simulación se crean
 * al empezarla, en tierra. Los de vuelo y los de simulación comparten
 * anillo: solo graba uno a la vez. */
static flujo_t s_sesion;
static flujo_t s_eventos;
static flujo_t s_salud;
static flujo_t s_telem;
static flujo_t s_vuelo;
static flujo_t s_empuje;
static flujo_t s_sim_vuelo;
static flujo_t s_sim_empuje;
static flujo_t *const s_flujos[] = {&s_sesion,  &s_eventos,   &s_salud,
                                    &s_telem,   &s_vuelo,     &s_empuje,
                                    &s_sim_vuelo, &s_sim_empuje};
#define N_FLUJOS (sizeof(s_flujos) / sizeof(s_flujos[0]))
static flujo_t *s_av; /* el de vuelo que graba */
static flujo_t *s_ae; /* el de trust curve que graba */

static dir_t s_dir[DIR_HUECOS];
static uint8_t s_dir_tmp[SECTOR] __attribute__((aligned(32)));

static muestra_t s_mu[MUESTRAS_N];
static uint32_t s_mu_cab;
static uint32_t s_mu_cola;
static uint32_t s_bloque_gnc = UINT32_MAX;

static imu_t s_imu[IMU_N];
static uint32_t s_imu_cab;
static uint32_t s_imu_cola;
static uint32_t s_steps_imu = UINT32_MAX;
static uint32_t s_cnt_imu1;
static uint32_t s_cnt_imu2;
static empuje_t s_emp;
static uint32_t s_emp_ini;
static uint32_t s_emp_fin;

static bool s_grabando;
static bool s_cerrando;
static bool s_sellar;
static bool s_es_sim;
static bool s_apogeo_visto;
static bool s_gnc_vuelo_prev;
static bool s_alto;        /* una IMU por encima de 3 g... */
static uint32_t s_t_alto;  /* ...desde este HAL_GetTick() */
static bool s_alto_usado;  /* ese tramo ya disparo un despegue */
static uint32_t s_t_limite; /* desde aquí cuentan los 20 minutos */
static cierre_t s_cierre;

static trabajo_t s_trab;
static flujo_t *s_trab_f;
static uint32_t s_trab_n;
static uint32_t s_trab_t0;
static uint32_t s_ultimo_poll;
static uint32_t s_ultimo_sync;
static uint32_t s_sync_i = N_FLUJOS; /* N_FLUJOS = no hay ronda en curso */

/* Hora: UTC en ms desde 1970 que corresponde a t_ms = 0 */
static bool s_hora;
static int64_t s_utc0_ms;
static uint32_t s_rmc_visto;
static bool s_renombrar;

static char s_fila[FILA_MAX];
static char s_linea[LINEA_MAX];
static char s_ev[EVENTO_MAX];
static char s_msg[MSG_MAX];

/* Salud (1 Hz) y eventos */
static uint32_t s_salud_t;      /* HAL_GetTick() de la siguiente fila */
static uint32_t s_bucle_n;      /* vueltas del bucle en este segundo */
static uint32_t s_bucle_max_us; /* la más larga entre dos vueltas */
static uint32_t s_bucle_t;      /* DWT de la vuelta anterior */
static uint32_t s_tick_max_us;  /* el tick del GNC más lento del segundo */
static uint32_t s_steps_vistos;
static uint32_t s_steps_salud;
static uint32_t s_escr_max_seg_us;
static uint32_t s_cnt_salud[6]; /* imu1 imu2 baro1 baro2 mag gps */
static uint32_t s_perdidos_prev;
static uint32_t s_can_err_prev[2];
static uint32_t s_radio_fallo_prev;
static uint32_t s_sd_perdidos_prev;

/* Lo último visto, para saber qué ha cambiado */
static struct {
  bool listo;
  int32_t estado;
  int32_t fase;
  uint8_t error;
  bool nav;
  bool gnss;
  bool armada;
  bool drogue;
  bool main;
  uint32_t o1a;
  uint32_t o1b;
  uint32_t o2a;
  uint32_t fallos;
  bool cont_ok;
  uint8_t cont;
  bool test;
  bool sim;
  bool aero;
  uint32_t piros;
  uint16_t rampa;
  uint16_t rumbo;
  bool power;
  bool avionica;
  bool rf;
  uint32_t resets;
  bool alineado;
  bool camaras;
  bool camaras_apagando;
  uint32_t cortes_camaras;
  uint32_t camara_firma[CAMARAS_N];
} s_prev;

/* La última posición GPS buena: para saber dónde cayó */
static struct {
  bool ok;
  int32_t lat_e7;
  int32_t lon_e7;
  int32_t alt_mm;
  uint8_t sats;
  uint32_t t_ms;
  uint32_t cnt;
} s_gps;
static char s_ruta[112];

#if ASPID_DATA_RELEER
static bool s_releer;
static char s_releer_ruta[2][48];
#endif

/* === Medir =============================================================== */

static uint32_t _t(void) { return DWT->CYCCNT; }
static uint32_t _us(uint32_t t0) {
  return (DWT->CYCCNT - t0) / (SystemCoreClock / 1000000U);
}

static void _filex_tiempo(uint32_t t0) {
  const uint32_t us = _us(t0);
  if (us > s_info.filex_max_us) {
    s_info.filex_max_us = us;
  }
}

/* === Formato rápido (sin printf: una fila son ~90 números a 100 Hz) ====== */

static const uint32_t s_pot10[] = {1U,      10U,      100U,      1000U,
                                   10000U,  100000U,  1000000U,  10000000U};

static char *_s(char *p, const char *s) {
  while (*s != '\0') {
    *p++ = *s++;
  }
  return p;
}

/* En 32 bits: la división de 64 es una llamada a la librería */
static char *_u32(char *p, uint32_t v) {
  char tmp[10];
  uint32_t n = 0U;
  do {
    tmp[n++] = (char)('0' + (v % 10U));
    v /= 10U;
  } while (v > 0U);
  while (n > 0U) {
    *p++ = tmp[--n];
  }
  return p;
}

static char *_u64(char *p, uint64_t v) {
  if (v <= UINT32_MAX) {
    return _u32(p, (uint32_t)v);
  }
  char tmp[20];
  uint32_t n = 0U;
  do {
    tmp[n++] = (char)('0' + (v % 10U));
    v /= 10U;
  } while (v > 0U);
  while (n > 0U) {
    *p++ = tmp[--n];
  }
  return p;
}

static char *_i32(char *p, int32_t v) {
  if (v < 0) {
    *p++ = '-';
    return _u64(p, (uint64_t)(-(int64_t)v));
  }
  return _u64(p, (uint64_t)v);
}

/* v / 10^dec con dec decimales */
static char *_fijo(char *p, int64_t v, uint32_t dec) {
  if (v < 0) {
    *p++ = '-';
    v = -v;
  }
  const uint64_t u = (uint64_t)v;
  uint32_t f;
  if (u <= UINT32_MAX) {
    p = _u32(p, (uint32_t)u / s_pot10[dec]);
    f = (uint32_t)u % s_pot10[dec];
  } else {
    p = _u64(p, u / s_pot10[dec]);
    f = (uint32_t)(u % s_pot10[dec]);
  }
  if (dec > 0U) {
    *p++ = '.';
    for (uint32_t i = dec; i > 0U; i--) {
      p[i - 1U] = (char)('0' + (f % 10U));
      f /= 10U;
    }
    p += dec;
  }
  return p;
}

static char *_f(char *p, float v, uint32_t dec) {
  if (isnan(v)) {
    return _s(p, "nan");
  }
  const double x = (double)v * (double)s_pot10[dec];
  if ((x > 9.0e15) || (x < -9.0e15)) {
    return _s(p, (v > 0.0f) ? "inf" : "-inf");
  }
  const double r = (x >= 0.0) ? (x + 0.5) : (x - 0.5);
  if ((r < 2.1e9) && (r > -2.1e9)) {
    return _fijo(p, (int32_t)r, dec); /* conversión de la FPU, sin librería */
  }
  return _fijo(p, (int64_t)r, dec);
}

static char *_hex(char *p, const uint8_t *d, uint32_t n) {
  static const char h[] = "0123456789ABCDEF";
  for (uint32_t i = 0U; i < n; i++) {
    if (i > 0U) {
      *p++ = ' ';
    }
    *p++ = h[d[i] >> 4];
    *p++ = h[d[i] & 0x0FU];
  }
  return p;
}

static char *_2d(char *p, uint32_t v) {
  *p++ = (char)('0' + ((v / 10U) % 10U));
  *p++ = (char)('0' + (v % 10U));
  return p;
}

/* Días desde 1970-01-01 (algoritmo de H. Hinnant, calendario gregoriano) */
static int32_t _dias(int32_t y, uint32_t m, uint32_t d) {
  y -= (m <= 2U) ? 1 : 0;
  const int32_t era = ((y >= 0) ? y : (y - 399)) / 400;
  const uint32_t yoe = (uint32_t)(y - (era * 400));
  const uint32_t doy = ((153U * ((m > 2U) ? (m - 3U) : (m + 9U))) + 2U) / 5U +
                       d - 1U;
  const uint32_t doe = (yoe * 365U) + (yoe / 4U) - (yoe / 100U) + doy;
  return (era * 146097) + (int32_t)doe - 719468;
}

static void _civil(int32_t z, int32_t *y, uint32_t *m, uint32_t *d) {
  z += 719468;
  const int32_t era = ((z >= 0) ? z : (z - 146096)) / 146097;
  const uint32_t doe = (uint32_t)(z - (era * 146097));
  const uint32_t yoe =
      (doe - (doe / 1460U) + (doe / 36524U) - (doe / 146096U)) / 365U;
  const uint32_t doy = doe - ((365U * yoe) + (yoe / 4U) - (yoe / 100U));
  const uint32_t mp = ((5U * doy) + 2U) / 153U;
  *d = doy - (((153U * mp) + 2U) / 5U) + 1U;
  *m = (mp < 10U) ? (mp + 3U) : (mp - 9U);
  *y = (int32_t)yoe + (era * 400) + ((*m <= 2U) ? 1 : 0);
}

/* "2026-10-05 14:30:12.345", o para el nombre de la carpeta
 * "2026-10-05_14-30-12" */
static char *_fecha(char *p, int64_t utc_ms, bool para_nombre) {
  const int64_t dia_ms = 86400000;
  int64_t dias = utc_ms / dia_ms;
  int64_t resto = utc_ms % dia_ms;
  if (resto < 0) {
    resto += dia_ms;
    dias--;
  }
  int32_t y;
  uint32_t m;
  uint32_t d;
  _civil((int32_t)dias, &y, &m, &d);
  const uint32_t ms = (uint32_t)resto;
  p = _u32(p, (uint32_t)y);
  *p++ = '-';
  p = _2d(p, m);
  *p++ = '-';
  p = _2d(p, d);
  *p++ = para_nombre ? '_' : ' ';
  p = _2d(p, ms / 3600000U);
  *p++ = para_nombre ? '-' : ':';
  p = _2d(p, (ms / 60000U) % 60U);
  *p++ = para_nombre ? '-' : ':';
  p = _2d(p, (ms / 1000U) % 60U);
  if (!para_nombre) {
    *p++ = '.';
    *p++ = (char)('0' + ((ms / 100U) % 10U));
    p = _2d(p, ms % 100U);
  }
  return p;
}

static char *_utc(char *p, uint32_t t_ms) {
  return s_hora ? _fecha(p, s_utc0_ms + (int64_t)t_ms, false) : p;
}

/* === Errores ============================================================= */

static void _error(uint32_t e) {
  s_info.errores++;
  s_info.ultimo_error = e;
  if (s_info.errores >= ERRORES_MAX) {
    s_info.estado = ASPID_SIN_SD;
  }
}

/* Una escritura por DMA ha fallado o no ha acabado a tiempo: la tarjeta se
 * da por perdida y no se vuelve a tocar (sd_card.c ya la ha dado de baja) */
static void _tarjeta_perdida(void) {
  s_info.tarjeta_perdida = true;
  s_info.errores++;
  s_info.ultimo_error = sd_card_last_error(ASPID_SD);
  s_info.estado = ASPID_SIN_SD;
  s_trab = T_NADA;
}

/* === Anillos ============================================================= */

static const char *_ruta(const char *rel) {
  (void)snprintf(s_ruta, sizeof(s_ruta), "%s/%s", s_info.carpeta, rel);
  return s_ruta;
}

static void _flujo(flujo_t *f, uint8_t *anillo, uint32_t tam,
                   uint32_t *bytes) {
  memset(f, 0, sizeof(*f));
  f->anillo = anillo;
  f->tam = tam;
  f->bytes = bytes;
  f->dir_hueco = -1;
}

static uint32_t _sitio(const flujo_t *f) {
  return f->tam - (f->cab - f->escrito);
}

/* Añade n bytes al archivo. false si no cabe (en RAM o en lo reservado):
 * se pierde y se cuenta, nunca se espera */
static bool _poner(flujo_t *f, const char *src, uint32_t n) {
  if (!f->abierto || f->cerrando) {
    return false;
  }
  if ((n > _sitio(f)) || ((f->cab + n) > f->reservado)) {
    s_info.bytes_perdidos += n;
    return false;
  }
  const uint32_t pos = f->cab % f->tam;
  const uint32_t hasta_fin = f->tam - pos;
  if (n <= hasta_fin) {
    memcpy(&f->anillo[pos], src, n);
  } else {
    memcpy(&f->anillo[pos], src, hasta_fin);
    memcpy(f->anillo, &src[hasta_fin], n - hasta_fin);
  }
  f->cab += n;
  return true;
}

static uint32_t _en_tarjeta(const flujo_t *f) {
  return (f->sync_cab > f->escrito) ? f->sync_cab : f->escrito;
}

/* === FileX (solo en tierra o al terminar: bloquea) ====================== */

static uint32_t _lba_cluster(uint32_t cluster) {
  return (uint32_t)s_m->fx_media_hidden_sectors +
         s_m->fx_media_data_sector_start +
         ((cluster - 2U) * s_m->fx_media_sectors_per_cluster);
}

/* Sector de la tarjeta donde va el byte 'off' del archivo (0 = fuera) */
static uint32_t _lba(const flujo_t *f, uint32_t off) {
  for (uint32_t i = 0U; i < f->n_tramos; i++) {
    if (off < f->tramo_bytes[i]) {
      return f->tramo_lba[i] + (off / SECTOR);
    }
    off -= f->tramo_bytes[i];
  }
  return 0U;
}

/* Otro tramo contiguo al final del archivo */
static bool _reservar(flujo_t *f, uint32_t bytes) {
  if (f->n_tramos >= MAX_TRAMOS) {
    return false;
  }
  const ULONG antes = f->file.fx_file_total_clusters;
  const uint32_t t0 = _t();
  const UINT st = fx_file_extended_allocate(&f->file, (ULONG64)bytes);
  _filex_tiempo(t0);
  if (st != FX_SUCCESS) {
    _error(st);
    return false;
  }
  const uint32_t n = (uint32_t)(f->file.fx_file_total_clusters - antes);
  const uint32_t primero =
      (uint32_t)f->file.fx_file_last_physical_cluster - n + 1U;
  f->tramo_lba[f->n_tramos] = _lba_cluster(primero);
  f->tramo_bytes[f->n_tramos] = n * s_cluster_bytes;
  f->n_tramos++;
  f->reservado += n * s_cluster_bytes;
  return true;
}

static bool _abrir(flujo_t *f, const char *rel, uint32_t reserva) {
  (void)snprintf(f->ruta, sizeof(f->ruta), "%s", rel);
  f->cab = 0U;
  f->escrito = 0U;
  f->sync_cab = 0U;
  f->en_dir = 0U;
  f->reservado = 0U;
  f->n_tramos = 0U;
  f->cerrando = false;
  f->dir_hueco = -1;

  const char *r = _ruta(rel);
  const uint32_t t0 = _t();
  UINT st = fx_file_create(s_m, (CHAR *)r);
  if ((st == FX_SUCCESS) || (st == FX_ALREADY_CREATED)) {
    st = fx_file_open(s_m, &f->file, (CHAR *)r, FX_OPEN_FOR_WRITE);
  }
  if (st == FX_SUCCESS) {
    st = fx_file_extended_truncate_release(&f->file, 0U);
  }
  _filex_tiempo(t0);
  if (st != FX_SUCCESS) {
    _error(st);
    return false;
  }
  f->abierto = true;
  return _reservar(f, reserva);
}

/*
 * Cierra con FileX con el tamaño de verdad. Con liberar, devuelve también lo
 * reservado de más; eso reescribe la FAT de toda la reserva (~50 ms con los
 * 64 MB del vuelo, un tick del GNC perdido), así que al terminar un vuelo de
 * verdad NO se hace: el archivo queda con su tamaño correcto (se lee bien en
 * el PC) y lo sobrante lo libera el arranque siguiente (aspid_reservas.txt).
 */
static void _finalizar(flujo_t *f, bool liberar) {
  if (!f->abierto) {
    return;
  }
  const uint32_t t0 = _t();
  f->file.fx_file_current_file_size = f->cab;
  f->file.fx_file_dir_entry.fx_dir_entry_file_size = f->cab;
  f->file.fx_file_modified = FX_TRUE;
  UINT st = liberar ? fx_file_extended_truncate_release(&f->file, f->cab)
                    : (UINT)FX_SUCCESS;
  const UINT st2 = fx_file_close(&f->file);
  _filex_tiempo(t0);
  if (st == FX_SUCCESS) {
    st = st2;
  }
  if (st != FX_SUCCESS) {
    _error(st);
  }
  if (f->bytes != NULL) {
    *f->bytes += f->cab - f->escrito; /* el último trozo */
  }
  f->abierto = false;
  f->cerrando = false;
  f->dir_hueco = -1;
}

/* Localiza la entrada de directorio de f en su carpeta (busca su primer
 * cluster en el sector que dice FileX y el siguiente, por si el nombre largo
 * empuja la entrada corta) y guarda una copia del sector */
static void _buscar_dir(flujo_t *f) {
  f->dir_hueco = -1;
  const uint32_t clu = (uint32_t)f->file.fx_file_first_physical_cluster;
  if (!f->abierto || (clu < 2U)) {
    return;
  }
  const uint32_t lba0 =
      (uint32_t)s_m->fx_media_hidden_sectors +
      (uint32_t)f->file.fx_file_dir_entry.fx_dir_entry_log_sector;
  for (uint32_t k = 0U; k < 2U; k++) {
    const uint32_t lba = lba0 + k;
    int32_t h = -1;
    for (uint32_t i = 0U; i < DIR_HUECOS; i++) {
      if (s_dir[i].usado && (s_dir[i].lba == lba)) {
        h = (int32_t)i;
      }
    }
    const uint8_t *sec = s_dir_tmp;
    if (h >= 0) {
      sec = s_dir[h].buf;
    } else if (!sd_card_read_blocks(ASPID_SD, s_dir_tmp, lba, 1U)) {
      return;
    }
    for (uint32_t off = 0U; off < SECTOR; off += 32U) {
      const uint8_t *e = &sec[off];
      if ((e[0] == 0x00U) || (e[0] == 0xE5U) || ((e[11] & 0x3FU) == 0x0FU)) {
        continue;
      }
      const uint32_t c = (uint32_t)e[26] | ((uint32_t)e[27] << 8) |
                         ((uint32_t)e[20] << 16) | ((uint32_t)e[21] << 24);
      if (c != clu) {
        continue;
      }
      if (h < 0) {
        for (uint32_t i = 0U; (i < DIR_HUECOS) && (h < 0); i++) {
          if (!s_dir[i].usado) {
            h = (int32_t)i;
            s_dir[i].usado = true;
            s_dir[i].lba = lba;
            memcpy(s_dir[i].buf, s_dir_tmp, SECTOR);
          }
        }
      }
      if (h >= 0) {
        f->dir_hueco = h;
        f->dir_off = off;
      }
      return;
    }
  }
}

/* Tras cualquier cosa de FileX: que FileX escriba lo suyo (con los tamaños
 * que ya puso el DMA) y volver a leer las copias de los sectores de carpeta */
static void _flush_y_dirs(void) {
  for (uint32_t i = 0U; i < N_FLUJOS; i++) {
    flujo_t *f = s_flujos[i];
    if (f->abierto) {
      f->file.fx_file_current_file_size = f->en_dir;
      f->file.fx_file_dir_entry.fx_dir_entry_file_size = f->en_dir;
      f->file.fx_file_modified = FX_TRUE;
    }
  }
  const uint32_t t0 = _t();
  const UINT st = fx_media_flush(s_m);
  _filex_tiempo(t0);
  if (st != FX_SUCCESS) {
    _error(st);
  }
  for (uint32_t i = 0U; i < DIR_HUECOS; i++) {
    s_dir[i].usado = false;
  }
  for (uint32_t i = 0U; i < N_FLUJOS; i++) {
    if (s_flujos[i]->abierto) {
      _buscar_dir(s_flujos[i]);
    }
  }
}

/* aspid_reservas.txt: los archivos con sitio reservado, para liberarlo en
 * el arranque siguiente si se corta la corriente antes de cerrarlos */
static void _guardar_reservas(void) {
  char *p = s_fila;
  for (uint32_t i = 0U; i < N_FLUJOS; i++) {
    if (s_flujos[i]->abierto &&
        ((uint32_t)(p - s_fila) < (FILA_MAX - 128U))) {
      p += snprintf(p, 128U, "%s/%s\r\n", s_info.carpeta,
                    s_flujos[i]->ruta);
    }
  }
  FX_FILE f;
  const uint32_t t0 = _t();
  UINT st = fx_file_create(s_m, (CHAR *)RESERVAS);
  if ((st == FX_SUCCESS) || (st == FX_ALREADY_CREATED)) {
    st = fx_file_open(s_m, &f, (CHAR *)RESERVAS, FX_OPEN_FOR_WRITE);
    if (st == FX_SUCCESS) {
      (void)fx_file_extended_truncate_release(&f, 0U);
      (void)fx_file_seek(&f, 0U);
      st = fx_file_write(&f, s_fila, (ULONG)(p - s_fila));
      (void)fx_file_close(&f);
    }
  }
  _filex_tiempo(t0);
  if (st != FX_SUCCESS) {
    _error(st);
  }
}

static void _limpiar_reservas(void) {
  FX_FILE f;
  ULONG n = 0U;
  if (fx_file_open(s_m, &f, (CHAR *)RESERVAS, FX_OPEN_FOR_READ) !=
      FX_SUCCESS) {
    return;
  }
  (void)fx_file_read(&f, s_fila, FILA_MAX - 1U, &n);
  (void)fx_file_close(&f);
  s_fila[n] = '\0';

  char *linea = s_fila;
  while (*linea != '\0') {
    char *fin = linea;
    while ((*fin != '\0') && (*fin != '\r') && (*fin != '\n')) {
      fin++;
    }
    const char sig = *fin;
    *fin = '\0';
    if (fin > linea) {
      FX_FILE a;
      if (fx_file_open(s_m, &a, (CHAR *)linea, FX_OPEN_FOR_WRITE) ==
          FX_SUCCESS) {
        const ULONG64 usado =
            ((a.fx_file_current_file_size + s_cluster_bytes - 1U) /
             s_cluster_bytes) *
            s_cluster_bytes;
        if (a.fx_file_current_available_size > usado) {
          if (fx_file_extended_truncate_release(
                  &a, a.fx_file_current_file_size) == FX_SUCCESS) {
            s_info.reservas_liberadas++;
          }
        }
        (void)fx_file_close(&a);
      }
    }
    linea = (sig == '\0') ? fin : (fin + 1);
  }
  (void)fx_media_flush(s_m);
}

/* === sesion.txt ========================================================== */

/* El texto del evento está en s_msg. A sesion.txt, con la hora delante */
static void _a_sesion(uint32_t t) {
  char *p = s_ev;
  p = _u32(p, t);
  while ((p - s_ev) < 10) {
    *p++ = ' ';
  }
  char *ini = p;
  p = s_hora ? _utc(p, t) : _s(p, "(sin hora)");
  while ((p - ini) < 25) {
    *p++ = ' ';
  }
  uint32_t cabe = EVENTO_MAX - (uint32_t)(p - s_ev) - 3U;
  for (const char *m = s_msg; (*m != '\0') && (cabe > 0U); m++, cabe--) {
    *p++ = *m;
  }
  p = _s(p, "\r\n");
  (void)_poner(&s_sesion, s_ev, (uint32_t)(p - s_ev));
}

/* Y a eventos.csv: t_ms,hora_utc,tipo,valor,detalle (sin comas dentro) */
static void _a_eventos(uint32_t t, const char *tipo, int32_t valor) {
  char *p = s_ev;
  p = _u32(p, t);
  *p++ = ',';
  p = _utc(p, t);
  *p++ = ',';
  p = _s(p, tipo);
  *p++ = ',';
  p = _i32(p, valor);
  *p++ = ',';
  uint32_t cabe = EVENTO_MAX - (uint32_t)(p - s_ev) - 3U;
  for (const char *m = s_msg; (*m != '\0') && (cabe > 0U); m++, cabe--) {
    *p++ = (*m == ',') ? ';' : (((*m == '\r') || (*m == '\n')) ? ' ' : *m);
  }
  p = _s(p, "\r\n");
  if (_poner(&s_eventos, s_ev, (uint32_t)(p - s_ev))) {
    s_info.eventos++;
  }
}

static void _msg(const char *fmt, va_list ap) {
  if (vsnprintf(s_msg, sizeof(s_msg), fmt, ap) < 0) {
    s_msg[0] = '\0';
  }
}

/* Un evento: solo a eventos.csv */
static void _ev(const char *tipo, int32_t valor, const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  _msg(fmt, ap);
  va_end(ap);
  _a_eventos(HAL_GetTick(), tipo, valor);
}

/* Un evento importante: a sesion.txt (para leerlo de un vistazo) y también
 * a eventos.csv */
static void _evento(const char *tipo, const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  _msg(fmt, ap);
  va_end(ap);
  const uint32_t t = HAL_GetTick();
  _a_sesion(t);
  _a_eventos(t, tipo, 0);
}

/* === Filas =============================================================== */

static char *_fila(char *p, const muestra_t *m) {
#define C (*p++ = ',')
  p = _u32(p, m->t_ms);
  C;
  p = _utc(p, m->t_ms);
  C;
  p = _u32(p, m->tick);
  C;
  p = _u32(p, m->estado);
  C;
  p = _u32(p, m->fase);
  C;
  p = _u32(p, m->error);
  for (uint32_t b = 0U; b < 6U; b++) {
    C;
    *p++ = (m->flags & (1U << b)) ? '1' : '0';
  }
  C;
  p = _f(p, m->alt, 2);
  for (uint32_t i = 0U; i < 3U; i++) {
    C;
    p = _f(p, m->pos[i], 2);
  }
  for (uint32_t i = 0U; i < 3U; i++) {
    C;
    p = _f(p, m->vel[i], 3);
  }
  for (uint32_t i = 0U; i < 4U; i++) {
    C;
    p = _f(p, m->quat[i], 5);
  }
  for (uint32_t i = 0U; i < 3U; i++) {
    C;
    p = _f(p, m->euler[i], 2);
  }
  for (uint32_t i = 0U; i < 3U; i++) {
    C;
    p = _f(p, m->acc_b[i], 4);
  }
  for (uint32_t i = 0U; i < 3U; i++) {
    C;
    p = _f(p, m->gyr_b[i], 5);
  }
  for (uint32_t i = 0U; i < 3U; i++) {
    C;
    p = _f(p, m->sesgo_a[i], 5);
  }
  for (uint32_t i = 0U; i < 3U; i++) {
    C;
    p = _f(p, m->sesgo_g[i], 6);
  }
  C;
  p = _f(p, m->apogeo, 1);
  C;
  p = _f(p, m->err_apogeo, 1);
  C;
  p = _f(p, m->t_apogeo, 2);
  C;
  p = _f(p, m->aero_gnc, 3);
  C;
  p = _f(p, m->aero_mando, 3);
  C;
  p = _u32(p, m->servo);

  const float *imu[4] = {m->imu1_a, m->imu1_g, m->imu2_a, m->imu2_g};
  for (uint32_t k = 0U; k < 4U; k++) {
    for (uint32_t i = 0U; i < 3U; i++) {
      C;
      p = _f(p, imu[k][i], ((k % 2U) == 0U) ? 4U : 5U);
    }
    if ((k % 2U) == 1U) {
      C;
      p = _u64(p, (k == 1U) ? m->imu1_t_us : m->imu2_t_us);
    }
  }
  C;
  p = _f(p, m->baro1_kpa, 4);
  C;
  p = _f(p, m->baro1_c, 2);
  C;
  p = _f(p, m->baro2_kpa, 4);
  C;
  p = _f(p, m->baro2_c, 2);
  for (uint32_t i = 0U; i < 3U; i++) {
    C;
    p = _f(p, m->mag_ut[i], 2);
  }

  /* GPS: los no válidos, vacíos */
  C;
  if (m->lat_e7 != SENSOR_BOARD_GPS_INVALID_I32) {
    p = _fijo(p, m->lat_e7, 7);
  }
  C;
  if (m->lon_e7 != SENSOR_BOARD_GPS_INVALID_I32) {
    p = _fijo(p, m->lon_e7, 7);
  }
  C;
  if (m->alt_mm != SENSOR_BOARD_GPS_INVALID_I32) {
    p = _fijo(p, m->alt_mm, 3);
  }
  C;
  p = _u32(p, m->gps_calidad);
  C;
  p = _u32(p, m->gps_sats);
  C;
  if (m->hdop != SENSOR_BOARD_GPS_INVALID_U16) {
    p = _fijo(p, m->hdop, 2);
  }
  C;
  p = _fijo(p, m->vn_mmps, 3);
  C;
  p = _fijo(p, m->ve_mmps, 3);
  C;
  p = _fijo(p, m->vd_mmps, 3);
  C;
  p = _u32(p, m->pvt_fix);

  for (uint32_t b = 0U; b < 4U; b++) {
    C;
    if (m->continuidad != 0xFFU) {
      *p++ = (m->continuidad & (1U << b)) ? '1' : '0';
    }
  }
  C;
  if (m->v11_mv > 0U) {
    p = _fijo(p, m->v11_mv, 3);
  }
  C;
  if (m->v11_mv > 0U) {
    p = _fijo(p, m->i11_ma, 3);
  }
  p = _s(p, "\r\n");
#undef C
  return p;
}

static float _norma(const float v[3]) {
  return sqrtf((v[0] * v[0]) + (v[1] * v[1]) + (v[2] * v[2]));
}

static char *_fila_imu(char *p, const imu_t *m) {
#define C (*p++ = ',')
  p = _u32(p, m->t_ms);
  C;
  p = _utc(p, m->t_ms);
  C;
  p = _u32(p, m->tick);
  C;
  p = _u32(p, m->fase);
  const float *v[4] = {m->a1, m->g1, m->a2, m->g2};
  for (uint32_t k = 0U; k < 4U; k++) {
    for (uint32_t i = 0U; i < 3U; i++) {
      C;
      p = _f(p, v[k][i], ((k % 2U) == 0U) ? 4U : 5U);
    }
    if ((k % 2U) == 1U) {
      C;
      p = _u64(p, (k == 1U) ? m->t1_us : m->t2_us);
      C;
      p = _u32(p, (k == 1U) ? m->n1 : m->n2);
    }
  }
  C;
  p = _f(p, _norma(m->a1) / G0, 3);
  C;
  p = _f(p, _norma(m->a2) / G0, 3);
  p = _s(p, "\r\n");
#undef C
  return p;
}

/* === Muestras ============================================================ */

static void _tomar(muestra_t *m, const sys_state *state, uint32_t ahora) {
  const gnc_data_t *g = gnc_get_data();
  const sensor_board_data_t *s = sensor_board_get_data();

  m->t_ms = ahora;
  m->tick = g->steps;
  m->estado = (uint8_t)g->rocket_state;
  m->fase = (uint8_t)g->flight_phase;
  m->error = g->error_code;
  m->flags = (uint8_t)((g->nav_valid ? 0x01U : 0U) |
                       (g->gnss_only ? 0x02U : 0U) |
                       (g->pyro_armed ? 0x04U : 0U) |
                       (despliegue_drogue_mandado() ? 0x08U : 0U) |
                       (despliegue_main_mandado() ? 0x10U : 0U) |
                       (gnc_playback_active() ? 0x20U : 0U));
  m->alt = g->alt_agl_m;
  memcpy(m->pos, g->pos_ned_m, sizeof(m->pos));
  memcpy(m->vel, g->vel_ned_mps, sizeof(m->vel));
  memcpy(m->quat, g->quat_nb, sizeof(m->quat));
  memcpy(m->euler, g->euler_rpy_deg, sizeof(m->euler));
  memcpy(m->acc_b, g->accel_body_mps2, sizeof(m->acc_b));
  memcpy(m->gyr_b, g->gyro_body_rps, sizeof(m->gyr_b));
  memcpy(m->sesgo_a, g->accel_bias_mps2, sizeof(m->sesgo_a));
  memcpy(m->sesgo_g, g->gyro_bias_rps, sizeof(m->sesgo_g));
  m->apogeo = g->apogee_pred_m;
  m->err_apogeo = g->apogee_error_m;
  m->t_apogeo = g->time_to_apogee_s;
  m->aero_gnc = g->actuator_cmd;
  m->aero_mando = airbrake_last_command();
  m->servo = airbrake_last_byte();

  memcpy(m->imu1_a, s->imu[0].accel_mps2, sizeof(m->imu1_a));
  memcpy(m->imu1_g, s->imu[0].gyro_rps, sizeof(m->imu1_g));
  memcpy(m->imu2_a, s->imu[1].accel_mps2, sizeof(m->imu2_a));
  memcpy(m->imu2_g, s->imu[1].gyro_rps, sizeof(m->imu2_g));
  m->imu1_t_us = s->imu[0].timestamp_us;
  m->imu2_t_us = s->imu[1].timestamp_us;
  m->baro1_kpa = s->baro[0].pressure_kpa;
  m->baro1_c = s->baro[0].temperature_c;
  m->baro2_kpa = s->baro[1].pressure_kpa;
  m->baro2_c = s->baro[1].temperature_c;
  for (uint32_t i = 0U; i < 3U; i++) {
    m->mag_ut[i] = s->mag.field_t[i] * 1.0e6f;
  }
  m->lat_e7 = s->gps_gga.latitude_e7;
  m->lon_e7 = s->gps_gga.longitude_e7;
  m->alt_mm = s->gps_gga.altitude_mm;
  m->gps_calidad = s->gps_gga.fix_quality;
  m->gps_sats = s->gps_gga.satellites;
  m->hdop = s->gps_gga.hdop_x100;
  m->vn_mmps = s->gps_pvt.vel_n_mmps;
  m->ve_mmps = s->gps_pvt.vel_e_mmps;
  m->vd_mmps = s->gps_pvt.vel_d_mmps;
  m->pvt_fix = s->gps_pvt.fix_type;

  m->continuidad =
      state->pyro_continuity_valid ? state->pyro_continuity : 0xFFU;
  m->v11_mv = state->power_rails_valid ? state->power_rails.voltage_11v_mv
                                       : 0U;
  m->i11_ma = state->power_rails.current_11v_ma;
}

/* === Empezar y terminar ================================================== */

static void _empezar(const char *motivo, uint32_t ahora) {
  s_es_sim = gnc_playback_active();
  if (s_es_sim) {
    /* Los archivos de la simulación se crean ahora: es en tierra, y FileX
     * bloquea unos ms */
    char rel[48];
    s_info.simulaciones++;
    (void)fx_directory_create(s_m, (CHAR *)_ruta("simulaciones"));
    (void)snprintf(rel, sizeof(rel), "simulaciones/sim_%02lu.csv",
                   (unsigned long)s_info.simulaciones);
    const bool ok1 = _abrir(&s_sim_vuelo, rel, RESERVA_SIM);
    (void)snprintf(rel, sizeof(rel), "simulaciones/sim_%02lu_trust_curve.csv",
                   (unsigned long)s_info.simulaciones);
    const bool ok2 = _abrir(&s_sim_empuje, rel, RESERVA_SIM_EMPUJE);
    _flush_y_dirs();
    _guardar_reservas();
    if (!ok1 || !ok2) {
      return;
    }
    s_av = &s_sim_vuelo;
    s_ae = &s_sim_empuje;
  } else {
    s_av = &s_vuelo;
    s_ae = &s_empuje;
    s_info.vuelos++;
  }
  (void)snprintf(s_info.archivo, sizeof(s_info.archivo), "%s", s_av->ruta);
  (void)_poner(s_av, CABECERA_VUELO, sizeof(CABECERA_VUELO) - 1U);
  (void)_poner(s_ae, CABECERA_EMPUJE, sizeof(CABECERA_EMPUJE) - 1U);

  s_grabando = true;
  s_info.estado = ASPID_GRABANDO;
  s_info.grabando_simulacion = s_es_sim;
  s_info.filas = 0U;
  s_info.filas_empuje = 0U;
  s_info.despegue_ms = ahora;
  s_info.altura_max_m = 0.0f;
  s_apogeo_visto = false;
  s_t_limite = ahora;
  s_emp = EMP_GRABANDO;
  s_emp_ini = ahora;
  s_emp_fin = 0U;
  s_info.empuje_grabando = true;
  {
    const gnc_data_t *g = gnc_get_data();
    _evento("sesgos",
            "sesgos del EKF al despegar: acc %.4f %.4f %.4f m/s2  gyr %.5f "
            "%.5f %.5f rad/s",
            (double)g->accel_bias_mps2[0], (double)g->accel_bias_mps2[1],
            (double)g->accel_bias_mps2[2], (double)g->gyro_bias_rps[0],
            (double)g->gyro_bias_rps[1], (double)g->gyro_bias_rps[2]);
  }
  _evento("despegue",
          "DESPEGUE (%s): graba %s y %s, con %lu + %lu filas de antes",
          motivo, s_av->ruta, s_ae->ruta,
          (unsigned long)(s_mu_cab - s_mu_cola),
          (unsigned long)(s_imu_cab - s_imu_cola));
}

/* Dónde cayó: la última posición GPS buena (se apunta a 50 m de altura,
 * bajando: el cohete aún se mueve unos segundos con el viento) y la del GNC
 * respecto a su origen */
static void _donde_cayo(void) {
  const gnc_data_t *g = gnc_get_data();
  const uint32_t ahora = HAL_GetTick();
  if (s_gps.ok) {
    const double lat = (double)s_gps.lat_e7 * 1.0e-7;
    const double lon = (double)s_gps.lon_e7 * 1.0e-7;
    _evento("caida",
            "DONDE CAYO (ultimo GPS bueno; se apunta a %.0f m de altura "
            "bajando): %.7f %.7f  %.1f m MSL  hace %.1f s  %u satelites  "
            "https://maps.google.com/?q=%.7f,%.7f",
            (double)g->alt_agl_m, lat, lon, (double)s_gps.alt_mm * 1.0e-3,
            (double)(ahora - s_gps.t_ms) * 1.0e-3, (unsigned)s_gps.sats, lat,
            lon);
  } else {
    _evento("caida", "DONDE CAYO: sin GPS bueno en toda la sesion");
  }
  _evento("caida_gnc",
          "posicion del GNC al terminar: N %.1f  E %.1f  D %.1f m del origen "
          "(donde se fijo el GPS)",
          (double)g->pos_ned_m[0], (double)g->pos_ned_m[1],
          (double)g->pos_ned_m[2]);
}

static void _terminar(const char *por) {
  s_grabando = false;
  s_cerrando = true;
  s_sellar = !s_es_sim && (s_info.altura_max_m >= ASPID_DATA_VUELO_REAL_M);
  if (s_sellar) {
    _donde_cayo();
  }
  s_cierre = CIERRE_DRENAR;
  if ((s_emp == EMP_GRABANDO) || (s_emp == EMP_ESPERA)) {
    s_emp = EMP_DRENANDO;
  }
  _evento("fin", "FIN (%s): %lu filas, %lu de trust curve, altura max %ld m%s",
          por,
          (unsigned long)s_info.filas, (unsigned long)s_info.filas_empuje,
          (long)s_info.altura_max_m,
          s_sellar ? ". Se cierra todo y la SD ya no se toca."
                   : (s_es_sim ? " (simulacion)"
                               : ". No paso de 100 m: no era un vuelo, se "
                                 "sigue grabando."));
}

/* A 100 Hz: ¿empieza o termina un vuelo? */
static void _vigilar(const muestra_t *m, uint32_t ahora) {
  const gnc_data_t *g = gnc_get_data();
  const bool gnc_vuelo = (g->rocket_state == GNC_ESTADO_EN_VUELO) &&
                         (g->flight_phase != GNC_FASE_EN_TIERRA);
  const bool gnc_flanco = gnc_vuelo && !s_gnc_vuelo_prev;
  s_gnc_vuelo_prev = gnc_vuelo;

  const float a = fmaxf(_norma(m->imu1_a), _norma(m->imu2_a));
  /* Se cuenta TIEMPO por encima de 3 g, no muestras: si el GNC se pone al
   * dia y se salta alguna muestra de 100 Hz, contar muestras alargaria los
   * 100 ms (igual que en camaras.c) */
  if (a >= (ASPID_DATA_DESPEGUE_G * G0)) {
    if (!s_alto) {
      s_alto = true;
      s_alto_usado = false;
      s_t_alto = ahora;
    }
  } else {
    s_alto = false;
  }
  const bool imu_flanco = s_alto && !s_alto_usado &&
                          ((ahora - s_t_alto) >= ASPID_DATA_DESPEGUE_MS);
  if (imu_flanco) {
    s_alto_usado = true;
  }

#if ASPID_DATA_PRUEBA
  const bool prueba_despegue = (ahora >= 30000U) && (s_info.vuelos == 0U);
#else
  const bool prueba_despegue = false;
#endif

  if (!s_grabando) {
    if (gnc_flanco) {
      _empezar("GNC EN VUELO", ahora);
    } else if (imu_flanco) {
      _empezar("IMU por encima de 3 g", ahora);
    } else if (prueba_despegue) {
      _empezar("PRUEBA", ahora);
      camaras_encender(CAMARAS_ORIGEN_DESPEGUE); /* como un despegue */
    }
    return;
  }

  if (gnc_flanco) {
    s_t_limite = ahora; /* el GNC lo confirma: de aquí cuentan los 20 min */
  }
  if ((m->fase >= GNC_FASE_APOGEO) && (m->fase <= GNC_FASE_EN_TIERRA)) {
    s_apogeo_visto = true;
  }
  const bool nav = (m->flags & 0x01U) != 0U;
  if (nav && isfinite(m->alt) && (m->alt > s_info.altura_max_m)) {
    s_info.altura_max_m = m->alt;
  }

#if ASPID_DATA_PRUEBA && ASPID_DATA_PRUEBA_MUERTE
  static bool muerta;
  if (!muerta && (ahora >= 38000U)) {
    /* Los pines del bus de la SD1 a analógico: para el SDMMC es como si la
     * tarjeta no estuviera (ni contesta ni confirma lo escrito) */
    muerta = true;
    GPIO_InitTypeDef g = {0};
    g.Mode = GPIO_MODE_ANALOG;
    g.Pull = GPIO_NOPULL;
    g.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
    HAL_GPIO_Init(GPIOC, &g);
    g.Pin = GPIO_PIN_2;
    HAL_GPIO_Init(GPIOD, &g);
    DebugOut_Printf("PRUEBA: SD1 desconectada a los %lu ms\r\n",
                    (unsigned long)ahora);
  }
#endif
#if ASPID_DATA_PRUEBA
  if (!s_es_sim && (ahora >= 45000U)) {
    s_apogeo_visto = true;
    s_info.altura_max_m = 1000.0f;
  }
#endif

  if (s_apogeo_visto && nav && isfinite(m->alt) &&
      (m->alt <= ASPID_DATA_FIN_ALTURA_M)) {
    _terminar("bajando de 50 m");
  } else if (m->fase == GNC_FASE_EN_TIERRA) {
    _terminar("GNC EN TIERRA");
  } else if ((ahora - s_t_limite) >= ASPID_DATA_MAX_VUELO_MS) {
    _terminar("20 minutos");
  }
}

static void _muestrear(const sys_state *state, uint32_t ahora) {
  if (s_cerrando) {
    return; /* lo que venga después ya no es de ese archivo */
  }
  /* El fin de una simulación se mira antes de tomar la muestra: tras ella
   * el GNC se reinicia y la fila saldría a ceros */
  if (s_grabando && s_es_sim && !gnc_playback_active()) {
    _terminar("fin de la simulacion");
    return;
  }
  const gnc_data_t *g = gnc_get_data();
  const uint32_t bloque = g->steps / (GNC_RATE_HZ / ASPID_DATA_HZ);
  if (bloque == s_bloque_gnc) {
    return;
  }
  s_bloque_gnc = bloque;

  muestra_t *m = &s_mu[s_mu_cab % MUESTRAS_N];
  _tomar(m, state, ahora);
  s_mu_cab++;
  const uint32_t limite = s_grabando ? MUESTRAS_N : ANTES_N;
  while ((s_mu_cab - s_mu_cola) > limite) {
    s_mu_cola++;
    if (s_grabando) {
      s_info.filas_perdidas++;
    }
  }
  _vigilar(m, ahora);
}

/* Una fila de IMU por tick del GNC (400 Hz) */
static void _muestrear_imu(uint32_t ahora) {
  const gnc_data_t *g = gnc_get_data();
  if (g->steps == s_steps_imu) {
    return;
  }
  s_steps_imu = g->steps;

  if (s_emp == EMP_DRENANDO) {
    if ((s_imu_cola == s_imu_cab) && (s_ae != NULL)) {
      s_ae->cerrando = true; /* se escribe lo que queda y su tamaño */
      s_emp = EMP_HECHO;
      s_info.empuje_grabando = false;
    }
    return;
  }
  if ((s_emp == EMP_HECHO) || s_cerrando) {
    return;
  }

  const sensor_board_data_t *s = sensor_board_get_data();
  imu_t *m = &s_imu[s_imu_cab % IMU_N];
  m->t_ms = ahora;
  m->tick = g->steps;
  m->fase = (uint8_t)g->flight_phase;
  m->n1 = (uint8_t)(s->imu[0].count - s_cnt_imu1);
  m->n2 = (uint8_t)(s->imu[1].count - s_cnt_imu2);
  s_cnt_imu1 = s->imu[0].count;
  s_cnt_imu2 = s->imu[1].count;
  memcpy(m->a1, s->imu[0].accel_mps2, sizeof(m->a1));
  memcpy(m->g1, s->imu[0].gyro_rps, sizeof(m->g1));
  memcpy(m->a2, s->imu[1].accel_mps2, sizeof(m->a2));
  memcpy(m->g2, s->imu[1].gyro_rps, sizeof(m->g2));
  m->t1_us = s->imu[0].timestamp_us;
  m->t2_us = s->imu[1].timestamp_us;
  s_imu_cab++;

  const bool grabando = (s_emp == EMP_GRABANDO);
  const uint32_t limite = grabando ? IMU_N : IMU_ANTES_N;
  while ((s_imu_cab - s_imu_cola) > limite) {
    s_imu_cola++;
    if (grabando) {
      s_info.filas_perdidas++;
    }
  }

  if (grabando) {
    /* Fin de la ventana: 5 s después de que el GNC dé el motor por
     * apagado, o como mucho 30 s desde el despegue */
    if ((s_emp_fin == 0U) && (g->flight_phase >= GNC_FASE_COAST) &&
        (g->flight_phase <= GNC_FASE_EN_TIERRA)) {
      s_emp_fin = ahora + ASPID_DATA_EMPUJE_DESPUES_MS;
    }
    if (((s_emp_fin != 0U) && ((int32_t)(ahora - s_emp_fin) >= 0)) ||
        ((ahora - s_emp_ini) >= ASPID_DATA_EMPUJE_MAX_MS)) {
      s_emp = EMP_DRENANDO;
      _evento("trust_curve", "trust curve terminada (%s): %lu filas",
              (s_emp_fin != 0U) ? "5 s tras el motor" : "30 s",
              (unsigned long)s_info.filas_empuje);
    }
  }
}

static void _formatear(void) {
  if ((s_grabando || s_cerrando) && (s_av != NULL) && !s_av->cerrando) {
    for (uint32_t k = 0U;
         (k < FORMATO_VUELO_VUELTA) && (s_mu_cola != s_mu_cab); k++) {
      if (_sitio(s_av) < FILA_MAX) {
        break; /* la tarjeta va lenta: espera en el anillo de muestras */
      }
      const uint32_t t0 = _t();
      char *p = _fila(s_fila, &s_mu[s_mu_cola % MUESTRAS_N]);
      const uint32_t us = _us(t0);
      if (us > s_info.formato_max_us) {
        s_info.formato_max_us = us;
      }
      (void)_poner(s_av, s_fila, (uint32_t)(p - s_fila));
      s_mu_cola++;
      s_info.filas++;
      s_info.filas_total++;
    }
  }
  if (((s_emp == EMP_GRABANDO) || (s_emp == EMP_DRENANDO)) &&
      (s_ae != NULL) && !s_ae->cerrando) {
    for (uint32_t k = 0U;
         (k < FORMATO_IMU_VUELTA) && (s_imu_cola != s_imu_cab); k++) {
      if (_sitio(s_ae) < LINEA_MAX) {
        break;
      }
      char *p = _fila_imu(s_fila, &s_imu[s_imu_cola % IMU_N]);
      (void)_poner(s_ae, s_fila, (uint32_t)(p - s_fila));
      s_imu_cola++;
      s_info.filas_empuje++;
    }
  }
}

/* === Hora del GPS y nombre de la carpeta ================================ */

static void _hora_gps(uint32_t ahora) {
  if (s_hora) {
    return;
  }
#if ASPID_DATA_PRUEBA
  if (ahora >= 15000U) {
    /* 2000-01-01 12:00:00 UTC en este instante: falsa y que se note */
    s_utc0_ms = ((int64_t)_dias(2000, 1U, 1U) * 86400000) + 43200000 -
                (int64_t)ahora;
    s_hora = true;
  }
#endif
  const gps_rmc_data_t *r = &sensor_board_get_data()->gps_rmc;
  if (!s_hora && (r->count != s_rmc_visto)) {
    s_rmc_visto = r->count;
    if ((r->valid == 1U) && (r->year >= 2024U) && (r->month >= 1U) &&
        (r->month <= 12U) && (r->day >= 1U) && (r->day <= 31U) &&
        (r->time_ms < 86400000U)) {
      s_utc0_ms = ((int64_t)_dias((int32_t)r->year, r->month, r->day) *
                   86400000) +
                  (int64_t)r->time_ms - (int64_t)ahora;
      s_hora = true;
    }
  }
  if (s_hora) {
    s_info.hora_conocida = true;
    char f[32];
    *_fecha(f, s_utc0_ms, false) = '\0';
    _evento("hora",
            "hora UTC del GPS: t_ms = 0 fue %s (+-0,2 s). Desde aqui las "
            "filas llevan hora_utc.", f);
    s_renombrar = true;
  }
}

static void _renombrar_carpeta(void) {
  s_renombrar = false;
  char nuevo[48];
  char *p = _s(nuevo, "aspid_data_");
  p = _fecha(p, s_utc0_ms, true);
  *p = '\0';
  for (uint32_t i = 1U; i <= 9U; i++) {
    if (i > 1U) {
      p[0] = '_';
      p[1] = (char)('0' + i);
      p[2] = '\0';
    }
    const uint32_t t0 = _t();
    const UINT st =
        fx_directory_rename(s_m, (CHAR *)s_info.carpeta, (CHAR *)nuevo);
    _filex_tiempo(t0);
    if (st == FX_SUCCESS) {
      _evento("carpeta", "carpeta renombrada: %s -> %s", s_info.carpeta,
              nuevo);
      (void)snprintf(s_info.carpeta, sizeof(s_info.carpeta), "%s", nuevo);
      _guardar_reservas();
      _flush_y_dirs();
      return;
    }
    if (st != FX_ALREADY_CREATED) {
      _error(st);
      _evento("carpeta", "no se pudo renombrar la carpeta (FileX 0x%02X)",
              (unsigned)st);
      return;
    }
  }
}

/* === Escritura por DMA =================================================== */

static void _lanzar(trabajo_t t, flujo_t *f, uint32_t n, uint32_t lba,
                    const uint8_t *buf, uint32_t sectores) {
  if (lba == 0U) {
    _error(FX_SECTOR_INVALID); /* fuera de lo reservado: no debería pasar */
    return;
  }
  if (!sd_card_write_start(ASPID_SD, buf, lba, sectores)) {
    _tarjeta_perdida();
    return;
  }
  s_trab = t;
  s_trab_f = f;
  s_trab_n = n;
  s_trab_t0 = _t();
}

static void _acabar_trabajo(void) {
  flujo_t *f = s_trab_f;
  const uint32_t us = _us(s_trab_t0);
  if (us > s_info.escritura_max_us) {
    s_info.escritura_max_us = us;
  }
  if (us > s_escr_max_seg_us) {
    s_escr_max_seg_us = us;
  }
  s_info.escrituras++;
  switch (s_trab) {
  case T_DATOS:
    f->escrito += BLOQUE;
    if (f->bytes != NULL) {
      *f->bytes += BLOQUE;
    }
    break;
  case T_PARCIAL:
    f->sync_cab = s_trab_n;
    break;
  case T_DIR:
    f->en_dir = s_trab_n;
    /* Que FileX diga lo mismo si un día escribe esta entrada */
    f->file.fx_file_current_file_size = s_trab_n;
    f->file.fx_file_dir_entry.fx_dir_entry_file_size = s_trab_n;
    f->file.fx_file_modified = FX_TRUE;
    break;
  case T_NADA:
  default:
    break;
  }
  s_trab = T_NADA;
}

/* Lo que tenga f a medias y su tamaño en la carpeta. true si lanzó algo */
static bool _lanzar_sync(flujo_t *f) {
  if (!f->abierto) {
    return false;
  }
  if (f->cab > _en_tarjeta(f)) {
    /* Menos de un bloque (los enteros ya han salido), desde un múltiplo de
     * 4 KB: contiguo en el anillo. Se escribe el sector entero; lo que pase
     * del final no cuenta, el tamaño lo dice la carpeta. */
    const uint32_t sect = ((f->cab - f->escrito) + SECTOR - 1U) / SECTOR;
    _lanzar(T_PARCIAL, f, f->cab, _lba(f, f->escrito),
            &f->anillo[f->escrito % f->tam], sect);
    return true;
  }
  const uint32_t en = _en_tarjeta(f);
  if ((f->dir_hueco >= 0) && (en != f->en_dir)) {
    dir_t *d = &s_dir[f->dir_hueco];
    uint8_t *e = &d->buf[f->dir_off + 28U];
    e[0] = (uint8_t)en;
    e[1] = (uint8_t)(en >> 8);
    e[2] = (uint8_t)(en >> 16);
    e[3] = (uint8_t)(en >> 24);
    _lanzar(T_DIR, f, en, d->lba, d->buf, 1U);
    return true;
  }
  return false;
}

/* Volcado de cada segundo: a cada archivo, como mucho su trozo a medias y
 * su tamaño UNA vez por ronda. Sin esto, uno que no para de crecer (el de
 * vuelo) acapararía la ronda: mientras se escribe su trozo llegan datos
 * nuevos y vuelta a empezar. */
static bool _ronda(flujo_t *f) {
  if (!f->abierto || (f->ronda == 0U)) {
    return false;
  }
  if (f->ronda == 1U) {
    f->ronda = 2U;
    if (f->cab > _en_tarjeta(f)) {
      const uint32_t sect = ((f->cab - f->escrito) + SECTOR - 1U) / SECTOR;
      _lanzar(T_PARCIAL, f, f->cab, _lba(f, f->escrito),
              &f->anillo[f->escrito % f->tam], sect);
      return true;
    }
  }
  f->ronda = 0U;
  const uint32_t en = _en_tarjeta(f);
  if ((f->dir_hueco >= 0) && (en != f->en_dir)) {
    dir_t *d = &s_dir[f->dir_hueco];
    uint8_t *e = &d->buf[f->dir_off + 28U];
    e[0] = (uint8_t)en;
    e[1] = (uint8_t)(en >> 8);
    e[2] = (uint8_t)(en >> 16);
    e[3] = (uint8_t)(en >> 24);
    _lanzar(T_DIR, f, en, d->lba, d->buf, 1U);
    return true;
  }
  return false;
}

/* Con el DMA libre: ¿qué se escribe ahora? true si lanzó algo */
static bool _siguiente(uint32_t ahora) {
  /* 1. Bloques enteros: el que más tenga pendiente */
  flujo_t *mejor = NULL;
  uint32_t max = 0U;
  for (uint32_t i = 0U; i < N_FLUJOS; i++) {
    flujo_t *f = s_flujos[i];
    const uint32_t pend = f->cab - f->escrito;
    if (f->abierto && (pend >= BLOQUE) && (pend > max)) {
      mejor = f;
      max = pend;
    }
  }
  if (mejor != NULL) {
    _lanzar(T_DATOS, mejor, BLOQUE, _lba(mejor, mejor->escrito),
            &mejor->anillo[mejor->escrito % mejor->tam], BLOQUE / SECTOR);
    return true;
  }
  /* 2. Los que se cierran: su último trozo y su tamaño, ya */
  for (uint32_t i = 0U; i < N_FLUJOS; i++) {
    if (s_flujos[i]->cerrando && _lanzar_sync(s_flujos[i])) {
      return true;
    }
  }
  /* 3. Cada segundo, todos, una vez cada uno */
  if ((s_sync_i >= N_FLUJOS) &&
      ((ahora - s_ultimo_sync) >= ASPID_DATA_SYNC_MS)) {
    s_sync_i = 0U;
    s_ultimo_sync = ahora;
    for (uint32_t i = 0U; i < N_FLUJOS; i++) {
      s_flujos[i]->ronda = 1U;
    }
  }
  while (s_sync_i < N_FLUJOS) {
    if (_ronda(s_flujos[s_sync_i])) {
      return true;
    }
    s_sync_i++;
  }
  return false;
}

/* === Eventos y salud ===================================================== */

static const char *_nombre_estado(int32_t e) {
  static const char *const n[] = {"POWER_ON", "MANTENIMIENTO", "EN_VUELO",
                                  "ERROR"};
  return ((e >= 0) && (e < 4)) ? n[e] : "?";
}

static const char *_nombre_fase(int32_t f) {
  static const char *const n[] = {"IDLE",   "BOOST",  "COAST", "CONTROL",
                                  "APOGEO", "DROGUE", "MAIN",  "EN_TIERRA"};
  return ((f >= 0) && (f < 8)) ? n[f] : "?";
}

/* Los de state_machine.c, como en tools/estacion/paquete.py */
static const char *_nombre_error(uint8_t e) {
  static const char *const n[] = {
      "sin error", "NAV INVALIDA DEMASIADO TIEMPO", "ESTANCAMIENTO",
      "ENTRADAS NO FINITAS", "PARADA ORDENADA DESDE TIERRA"};
  return (e < 5U) ? n[e] : "desconocido";
}

static const char *_piro_nombre(uint8_t canal) {
  static const char *const n[] = {"?", "1A", "1B", "2A", "2B"};
  return (canal < 5U) ? n[canal] : "?";
}

static const char *_si(bool v) { return v ? "si" : "--"; }

/* En cada vuelta: ¿ha cambiado algo? */
static void _vigilar_eventos(const sys_state *st, uint32_t ahora) {
  const gnc_data_t *g = gnc_get_data();
  const sensor_board_data_t *sb = sensor_board_get_data();

  /* Última posición GPS buena */
  const gps_gga_data_t *gg = &sb->gps_gga;
  if (gg->count != s_gps.cnt) {
    s_gps.cnt = gg->count;
    if ((gg->fix_quality > 0U) &&
        (gg->latitude_e7 != SENSOR_BOARD_GPS_INVALID_I32) &&
        (gg->longitude_e7 != SENSOR_BOARD_GPS_INVALID_I32)) {
      if (!s_gps.ok) {
        _ev("gps_fix", gg->satellites,
            "primer fix del GPS: %.7f %.7f con %u satelites",
            (double)gg->latitude_e7 * 1.0e-7,
            (double)gg->longitude_e7 * 1.0e-7, (unsigned)gg->satellites);
      }
      s_gps.ok = true;
      s_gps.lat_e7 = gg->latitude_e7;
      s_gps.lon_e7 = gg->longitude_e7;
      s_gps.alt_mm = gg->altitude_mm;
      s_gps.sats = gg->satellites;
      s_gps.t_ms = ahora;
    }
  }

  const bool sim = gnc_playback_active();
  const bool test = uplink_modo_test();
  const bool aero = airbrake_prueba_activa();
  const uint8_t cont = st->pyro_continuity;
  if (!s_prev.listo) {
    s_prev.listo = true;
    s_prev.estado = g->rocket_state;
    s_prev.fase = g->flight_phase;
    s_prev.error = g->error_code;
    s_prev.nav = g->nav_valid;
    s_prev.gnss = g->gnss_only;
    s_prev.armada = g->pyro_armed;
    s_prev.drogue = g->drogue_flag;
    s_prev.main = g->main_flag;
    s_prev.o1a = despliegue_ordenes_1a();
    s_prev.o1b = despliegue_ordenes_1b();
    s_prev.o2a = despliegue_ordenes_2a();
    s_prev.fallos = despliegue_fallos();
    s_prev.cont_ok = st->pyro_continuity_valid;
    s_prev.cont = cont;
    s_prev.test = test;
    s_prev.sim = sim;
    s_prev.aero = aero;
    s_prev.piros = uplink_piros_aceptados();
    s_prev.rampa = lanzamiento_centesimas();
    s_prev.rumbo = lanzamiento_rumbo_centesimas();
    s_prev.power = st->power;
    s_prev.avionica = st->avionics;
    s_prev.rf = st->rf;
    s_prev.resets = sb->board_resets;
    s_prev.camaras = camaras_encendidas();
    s_prev.camaras_apagando = camaras_apagando();
    return;
  }
  if (camaras_encendidas() != s_prev.camaras) {
    s_prev.camaras = camaras_encendidas();
    static const char *const origen[] = {"", "por el despegue",
                                         "desde tierra",
                                         "al terminar el vuelo (50 m)"};
    const bool por_plazo = camaras_cortes_por_plazo() != s_prev.cortes_camaras;
    s_prev.cortes_camaras = camaras_cortes_por_plazo();
    _ev("camaras", s_prev.camaras ? 1 : 0, "camaras %s %s%s",
        s_prev.camaras ? "ENCENDIDAS" : "apagadas",
        origen[camaras_origen()],
        por_plazo ? " (corriente cortada POR PLAZO, sin confirmar que "
                    "pararan de grabar)"
                  : "");
  }
  if (camaras_apagando() != s_prev.camaras_apagando) {
    s_prev.camaras_apagando = camaras_apagando();
    if (s_prev.camaras_apagando) {
      _ev("camaras", 2, "apagando camaras: parando la grabacion antes de "
                        "cortar la corriente");
    }
  }
  /* Cada camara: estado, NO GRABA, reloj puesto */
  for (uint32_t i = 0U; i < CAMARAS_N; i++) {
    const camara_t *c = camaras_camara(i);
    const uint32_t f = (uint32_t)c->estado | (c->no_graba ? 0x100U : 0U) |
                       (c->reloj_ok ? 0x200U : 0U);
    if (f != s_prev.camara_firma[i]) {
      s_prev.camara_firma[i] = f;
      _ev("camara", (int32_t)c->estado,
          "camara %u %s%s, %s, SD %lu/%lu MB, quedan %lu s, reloj %s, "
          "boton %lu",
          (unsigned)c->num, camara_estado_nombre(c->estado),
          c->no_graba ? " NO GRABA" : "",
          c->ajustes_leidos ? c->resolucion : "?",
          (unsigned long)c->libre_mb, (unsigned long)c->total_mb,
          (unsigned long)c->queda_s,
          c->reloj_ok ? c->hora_camara : "sin poner",
          (unsigned long)c->pulsaciones);
    }
  }

  if (g->rocket_state != s_prev.estado) {
    _ev("estado", g->rocket_state, "GNC %s -> %s",
        _nombre_estado(s_prev.estado), _nombre_estado(g->rocket_state));
    s_prev.estado = g->rocket_state;
  }
  if (g->flight_phase != s_prev.fase) {
    _ev("fase", g->flight_phase, "fase %s -> %s (altura %.1f m%s)",
        _nombre_fase(s_prev.fase), _nombre_fase(g->flight_phase),
        (double)g->alt_agl_m, sim ? ", SIMULACION" : "");
    s_prev.fase = g->flight_phase;
  }
  if (g->error_code != s_prev.error) {
    _ev("error_gnc", g->error_code, "error del GNC: %s",
        _nombre_error(g->error_code));
    s_prev.error = g->error_code;
  }
  if (g->nav_valid != s_prev.nav) {
    _ev("nav", g->nav_valid ? 1 : 0,
        g->nav_valid ? "navegacion valida" : "navegacion NO valida");
    s_prev.nav = g->nav_valid;
    if (g->nav_valid && !s_prev.alineado && !sim) {
      s_prev.alineado = true;
      _evento("alineado",
              "EKF alineado: rampa %.2f grados desde la vertical rumbo %.2f; "
              "gravedad medida a %.2f grados de la esperada; sesgos acc %.4f "
              "%.4f %.4f m/s2 gyr %.5f %.5f %.5f rad/s",
              (double)g->rampa_vertical_deg, (double)g->rampa_rumbo_deg,
              (double)g->rampa_error_deg, (double)g->accel_bias_mps2[0],
              (double)g->accel_bias_mps2[1], (double)g->accel_bias_mps2[2],
              (double)g->gyro_bias_rps[0], (double)g->gyro_bias_rps[1],
              (double)g->gyro_bias_rps[2]);
    }
  }
  if (g->gnss_only != s_prev.gnss) {
    _ev("solo_gnss", g->gnss_only ? 1 : 0,
        g->gnss_only ? "el EKF pasa a usar solo el GNSS"
                     : "el EKF vuelve a usar la inercial");
    s_prev.gnss = g->gnss_only;
  }
  if (g->pyro_armed != s_prev.armada) {
    _ev("piro_armada", g->pyro_armed ? 1 : 0,
        g->pyro_armed ? "el guiado arma la pirotecnia (por encima de 100 m)"
                      : "pirotecnia desarmada");
    s_prev.armada = g->pyro_armed;
  }
  if (g->drogue_flag && !s_prev.drogue) {
    _ev("gnc_drogue", 1, "el GNC pide el drogue (altura %.1f m)",
        (double)g->alt_agl_m);
  }
  s_prev.drogue = g->drogue_flag;
  if (g->main_flag && !s_prev.main) {
    _ev("gnc_main", 1, "el GNC pide el main (altura %.1f m)",
        (double)g->alt_agl_m);
  }
  s_prev.main = g->main_flag;

  const uint32_t o1a = despliegue_ordenes_1a();
  const uint32_t o1b = despliegue_ordenes_1b();
  const uint32_t o2a = despliegue_ordenes_2a();
  const uint32_t fallos = despliegue_fallos();
  if (o1a != s_prev.o1a) {
    _ev("despliegue_1a", (int32_t)o1a, "power acepta la orden de 1A (%lu)",
        (unsigned long)o1a);
    s_prev.o1a = o1a;
  }
  if (o1b != s_prev.o1b) {
    _ev("despliegue_1b", (int32_t)o1b, "power acepta la orden de 1B (%lu)",
        (unsigned long)o1b);
    s_prev.o1b = o1b;
  }
  if (o2a != s_prev.o2a) {
    _ev("despliegue_2a", (int32_t)o2a, "power acepta la orden de 2A (%lu)",
        (unsigned long)o2a);
    s_prev.o2a = o2a;
  }
  if (fallos != s_prev.fallos) {
    _ev("despliegue_fallo", (int32_t)fallos,
        "power NO acepta una orden de despliegue (%lu en total)",
        (unsigned long)fallos);
    s_prev.fallos = fallos;
  }

  if (st->pyro_continuity_valid &&
      (!s_prev.cont_ok || (cont != s_prev.cont))) {
    _ev("continuidad", cont, "continuidad 1A %s  1B %s  2A %s  2B %s",
        _si((cont & SYS_PYRO_1A) != 0U), _si((cont & SYS_PYRO_1B) != 0U),
        _si((cont & SYS_PYRO_2A) != 0U), _si((cont & SYS_PYRO_2B) != 0U));
    s_prev.cont = cont;
  }
  s_prev.cont_ok = st->pyro_continuity_valid;

  if (test != s_prev.test) {
    _ev("modo_test", test ? 1 : 0,
        test ? "entra en modo test" : "sale del modo test");
    s_prev.test = test;
  }
  if (sim != s_prev.sim) {
    _ev("simulacion", sim ? 1 : 0,
        sim ? "empieza SIMULAR RUTA" : "termina la simulacion");
    s_prev.sim = sim;
  }
  if (aero != s_prev.aero) {
    _ev("aerofreno_prueba", aero ? 1 : 0,
        aero ? "prueba de aerofreno desde tierra" : "fin de la prueba de "
                                                     "aerofreno");
    s_prev.aero = aero;
  }
  const uint32_t piros = uplink_piros_aceptados();
  if (piros != s_prev.piros) {
    _ev("piro_manual", uplink_ultimo_piro(),
        "pirotecnico %s disparado desde tierra (modo test)",
        _piro_nombre(uplink_ultimo_piro()));
    s_prev.piros = piros;
  }
  const uint16_t rampa = lanzamiento_centesimas();
  const uint16_t rumbo = lanzamiento_rumbo_centesimas();
  if ((rampa != s_prev.rampa) || (rumbo != s_prev.rumbo)) {
    _ev("rampa", rampa, "rampa %.2f grados desde la vertical  rumbo %.2f",
        (double)rampa * 0.01, (double)rumbo * 0.01);
    s_prev.rampa = rampa;
    s_prev.rumbo = rumbo;
  }
  if (st->power != s_prev.power) {
    _ev("placa", st->power ? 1 : 0, "power %s",
        st->power ? "responde" : "NO responde");
    s_prev.power = st->power;
  }
  if (st->avionics != s_prev.avionica) {
    _ev("placa", st->avionics ? 1 : 0, "avionica %s",
        st->avionics ? "responde" : "NO responde");
    s_prev.avionica = st->avionics;
  }
  if (st->rf != s_prev.rf) {
    _ev("placa", st->rf ? 1 : 0, "rf %s", st->rf ? "responde" : "NO responde");
    s_prev.rf = st->rf;
  }
  if (sb->board_resets != s_prev.resets) {
    _ev("avionica_reset", (int32_t)sb->board_resets,
        "la placa de sensores se ha reiniciado (su reloj volvio a 0)");
    s_prev.resets = sb->board_resets;
  }
}

/* Una fila de salud por segundo, y los contadores que han subido, como
 * eventos */
static void _fila_salud(const sys_state *st, uint32_t ahora) {
  const gnc_data_t *g = gnc_get_data();
  const sensor_board_data_t *sb = sensor_board_get_data();
  can_driver_stats_t c[2];
  can_driver_get_stats(CAN_PORT_1, &c[0]);
  can_driver_get_stats(CAN_PORT_2, &c[1]);
  const uint32_t cnt[6] = {sb->imu[0].count,  sb->imu[1].count,
                           sb->baro[0].count, sb->baro[1].count,
                           sb->mag.count,     sb->gps_gga.count};
  static const char *const reg[] = {"SIN_SD", "ESPERANDO", "GRABANDO",
                                    "SELLADO"};

  char *p = s_fila;
#define C (*p++ = ',')
  p = _u32(p, ahora);
  C;
  p = _utc(p, ahora);
  C;
  p = _s(p, reg[s_info.estado]);
  C;
  p = _u32(p, s_bucle_n);
  C;
  p = _fijo(p, s_bucle_max_us / 100U, 1);
  C;
  p = _u32(p, g->steps - s_steps_salud);
  C;
  p = _u32(p, s_tick_max_us);
  C;
  p = _u32(p, g->overruns);
  C;
  p = _u32(p, g->ticks_recuperados);
  C;
  p = _i32(p, g->rocket_state);
  C;
  p = _i32(p, g->flight_phase);
  C;
  p = _u32(p, g->error_code);
  C;
  *p++ = g->nav_valid ? '1' : '0';
  C;
  p = _u32(p, g->align_count);
  for (uint32_t i = 0U; i < 6U; i++) {
    C;
    p = _u32(p, cnt[i] - s_cnt_salud[i]);
    s_cnt_salud[i] = cnt[i];
  }
  C;
  p = _u32(p, sb->board_resets);
  C;
  p = _u32(p, status_packet_can_hz(0U));
  C;
  p = _u32(p, status_packet_can_hz(1U));
  for (uint32_t i = 0U; i < 2U; i++) {
    C;
    p = _u32(p, c[i].rx_overflows);
    C;
    p = _u32(p, c[i].fifo_msg_lost);
    C;
    p = _u32(p, c[i].tx_errors);
  }
  const uint32_t radio[8] = {
      rf_telemetry_sent(),          rf_telemetry_failed(),
      rf_telemetry_estado_enviados(), rf_telemetry_acuses_enviados(),
      uplink_recibidos(),           uplink_validos(),
      uplink_rechazados(),          uplink_denegados()};
  for (uint32_t i = 0U; i < 8U; i++) {
    C;
    p = _u32(p, radio[i]);
  }
  C;
  *p++ = st->power ? '1' : '0';
  C;
  *p++ = st->avionics ? '1' : '0';
  C;
  *p++ = st->rf ? '1' : '0';
  C;
  p = _u32(p, st->wake_up_retries);
  const sys_power_rails *r = &st->power_rails;
  const uint16_t rail[6] = {r->voltage_3v3_mv, r->current_3v3_ma,
                            r->voltage_5v_mv,  r->current_5v_ma,
                            r->voltage_11v_mv, r->current_11v_ma};
  for (uint32_t i = 0U; i < 6U; i++) {
    C;
    if (st->power_rails_valid) {
      p = _fijo(p, rail[i], 3);
    }
  }
  for (uint32_t b = 0U; b < 4U; b++) {
    C;
    if (st->pyro_continuity_valid) {
      *p++ = (st->pyro_continuity & (1U << b)) ? '1' : '0';
    }
  }
  C;
  p = _u32(p, s_info.escrituras);
  C;
  p = _fijo(p, s_escr_max_seg_us / 100U, 1);
  C;
  p = _u32(p, s_info.bytes_perdidos);
  C;
  p = _u32(p, s_info.filas_perdidas);
  C;
  p = _u32(p, s_info.errores);
  C;
  p = _u32(p, DebugOut_DroppedBytes());
  C;
  *p++ = camaras_encendidas() ? '1' : '0';
  p = _s(p, "\r\n");
#undef C
  if (_poner(&s_salud, s_fila, (uint32_t)(p - s_fila))) {
    s_info.filas_salud++;
  }

  /* Lo que no debería subir, como evento */
  if (g->overruns != s_perdidos_prev) {
    _ev("gnc_ticks_perdidos", (int32_t)(g->overruns - s_perdidos_prev),
        "el GNC perdio %lu ticks en el ultimo segundo (vuelta de bucle mas "
        "larga %.1f ms)",
        (unsigned long)(g->overruns - s_perdidos_prev),
        (double)s_bucle_max_us * 1.0e-3);
    s_perdidos_prev = g->overruns;
  }
  for (uint32_t i = 0U; i < 2U; i++) {
    const uint32_t err = c[i].rx_overflows + c[i].fifo_msg_lost +
                         c[i].tx_errors + c[i].bus_off;
    if (err != s_can_err_prev[i]) {
      _ev((i == 0U) ? "can1" : "can2", (int32_t)(err - s_can_err_prev[i]),
          "CAN%lu: desbordes %lu  perdidas %lu  errores tx %lu  bus-off %lu "
          "(en total)",
          (unsigned long)(i + 1U), (unsigned long)c[i].rx_overflows,
          (unsigned long)c[i].fifo_msg_lost, (unsigned long)c[i].tx_errors,
          (unsigned long)c[i].bus_off);
      s_can_err_prev[i] = err;
    }
  }
  if (rf_telemetry_failed() != s_radio_fallo_prev) {
    _ev("radio_fallo", (int32_t)(rf_telemetry_failed() - s_radio_fallo_prev),
        "paquetes de radio que no se pudieron pasar a la RF por CAN");
    s_radio_fallo_prev = rf_telemetry_failed();
  }
  if (s_info.bytes_perdidos != s_sd_perdidos_prev) {
    _ev("sd_perdidos", (int32_t)(s_info.bytes_perdidos - s_sd_perdidos_prev),
        "bytes que no cupieron en RAM esperando a la tarjeta");
    s_sd_perdidos_prev = s_info.bytes_perdidos;
  }

  s_steps_salud = g->steps;
  s_bucle_n = 0U;
  s_bucle_max_us = 0U;
  s_tick_max_us = 0U;
  s_escr_max_seg_us = 0U;
}

/* === Cerrar ============================================================== */

static void _paso_cierre(void) {
  switch (s_cierre) {
  case CIERRE_DRENAR:
    /* Que se formatee todo lo que quedaba en los anillos de muestras */
    if ((s_mu_cola != s_mu_cab) || (s_emp == EMP_DRENANDO)) {
      return;
    }
    if (s_av != NULL) {
      s_av->cerrando = true;
    }
    if (s_ae != NULL) {
      s_ae->cerrando = true;
    }
    if (s_sellar) {
      _evento("sellado", "SD SELLADA: no se escribe nada mas hasta el "
                         "proximo arranque");
      s_telem.cerrando = true;
      s_sesion.cerrando = true;
      s_eventos.cerrando = true;
      s_salud.cerrando = true;
    }
    s_cierre = CIERRE_DATOS;
    break;
  case CIERRE_DATOS:
    for (uint32_t i = 0U; i < N_FLUJOS; i++) {
      const flujo_t *f = s_flujos[i];
      if (f->abierto && f->cerrando && (_en_tarjeta(f) != f->cab)) {
        return; /* aún sale por DMA */
      }
    }
    s_cierre = CIERRE_FILEX;
    break;
  case CIERRE_FILEX:
    for (uint32_t i = 0U; i < N_FLUJOS; i++) {
      if (s_flujos[i]->abierto && s_flujos[i]->cerrando) {
        _finalizar(s_flujos[i], !s_sellar);
        return; /* uno por vuelta */
      }
    }
    s_cierre = CIERRE_FLUSH;
    break;
  case CIERRE_FLUSH:
    _flush_y_dirs();
    s_cierre = CIERRE_SIGUIENTE;
    break;
  case CIERRE_SIGUIENTE:
  default:
#if ASPID_DATA_RELEER
    (void)snprintf(s_releer_ruta[0], sizeof(s_releer_ruta[0]), "%s",
                   (s_av != NULL) ? s_av->ruta : "");
    (void)snprintf(s_releer_ruta[1], sizeof(s_releer_ruta[1]), "%s",
                   (s_ae != NULL) ? s_ae->ruta : "");
    s_releer = true;
#endif
    s_cierre = CIERRE_NADA;
    s_cerrando = false;
    s_av = NULL;
    s_ae = NULL;
    s_mu_cola = s_mu_cab; /* lo de durante el cierre no vale de "antes" */
    s_imu_cola = s_imu_cab;
    s_emp = EMP_ESPERA;
    s_info.grabando_simulacion = false;
    s_info.empuje_grabando = false;
    if (s_sellar) {
      s_info.estado = ASPID_SELLADO;
      return;
    }
    if (!s_es_sim) {
      /* No era un vuelo: los siguientes, ya abiertos y reservados */
      char rel[48];
      (void)snprintf(rel, sizeof(rel), "vuelo/vuelo_%02lu.csv",
                     (unsigned long)(s_info.vuelos + 1U));
      (void)_abrir(&s_vuelo, rel, RESERVA_VUELO);
      (void)snprintf(rel, sizeof(rel), "trust_curve/trust_curve_%02lu.csv",
                     (unsigned long)(s_info.vuelos + 1U));
      (void)_abrir(&s_empuje, rel, RESERVA_EMPUJE);
      _guardar_reservas();
      _flush_y_dirs();
    }
    if (s_info.estado != ASPID_SIN_SD) {
      s_info.estado = ASPID_ESPERANDO;
    }
    break;
  }
}

/* Con el DMA parado: lo que hace falta de FileX. Nunca grabando. */
static void _filex(void) {
  if (s_cerrando) {
    _paso_cierre();
  } else if (s_grabando) {
    return;
  } else if (s_renombrar) {
    _renombrar_carpeta();
  } else {
    /* Los que crecen todo el rato en tierra: otro tramo si queda poco */
    for (uint32_t i = 0U; i < N_FLUJOS; i++) {
      flujo_t *f = s_flujos[i];
      if (f->abierto && !f->cerrando && (f->ampliar > 0U) &&
          ((f->reservado - f->cab) < f->ampliar_min)) {
        if (_reservar(f, f->ampliar)) {
          _flush_y_dirs();
        }
        return; /* uno por vuelta */
      }
    }
  }
}

/* === Comprobación en la mesa ============================================= */

#if ASPID_DATA_PRUEBA || ASPID_DATA_RELEER
/* Relee un archivo cerrado y cuenta sus líneas por USB */
static void _prueba_mostrar(const char *rel, uint32_t lineas_mostrar) {
  FX_FILE f;
  const char *ruta = _ruta(rel);
  if (fx_file_open(s_m, &f, (CHAR *)ruta, FX_OPEN_FOR_READ) != FX_SUCCESS) {
    DebugOut_Printf("PRUEBA %s: NO SE PUEDE ABRIR\r\n", ruta);
    return;
  }
  uint32_t lineas = 0U;
  uint32_t bytes = 0U;
  uint32_t mostradas = 0U;
  char linea[200];
  uint32_t n_linea = 0U;
  char ultima[200];
  uint32_t n_ultima = 0U;
  ULONG leidos = 0U;
  DebugOut_Printf("PRUEBA ---- %s\r\n", ruta);
  while ((fx_file_read(&f, s_mem_vuelo, sizeof(s_mem_vuelo), &leidos) ==
          FX_SUCCESS) &&
         (leidos > 0U)) {
    for (uint32_t i = 0U; i < leidos; i++) {
      const char c = (char)s_mem_vuelo[i];
      bytes++;
      if (c == '\n') {
        lineas++;
        linea[n_linea] = '\0';
        if (mostradas < lineas_mostrar) {
          DebugOut_Printf("  | %s\r\n", linea);
          mostradas++;
        }
        memcpy(ultima, linea, n_linea + 1U);
        n_ultima = n_linea;
        n_linea = 0U;
      } else if ((c != '\r') && (n_linea < (sizeof(linea) - 1U))) {
        linea[n_linea++] = c;
      }
    }
    HAL_Delay(20); /* que el USB vacíe lo impreso */
  }
  (void)fx_file_close(&f);
  if (n_ultima > 0U) {
    DebugOut_Printf("  | ...\r\n  | %s\r\n", ultima);
  }
  DebugOut_Printf("  = %lu lineas, %lu bytes\r\n", (unsigned long)lineas,
                  (unsigned long)bytes);
  HAL_Delay(50);
}
#endif

/* === API ================================================================= */

void aspid_data_init(const sys_state *state) {
  memset(&s_info, 0, sizeof(s_info));
  s_info.estado = ASPID_SIN_SD;
  _flujo(&s_sesion, s_mem_sesion, ANILLO_SESION, NULL);
  _flujo(&s_eventos, s_mem_eventos, ANILLO_EVENTOS, NULL);
  _flujo(&s_salud, s_mem_salud, ANILLO_SALUD, NULL);
  _flujo(&s_telem, s_mem_telem, ANILLO_TELEM, &s_info.bytes_telemetria);
  s_telem.ampliar = RESERVA_TELEM;
  s_telem.ampliar_min = RESERVA_TELEM_MIN;
  s_salud.ampliar = RESERVA_SALUD;
  s_salud.ampliar_min = RESERVA_SALUD_MIN;
  _flujo(&s_vuelo, s_mem_vuelo, ANILLO_VUELO, &s_info.bytes_vuelo);
  _flujo(&s_empuje, s_mem_empuje, ANILLO_EMPUJE, &s_info.bytes_empuje);
  _flujo(&s_sim_vuelo, s_mem_vuelo, ANILLO_VUELO, &s_info.bytes_vuelo);
  _flujo(&s_sim_empuje, s_mem_empuje, ANILLO_EMPUJE, &s_info.bytes_empuje);

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  s_cnt_imu1 = sensor_board_get_data()->imu[0].count;
  s_cnt_imu2 = sensor_board_get_data()->imu[1].count;

  void *h = storage_media_handle(ASPID_SD);
  if (h == NULL) {
    return; /* sin SD1: no se graba nada (se ve en la consola) */
  }
  s_m = storage_media_fx(h);
  s_cluster_bytes =
      s_m->fx_media_sectors_per_cluster * s_m->fx_media_bytes_per_sector;
  if ((s_m->fx_media_bytes_per_sector != SECTOR) || (s_cluster_bytes == 0U)) {
    return;
  }

  _limpiar_reservas();

  const uint32_t arranque = storage_get_info(ASPID_SD)->boot_count;
  bool creada = false;
  for (uint32_t i = 1U; (i <= 9U) && !creada; i++) {
    if (i == 1U) {
      (void)snprintf(s_info.carpeta, sizeof(s_info.carpeta),
                     "aspid_data_sin_fecha_%04lu", (unsigned long)arranque);
    } else {
      (void)snprintf(s_info.carpeta, sizeof(s_info.carpeta),
                     "aspid_data_sin_fecha_%04lu_%lu",
                     (unsigned long)arranque, (unsigned long)i);
    }
    const UINT st = fx_directory_create(s_m, (CHAR *)s_info.carpeta);
    if (st == FX_SUCCESS) {
      creada = true;
    } else if (st != FX_ALREADY_CREATED) {
      _error(st);
      return;
    }
  }
  if (!creada) {
    return;
  }
  (void)fx_directory_create(s_m, (CHAR *)_ruta("eventos"));
  (void)fx_directory_create(s_m, (CHAR *)_ruta("salud"));
  (void)fx_directory_create(s_m, (CHAR *)_ruta("telemetria"));
  (void)fx_directory_create(s_m, (CHAR *)_ruta("vuelo"));
  (void)fx_directory_create(s_m, (CHAR *)_ruta("trust_curve"));

  if (!_abrir(&s_sesion, "sesion.txt", RESERVA_SESION) ||
      !_abrir(&s_eventos, "eventos/eventos.csv", RESERVA_EVENTOS) ||
      !_abrir(&s_salud, "salud/salud.csv", RESERVA_SALUD) ||
      !_abrir(&s_telem, "telemetria/telemetria.csv", RESERVA_TELEM) ||
      !_abrir(&s_vuelo, "vuelo/vuelo_01.csv", RESERVA_VUELO) ||
      !_abrir(&s_empuje, "trust_curve/trust_curve_01.csv",
              RESERVA_EMPUJE)) {
    s_info.estado = ASPID_SIN_SD;
    return;
  }
  _guardar_reservas();
  _flush_y_dirs();

  (void)_poner(&s_telem, CABECERA_TELEM, sizeof(CABECERA_TELEM) - 1U);
  (void)_poner(&s_eventos, CABECERA_EVENTOS, sizeof(CABECERA_EVENTOS) - 1U);
  (void)_poner(&s_salud, CABECERA_SALUD, sizeof(CABECERA_SALUD) - 1U);

  /* Ficha de la configuración con la que se arrancó */
  const storage_sd_info_t *sd1 = storage_get_info(SD_CARD_1);
  const storage_sd_info_t *sd2 = storage_get_info(SD_CARD_2);
  const uint32_t div1 = sd_card_get_clock_div(SD_CARD_1);
  const uint32_t div2 = sd_card_get_clock_div(SD_CARD_2);
  int n = snprintf(
      s_fila, FILA_MAX,
      "ASPID - sesion de la core (App/aspid_data.c)\r\n\r\n"
      "== CONFIGURACION AL ARRANCAR ==\r\n"
      "firmware compilado %s %s, GNC ASPID v1.5, protocolo CAN1 v%u.%u, CPU "
      "%lu MHz\r\n"
      "SD1: arranque %lu, %lu MB (libres %lu), cluster %lu B, bus %lu kHz\r\n"
      "SD2: %s, %lu MB, bus %lu kHz\r\n"
      "placas: power %s, avionica %s, rf %s (reintentos de wake up %lu)\r\n",
      __DATE__, __TIME__, (unsigned)CAN_PROTOCOL_VERSION_MAJOR,
      (unsigned)CAN_PROTOCOL_VERSION_MINOR,
      (unsigned long)(SystemCoreClock / 1000000U),
      (unsigned long)sd1->boot_count, (unsigned long)sd1->capacity_mb,
      (unsigned long)sd1->free_mb, (unsigned long)s_cluster_bytes,
      (unsigned long)((div1 == 0U) ? 20000U : (10000U / div1)),
      (sd2->state == STORAGE_SD_MOUNTED) ? "montada" : "NO montada",
      (unsigned long)sd2->capacity_mb,
      (unsigned long)((div2 == 0U) ? 20000U : (10000U / div2)),
      state->power ? "OK" : "NO", state->avionics ? "OK" : "NO",
      state->rf ? "OK" : "NO", (unsigned long)state->wake_up_retries);
  if ((n > 0) && ((uint32_t)n < FILA_MAX)) {
    (void)_poner(&s_sesion, s_fila, (uint32_t)n);
  }

  /* Sensores de la aviónica, según su INIT_STATUS */
  {
    static const char *const nombre[] = {"baro1", "baro2", "imu1", "imu2",
                                         "mag",   "gps",   "vn100"};
    const sys_init_status *ai = &state->avionics_init;
    char *p = s_fila;
    p = _s(p, "sensores de la avionica:");
    for (uint32_t i = 0U; i < 7U; i++) {
      const uint32_t b = 1UL << i;
      *p++ = ' ';
      p = _s(p, nombre[i]);
      *p++ = ' ';
      p = _s(p, !ai->valid ? "?"
                : ((ai->ok & b) != 0U)
                    ? "OK"
                    : (((ai->supported & b) != 0U) ? "FALLO" : "no-lleva"));
    }
    p = _s(p, ai->valid ? "\r\n" : "  (sin INIT_STATUS)\r\n");
    (void)_poner(&s_sesion, s_fila, (uint32_t)(p - s_fila));
  }

  n = snprintf(
      s_fila, FILA_MAX,
      "rampa: %.2f grados desde la vertical, rumbo %.2f (origen: %s)\r\n\r\n"
      "== QUE HAY EN ESTA CARPETA ==\r\n"
      "eventos/eventos.csv   todo lo que pasa, una linea por evento\r\n"
      "salud/salud.csv       1 Hz desde el arranque: bucle, GNC, sensores, "
      "buses, radio, power y SD\r\n"
      "telemetria/           cada paquete enviado y recibido por radio\r\n"
      "vuelo/                %u Hz desde %u ms antes del despegue hasta bajar "
      "de %.0f m tras el apogeo\r\n"
      "trust_curve/          las IMU a %u Hz desde %u ms antes del despegue "
      "hasta %u s tras el motor\r\n"
      "simulaciones/         lo mismo en un SIMULAR RUTA (no son vuelos)\r\n"
      "t_ms = ms desde el arranque de la core; la hora UTC sale del GPS "
      "(abajo, a que t_ms corresponde). Si un vuelo pasa de %.0f m, al "
      "terminar se cierra todo y la SD no se toca mas en este arranque.\r\n"
      "\r\n== EVENTOS IMPORTANTES (todos en eventos/eventos.csv) ==\r\n"
      "t_ms      hora UTC                 evento\r\n",
      (double)lanzamiento_grados(), (double)lanzamiento_rumbo_grados(),
      lanzamiento_origen(), (unsigned)ASPID_DATA_HZ,
      (unsigned)ASPID_DATA_ANTES_MS, (double)ASPID_DATA_FIN_ALTURA_M,
      (unsigned)GNC_RATE_HZ, (unsigned)ASPID_DATA_EMPUJE_ANTES_MS,
      (unsigned)(ASPID_DATA_EMPUJE_DESPUES_MS / 1000U),
      (double)ASPID_DATA_VUELO_REAL_M);
  if ((n > 0) && ((uint32_t)n < FILA_MAX)) {
    (void)_poner(&s_sesion, s_fila, (uint32_t)n);
  }
  _evento("arranque", "arranque: carpeta %s, %lu MB reservados%s",
          s_info.carpeta,
          (unsigned long)((s_sesion.reservado + s_eventos.reservado +
                           s_salud.reservado + s_telem.reservado +
                           s_vuelo.reservado + s_empuje.reservado) /
                          MB),
          ASPID_DATA_PRUEBA ? "  (ASPID_DATA_PRUEBA: fecha y vuelo FALSOS)"
                            : "");
  if (s_info.reservas_liberadas > 0U) {
    _evento("reservas",
            "liberado lo reservado y sin usar de %lu archivos de la sesion "
            "anterior (se corto la corriente sin cerrarlos)",
            (unsigned long)s_info.reservas_liberadas);
  }

  /* Contadores de partida para la salud */
  const sensor_board_data_t *sb = sensor_board_get_data();
  s_cnt_salud[0] = sb->imu[0].count;
  s_cnt_salud[1] = sb->imu[1].count;
  s_cnt_salud[2] = sb->baro[0].count;
  s_cnt_salud[3] = sb->baro[1].count;
  s_cnt_salud[4] = sb->mag.count;
  s_cnt_salud[5] = sb->gps_gga.count;
  s_steps_salud = gnc_get_data()->steps;
  s_perdidos_prev = gnc_get_data()->overruns;
  s_radio_fallo_prev = rf_telemetry_failed();
  s_salud_t = HAL_GetTick() + 1000U;

  s_info.estado = ASPID_ESPERANDO;
  s_ultimo_sync = HAL_GetTick();
}

void aspid_data_update(const sys_state *state) {
  /* Cuánto ha pasado desde la vuelta anterior: si algo bloquea el bucle,
   * sea lo que sea, sale aquí (columna bucle_max_ms de la salud) */
  const uint32_t ciclo = DWT->CYCCNT;
  if (s_bucle_t != 0U) {
    const uint32_t us =
        (ciclo - s_bucle_t) / (SystemCoreClock / 1000000U);
    if (us > s_bucle_max_us) {
      s_bucle_max_us = us;
    }
  }
  s_bucle_t = ciclo;
  s_bucle_n++;

#if ASPID_DATA_RELEER
  if (s_releer) {
    s_releer = false;
    _prueba_mostrar(s_releer_ruta[0], 3U);
    _prueba_mostrar(s_releer_ruta[1], 3U);
  }
#endif
#if ASPID_DATA_PRUEBA
  static bool releido;
  if ((s_info.estado == ASPID_SELLADO) && !releido &&
      (HAL_GetTick() > 70000U)) {
    releido = true;
    _prueba_mostrar("sesion.txt", 40U);
    _prueba_mostrar("eventos/eventos.csv", 40U);
    _prueba_mostrar("salud/salud.csv", 4U);
    _prueba_mostrar("telemetria/telemetria.csv", 3U);
    _prueba_mostrar("vuelo/vuelo_01.csv", 3U);
    _prueba_mostrar("trust_curve/trust_curve_01.csv", 3U);
  }
#endif
  if ((s_info.estado != ASPID_ESPERANDO) &&
      (s_info.estado != ASPID_GRABANDO)) {
    return;
  }
  const uint32_t ahora = HAL_GetTick();
  {
    const gnc_data_t *g = gnc_get_data();
    if (g->steps != s_steps_vistos) {
      s_steps_vistos = g->steps;
      if (g->last_step_us > s_tick_max_us) {
        s_tick_max_us = g->last_step_us;
      }
    }
  }
  _vigilar_eventos(state, ahora);
  if ((int32_t)(ahora - s_salud_t) >= 0) {
    s_salud_t = ((ahora - s_salud_t) > 1000U) ? (ahora + 1000U)
                                               : (s_salud_t + 1000U);
    _fila_salud(state, ahora);
  }
  _hora_gps(ahora);
  _muestrear_imu(ahora);
  _muestrear(state, ahora);
  _formatear();

  /* La escritura en marcha: se pregunta sin esperar */
  if (s_trab != T_NADA) {
    if ((_t() - s_ultimo_poll) <
        (POLL_US * (SystemCoreClock / 1000000U))) {
      return;
    }
    s_ultimo_poll = _t();
    const sd_card_async_t r = sd_card_write_poll(ASPID_SD);
    if (r == SD_CARD_ASYNC_OCUPADA) {
      return;
    }
    if (r == SD_CARD_ASYNC_FALLO) {
      _tarjeta_perdida();
      return;
    }
    _acabar_trabajo();
  }
  if ((s_info.estado != ASPID_ESPERANDO) &&
      (s_info.estado != ASPID_GRABANDO)) {
    return;
  }
  if (!_siguiente(ahora)) {
    _filex();
  }
}

static const char *_tipo_tx(aspid_tx_tipo_t tipo) {
  switch (tipo) {
  case ASPID_TX_VUELO:
    return "vuelo";
  case ASPID_TX_ESTADO:
    return "estado";
  case ASPID_TX_ACUSE:
    return "acuse";
  case ASPID_TX_CAMARAS:
    return "camaras";
  default:
    return "?";
  }
}

/* Nombres de los comandos de App/uplink.h */
static const char *_tipo_rx(const uint8_t *d, uint8_t len) {
  if ((len < 2U) || (d[0] != 0x5AU)) {
    return "?";
  }
  switch (d[1]) {
  case 0x01U:
    return "pedir_estado";
  case 0x02U:
    return "ping";
  case 0x10U:
    return "modo_test";
  case 0x11U:
    return "pirotecnico";
  case 0x12U:
    return "simular_ruta";
  case 0x13U:
    return "angulo";
  case 0x14U:
    return "prueba_aerofreno";
  case 0x15U:
    return "extension_aerofreno";
  case 0x16U:
    return "rumbo";
  default:
    return "desconocido";
  }
}

/* uplink_denegado_t, en el mismo orden */
static const char *const s_motivos[] = {
    "",           "fuera de tierra",   "sin modo test",
    "canal mal",  "power no responde", "simulando",
    "sin VUELO.BIN", "fuera de rango"};

static char *_linea_telem(uint32_t t, const char *sentido, const char *tipo,
                          uint8_t len) {
  char *p = s_linea;
  p = _u32(p, t);
  *p++ = ',';
  p = _utc(p, t);
  *p++ = ',';
  p = _s(p, sentido);
  *p++ = ',';
  p = _s(p, tipo);
  *p++ = ',';
  return _u32(p, len);
}

static bool _telem_activa(void) {
  return ((s_info.estado == ASPID_ESPERANDO) ||
          (s_info.estado == ASPID_GRABANDO)) &&
         s_telem.abierto && !s_telem.cerrando;
}

void aspid_data_tx(aspid_tx_tipo_t tipo, const uint8_t *datos, uint8_t len,
                   bool ok) {
  if (!_telem_activa() || (datos == NULL)) {
    return;
  }
  if (len > 32U) {
    len = 32U;
  }
  char *p = _linea_telem(HAL_GetTick(), "enviado", _tipo_tx(tipo), len);
  p = _s(p, ",,,,");
  p = _s(p, ok ? "encolado" : "fallo al encolar en CAN");
  *p++ = ',';
  p = _hex(p, datos, len);
  p = _s(p, "\r\n");
  if (_poner(&s_telem, s_linea, (uint32_t)(p - s_linea))) {
    s_info.lineas_telemetria++;
  }
}

void aspid_data_rx(const uint8_t *datos, uint8_t len,
                   const sys_rf_rx_meta_t *meta,
                   aspid_rx_resultado_t resultado, uint8_t motivo) {
  if (!_telem_activa() || (datos == NULL)) {
    return;
  }
  if (len > 32U) {
    len = 32U;
  }
  const char *tipo = (resultado == ASPID_RX_MAL_FORMADO)
                         ? "mal_formado"
                         : _tipo_rx(datos, len);
  char *p = _linea_telem(HAL_GetTick(), "recibido", tipo, len);
  *p++ = ',';
  if ((meta != NULL) && meta->valida) {
    p = _i32(p, meta->rssi_dbm);
    *p++ = ',';
    p = _i32(p, meta->snr_db);
    *p++ = ',';
    p = _i32(p, meta->df_hz);
  } else {
    p = _s(p, ",,");
  }
  *p++ = ',';
  switch (resultado) {
  case ASPID_RX_EJECUTADO:
    p = _s(p, "ejecutado");
    break;
  case ASPID_RX_DENEGADO:
    p = _s(p, "denegado: ");
    p = _s(p, (motivo < (sizeof(s_motivos) / sizeof(s_motivos[0])))
                  ? s_motivos[motivo]
                  : "?");
    break;
  case ASPID_RX_DESCONOCIDO:
    p = _s(p, "comando desconocido");
    break;
  case ASPID_RX_MAL_FORMADO:
  default:
    p = _s(p, "ignorado: no es un comando (marca o XOR)");
    break;
  }
  *p++ = ',';
  p = _hex(p, datos, len);
  p = _s(p, "\r\n");
  if (_poner(&s_telem, s_linea, (uint32_t)(p - s_linea))) {
    s_info.lineas_telemetria++;
  }
}

const aspid_data_info_t *aspid_data_info(void) { return &s_info; }
