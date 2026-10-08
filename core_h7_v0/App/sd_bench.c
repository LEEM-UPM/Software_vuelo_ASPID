#include "sd_bench.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "debug_out.h"
#include "fx_api.h"
#include "main.h"
#include "sd_card.h"
#include "storage.h"

/* De storage.c, como en storage_stream.c: la FX_MEDIA de una tarjeta
 * montada, para usar FileX directamente y medir cada llamada */
void *storage_media_handle(sd_card_id_t sd);
FX_MEDIA *storage_media_fx(void *handle);

#define BENCH_SD SD_CARD_1
#define BENCH_FILE "SDBENCH.BIN"
#define BENCH_REPORT "SDBENCH.TXT"

#define BENCH_SDMMC_HZ 20000000U /* SDMMCCLK, de PLL1Q */

#define BENCH_SMALL_CHUNK 512U
#define BENCH_BIG_CHUNK (16U * 1024U)
#define BENCH_SWEEP_SMALL_BYTES (256U * 1024U)
#define BENCH_SWEEP_BIG_BYTES (1024U * 1024U)
#define BENCH_SUSTAINED_BYTES (8U * 1024U * 1024U)
#define BENCH_FLUSH_EVERY (1024U * 1024U)

/* Simulación del registro de vuelo */
#define BENCH_LOG_HZ 400U
#define BENCH_LOG_RECORD 256U
#define BENCH_LOG_SECONDS 20U
#define BENCH_LOG_TICKS (BENCH_LOG_HZ * BENCH_LOG_SECONDS)
#define BENCH_LOG_TICK_US (1000000U / BENCH_LOG_HZ)
#define BENCH_LOG_CATCHUP_US 20000U /* GNC_MAX_RETRASO_TICKS = 8 ticks */

/* Divisores a probar, de lento a rápido: SDMMC_CK = 20 MHz / (2 * div), y 0
 * es sin dividir (20 MHz) */
static const uint32_t s_divs[] = {16U, 8U, 4U, 2U, 1U, 0U};
#define BENCH_N_DIVS (sizeof(s_divs) / sizeof(s_divs[0]))

/* Histograma de lo que bloquea cada llamada, en us */
static const uint32_t s_hist_lim_us[] = {1000U,  2500U,   5000U,  10000U,
                                         20000U, 50000U, 100000U, 250000U};
static const char *const s_hist_name[] = {
    "<1 ms",     "1-2,5 ms",   "2,5-5 ms",   "5-10 ms",   "10-20 ms",
    "20-50 ms",  "50-100 ms",  "100-250 ms", ">=250 ms"};
#define BENCH_HIST_N 9U

typedef struct {
  uint32_t n;
  uint32_t min_us;
  uint32_t max_us;
  uint64_t sum_us;
  uint32_t hist[BENCH_HIST_N];
} stats_t;

typedef struct {
  bool ok; /* escrito, releído y comparado sin errores */
  uint32_t bytes;
  uint64_t write_us; /* escrituras + flushes + cierre */
  uint64_t read_us;
  stats_t calls;  /* cada fx_file_write */
  stats_t flushes; /* cada fx_media_flush intermedio */
  uint32_t close_us;
  uint32_t alloc_us;
  uint32_t bad_words;
  uint32_t fx_error;
  uint32_t hal_error;
} run_t;

/* Fuera de la DTCM y alineado a 32: FileX lo pasa tal cual al IDMA sin el
 * buffer intermedio de sd_card.c (si no, se mediría ese rodeo) */
static uint8_t s_buf[BENCH_BIG_CHUNK] __attribute__((aligned(32)));
static run_t s_run;
static char s_report[8192];
static uint32_t s_report_len;
static char s_line[256];
static uint32_t s_cyc_per_us;

/* === Utilidades ========================================================= */

static void _say(const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(s_line, sizeof(s_line), fmt, ap);
  va_end(ap);
  if (n < 0) {
    return;
  }
  if ((uint32_t)n >= sizeof(s_line)) {
    n = (int)sizeof(s_line) - 1;
  }
  DebugOut_Printf("%s", s_line);
  if ((s_report_len + (uint32_t)n) < sizeof(s_report)) {
    memcpy(&s_report[s_report_len], s_line, (uint32_t)n);
    s_report_len += (uint32_t)n;
  }
}

static uint32_t _t(void) { return DWT->CYCCNT; }
static uint32_t _us(uint32_t t0) { return (DWT->CYCCNT - t0) / s_cyc_per_us; }

static void _stats_reset(stats_t *s) {
  memset(s, 0, sizeof(*s));
  s->min_us = UINT32_MAX;
}

static void _stats_add(stats_t *s, uint32_t us) {
  s->n++;
  s->sum_us += us;
  if (us < s->min_us) {
    s->min_us = us;
  }
  if (us > s->max_us) {
    s->max_us = us;
  }
  uint32_t i = 0U;
  while ((i < (BENCH_HIST_N - 1U)) && (us >= s_hist_lim_us[i])) {
    i++;
  }
  s->hist[i]++;
}

static uint32_t _avg_us(const stats_t *s) {
  return (s->n > 0U) ? (uint32_t)(s->sum_us / s->n) : 0U;
}

/* KB/s a partir de bytes y microsegundos */
static uint32_t _kbps(uint32_t bytes, uint64_t us) {
  return (us > 0U) ? (uint32_t)(((uint64_t)bytes * 1000000U) / (us * 1024U))
                   : 0U;
}

/* "12.3" ms con un decimal, sin flotantes en printf */
static const char *_ms(uint32_t us, char *buf) {
  (void)snprintf(buf, 16, "%lu.%lu", (unsigned long)(us / 1000U),
                 (unsigned long)((us % 1000U) / 100U));
  return buf;
}

static uint32_t _clock_khz(uint32_t div) {
  return ((div == 0U) ? BENCH_SDMMC_HZ : (BENCH_SDMMC_HZ / (2U * div))) /
         1000U;
}

/* Patrón que depende de la posición y de la semilla: un trozo escrito en el
 * sitio equivocado, o los datos de una pasada anterior, no cuadran */
static void _fill(uint32_t offset, uint32_t len, uint32_t seed) {
  uint32_t *w = (uint32_t *)s_buf;
  const uint32_t i0 = offset / 4U;
  for (uint32_t i = 0U; i < (len / 4U); i++) {
    w[i] = ((i0 + i) * 0x9E3779B1U) ^ seed;
  }
}

static uint32_t _check(uint32_t offset, uint32_t len, uint32_t seed) {
  const uint32_t *w = (const uint32_t *)s_buf;
  const uint32_t i0 = offset / 4U;
  uint32_t bad = 0U;
  for (uint32_t i = 0U; i < (len / 4U); i++) {
    if (w[i] != (((i0 + i) * 0x9E3779B1U) ^ seed)) {
      bad++;
    }
  }
  return bad;
}

static void _print_hist(const stats_t *s) {
  for (uint32_t i = 0U; i < BENCH_HIST_N; i++) {
    if (s->hist[i] > 0U) {
      _say("      %-11s %6lu\r\n", s_hist_name[i], (unsigned long)s->hist[i]);
    }
  }
}

/* === Escribir y comprobar ============================================== */

static bool _fail(run_t *r, UINT status) {
  r->fx_error = (uint32_t)status;
  r->hal_error = sd_card_last_error(BENCH_SD);
  r->ok = false;
  return false;
}

/*
 * Escribe 'total' bytes en trozos de 'chunk', midiendo cada llamada. Con
 * flush_every > 0 hace un fx_media_flush cada tantos bytes (lo que haría el
 * registro para sobrevivir a un corte), medido aparte.
 */
static bool _write(FX_MEDIA *m, uint32_t total, uint32_t chunk, uint32_t seed,
                   bool prealloc, uint32_t flush_every, run_t *r) {
  memset(r, 0, sizeof(*r));
  _stats_reset(&r->calls);
  _stats_reset(&r->flushes);
  r->bytes = total;

  FX_FILE f;
  (void)fx_file_delete(m, (CHAR *)BENCH_FILE);
  UINT st = fx_file_create(m, (CHAR *)BENCH_FILE);
  if (st == FX_SUCCESS) {
    st = fx_file_open(m, &f, (CHAR *)BENCH_FILE, FX_OPEN_FOR_WRITE);
  }
  if (st != FX_SUCCESS) {
    return _fail(r, st);
  }

  if (prealloc) {
    const uint32_t t0 = _t();
    st = fx_file_extended_allocate(&f, (ULONG64)total);
    r->alloc_us = _us(t0);
    if (st != FX_SUCCESS) {
      (void)fx_file_close(&f);
      return _fail(r, st);
    }
  }

  uint32_t since_flush = 0U;
  for (uint32_t off = 0U; off < total; off += chunk) {
    _fill(off, chunk, seed);
    uint32_t t0 = _t();
    st = fx_file_write(&f, s_buf, chunk);
    uint32_t us = _us(t0);
    if (st != FX_SUCCESS) {
      (void)fx_file_close(&f);
      return _fail(r, st);
    }
    _stats_add(&r->calls, us);
    r->write_us += us;

    since_flush += chunk;
    if ((flush_every > 0U) && (since_flush >= flush_every)) {
      since_flush = 0U;
      t0 = _t();
      st = fx_media_flush(m);
      us = _us(t0);
      if (st != FX_SUCCESS) {
        (void)fx_file_close(&f);
        return _fail(r, st);
      }
      _stats_add(&r->flushes, us);
      r->write_us += us;
    }
  }

  const uint32_t t0 = _t();
  st = fx_file_close(&f);
  if (st == FX_SUCCESS) {
    st = fx_media_flush(m);
  }
  r->close_us = _us(t0);
  r->write_us += r->close_us;
  if (st != FX_SUCCESS) {
    return _fail(r, st);
  }
  return true;
}

/* Relee el archivo entero y lo compara con el patrón */
static bool _verify(FX_MEDIA *m, uint32_t seed, run_t *r) {
  FX_FILE f;
  UINT st = fx_file_open(m, &f, (CHAR *)BENCH_FILE, FX_OPEN_FOR_READ);
  if (st != FX_SUCCESS) {
    return _fail(r, st);
  }
  for (uint32_t off = 0U; off < r->bytes; off += BENCH_BIG_CHUNK) {
    const uint32_t len = ((r->bytes - off) < BENCH_BIG_CHUNK)
                             ? (r->bytes - off)
                             : BENCH_BIG_CHUNK;
    memset(s_buf, 0, len);
    ULONG actual = 0U;
    const uint32_t t0 = _t();
    st = fx_file_read(&f, s_buf, len, &actual);
    r->read_us += _us(t0);
    if ((st != FX_SUCCESS) || (actual != len)) {
      (void)fx_file_close(&f);
      return _fail(r, (st != FX_SUCCESS) ? st : FX_END_OF_FILE);
    }
    r->bad_words += _check(off, len, seed);
  }
  (void)fx_file_close(&f);
  r->ok = (r->bad_words == 0U);
  return r->ok;
}

static void _print_run(const char *label, const run_t *r) {
  char a[16];
  char b[16];
  if (r->fx_error != 0U) {
    _say("   %-8s FALLO: FileX 0x%02lX, HAL 0x%08lX tras %lu escrituras\r\n",
         label, (unsigned long)r->fx_error, (unsigned long)r->hal_error,
         (unsigned long)r->calls.n);
    return;
  }
  _say("   %-8s escribe %5lu KB/s  llamada media %s ms, max %s ms  | lee %5lu "
       "KB/s | %s",
       label, (unsigned long)_kbps(r->bytes, r->write_us),
       _ms(_avg_us(&r->calls), a), _ms(r->calls.max_us, b),
       (unsigned long)_kbps(r->bytes, r->read_us),
       r->ok ? "OK" : "DATOS MAL");
  if (r->bad_words > 0U) {
    _say(" (%lu palabras de 32 bits distintas)", (unsigned long)r->bad_words);
  }
  _say("\r\n");
}

/* Prueba completa a un divisor: trozos pequeños y grandes, releyendo */
static bool _sweep_one(FX_MEDIA *m, uint32_t div) {
  _say(" %5lu kHz (div %lu)\r\n", (unsigned long)_clock_khz(div),
       (unsigned long)div);
  if (!sd_card_set_clock_div(BENCH_SD, div)) {
    _say("   no se pudo cambiar el reloj\r\n");
    return false;
  }

  const uint32_t seed_small = 0x5D000000U ^ (div << 8) ^ 0x01U;
  bool ok = _write(m, BENCH_SWEEP_SMALL_BYTES, BENCH_SMALL_CHUNK, seed_small,
                   false, 0U, &s_run) &&
            _verify(m, seed_small, &s_run);
  _print_run("512 B", &s_run);
  if (!ok) {
    return false;
  }

  const uint32_t seed_big = 0x5D000000U ^ (div << 8) ^ 0x02U;
  ok = _write(m, BENCH_SWEEP_BIG_BYTES, BENCH_BIG_CHUNK, seed_big, false, 0U,
              &s_run) &&
       _verify(m, seed_big, &s_run);
  _print_run("16 KB", &s_run);
  return ok;
}

/* Escritura larga: aquí es donde salen los picos de la tarjeta */
static void _sustained(FX_MEDIA *m, bool prealloc) {
  char a[16];
  char b[16];
  char c[16];
  const uint32_t seed = prealloc ? 0xC0FFEE01U : 0xC0FFEE02U;
  _say(" %lu MB a trozos de 16 KB, flush cada 1 MB, %s\r\n",
       (unsigned long)(BENCH_SUSTAINED_BYTES / (1024U * 1024U)),
       prealloc ? "archivo RESERVADO antes (fx_file_allocate)"
                : "el archivo crece segun se escribe");
  const bool ok = _write(m, BENCH_SUSTAINED_BYTES, BENCH_BIG_CHUNK, seed,
                         prealloc, BENCH_FLUSH_EVERY, &s_run) &&
                  _verify(m, seed, &s_run);
  _print_run("16 KB", &s_run);
  if (prealloc) {
    _say("   reservar %lu MB: %s ms\r\n",
         (unsigned long)(BENCH_SUSTAINED_BYTES / (1024U * 1024U)),
         _ms(s_run.alloc_us, a));
  }
  _say("   flush: %lu, medio %s ms, max %s ms;  cierre %s ms\r\n",
       (unsigned long)s_run.flushes.n, _ms(_avg_us(&s_run.flushes), a),
       _ms(s_run.flushes.max_us, b), _ms(s_run.close_us, c));
  if (ok || (s_run.calls.n > 0U)) {
    _say("   bloqueo de cada escritura de 16 KB (%lu llamadas):\r\n",
         (unsigned long)s_run.calls.n);
    _print_hist(&s_run.calls);
  }
}

/*
 * El registro de vuelo tal cual iría en el bucle: cada 2,5 ms se añade un
 * registro de 256 B a un buffer de 16 KB, y cuando se llena se escribe
 * (bloqueando, como todo hoy). Un flush por segundo. Lo que se mide es lo
 * que vería el GNC: cuánto llega tarde cada tick.
 */
static void _log_sim(FX_MEDIA *m, uint32_t div) {
  char a[16];
  char b[16];
  _say(" %lu kHz: %lu s a %lu Hz, %lu B por tick (%lu KB/s), bloques de 16 KB"
       ", flush cada 1 s\r\n",
       (unsigned long)_clock_khz(div), (unsigned long)BENCH_LOG_SECONDS,
       (unsigned long)BENCH_LOG_HZ, (unsigned long)BENCH_LOG_RECORD,
       (unsigned long)((BENCH_LOG_HZ * BENCH_LOG_RECORD) / 1024U));
  if (!sd_card_set_clock_div(BENCH_SD, div)) {
    _say("   no se pudo cambiar el reloj\r\n");
    return;
  }

  FX_FILE f;
  (void)fx_file_delete(m, (CHAR *)BENCH_FILE);
  UINT st = fx_file_create(m, (CHAR *)BENCH_FILE);
  if (st == FX_SUCCESS) {
    st = fx_file_open(m, &f, (CHAR *)BENCH_FILE, FX_OPEN_FOR_WRITE);
  }
  if (st == FX_SUCCESS) {
    st = fx_file_extended_allocate(
        &f, (ULONG64)BENCH_LOG_TICKS * BENCH_LOG_RECORD);
  }
  if (st != FX_SUCCESS) {
    _say("   FALLO al preparar el archivo: FileX 0x%02lX\r\n",
         (unsigned long)st);
    return;
  }

  stats_t writes;
  stats_t flushes;
  stats_t late; /* retraso de cada tick sobre su hora */
  _stats_reset(&writes);
  _stats_reset(&flushes);
  _stats_reset(&late);
  uint32_t lost_ticks = 0U;  /* ticks enteros perdidos (> 2,5 ms tarde) */
  uint32_t beyond = 0U;      /* más tarde de lo que el GNC recupera */
  uint32_t used = 0U;
  uint32_t errors = 0U;

  const uint32_t tick_cyc = BENCH_LOG_TICK_US * s_cyc_per_us;
  uint32_t next = _t() + tick_cyc;
  for (uint32_t k = 0U; k < BENCH_LOG_TICKS; k++) {
    while ((int32_t)(_t() - next) < 0) {
    }
    const uint32_t late_us = (_t() - next) / s_cyc_per_us;
    _stats_add(&late, late_us);
    if (late_us >= BENCH_LOG_TICK_US) {
      lost_ticks++;
    }
    if (late_us >= BENCH_LOG_CATCHUP_US) {
      beyond++;
    }
    next += tick_cyc;

    /* "Registro" del tick: contador y relleno */
    uint32_t *rec = (uint32_t *)&s_buf[used];
    for (uint32_t i = 0U; i < (BENCH_LOG_RECORD / 4U); i++) {
      rec[i] = k ^ (i << 24);
    }
    used += BENCH_LOG_RECORD;

    if (used >= BENCH_BIG_CHUNK) {
      const uint32_t t0 = _t();
      if (fx_file_write(&f, s_buf, used) != FX_SUCCESS) {
        errors++;
      }
      _stats_add(&writes, _us(t0));
      used = 0U;
    }
    if (((k + 1U) % BENCH_LOG_HZ) == 0U) {
      const uint32_t t0 = _t();
      if (fx_media_flush(m) != FX_SUCCESS) {
        errors++;
      }
      _stats_add(&flushes, _us(t0));
    }
  }
  (void)fx_file_close(&f);
  (void)fx_media_flush(m);

  _say("   escrituras de 16 KB: %lu, media %s ms, max %s ms\r\n",
       (unsigned long)writes.n, _ms(_avg_us(&writes), a),
       _ms(writes.max_us, b));
  _say("   flush por segundo: medio %s ms, max %s ms\r\n",
       _ms(_avg_us(&flushes), a), _ms(flushes.max_us, b));
  _say("   ticks: %lu, retraso max %s ms; con al menos un tick perdido %lu, "
       "mas de 20 ms (el GNC no lo recupera) %lu%s\r\n",
       (unsigned long)late.n, _ms(late.max_us, a), (unsigned long)lost_ticks,
       (unsigned long)beyond, (errors > 0U) ? "  ERRORES DE ESCRITURA" : "");
  _say("   retraso de los ticks:\r\n");
  _print_hist(&late);
}

/* === API ================================================================= */

void sd_bench_run(void) {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  s_cyc_per_us = SystemCoreClock / 1000000U;
  s_report_len = 0U;

  _say("\r\n=== BANCO DE LA SD1 (sd_bench.c) ===\r\n");
  void *h = storage_media_handle(BENCH_SD);
  if (h == NULL) {
    _say(" la SD1 no esta montada: no se hace nada\r\n");
    return;
  }
  FX_MEDIA *m = storage_media_fx(h);
  const storage_sd_info_t *info = storage_get_info(BENCH_SD);
  const uint32_t div0 = sd_card_get_clock_div(BENCH_SD);
  _say(" tarjeta %lu MB, libres %lu MB, FAT%s, cluster %lu B; bus hoy a %lu "
       "kHz\r\n",
       (unsigned long)info->capacity_mb, (unsigned long)info->free_mb,
       m->fx_media_32_bit_FAT ? "32" : "12/16",
       (unsigned long)(m->fx_media_sectors_per_cluster *
                       m->fx_media_bytes_per_sector),
       (unsigned long)_clock_khz(div0));
  const uint32_t t_start = HAL_GetTick();

  /* 1. Barrido */
  _say("\r\n-- 1. Velocidad del bus (todo se relee y compara) --\r\n");
  int best = -1;
  for (uint32_t i = 0U; i < BENCH_N_DIVS; i++) {
    if (!_sweep_one(m, s_divs[i])) {
      _say("   -> falla a %lu kHz: no se prueba mas rapido\r\n",
           (unsigned long)_clock_khz(s_divs[i]));
      break;
    }
    best = (int)i;
  }

  if (best < 0) {
    _say(" ni siquiera a la velocidad de hoy: algo va mal\r\n");
  } else {
    const uint32_t div_best = s_divs[best];
    _say(" -> la mas rapida que pasa: %lu kHz\r\n",
         (unsigned long)_clock_khz(div_best));

    /* 2. Sostenida */
    _say("\r\n-- 2. Escritura sostenida a %lu kHz --\r\n",
         (unsigned long)_clock_khz(div_best));
    (void)sd_card_set_clock_div(BENCH_SD, div_best);
    _sustained(m, false);
    _sustained(m, true);

    /* 3. El registro de vuelo, a la mejor velocidad y a la de hoy */
    _say("\r\n-- 3. Simulacion del registro de vuelo (bloqueante) --\r\n");
    _log_sim(m, div_best);
    if (div_best != div0) {
      _log_sim(m, div0);
    }
  }

  /* Todo como estaba */
  (void)sd_card_set_clock_div(BENCH_SD, div0);
  (void)fx_file_delete(m, (CHAR *)BENCH_FILE);
  (void)fx_media_flush(m);
  _say("\r\n bus de vuelta a %lu kHz. Banco: %lu s. Informe en %s\r\n",
       (unsigned long)_clock_khz(div0),
       (unsigned long)((HAL_GetTick() - t_start) / 1000U), BENCH_REPORT);
  _say("=== FIN DEL BANCO ===\r\n");
  if (!storage_write_file(BENCH_SD, BENCH_REPORT, (const uint8_t *)s_report,
                          s_report_len, false)) {
    DebugOut_Printf(" (no se pudo guardar %s)\r\n", BENCH_REPORT);
  }
}
