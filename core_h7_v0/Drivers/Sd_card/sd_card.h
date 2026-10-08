/**
 * @file sd_card.h
 * @brief Acceso por bloques a las dos tarjetas microSD del core.
 *
 * SD1 en SDMMC1 (PC8-PC12, PD2, detección PC7) y SD2 en SDMMC2 (PG9-PG11, PB4,
 * PD6, PD7, detección PA8), bus de 4 bits. Las transferencias van por DMA
 * (IDMA del SDMMC) y las funciones esperan a que terminen: son bloqueantes.
 *
 * A diferencia de MX_SDMMCx_SD_Init(), si no hay tarjeta o falla no se llama a
 * Error_Handler(): sd_card_init() devuelve false y el resto del core sigue.
 */

#ifndef SD_CARD_H
#define SD_CARD_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
  SD_CARD_1 = 0, /* SDMMC1 */
  SD_CARD_2,     /* SDMMC2 */
  SD_CARD_COUNT
} sd_card_id_t;

#define SD_CARD_BLOCK_SIZE 512U

/* Inicializa la tarjeta (identificación, bus de 4 bits). Se puede volver a
 * llamar tras un fallo para reintentar. */
bool sd_card_init(sd_card_id_t id);
void sd_card_deinit(sd_card_id_t id);
bool sd_card_is_ready(sd_card_id_t id);

/* Nivel crudo del pin de detección de tarjeta (SDx_CD) */
bool sd_card_cd_pin_high(sd_card_id_t id);

/* Número de bloques de 512 bytes de la tarjeta (0 si no está lista) */
uint32_t sd_card_block_count(sd_card_id_t id);

/* Divisor del reloj del bus: SDMMC_CK = 20 MHz / (2 * div), y div = 0 es
 * sin dividir (20 MHz). Solo entre transferencias, que desde el bucle es
 * siempre (todas son bloqueantes). Lo usa el banco de App/sd_bench.c. */
bool sd_card_set_clock_div(sd_card_id_t id, uint32_t div);
uint32_t sd_card_get_clock_div(sd_card_id_t id);

/*
 * ESCRITURA SIN BLOQUEAR (la usa el registro de vuelo, App/aspid_data.c).
 *
 * sd_card_write_start() lanza el DMA y vuelve enseguida; sd_card_write_poll()
 * dice cómo va, también sin esperar: OCUPADA mientras corre el DMA y la
 * tarjeta programa, HECHA al acabar y FALLO si algo falla o tarda más de lo
 * que permite la norma SD (250 ms de programación, con margen). Con un FALLO
 * la tarjeta se da por perdida (deja de estar lista) y no se reintenta nada:
 * una tarjeta que muere nunca para el bucle. HECHA o FALLO se mantienen hasta
 * la siguiente sd_card_write_start().
 *
 * Una escritura cada vez. El buffer tiene que estar alineado a 32 bytes y
 * fuera de la DTCM, y no se puede tocar hasta que acabe. Las lecturas y
 * escrituras bloqueantes de siempre esperan a que termine la que esté en
 * marcha (lo que tarde una escritura sana, unos ms).
 */
typedef enum {
  SD_CARD_ASYNC_LIBRE = 0,
  SD_CARD_ASYNC_OCUPADA,
  SD_CARD_ASYNC_HECHA,
  SD_CARD_ASYNC_FALLO,
} sd_card_async_t;

#define SD_CARD_ASYNC_DMA_MS 500U  /* hasta el fin de la transferencia */
#define SD_CARD_ASYNC_PROG_MS 500U /* y hasta que la tarjeta termina de
                                      programar (la norma da 250 ms) */

bool sd_card_write_start(sd_card_id_t id, const uint8_t *buf, uint32_t block,
                         uint32_t count);
sd_card_async_t sd_card_write_poll(sd_card_id_t id);

/* Lo más que han tardado, en us: lanzar una escritura sin bloquear, una
 * llamada a sd_card_write_poll() (solo registros: deberían ser unos us) y la
 * interrupción del SDMMC (solo la usan las operaciones bloqueantes, y ahí el
 * HAL manda el CMD12 y espera su respuesta). */
typedef struct {
  uint32_t inicio_max_us;
  uint32_t estado_max_us;
  uint32_t irq_max_us;
} sd_card_tiempos_t;

sd_card_tiempos_t sd_card_tiempos(sd_card_id_t id);
void sd_card_irq_medir(sd_card_id_t id, uint32_t us); /* desde la IRQ */

/* Último código de error del HAL (HAL_SD_ERROR_*), 0 si no hubo */
uint32_t sd_card_last_error(sd_card_id_t id);

/* Lectura / escritura de 'count' bloques a partir de 'block'. El buffer debe
 * estar en RAM accesible por DMA (RAM_D1, donde el linker pone todo ahora). */
bool sd_card_read_blocks(sd_card_id_t id, uint8_t *buf, uint32_t block,
                         uint32_t count);
bool sd_card_write_blocks(sd_card_id_t id, const uint8_t *buf, uint32_t block,
                          uint32_t count);

#endif /* SD_CARD_H */
