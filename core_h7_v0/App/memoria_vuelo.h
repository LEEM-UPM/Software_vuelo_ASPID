/*
 * Memoria del vuelo en la flash interna: lo justo para que, si la core se
 * reinicia en el aire, sepa en qué punto del vuelo estaba (emergencia.h).
 *
 * DÓNDE. El STM32H723ZE tiene 512 KB de flash en 4 sectores de 128 KB y un
 * solo banco. El firmware usa los tres primeros (el enlazador lo limita a
 * 384 KB, STM32H723XX_FLASH.ld) y el sector 3 (0x08060000) es solo para
 * esto. Si algún día el firmware no cupiera y pisara el sector, al arrancar
 * se nota (memoria_vuelo_disponible() = false) y no se usa.
 *
 * CÓMO. Registros de 32 bytes, una "palabra de flash" del H7 (lo mínimo que
 * se programa de una vez), uno detrás de otro: caben 4096. El estado es el
 * último válido (CRC bien, secuencia más alta). Escribir uno tarda decenas
 * de µs y se puede en vuelo. BORRAR BLOQUEA TODO EL CHIP 1-2 s (un solo
 * banco: mientras borra no se puede leer código de la flash), así que solo
 * se borra al arrancar en tierra y cuando queda poco sitio
 * (memoria_vuelo_mantener). En vuelo, solo se añade.
 *
 * SI SE VA LA CORRIENTE ESCRIBIENDO, la palabra puede quedar con el ECC mal,
 * y leerla da un fallo de bus. La lectura del arranque los ignora (FAULTMASK
 * + BFHFNMIGN) y salta esa palabra: vale la anterior.
 *
 * FORMATO de cada registro (little endian):
 *   0     0xA7, marca              1   versión (MEMORIA_VUELO_VERSION)
 *   2-3   número de vuelo          4-7 secuencia, +1 por registro
 *   8-11  ms de vuelo (desde el despegue) al escribirlo
 *   12    estado del GNC           13  fase del GNC
 *   14    MV_* (abajo)             15  reinicios en vuelo
 *   16-19 presión de referencia (suelo) del baro 1, float Pa
 *   20-23 la del baro 2
 *   24-25 altura máxima del baro 1, int16 en 0,5 m
 *   26-27 la del baro 2
 *   28-31 CRC-32 de los bytes 0-27
 */
#ifndef MEMORIA_VUELO_H
#define MEMORIA_VUELO_H

#include <stdbool.h>
#include <stdint.h>

#define MEMORIA_VUELO_MARCA 0xA7U
#define MEMORIA_VUELO_VERSION 1U

#define MEMORIA_VUELO_DIRECCION 0x08060000U /* sector 3 */
#define MEMORIA_VUELO_SECTOR_BYTES 0x20000U
#define MEMORIA_VUELO_REGISTRO_BYTES 32U
#define MEMORIA_VUELO_REGISTROS \
  (MEMORIA_VUELO_SECTOR_BYTES / MEMORIA_VUELO_REGISTRO_BYTES)

/* Al arrancar en tierra se borra si está más llena que esto */
#define MEMORIA_VUELO_BORRAR_DESDE ((MEMORIA_VUELO_REGISTROS * 3U) / 4U)

/* flags */
#define MV_DESPEGUE (1U << 0)   /* hubo despegue: hay un vuelo en marcha */
#define MV_APOGEO (1U << 1)     /* apogeo visto (GNC o barómetro) */
#define MV_DROGUE (1U << 2)     /* drogue (1A y 1B) mandado */
#define MV_MAIN (1U << 3)       /* main (2A) mandado */
#define MV_TERMINADO (1U << 4)  /* vuelo terminado (50 m, en tierra o 20 min) */
#define MV_SIMULACION (1U << 5) /* nunca se escribe: si aparece, no vale */
#define MV_EMERGENCIA (1U << 6) /* este vuelo ha pasado por emergencia */

typedef struct {
  uint16_t vuelo;
  uint32_t t_vuelo_ms;
  uint8_t estado;
  uint8_t fase;
  uint8_t flags;
  uint8_t reinicios;
  float p0_pa[2];
  float max_m[2];
} memoria_vuelo_t;

/* Por qué se reinició la core (RCC_RSR, leído y borrado al arrancar) */
typedef enum {
  MEMORIA_RESET_ENCENDIDO = 0, /* encendido o bajón de tensión (POR/BOR) */
  MEMORIA_RESET_WATCHDOG,      /* IWDG: el bucle se colgó */
  MEMORIA_RESET_SOFTWARE,      /* NVIC_SystemReset: un fallo, o a propósito */
  MEMORIA_RESET_PIN,           /* el pin NRST: botón o ST-Link */
  MEMORIA_RESET_OTRO,
} memoria_reset_t;

/* LO PRIMERO en main(), antes de encender las cachés: la causa del reset y
 * el último registro válido del sector. No usa el HAL. */
void memoria_vuelo_arrancar(void);

memoria_reset_t memoria_vuelo_causa(void);
const char *memoria_vuelo_causa_nombre(memoria_reset_t c);

/* false si el sector no se puede usar (el firmware lo pisa) */
bool memoria_vuelo_disponible(void);

/* El último registro: el leído al arrancar o el último escrito. false si no
 * hay ninguno válido. */
bool memoria_vuelo_ultimo(memoria_vuelo_t *m);

/* Añade un registro. false si no cabe o la flash falla. */
bool memoria_vuelo_guardar(const memoria_vuelo_t *m);

/* SOLO al arrancar en tierra: si el sector pasa de MEMORIA_VUELO_BORRAR_DESDE
 * lo borra (1-2 s bloqueado) y vuelve a escribir el último registro. */
void memoria_vuelo_mantener(void);

/* Para la consola */
uint32_t memoria_vuelo_usados(void);    /* registros ocupados */
uint32_t memoria_vuelo_escritos(void);  /* escritos desde el arranque */
uint32_t memoria_vuelo_fallos(void);    /* escrituras que fallaron */
uint32_t memoria_vuelo_malos(void);     /* registros con ECC o CRC mal */
uint32_t memoria_vuelo_borrados(void);  /* borrados del sector en este arranque */
uint32_t memoria_vuelo_borrado_ms(void); /* lo que tardó el último borrado */
uint32_t memoria_vuelo_escritura_max_us(void); /* la escritura más lenta */

#endif /* MEMORIA_VUELO_H */
