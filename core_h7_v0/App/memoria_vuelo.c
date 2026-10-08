#include "memoria_vuelo.h"

#include <string.h>

#include "main.h"

/* Fin de lo que el firmware ocupa en la flash (STM32H723XX_FLASH.ld) */
extern uint32_t __data_source_end;

static memoria_reset_t s_causa = MEMORIA_RESET_OTRO;
static bool s_disponible;
static bool s_hay;
static memoria_vuelo_t s_ultimo;
static uint32_t s_secuencia;
static uint32_t s_siguiente; /* el primer hueco libre */
static uint32_t s_escritos;
static uint32_t s_fallos;
static uint32_t s_malos;
static uint32_t s_borrados;
static uint32_t s_borrado_ms;
static uint32_t s_escritura_max_us;

/* CRC-32 (el de Ethernet/zip), sin tabla: son 28 bytes */
static uint32_t _crc32(const uint8_t *p, uint32_t n) {
  uint32_t crc = 0xFFFFFFFFU;
  for (uint32_t i = 0U; i < n; i++) {
    crc ^= p[i];
    for (uint32_t b = 0U; b < 8U; b++) {
      crc = (crc & 1U) ? ((crc >> 1) ^ 0xEDB88320U) : (crc >> 1);
    }
  }
  return ~crc;
}

static void _u16(uint8_t *p, uint32_t v) {
  p[0] = (uint8_t)(v & 0xFFU);
  p[1] = (uint8_t)(v >> 8);
}

static void _u32(uint8_t *p, uint32_t v) {
  for (uint32_t i = 0U; i < 4U; i++) {
    p[i] = (uint8_t)(v >> (8U * i));
  }
}

static uint32_t _l16(const uint8_t *p) { return p[0] | ((uint32_t)p[1] << 8); }

static uint32_t _l32(const uint8_t *p) {
  return p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
         ((uint32_t)p[3] << 24);
}

static int16_t _medio_metro(float m) {
  const float v = m * 2.0f;
  if (!(v > -32767.0f)) {
    return -32767;
  }
  if (v > 32767.0f) {
    return 32767;
  }
  return (int16_t)v;
}

static void _a_bytes(const memoria_vuelo_t *m, uint32_t secuencia,
                     uint8_t b[MEMORIA_VUELO_REGISTRO_BYTES]) {
  memset(b, 0, MEMORIA_VUELO_REGISTRO_BYTES);
  b[0] = MEMORIA_VUELO_MARCA;
  b[1] = MEMORIA_VUELO_VERSION;
  _u16(&b[2], m->vuelo);
  _u32(&b[4], secuencia);
  _u32(&b[8], m->t_vuelo_ms);
  b[12] = m->estado;
  b[13] = m->fase;
  b[14] = m->flags;
  b[15] = m->reinicios;
  memcpy(&b[16], &m->p0_pa[0], 4U);
  memcpy(&b[20], &m->p0_pa[1], 4U);
  _u16(&b[24], (uint16_t)_medio_metro(m->max_m[0]));
  _u16(&b[26], (uint16_t)_medio_metro(m->max_m[1]));
  _u32(&b[28], _crc32(b, 28U));
}

static bool _de_bytes(const uint8_t b[MEMORIA_VUELO_REGISTRO_BYTES],
                      memoria_vuelo_t *m, uint32_t *secuencia) {
  if ((b[0] != MEMORIA_VUELO_MARCA) || (b[1] != MEMORIA_VUELO_VERSION) ||
      (_l32(&b[28]) != _crc32(b, 28U))) {
    return false;
  }
  m->vuelo = (uint16_t)_l16(&b[2]);
  *secuencia = _l32(&b[4]);
  m->t_vuelo_ms = _l32(&b[8]);
  m->estado = b[12];
  m->fase = b[13];
  m->flags = b[14];
  m->reinicios = b[15];
  memcpy(&m->p0_pa[0], &b[16], 4U);
  memcpy(&m->p0_pa[1], &b[20], 4U);
  m->max_m[0] = (float)(int16_t)_l16(&b[24]) * 0.5f;
  m->max_m[1] = (float)(int16_t)_l16(&b[26]) * 0.5f;
  return true;
}

/* Lee un registro de la flash sin morir si su ECC está roto: con FAULTMASK
 * y BFHFNMIGN el fallo de bus se ignora, y luego se mira el error de ECC
 * doble en FLASH_SR1. Devuelve false si estaba roto. */
static bool _leer(uint32_t direccion, uint8_t b[MEMORIA_VUELO_REGISTRO_BYTES]) {
  const uint32_t ccr = SCB->CCR;
  __set_FAULTMASK(1U);
  SCB->CCR = ccr | SCB_CCR_BFHFNMIGN_Msk;
  __DSB();
  __ISB();
  const volatile uint32_t *f = (const volatile uint32_t *)direccion;
  uint32_t w[MEMORIA_VUELO_REGISTRO_BYTES / 4U];
  for (uint32_t i = 0U; i < (MEMORIA_VUELO_REGISTRO_BYTES / 4U); i++) {
    w[i] = f[i];
  }
  __DSB();
  SCB->CCR = ccr;
  __ISB();
  __set_FAULTMASK(0U);
  const bool roto = (FLASH->SR1 & FLASH_SR_DBECCERR) != 0U;
  FLASH->CCR1 = FLASH_CCR_CLR_DBECCERR | FLASH_CCR_CLR_SNECCERR;
  memcpy(b, w, MEMORIA_VUELO_REGISTRO_BYTES);
  return !roto;
}

static bool _vacio(const uint8_t *b) {
  for (uint32_t i = 0U; i < MEMORIA_VUELO_REGISTRO_BYTES; i++) {
    if (b[i] != 0xFFU) {
      return false;
    }
  }
  return true;
}

static void _causa(void) {
  const uint32_t rsr = RCC->RSR;
  if ((rsr & (RCC_RSR_PORRSTF | RCC_RSR_BORRSTF)) != 0U) {
    s_causa = MEMORIA_RESET_ENCENDIDO;
  } else if ((rsr & RCC_RSR_IWDG1RSTF) != 0U) {
    s_causa = MEMORIA_RESET_WATCHDOG;
  } else if ((rsr & RCC_RSR_SFTRSTF) != 0U) {
    s_causa = MEMORIA_RESET_SOFTWARE;
  } else if ((rsr & RCC_RSR_PINRSTF) != 0U) {
    s_causa = MEMORIA_RESET_PIN;
  } else {
    s_causa = MEMORIA_RESET_OTRO;
  }
  RCC->RSR |= RCC_RSR_RMVF; /* que el siguiente arranque vea solo el suyo */
}

void memoria_vuelo_arrancar(void) {
  _causa();
  s_disponible = ((uint32_t)&__data_source_end <= MEMORIA_VUELO_DIRECCION);
  if (!s_disponible) {
    return;
  }
  s_hay = false;
  s_siguiente = 0U;
  for (uint32_t i = 0U; i < MEMORIA_VUELO_REGISTROS; i++) {
    uint8_t b[MEMORIA_VUELO_REGISTRO_BYTES];
    const uint32_t dir = MEMORIA_VUELO_DIRECCION + (i * MEMORIA_VUELO_REGISTRO_BYTES);
    const bool sano = _leer(dir, b);
    if (sano && _vacio(b)) {
      continue;
    }
    s_siguiente = i + 1U; /* nunca se escribe antes de algo ya escrito */
    memoria_vuelo_t m;
    uint32_t sec;
    if (!sano || !_de_bytes(b, &m, &sec)) {
      s_malos++;
      continue;
    }
    if (!s_hay || ((int32_t)(sec - s_secuencia) > 0)) {
      s_hay = true;
      s_secuencia = sec;
      s_ultimo = m;
    }
  }
}

memoria_reset_t memoria_vuelo_causa(void) { return s_causa; }

const char *memoria_vuelo_causa_nombre(memoria_reset_t c) {
  switch (c) {
  case MEMORIA_RESET_ENCENDIDO:
    return "encendido";
  case MEMORIA_RESET_WATCHDOG:
    return "WATCHDOG";
  case MEMORIA_RESET_SOFTWARE:
    return "software";
  case MEMORIA_RESET_PIN:
    return "pin NRST";
  default:
    return "?";
  }
}

bool memoria_vuelo_disponible(void) { return s_disponible; }

bool memoria_vuelo_ultimo(memoria_vuelo_t *m) {
  if (!s_hay) {
    return false;
  }
  *m = s_ultimo;
  return true;
}

bool memoria_vuelo_guardar(const memoria_vuelo_t *m) {
  if (!s_disponible || (s_siguiente >= MEMORIA_VUELO_REGISTROS)) {
    s_fallos++;
    return false;
  }
  static uint32_t palabra[MEMORIA_VUELO_REGISTRO_BYTES / 4U]
      __attribute__((aligned(32)));
  _a_bytes(m, s_secuencia + 1U, (uint8_t *)palabra);
  const uint32_t dir =
      MEMORIA_VUELO_DIRECCION + (s_siguiente * MEMORIA_VUELO_REGISTRO_BYTES);
  s_siguiente++; /* si falla, ese hueco queda perdido: no se reintenta en él */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  const uint32_t c0 = DWT->CYCCNT;
  HAL_FLASH_Unlock();
  const HAL_StatusTypeDef st =
      HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, dir, (uint32_t)palabra);
  HAL_FLASH_Lock();
  /* Comprobar lo escrito: la caché de datos puede tener la línea vieja */
  SCB_InvalidateDCache_by_Addr((void *)dir, (int32_t)MEMORIA_VUELO_REGISTRO_BYTES);
  const uint32_t us = (DWT->CYCCNT - c0) / (SystemCoreClock / 1000000U);
  if (us > s_escritura_max_us) {
    s_escritura_max_us = us;
  }
  if ((st != HAL_OK) ||
      (memcmp((const void *)dir, palabra, MEMORIA_VUELO_REGISTRO_BYTES) != 0)) {
    s_fallos++;
    return false;
  }
  s_secuencia++;
  s_ultimo = *m;
  s_hay = true;
  s_escritos++;
  return true;
}

void memoria_vuelo_mantener(void) {
  if (!s_disponible || (s_siguiente < MEMORIA_VUELO_BORRAR_DESDE)) {
    return;
  }
  FLASH_EraseInitTypeDef e = {0};
  e.TypeErase = FLASH_TYPEERASE_SECTORS;
  e.Banks = FLASH_BANK_1;
  e.Sector = FLASH_SECTOR_3;
  e.NbSectors = 1U;
  e.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  uint32_t sector_malo = 0U;
  /* Medido con el contador de ciclos: mientras borra no corre ninguna
   * interrupción (están en la flash), tampoco la del HAL_GetTick() */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  const uint32_t c0 = DWT->CYCCNT;
  HAL_FLASH_Unlock();
  const HAL_StatusTypeDef st = HAL_FLASHEx_Erase(&e, &sector_malo);
  HAL_FLASH_Lock();
  s_borrado_ms = (DWT->CYCCNT - c0) / (SystemCoreClock / 1000U);
  SCB_InvalidateDCache_by_Addr((void *)MEMORIA_VUELO_DIRECCION,
                               (int32_t)MEMORIA_VUELO_SECTOR_BYTES);
  if (st != HAL_OK) {
    s_fallos++;
    return;
  }
  s_borrados++;
  s_siguiente = 0U;
  if (s_hay) {
    (void)memoria_vuelo_guardar(&s_ultimo); /* que no se pierda el estado */
  }
}

uint32_t memoria_vuelo_usados(void) { return s_siguiente; }
uint32_t memoria_vuelo_escritos(void) { return s_escritos; }
uint32_t memoria_vuelo_fallos(void) { return s_fallos; }
uint32_t memoria_vuelo_malos(void) { return s_malos; }
uint32_t memoria_vuelo_borrados(void) { return s_borrados; }
uint32_t memoria_vuelo_borrado_ms(void) { return s_borrado_ms; }
uint32_t memoria_vuelo_escritura_max_us(void) { return s_escritura_max_us; }
