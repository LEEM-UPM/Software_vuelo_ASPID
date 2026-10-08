/*
 * Escritura y lectura de campos en el buffer de datos de una trama CAN
 * (ver ICD.md, sección 3).
 *
 * - Todo en little-endian, independientemente del micro.
 * - Solo tipos de tamaño fijo y float32 IEEE-754.
 * - Cada función escribe o lee en buf[0], buf[1]... El que llama pasa el
 *   puntero ya desplazado al byte del campo: can_put_u16_le(&data[3], v).
 * - El que llama es responsable de que el campo quepa en el buffer.
 *
 * Funciones static inline: cada archivo que incluya esta cabecera tiene su
 * copia, sin errores de definición múltiple al enlazar.
 */

#ifndef CAN_PACK_H
#define CAN_PACK_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#if defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L
_Static_assert(sizeof(float) == 4U, "can_pack: float debe ser de 32 bits");
#endif

/* ------------------------------------------------------------------------
 * Escritura
 * ------------------------------------------------------------------------ */
static inline void can_put_u8(uint8_t *buf, uint8_t v) { buf[0] = v; }

static inline void can_put_i8(uint8_t *buf, int8_t v) {
  memcpy(buf, &v, sizeof(v));
}

static inline void can_put_u16_le(uint8_t *buf, uint16_t v) {
  buf[0] = (uint8_t)v;
  buf[1] = (uint8_t)(v >> 8);
}

static inline void can_put_u32_le(uint8_t *buf, uint32_t v) {
  buf[0] = (uint8_t)v;
  buf[1] = (uint8_t)(v >> 8);
  buf[2] = (uint8_t)(v >> 16);
  buf[3] = (uint8_t)(v >> 24);
}

static inline void can_put_u64_le(uint8_t *buf, uint64_t v) {
  can_put_u32_le(&buf[0], (uint32_t)v);
  can_put_u32_le(&buf[4], (uint32_t)(v >> 32));
}

/*
 * Con signo: se copian los bits al entero sin signo del mismo tamaño. Los
 * tipos intN_t son complemento a dos por definición, así que es portable.
 */
static inline void can_put_i16_le(uint8_t *buf, int16_t v) {
  uint16_t u;
  memcpy(&u, &v, sizeof(u));
  can_put_u16_le(buf, u);
}

static inline void can_put_i32_le(uint8_t *buf, int32_t v) {
  uint32_t u;
  memcpy(&u, &v, sizeof(u));
  can_put_u32_le(buf, u);
}

static inline void can_put_f32_le(uint8_t *buf, float v) {
  uint32_t u;
  memcpy(&u, &v, sizeof(u));
  can_put_u32_le(buf, u);
}

/* Booleano en un byte: 0 = falso, 1 = verdadero */
static inline void can_put_bool(uint8_t *buf, bool v) {
  buf[0] = v ? 1U : 0U;
}

/* ------------------------------------------------------------------------
 * Lectura
 * ------------------------------------------------------------------------ */
static inline uint8_t can_get_u8(const uint8_t *buf) { return buf[0]; }

static inline int8_t can_get_i8(const uint8_t *buf) {
  int8_t v;
  memcpy(&v, buf, sizeof(v));
  return v;
}

static inline uint16_t can_get_u16_le(const uint8_t *buf) {
  return (uint16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
}

static inline uint32_t can_get_u32_le(const uint8_t *buf) {
  return (uint32_t)buf[0] | ((uint32_t)buf[1] << 8) |
         ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24);
}

static inline uint64_t can_get_u64_le(const uint8_t *buf) {
  return (uint64_t)can_get_u32_le(&buf[0]) |
         ((uint64_t)can_get_u32_le(&buf[4]) << 32);
}

static inline int16_t can_get_i16_le(const uint8_t *buf) {
  uint16_t u = can_get_u16_le(buf);
  int16_t v;
  memcpy(&v, &u, sizeof(v));
  return v;
}

static inline int32_t can_get_i32_le(const uint8_t *buf) {
  uint32_t u = can_get_u32_le(buf);
  int32_t v;
  memcpy(&v, &u, sizeof(v));
  return v;
}

static inline float can_get_f32_le(const uint8_t *buf) {
  uint32_t u = can_get_u32_le(buf);
  float v;
  memcpy(&v, &u, sizeof(v));
  return v;
}

/* Cualquier valor distinto de 0 se interpreta como verdadero */
static inline bool can_get_bool(const uint8_t *buf) { return buf[0] != 0U; }

#endif /* CAN_PACK_H */
