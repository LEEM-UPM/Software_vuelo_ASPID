/*
 * Archivos abiertos de la SD, para leerlos o escribirlos por trozos.
 *
 * storage.h trabaja con el archivo entero: abre, lee o escribe todo y cierra.
 * Eso vale para BOOTCNT.TXT, pero no para un archivo de 2 MB ni para ir
 * añadiendo líneas miles de veces, porque cada llamada volvería a abrir,
 * recorrer la FAT y volcar la caché.
 *
 * Aquí el archivo se queda abierto entre llamadas. La escritura va por un
 * buffer que el llamante proporciona: se acumulan bytes y solo se baja a la
 * tarjeta cuando se llena, que es lo que hace que escribir 21000 líneas sea
 * viable.
 *
 * Nada de esto es reentrante ni seguro entre interrupciones: es para el bucle
 * principal, como el resto del módulo.
 *
 * Hay que cerrar SIEMPRE con storage_stream_close(), que es quien vacía lo que
 * quede en el buffer y actualiza el tamaño del archivo en la tarjeta.
 */

#ifndef STORAGE_STREAM_H
#define STORAGE_STREAM_H

#include <stdbool.h>
#include <stdint.h>

#include "fx_api.h"
#include "storage.h"

typedef struct {
  FX_FILE file;
  void *media;      /* storage_sd_t*, opaco fuera de storage.c */
  sd_card_id_t sd;
  bool open;
  bool writing;

  /* Solo en escritura: buffer del llamante */
  uint8_t *buf;
  uint32_t buf_size;
  uint32_t buf_used;

  uint32_t size_bytes; /* tamaño al abrir, en lectura */
  uint32_t error;      /* último FX_* distinto de cero */
} storage_stream_t;

/*
 * Abre un archivo para leer. Deja en size_bytes su tamaño. Devuelve false si
 * la tarjeta no está montada o el archivo no existe.
 */
bool storage_stream_open_read(storage_stream_t *st, sd_card_id_t sd,
                              const char *path);

/*
 * Crea el archivo (o lo vacía si ya existe) y lo abre para escribir. 'buf' es
 * el buffer de acumulación, que tiene que seguir vivo hasta el close; cuanto
 * más grande, menos accesos a la tarjeta (4 KB o más va bien).
 */
bool storage_stream_open_write(storage_stream_t *st, sd_card_id_t sd,
                               const char *path, uint8_t *buf,
                               uint32_t buf_size);

/* Lee exactamente 'len' bytes. false si se acabó el archivo o hubo error. */
bool storage_stream_read(storage_stream_t *st, void *dst, uint32_t len);

/* Añade bytes al buffer, bajándolos a la tarjeta cuando se llena. */
bool storage_stream_write(storage_stream_t *st, const void *src, uint32_t len);

/* Igual, con una cadena terminada en cero (sin el cero). */
bool storage_stream_puts(storage_stream_t *st, const char *text);

/* Vacía el buffer, cierra y sincroniza la tarjeta. Se puede llamar aunque ya
 * esté cerrado. Devuelve false si algo falló en el camino. */
bool storage_stream_close(storage_stream_t *st);

#endif /* STORAGE_STREAM_H */
