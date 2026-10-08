#include "storage_stream.h"

#include <string.h>

/* Definido en storage.c: devuelve el storage_sd_t de una tarjeta montada, o
 * NULL. Se declara aquí para no exponer el tipo en storage.h. */
void *storage_media_handle(sd_card_id_t sd);
FX_MEDIA *storage_media_fx(void *handle);
void storage_media_note_error(void *handle, sd_card_id_t sd, uint32_t fx_error);

static bool _fail(storage_stream_t *st, UINT status) {
  st->error = (uint32_t)status;
  if (st->media != NULL) {
    storage_media_note_error(st->media, st->sd, (uint32_t)status);
  }
  return false;
}

/* Baja a la tarjeta lo que haya acumulado */
static bool _flush(storage_stream_t *st) {
  if (st->buf_used == 0U) {
    return true;
  }
  const UINT status = fx_file_write(&st->file, st->buf, st->buf_used);
  if (status != FX_SUCCESS) {
    return _fail(st, status);
  }
  st->buf_used = 0U;
  return true;
}

static bool _prepare(storage_stream_t *st, sd_card_id_t sd) {
  memset(st, 0, sizeof(*st));
  st->sd = sd;
  st->media = storage_media_handle(sd);
  return (st->media != NULL);
}

bool storage_stream_open_read(storage_stream_t *st, sd_card_id_t sd,
                              const char *path) {
  if ((st == NULL) || (path == NULL)) {
    return false;
  }
  if (!_prepare(st, sd)) {
    return false;
  }

  const UINT status = fx_file_open(storage_media_fx(st->media), &st->file,
                                   (CHAR *)path, FX_OPEN_FOR_READ);
  if (status != FX_SUCCESS) {
    return _fail(st, status);
  }

  st->open = true;
  st->writing = false;
  st->size_bytes = (uint32_t)st->file.fx_file_current_file_size;
  return true;
}

bool storage_stream_open_write(storage_stream_t *st, sd_card_id_t sd,
                               const char *path, uint8_t *buf,
                               uint32_t buf_size) {
  if ((st == NULL) || (path == NULL) || (buf == NULL) || (buf_size == 0U)) {
    return false;
  }
  if (!_prepare(st, sd)) {
    return false;
  }

  FX_MEDIA *media = storage_media_fx(st->media);
  UINT status = fx_file_create(media, (CHAR *)path);
  if ((status != FX_SUCCESS) && (status != FX_ALREADY_CREATED)) {
    return _fail(st, status);
  }

  status = fx_file_open(media, &st->file, (CHAR *)path, FX_OPEN_FOR_WRITE);
  if (status != FX_SUCCESS) {
    return _fail(st, status);
  }

  /* Se vacía: si quedara contenido de una ejecución anterior más larga, el
   * archivo tendría una cola que no corresponde a esta. */
  status = fx_file_truncate_release(&st->file, 0U);
  if (status == FX_SUCCESS) {
    status = fx_file_seek(&st->file, 0U);
  }
  if (status != FX_SUCCESS) {
    (void)fx_file_close(&st->file);
    return _fail(st, status);
  }

  st->open = true;
  st->writing = true;
  st->buf = buf;
  st->buf_size = buf_size;
  st->buf_used = 0U;
  return true;
}

bool storage_stream_read(storage_stream_t *st, void *dst, uint32_t len) {
  if ((st == NULL) || !st->open || st->writing || (dst == NULL)) {
    return false;
  }
  if (len == 0U) {
    return true;
  }

  ULONG actual = 0U;
  const UINT status = fx_file_read(&st->file, dst, len, &actual);
  if (status != FX_SUCCESS) {
    return _fail(st, status);
  }
  if (actual != (ULONG)len) {
    return _fail(st, FX_END_OF_FILE); /* archivo más corto de lo esperado */
  }
  return true;
}

bool storage_stream_write(storage_stream_t *st, const void *src,
                          uint32_t len) {
  if ((st == NULL) || !st->open || !st->writing || (src == NULL)) {
    return false;
  }

  const uint8_t *in = (const uint8_t *)src;
  while (len > 0U) {
    uint32_t space = st->buf_size - st->buf_used;
    if (space == 0U) {
      if (!_flush(st)) {
        return false;
      }
      space = st->buf_size;
    }
    const uint32_t chunk = (len < space) ? len : space;
    memcpy(&st->buf[st->buf_used], in, chunk);
    st->buf_used += chunk;
    in += chunk;
    len -= chunk;
  }
  return true;
}

bool storage_stream_puts(storage_stream_t *st, const char *text) {
  if (text == NULL) {
    return false;
  }
  return storage_stream_write(st, text, (uint32_t)strlen(text));
}

bool storage_stream_close(storage_stream_t *st) {
  if ((st == NULL) || !st->open) {
    return (st != NULL) && (st->error == 0U);
  }

  bool ok = true;
  if (st->writing) {
    ok = _flush(st);
  }

  UINT status = fx_file_close(&st->file);
  if ((status != FX_SUCCESS) && ok) {
    ok = _fail(st, status);
  }

  if (st->writing) {
    status = fx_media_flush(storage_media_fx(st->media));
    if ((status != FX_SUCCESS) && ok) {
      ok = _fail(st, status);
    }
  }

  st->open = false;
  return ok;
}
