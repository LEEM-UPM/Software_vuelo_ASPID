/**
 * @file      sx126x_hal.c
 *
 * @brief     Implements the SX126x radio HAL functions for STM32 (HAL de ST).
 *
 * Reglas (cada una corresponde a un fallo real ya depurado en esta placa):
 *
 *  1. Antes de CADA transacción SPI se espera a que BUSY baje, con timeout de
 *     SX126X_HAL_BUSY_TIMEOUT_MS. Un bucle de espera sin timeout cuelga el
 *     firmware para siempre en cuanto el radio falla (p. ej. si la
 *     calibración no termina y BUSY se queda en alto).
 *  2. reset(): NRESET a bajo SX126X_HAL_RESET_PULSE_MS (el datasheet pide
 *     >= 100 µs), soltar y esperar a que BUSY baje antes de volver.
 *  3. write()/read(): NSS bajo durante todo el comando (mandato + datos). No
 *     se llama a la transferencia SPI con longitud 0: el HAL de ST devuelve
 *     error en ese caso.
 */

#include <stddef.h>

#include "sx126x_hal.h"
#include "sx126x_hal_context_stm32.h"

/** Tiempo máximo de espera a que BUSY baje antes de una transacción. */
#define SX126X_HAL_BUSY_TIMEOUT_MS 100U

/** Duración del pulso de NRESET. */
#define SX126X_HAL_RESET_PULSE_MS 2U

/** Timeout de cada transferencia SPI (a 8 MHz, 255 bytes tardan < 1 ms). */
#define SX126X_HAL_SPI_TIMEOUT_MS 100U

/**
 * @brief Espera a que BUSY baje, como mucho SX126X_HAL_BUSY_TIMEOUT_MS.
 * @return SX126X_HAL_STATUS_ERROR si BUSY sigue en alto al agotarse.
 */
static sx126x_hal_status_t sx126x_hal_wait_on_busy(
    const sx126x_hal_context_t *ctx) {
  uint32_t start = HAL_GetTick();

  while (HAL_GPIO_ReadPin(ctx->busy.port, ctx->busy.pin) == GPIO_PIN_SET) {
    if ((HAL_GetTick() - start) >= SX126X_HAL_BUSY_TIMEOUT_MS) {
      return SX126X_HAL_STATUS_ERROR;
    }
  }
  return SX126X_HAL_STATUS_OK;
}

sx126x_hal_status_t sx126x_hal_reset(const void *context) {
  const sx126x_hal_context_t *ctx = (const sx126x_hal_context_t *)context;

  HAL_GPIO_WritePin(ctx->reset.port, ctx->reset.pin, GPIO_PIN_RESET);
  HAL_Delay(SX126X_HAL_RESET_PULSE_MS);
  HAL_GPIO_WritePin(ctx->reset.port, ctx->reset.pin, GPIO_PIN_SET);

  return sx126x_hal_wait_on_busy(ctx);
}

sx126x_hal_status_t sx126x_hal_wakeup(const void *context) {
  const sx126x_hal_context_t *ctx = (const sx126x_hal_context_t *)context;

  /* Un flanco de bajada en NSS saca al chip de sleep */
  HAL_GPIO_WritePin(ctx->nss.port, ctx->nss.pin, GPIO_PIN_RESET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(ctx->nss.port, ctx->nss.pin, GPIO_PIN_SET);

  return sx126x_hal_wait_on_busy(ctx);
}

sx126x_hal_status_t sx126x_hal_write(const void *context,
                                     const uint8_t *command,
                                     const uint16_t command_length,
                                     const uint8_t *data,
                                     const uint16_t data_length) {
  const sx126x_hal_context_t *ctx = (const sx126x_hal_context_t *)context;

  if (sx126x_hal_wait_on_busy(ctx) != SX126X_HAL_STATUS_OK) {
    return SX126X_HAL_STATUS_ERROR;
  }

  HAL_StatusTypeDef status = HAL_OK;

  HAL_GPIO_WritePin(ctx->nss.port, ctx->nss.pin, GPIO_PIN_RESET);
  if (command_length > 0U) {
    status = HAL_SPI_Transmit(ctx->spi, (uint8_t *)command, command_length,
                              SX126X_HAL_SPI_TIMEOUT_MS);
  }
  if ((status == HAL_OK) && (data_length > 0U)) {
    status = HAL_SPI_Transmit(ctx->spi, (uint8_t *)data, data_length,
                              SX126X_HAL_SPI_TIMEOUT_MS);
  }
  HAL_GPIO_WritePin(ctx->nss.port, ctx->nss.pin, GPIO_PIN_SET);

  return (status == HAL_OK) ? SX126X_HAL_STATUS_OK : SX126X_HAL_STATUS_ERROR;
}

sx126x_hal_status_t sx126x_hal_read(const void *context, const uint8_t *command,
                                    const uint16_t command_length,
                                    uint8_t *data, const uint16_t data_length) {
  const sx126x_hal_context_t *ctx = (const sx126x_hal_context_t *)context;

  if (sx126x_hal_wait_on_busy(ctx) != SX126X_HAL_STATUS_OK) {
    return SX126X_HAL_STATUS_ERROR;
  }

  HAL_StatusTypeDef status = HAL_OK;

  HAL_GPIO_WritePin(ctx->nss.port, ctx->nss.pin, GPIO_PIN_RESET);
  if (command_length > 0U) {
    status = HAL_SPI_Transmit(ctx->spi, (uint8_t *)command, command_length,
                              SX126X_HAL_SPI_TIMEOUT_MS);
  }
  if ((status == HAL_OK) && (data_length > 0U)) {
    status = HAL_SPI_Receive(ctx->spi, data, data_length,
                             SX126X_HAL_SPI_TIMEOUT_MS);
  }
  HAL_GPIO_WritePin(ctx->nss.port, ctx->nss.pin, GPIO_PIN_SET);

  return (status == HAL_OK) ? SX126X_HAL_STATUS_OK : SX126X_HAL_STATUS_ERROR;
}
