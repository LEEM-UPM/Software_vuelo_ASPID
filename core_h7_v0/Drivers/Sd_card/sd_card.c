#include "sd_card.h"

#include <string.h>

#include "main.h"
#include "sdmmc.h"

/* Reloj del bus: SDMMCCLK (20 MHz, PLL1Q) / (2 * div). 16 -> 625 kHz, igual
 * que el .ioc. La identificación la hace el HAL a 400 kHz por su cuenta. */
#define SD_CARD_CLOCK_DIV 16U

/*
 * Reloj del bus de datos de cada tarjeta, una vez identificada (la
 * identificación la hace el HAL a 400 kHz). SD1 a 20 MHz, sin dividir:
 * medido con App/sd_bench.c el 2026-10-05, ~20 MB escritos, releídos y
 * comparados sin un error, 5 MB/s escribiendo (a 625 kHz eran 281 KB/s y un
 * bloque de 16 KB bloqueaba 56 ms). La SD2 sigue a 625 kHz hasta probarla
 * igual.
 */
static const uint32_t s_div_datos[SD_CARD_COUNT] = {
    [SD_CARD_1] = 0U,
    [SD_CARD_2] = SD_CARD_CLOCK_DIV,
};

#define SD_CARD_TRANSFER_TIMEOUT_MS 2000U /* fin de la transferencia DMA */
#define SD_CARD_READY_TIMEOUT_MS 1000U    /* tarjeta de vuelta en TRANSFER */

typedef struct {
  SD_HandleTypeDef *hsd;
  SDMMC_TypeDef *instance;
  GPIO_TypeDef *cd_port;
  uint16_t cd_pin;
  bool ready;
  uint32_t last_error;
  volatile bool rx_done;
  volatile bool tx_done;
  volatile bool failed;
  /* Escritura sin bloquear: estado, fase (1 DMA, 2 programando) e instante
   * en que empezó la fase */
  sd_card_async_t async;
  uint8_t async_fase;
  bool async_multi;
  uint32_t async_t0;    /* inicio de la fase (datos, programación) */
  uint32_t async_t_cmd; /* envío del último comando */
  sd_card_tiempos_t tiempos;
} sd_card_t;

static uint32_t _us_desde(uint32_t t0) {
  return (DWT->CYCCNT - t0) / (SystemCoreClock / 1000000U);
}

static sd_card_t s_cards[SD_CARD_COUNT] = {
    [SD_CARD_1] = {.hsd = &hsd1,
                   .instance = SDMMC1,
                   .cd_port = SD1_CD_GPIO_Port,
                   .cd_pin = SD1_CD_Pin},
    [SD_CARD_2] = {.hsd = &hsd2,
                   .instance = SDMMC2,
                   .cd_port = SD2_CD_GPIO_Port,
                   .cd_pin = SD2_CD_Pin},
};

/*
 * CACHÉ DE DATOS (encendida en flight_software.c). El IDMA lee y escribe la
 * RAM directamente, sin pasar por la caché del Cortex-M7, así que:
 *   - antes de ESCRIBIR en la tarjeta se limpia la caché del buffer, para que
 *     lo que esté solo en la caché llegue a la RAM y el IDMA lo vea;
 *   - al LEER, se limpia e invalida antes (que ninguna línea sucia se vacíe
 *     encima de lo que está llegando) y se invalida después (que la CPU lea
 *     lo que trajo el IDMA y no lo que tenía en caché).
 * Invalidar trabaja por líneas de 32 bytes: con un buffer que no empiece en
 * una línea se tiraría también lo que tenga al lado. Por eso un buffer que no
 * esté alineado a 32 bytes (o que el IDMA no alcance) se transfiere bloque a
 * bloque a través de este, que sí lo está. El IDMA además trabaja con
 * palabras de 32 bits, así que esto cubre también el caso de antes.
 */
#define SD_CARD_CACHE_LINE 32U
static uint8_t s_scratch[SD_CARD_BLOCK_SIZE]
    __attribute__((aligned(SD_CARD_CACHE_LINE)));

static sd_card_t *_card(sd_card_id_t id) {
  return ((unsigned)id < (unsigned)SD_CARD_COUNT) ? &s_cards[id] : NULL;
}

static sd_card_t *_card_from_handle(const SD_HandleTypeDef *hsd) {
  for (int i = 0; i < (int)SD_CARD_COUNT; i++) {
    if (s_cards[i].hsd == hsd) {
      return &s_cards[i];
    }
  }
  return NULL;
}

/* Espera a que la tarjeta vuelva al estado TRANSFER (lista para otra orden) */
static bool _wait_transfer_state(sd_card_t *c) {
  const uint32_t start = HAL_GetTick();
  while ((HAL_GetTick() - start) < SD_CARD_READY_TIMEOUT_MS) {
    if (HAL_SD_GetCardState(c->hsd) == HAL_SD_CARD_TRANSFER) {
      return true;
    }
  }
  c->last_error = HAL_SD_ERROR_TIMEOUT;
  return false;
}

/* Espera al callback de fin de transferencia (o de error) */
static bool _wait_done(sd_card_t *c, const volatile bool *done) {
  const uint32_t start = HAL_GetTick();
  while (!*done && !c->failed) {
    if ((HAL_GetTick() - start) >= SD_CARD_TRANSFER_TIMEOUT_MS) {
      (void)HAL_SD_Abort(c->hsd);
      c->last_error = HAL_SD_ERROR_TIMEOUT;
      return false;
    }
  }
  if (c->failed) {
    c->last_error = HAL_SD_GetError(c->hsd);
    return false;
  }
  return _wait_transfer_state(c);
}

/*
 * ESCRITURA SIN BLOQUEAR, A MANO SOBRE EL SDMMC. No se usa el HAL porque
 * espera cada respuesta de la tarjeta en un bucle (SDMMC_GetCmdResp1) que,
 * si la respuesta no llega, tarda ~14 s en rendirse: medido desconectando la
 * SD1 en mitad de un vuelo de prueba, el bucle principal (y el GNC) se quedó
 * parado 13,8 s dentro de HAL_SD_WriteBlocks_DMA. Aquí cada paso se lanza y
 * se mira en llamadas sucesivas, con su propio plazo:
 *
 *   CMD25/24 (escribir) -> respuesta -> datos por el IDMA hasta DATAEND ->
 *   CMD12 (parar, solo si son varios bloques) -> CMD13 hasta que la tarjeta
 *   vuelve a TRANSFER (ha terminado de programar)
 *
 * Sin interrupciones: se mira el registro STA. Al acabar el periférico queda
 * como lo deja el HAL, para que las operaciones bloqueantes de siempre
 * (FileX) sigan funcionando.
 */
#define SD_CARD_ASYNC_CMD_MS 10U /* respuesta a un comando: son microsegundos */
#define SD_CARD_R1_ESTADO(r) (((r) >> 9) & 0x0FU)
#define SD_CARD_R1_TRANSFER 4U

enum {
  FASE_NADA = 0,
  FASE_CMD_ESCRIBIR,
  FASE_DATOS,
  FASE_CMD12,
  FASE_CMD13,
};

static void _cmd(SDMMC_TypeDef *s, uint32_t indice, uint32_t arg) {
  SDMMC_CmdInitTypeDef c;
  c.Argument = arg;
  c.CmdIndex = indice;
  c.Response = SDMMC_RESPONSE_SHORT;
  c.WaitForInterrupt = SDMMC_WAIT_NO;
  c.CPSM = SDMMC_CPSM_ENABLE;
  (void)SDMMC_SendCommand(s, &c);
}

/* Respuesta del último comando, sin esperar: 0 aún no, 1 llegó, -1 mal */
static int _respuesta(SDMMC_TypeDef *s) {
  const uint32_t sta = s->STA;
  if ((sta & SDMMC_FLAG_CMDACT) != 0U) {
    return 0;
  }
  if ((sta & (SDMMC_FLAG_CTIMEOUT | SDMMC_FLAG_CCRCFAIL)) != 0U) {
    __SDMMC_CLEAR_FLAG(s, SDMMC_STATIC_CMD_FLAGS);
    return -1;
  }
  if ((sta & SDMMC_FLAG_CMDREND) != 0U) {
    __SDMMC_CLEAR_FLAG(s, SDMMC_STATIC_CMD_FLAGS);
    return 1;
  }
  return 0;
}

static void _cmd13(sd_card_t *c) {
  _cmd(c->instance, SDMMC_CMD_SEND_STATUS,
       (uint32_t)c->hsd->SdCard.RelCardAdd << 16U);
  c->async_t_cmd = HAL_GetTick();
}

/* Deja el camino de datos parado y limpio, como al acabar el HAL */
static void _datos_parar(SDMMC_TypeDef *s) {
  s->IDMACTRL = SDMMC_DISABLE_IDMA;
  __SDMMC_CMDTRANS_DISABLE(s);
  __SDMMC_CMDSTOP_DISABLE(s);
  s->DLEN = 0U;
  s->DCTRL = 0U;
  __SDMMC_CLEAR_FLAG(s, SDMMC_STATIC_FLAGS);
}

static sd_card_async_t _async_fallo(sd_card_t *c, uint32_t error) {
  _datos_parar(c->instance);
  c->last_error = error;
  c->ready = false; /* perdida: no se vuelve a tocar */
  c->async = SD_CARD_ASYNC_FALLO;
  c->async_fase = FASE_NADA;
  return c->async;
}

/* Avanza la escritura sin bloquear: solo lee y escribe registros */
static sd_card_async_t _async_poll(sd_card_t *c) {
  if (c->async != SD_CARD_ASYNC_OCUPADA) {
    return c->async;
  }
  const uint32_t t0 = DWT->CYCCNT;
  SDMMC_TypeDef *s = c->instance;
  const uint32_t ahora = HAL_GetTick();
  int r;

  switch (c->async_fase) {
  case FASE_CMD_ESCRIBIR:
    r = _respuesta(s);
    if (r == 0) {
      if ((ahora - c->async_t_cmd) >= SD_CARD_ASYNC_CMD_MS) {
        return _async_fallo(c, HAL_SD_ERROR_CMD_RSP_TIMEOUT);
      }
      break;
    }
    if ((r < 0) ||
        ((SDMMC_GetResponse(s, SDMMC_RESP1) & SDMMC_OCR_ERRORBITS) != 0U)) {
      return _async_fallo(c, HAL_SD_ERROR_CMD_RSP_TIMEOUT);
    }
    c->async_fase = FASE_DATOS;
    c->async_t0 = ahora;
    /* fall through */
  case FASE_DATOS: {
    const uint32_t sta = s->STA;
    if ((sta & (SDMMC_FLAG_DCRCFAIL | SDMMC_FLAG_DTIMEOUT |
                SDMMC_FLAG_TXUNDERR)) != 0U) {
      return _async_fallo(c, HAL_SD_ERROR_DATA_CRC_FAIL);
    }
    if ((sta & SDMMC_FLAG_DATAEND) == 0U) {
      if ((ahora - c->async_t0) >= SD_CARD_ASYNC_DMA_MS) {
        return _async_fallo(c, HAL_SD_ERROR_TIMEOUT);
      }
      break;
    }
    _datos_parar(s);
    if (c->async_multi) {
      __SDMMC_CMDSTOP_ENABLE(s);
      _cmd(s, SDMMC_CMD_STOP_TRANSMISSION, 0U);
      c->async_t_cmd = ahora;
      c->async_fase = FASE_CMD12;
    } else {
      _cmd13(c);
      c->async_fase = FASE_CMD13;
    }
    c->async_t0 = ahora;
    break;
  }
  case FASE_CMD12:
    r = _respuesta(s);
    if (r == 0) {
      if ((ahora - c->async_t_cmd) >= SD_CARD_ASYNC_CMD_MS) {
        return _async_fallo(c, HAL_SD_ERROR_CMD_RSP_TIMEOUT);
      }
      break;
    }
    __SDMMC_CMDSTOP_DISABLE(s);
    if (r < 0) {
      return _async_fallo(c, HAL_SD_ERROR_CMD_RSP_TIMEOUT);
    }
    _cmd13(c);
    c->async_fase = FASE_CMD13;
    break;
  case FASE_CMD13:
    /* La tarjeta programa lo recibido: se le pregunta hasta que acaba */
    r = _respuesta(s);
    if (r == 0) {
      if ((ahora - c->async_t_cmd) >= SD_CARD_ASYNC_CMD_MS) {
        return _async_fallo(c, HAL_SD_ERROR_CMD_RSP_TIMEOUT);
      }
      break;
    }
    if (r < 0) {
      return _async_fallo(c, HAL_SD_ERROR_CMD_RSP_TIMEOUT);
    }
    if (SD_CARD_R1_ESTADO(SDMMC_GetResponse(s, SDMMC_RESP1)) ==
        SD_CARD_R1_TRANSFER) {
      c->async = SD_CARD_ASYNC_HECHA;
      c->async_fase = FASE_NADA;
      break;
    }
    if ((ahora - c->async_t0) >= SD_CARD_ASYNC_PROG_MS) {
      return _async_fallo(c, HAL_SD_ERROR_TIMEOUT);
    }
    _cmd13(c);
    break;
  default:
    return _async_fallo(c, HAL_SD_ERROR_GENERAL_UNKNOWN_ERR);
  }

  const uint32_t us = _us_desde(t0);
  if (us > c->tiempos.estado_max_us) {
    c->tiempos.estado_max_us = us;
  }
  return c->async;
}

/* Las operaciones bloqueantes esperan a la escritura sin bloquear que haya en
 * marcha: como mucho SD_CARD_ASYNC_DMA_MS + SD_CARD_ASYNC_PROG_MS */
static bool _async_esperar(sd_card_t *c) {
  while (_async_poll(c) == SD_CARD_ASYNC_OCUPADA) {
  }
  return c->ready;
}

static bool _read(sd_card_t *c, uint8_t *buf, uint32_t block, uint32_t count) {
  const int32_t bytes = (int32_t)(count * SD_CARD_BLOCK_SIZE);
  SCB_CleanInvalidateDCache_by_Addr((void *)buf, bytes);
  c->rx_done = false;
  c->failed = false;
  if (HAL_SD_ReadBlocks_DMA(c->hsd, buf, block, count) != HAL_OK) {
    c->last_error = c->hsd->ErrorCode;
    return false;
  }
  const bool ok = _wait_done(c, &c->rx_done);
  SCB_InvalidateDCache_by_Addr((void *)buf, bytes);
  return ok;
}

static bool _write(sd_card_t *c, const uint8_t *buf, uint32_t block,
                   uint32_t count) {
  SCB_CleanDCache_by_Addr((void *)buf, (int32_t)(count * SD_CARD_BLOCK_SIZE));
  c->tx_done = false;
  c->failed = false;
  if (HAL_SD_WriteBlocks_DMA(c->hsd, buf, block, count) != HAL_OK) {
    c->last_error = c->hsd->ErrorCode;
    return false;
  }
  return _wait_done(c, &c->tx_done);
}

/* ¿Se puede pasar tal cual al IDMA? Alineado a una línea de caché (ver
 * s_scratch) y fuera de la DTCM, que el IDMA no alcanza. */
static bool _aligned(const uint8_t *buf) {
  const uintptr_t a = (uintptr_t)buf;
  const bool en_dtcm = (a >= 0x20000000U) && (a < 0x20020000U);
  return ((a & (SD_CARD_CACHE_LINE - 1U)) == 0U) && !en_dtcm;
}

/* ===========================================================================
 * API
 * ======================================================================== */

bool sd_card_init(sd_card_id_t id) {
  sd_card_t *c = _card(id);
  if (c == NULL) {
    return false;
  }
  if (c->ready) {
    return true;
  }

  /* Tras un intento fallido el handle queda en ERROR: volver a RESET para que
   * HAL_SD_Init() rehaga todo (reloj, pines) */
  if (c->hsd->State != HAL_SD_STATE_RESET) {
    (void)HAL_SD_DeInit(c->hsd);
  }

  c->hsd->Instance = c->instance;
  c->hsd->Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  c->hsd->Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  c->hsd->Init.BusWide = SDMMC_BUS_WIDE_4B;
  c->hsd->Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  c->hsd->Init.ClockDiv = SD_CARD_CLOCK_DIV;

  if (HAL_SD_Init(c->hsd) != HAL_OK) {
    c->last_error = c->hsd->ErrorCode;
    return false;
  }

  c->last_error = 0U;
  c->ready = true;
  (void)sd_card_set_clock_div(id, s_div_datos[id]);
  return true;
}

void sd_card_deinit(sd_card_id_t id) {
  sd_card_t *c = _card(id);
  if (c == NULL) {
    return;
  }
  (void)HAL_SD_DeInit(c->hsd);
  c->ready = false;
}

bool sd_card_is_ready(sd_card_id_t id) {
  const sd_card_t *c = _card(id);
  return (c != NULL) && c->ready;
}

bool sd_card_cd_pin_high(sd_card_id_t id) {
  const sd_card_t *c = _card(id);
  return (c != NULL) && (HAL_GPIO_ReadPin(c->cd_port, c->cd_pin) == GPIO_PIN_SET);
}

uint32_t sd_card_block_count(sd_card_id_t id) {
  const sd_card_t *c = _card(id);
  if ((c == NULL) || !c->ready) {
    return 0U;
  }
  HAL_SD_CardInfoTypeDef info;
  if (HAL_SD_GetCardInfo(c->hsd, &info) != HAL_OK) {
    return 0U;
  }
  /* LogBlockSize es 512 en SDSC y SDHC/SDXC */
  return info.LogBlockNbr;
}

bool sd_card_set_clock_div(sd_card_id_t id, uint32_t div) {
  sd_card_t *c = _card(id);
  if ((c == NULL) || !c->ready || (div > (SDMMC_CLKCR_CLKDIV_Msk >>
                                          SDMMC_CLKCR_CLKDIV_Pos))) {
    return false;
  }
  MODIFY_REG(c->instance->CLKCR, SDMMC_CLKCR_CLKDIV,
             div << SDMMC_CLKCR_CLKDIV_Pos);
  c->hsd->Init.ClockDiv = div;
  return true;
}

uint32_t sd_card_get_clock_div(sd_card_id_t id) {
  const sd_card_t *c = _card(id);
  if (c == NULL) {
    return 0U;
  }
  return (c->instance->CLKCR & SDMMC_CLKCR_CLKDIV_Msk) >>
         SDMMC_CLKCR_CLKDIV_Pos;
}

uint32_t sd_card_last_error(sd_card_id_t id) {
  const sd_card_t *c = _card(id);
  return (c != NULL) ? c->last_error : 0U;
}

bool sd_card_read_blocks(sd_card_id_t id, uint8_t *buf, uint32_t block,
                         uint32_t count) {
  sd_card_t *c = _card(id);
  if ((c == NULL) || !c->ready || (buf == NULL) || (count == 0U) ||
      !_async_esperar(c)) {
    return false;
  }

  if (_aligned(buf)) {
    return _read(c, buf, block, count);
  }

  for (uint32_t i = 0U; i < count; i++) {
    if (!_read(c, s_scratch, block + i, 1U)) {
      return false;
    }
    memcpy(&buf[i * SD_CARD_BLOCK_SIZE], s_scratch, SD_CARD_BLOCK_SIZE);
  }
  return true;
}

bool sd_card_write_blocks(sd_card_id_t id, const uint8_t *buf, uint32_t block,
                          uint32_t count) {
  sd_card_t *c = _card(id);
  if ((c == NULL) || !c->ready || (buf == NULL) || (count == 0U) ||
      !_async_esperar(c)) {
    return false;
  }

  if (_aligned(buf)) {
    return _write(c, buf, block, count);
  }

  for (uint32_t i = 0U; i < count; i++) {
    memcpy(s_scratch, &buf[i * SD_CARD_BLOCK_SIZE], SD_CARD_BLOCK_SIZE);
    if (!_write(c, s_scratch, block + i, 1U)) {
      return false;
    }
  }
  return true;
}

bool sd_card_write_start(sd_card_id_t id, const uint8_t *buf, uint32_t block,
                         uint32_t count) {
  sd_card_t *c = _card(id);
  if ((c == NULL) || !c->ready || (buf == NULL) || (count == 0U) ||
      (c->async == SD_CARD_ASYNC_OCUPADA) || !_aligned(buf) ||
      (c->hsd->State != HAL_SD_STATE_READY)) {
    return false;
  }
  const uint32_t t0 = DWT->CYCCNT;
  SCB_CleanDCache_by_Addr((void *)buf, (int32_t)(count * SD_CARD_BLOCK_SIZE));

  /* Lo mismo que HAL_SD_WriteBlocks_DMA, salvo esperar la respuesta */
  SDMMC_TypeDef *s = c->instance;
  s->DCTRL = 0U;
  __SDMMC_CLEAR_FLAG(s, SDMMC_STATIC_FLAGS);
  SDMMC_DataInitTypeDef d;
  d.DataTimeOut = SDMMC_DATATIMEOUT;
  d.DataLength = count * SD_CARD_BLOCK_SIZE;
  d.DataBlockSize = SDMMC_DATABLOCK_SIZE_512B;
  d.TransferDir = SDMMC_TRANSFER_DIR_TO_CARD;
  d.TransferMode = SDMMC_TRANSFER_MODE_BLOCK;
  d.DPSM = SDMMC_DPSM_DISABLE;
  (void)SDMMC_ConfigData(s, &d);
  __SDMMC_CMDTRANS_ENABLE(s);
  s->IDMABASE0 = (uint32_t)buf;
  s->IDMACTRL = SDMMC_ENABLE_IDMA_SINGLE_BUFF;

  uint32_t dir = block;
  if (c->hsd->SdCard.CardType != CARD_SDHC_SDXC) {
    dir *= SD_CARD_BLOCK_SIZE; /* SDSC: dirección en bytes */
  }
  c->async_multi = (count > 1U);
  _cmd(s, c->async_multi ? SDMMC_CMD_WRITE_MULT_BLOCK
                         : SDMMC_CMD_WRITE_SINGLE_BLOCK,
       dir);
  c->async = SD_CARD_ASYNC_OCUPADA;
  c->async_fase = FASE_CMD_ESCRIBIR;
  c->async_t0 = HAL_GetTick();
  c->async_t_cmd = c->async_t0;

  const uint32_t us = _us_desde(t0);
  if (us > c->tiempos.inicio_max_us) {
    c->tiempos.inicio_max_us = us;
  }
  return true;
}

sd_card_tiempos_t sd_card_tiempos(sd_card_id_t id) {
  const sd_card_t *c = _card(id);
  const sd_card_tiempos_t cero = {0};
  return (c != NULL) ? c->tiempos : cero;
}

void sd_card_irq_medir(sd_card_id_t id, uint32_t us) {
  sd_card_t *c = _card(id);
  if ((c != NULL) && (us > c->tiempos.irq_max_us)) {
    c->tiempos.irq_max_us = us;
  }
}

sd_card_async_t sd_card_write_poll(sd_card_id_t id) {
  sd_card_t *c = _card(id);
  return (c != NULL) ? _async_poll(c) : SD_CARD_ASYNC_FALLO;
}

/* ===========================================================================
 * Callbacks del HAL (desde SDMMCx_IRQHandler)
 * ======================================================================== */

void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsd) {
  sd_card_t *c = _card_from_handle(hsd);
  if (c != NULL) {
    c->rx_done = true;
  }
}

void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsd) {
  sd_card_t *c = _card_from_handle(hsd);
  if (c != NULL) {
    c->tx_done = true;
  }
}

void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd) {
  sd_card_t *c = _card_from_handle(hsd);
  if (c != NULL) {
    c->failed = true;
  }
}
