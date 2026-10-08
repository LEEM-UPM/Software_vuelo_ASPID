#include "runcam.h"

#include <string.h>

#define RUNCAM_CABECERA 0xCCU
#define RUNCAM_CMD_CONTROL 0x01U
#define RUNCAM_INFO_BYTES 5U
#define NADA 0xFFU

static uint8_t _crc(uint8_t crc, uint8_t b) {
  crc ^= b;
  for (uint32_t i = 0U; i < 8U; i++) {
    crc = (crc & 0x80U) ? (uint8_t)((crc << 1) ^ 0xD5U) : (uint8_t)(crc << 1);
  }
  return crc;
}

static uint8_t _crc_de(const uint8_t *p, uint32_t n) {
  uint8_t crc = 0U;
  for (uint32_t i = 0U; i < n; i++) {
    crc = _crc(crc, p[i]);
  }
  return crc;
}

/* El paquete entero al anillo de salida, o nada si no cabe */
static bool _enviar(runcam_t *c, const uint8_t *p, uint32_t n) {
  if (!c->iniciado) {
    return false;
  }
  if (n > (RUNCAM_TX - (c->tx_cab - c->tx_cola))) {
    c->no_cabia++;
    return false;
  }
  for (uint32_t i = 0U; i < n; i++) {
    c->tx[c->tx_cab % RUNCAM_TX] = p[i];
    c->tx_cab++;
  }
  /* La interrupción lo pasa a la FIFO. ATOMIC: la interrupción también toca
   * CR1 (lo apaga al vaciar el anillo) */
  ATOMIC_SET_BIT(c->huart->Instance->CR1, USART_CR1_TXEIE_TXFNFIE);
  c->enviados++;
  return true;
}

static bool _paquete(runcam_t *c, uint8_t cmd, const uint8_t *param,
                     uint32_t n, uint8_t espera) {
  uint8_t p[2U + 1U + 32U];
  uint32_t k = 0U;
  p[k++] = RUNCAM_CABECERA;
  p[k++] = cmd;
  for (uint32_t i = 0U; (i < n) && (k < (sizeof(p) - 1U)); i++) {
    p[k++] = param[i];
  }
  p[k] = _crc_de(p, k);
  k++;
  /* Lo que llegue a partir de ahora es la respuesta a esto */
  c->esperando = espera;
  c->pedido_ms = HAL_GetTick();
  c->resp_n = 0U;
  c->resp_lista = false;
  return _enviar(c, p, k);
}

bool runcam_init(runcam_t *c, UART_HandleTypeDef *huart, IRQn_Type irq) {
  memset((void *)c, 0, sizeof(*c));
  c->huart = huart;
  c->esperando = NADA;
  if (HAL_UARTEx_EnableFifoMode(huart) != HAL_OK) {
    return false;
  }
  /* Recepción por interrupción: cada byte (o error) la dispara */
  SET_BIT(huart->Instance->CR1, USART_CR1_RXNEIE_RXFNEIE);
  SET_BIT(huart->Instance->CR3, USART_CR3_EIE);
  HAL_NVIC_SetPriority(irq, 8U, 0U);
  HAL_NVIC_EnableIRQ(irq);
  c->iniciado = true;
  return true;
}

/* Desde el IRQHandler del UART: solo copiar */
void runcam_irq(runcam_t *c) {
  USART_TypeDef *u = c->huart->Instance;
  const uint32_t errores =
      USART_ISR_ORE | USART_ISR_NE | USART_ISR_FE | USART_ISR_PE;
  if ((u->ISR & errores) != 0U) {
    c->errores_uart++;
    u->ICR = USART_ICR_ORECF | USART_ICR_NECF | USART_ICR_FECF |
             USART_ICR_PECF;
  }
  while ((u->ISR & USART_ISR_RXNE_RXFNE) != 0U) {
    const uint8_t b = (uint8_t)u->RDR;
    if ((c->cab - c->cola) < RUNCAM_ANILLO) {
      c->anillo[c->cab % RUNCAM_ANILLO] = b;
      c->cab++;
    }
  }
  /* Salida: rellenar la FIFO mientras quede algo y haya hueco */
  if ((u->CR1 & USART_CR1_TXEIE_TXFNFIE) != 0U) {
    while ((c->tx_cola != c->tx_cab) &&
           ((u->ISR & USART_ISR_TXE_TXFNF) != 0U)) {
      u->TDR = c->tx[c->tx_cola % RUNCAM_TX];
      c->tx_cola++;
    }
    if (c->tx_cola == c->tx_cab) {
      CLEAR_BIT(u->CR1, USART_CR1_TXEIE_TXFNFIE);
    }
  }
}

void runcam_olvidar(runcam_t *c) {
  c->esperando = NADA;
  c->resp_n = 0U;
  c->resp_lista = false;
  c->info_ok = false;
}

bool runcam_pedir_info(runcam_t *c) {
  return _paquete(c, RUNCAM_CMD_INFO, NULL, 0U, RUNCAM_CMD_INFO);
}

bool runcam_control(runcam_t *c, uint8_t operacion) {
  return _paquete(c, RUNCAM_CMD_CONTROL, &operacion, 1U, NADA);
}

bool runcam_pedir_ajustes(runcam_t *c, uint8_t carpeta, uint8_t trozo) {
  const uint8_t p[2] = {carpeta, trozo};
  return _paquete(c, RUNCAM_CMD_AJUSTES, p, 2U, RUNCAM_CMD_AJUSTES);
}

bool runcam_pedir_detalle(runcam_t *c, uint8_t ajuste, uint8_t trozo) {
  const uint8_t p[2] = {ajuste, trozo};
  return _paquete(c, RUNCAM_CMD_DETALLE, p, 2U, RUNCAM_CMD_DETALLE);
}

bool runcam_escribir(runcam_t *c, uint8_t ajuste, const uint8_t *valor,
                     uint32_t n) {
  uint8_t p[1U + 31U];
  if (n > (sizeof(p) - 1U)) {
    return false;
  }
  p[0] = ajuste;
  memcpy(&p[1], valor, n);
  return _paquete(c, RUNCAM_CMD_ESCRIBIR, p, n + 1U, RUNCAM_CMD_ESCRIBIR);
}

bool runcam_ocupada(const runcam_t *c) {
  return c->resp_lista ||
         ((c->esperando != NADA) &&
          ((HAL_GetTick() - c->pedido_ms) < RUNCAM_PLAZO_MS));
}

const uint8_t *runcam_respuesta(runcam_t *c, uint32_t *n) {
  if (!c->resp_lista) {
    return NULL;
  }
  c->resp_lista = false; /* recogida: resp no cambia hasta otra petición */
  *n = c->resp_n;
  return c->resp;
}

/* ¿Cuánto mide entera la respuesta que se está formando? 0 = aún no se sabe */
static uint32_t _largo(const runcam_t *c) {
  if (c->esperando == RUNCAM_CMD_INFO) {
    return RUNCAM_INFO_BYTES;
  }
  /* ESCRIBIR: primero la forma corta, CC | resultado | releer | CRC */
  if ((c->esperando == RUNCAM_CMD_ESCRIBIR) && (c->resp_n == 4U) &&
      (_crc_de(c->resp, 3U) == c->resp[3])) {
    return 4U;
  }
  /* CC | faltan | largo | datos[largo] | CRC */
  return (c->resp_n >= 3U) ? (4U + c->resp[2]) : 0U;
}

static void _byte(runcam_t *c, uint8_t b) {
  c->bytes_rx++;
  if (c->n_ultimos < sizeof(c->ultimos)) {
    c->ultimos[c->n_ultimos++] = b;
  } else {
    memmove(c->ultimos, &c->ultimos[1], sizeof(c->ultimos) - 1U);
    c->ultimos[sizeof(c->ultimos) - 1U] = b;
  }

  if ((c->esperando == NADA) || c->resp_lista) {
    c->ignorados++;
    return;
  }
  if ((c->resp_n == 0U) && (b != RUNCAM_CABECERA)) {
    c->ignorados++; /* basura hasta la cabecera */
    return;
  }
  if (c->resp_n >= RUNCAM_RESP_MAX) {
    c->resp_n = 0U; /* demasiado larga: no es lo que se espera */
    c->errores_crc++;
    return;
  }
  c->resp[c->resp_n++] = b;

  const uint32_t total = _largo(c);
  if ((total == 0U) || (c->resp_n < total)) {
    return;
  }
  if (_crc_de(c->resp, total - 1U) != c->resp[total - 1U]) {
    c->errores_crc++;
    c->resp_n = 0U;
    return;
  }
  c->respuestas++;
  if (c->esperando == RUNCAM_CMD_INFO) {
    c->info_ok = true;
    c->version = c->resp[1];
    c->funciones = (uint16_t)(c->resp[2] | ((uint16_t)c->resp[3] << 8));
    c->info_ms = HAL_GetTick();
    c->esperando = NADA;
    c->resp_n = 0U;
    return;
  }
  c->resp_lista = true; /* la recoge quien la pidió */
  c->esperando = NADA;
}

void runcam_update(runcam_t *c) {
  if (!c->iniciado) {
    return;
  }
  while (c->cola != c->cab) {
    _byte(c, c->anillo[c->cola % RUNCAM_ANILLO]);
    c->cola++;
  }
}
