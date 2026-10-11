#include "boton_memoria.h"

#include <stdint.h>

#include "buzzer.h"
#include "debug_out.h"
#include "main.h"
#include "memoria_vuelo.h"
#include "watchdog.h"

static bool s_pulsado;
static uint32_t s_t_pulsado;
static bool s_avisado; /* "no se puede" dicho una vez por pulsación */

void boton_memoria_update(bool permitido) {
  const uint32_t ahora = HAL_GetTick();
  if (HAL_GPIO_ReadPin(BUTTON1_GPIO_Port, BUTTON1_Pin) != GPIO_PIN_SET) {
    s_pulsado = false;
    s_avisado = false;
    return;
  }
  if (!s_pulsado) {
    s_pulsado = true;
    s_t_pulsado = ahora;
  }
  if ((ahora - s_t_pulsado) < BOTON_MEMORIA_MS) {
    return;
  }
  if (!permitido) {
    if (!s_avisado) {
      s_avisado = true;
      DebugOut_Printf("BOTON: con un vuelo en marcha NO se borra la memoria "
                      "de vuelo\r\n");
    }
    return;
  }
  if (!memoria_vuelo_disponible()) {
    if (!s_avisado) {
      s_avisado = true;
      DebugOut_Printf("BOTON: memoria de vuelo NO disponible, nada que "
                      "borrar\r\n");
    }
    return;
  }

  DebugOut_Printf("BOTON: borrando la memoria de vuelo y reiniciando...\r\n");
  buzzer_beep(2000U, 150U); /* también deja salir el texto por el USB */

  /* El borrado para el chip 1-2 s: más que el plazo del watchdog */
  watchdog_alargar();
  const bool ok = memoria_vuelo_borrar();

  DebugOut_Printf("BOTON: %s (%lu ms)\r\n",
                  ok ? "memoria de vuelo borrada" : "FALLO AL BORRAR",
                  (unsigned long)memoria_vuelo_borrado_ms());
  buzzer_beep(ok ? 3000U : 400U, ok ? 150U : 600U);
  HAL_Delay(100U);
  NVIC_SystemReset();
}
