/*
 * Diagnostico del ADC de la placa de power, para leer por SWD (sin consola).
 *
 * Cada ADC_DIAG_PERIODO_MS deja en g_adc_diag las cuentas crudas de los
 * canales de tension y la lectura de VREFINT, que es lo que permite saber
 * si la referencia del ADC esta bien: VREFINT es ~1,21 V internos con su
 * valor de fabrica grabado en el chip, asi que de su lectura sale la
 * referencia real (vref_mv). Si un canal lee 4095 con una referencia sana,
 * el problema esta en esa entrada; si la referencia esta mal, lo estan
 * todos los canales a la vez.
 *
 * Se lee con: STM32_Programmer_CLI ... mode=HOTPLUG -r32 <&g_adc_diag> 48
 * (la direccion sale de arm-none-eabi-nm sobre el .elf).
 */

#ifndef ADC_DIAG_H
#define ADC_DIAG_H

#include <stdint.h>

#ifndef ADC_DIAG_ENABLE
#define ADC_DIAG_ENABLE 1
#endif

#define ADC_DIAG_MAGIC   0xADC0D1A6U
#define ADC_DIAG_CANALES 5U /* 3V3, 5V, 11V, PYRO1, PYRO2 */

typedef struct {
  uint32_t magic;
  uint32_t vueltas;
  uint32_t errores;
  uint16_t vrefint_cal;            /* de fabrica, con VREF+ = 3,0 V */
  uint16_t vref_mv;                /* referencia real calculada */
  uint16_t vrefint[3];             /* media, min, max */
  uint16_t raw[ADC_DIAG_CANALES][3]; /* media, min, max por canal */
  uint16_t vbat[3];                /* VBAT/3: media, min, max */
  uint16_t ref_malas;              /* lecturas descartadas por referencia mala
                                      (Analog_BadReferenceCount, saturado) */
  uint16_t reserva;
  uint16_t pin_mv[ADC_DIAG_CANALES]; /* lo que da Analog_ReadPinMillivolts;
                                        0xFFFF = no valido (saturado...) */
} adc_diag_t;

extern volatile adc_diag_t g_adc_diag;

/* Llamar en el bucle principal */
void AdcDiag_Update(void);

#endif /* ADC_DIAG_H */
