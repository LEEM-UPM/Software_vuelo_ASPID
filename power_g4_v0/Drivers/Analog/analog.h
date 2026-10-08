#ifndef ANALOG_H
#define ANALOG_H

/*
 * Lectura de las entradas analógicas de medida de la placa: tensión en el
 * PIN del micro, en milivoltios. Pasar de esa tensión a la magnitud física
 * (tensión del raíl, corriente) depende de los divisores, shunts y
 * amplificadores del esquemático, y NO se hace aquí.
 *
 * Mapa de señales (esquemático de power y Power_MK2.ioc):
 *
 *   Canal            Pin    Red del esquemático  Camino             ADC
 *   ANALOG_V_3V3     PB15   +3V3_VSensing        directo            ADC2_IN15
 *   ANALOG_V_5V      PA0    +5V_VSensing         directo            ADC1_IN1  (*)
 *   ANALOG_V_11V     PA2    +11V_VSensing        directo            ADC1_IN3  (*)
 *   ANALOG_V_PYRO1   PA5    VSensing_Pyro1       directo            ADC2_IN13
 *   ANALOG_V_PYRO2   PB12   VSensing_Pyro2       directo            ADC1_IN11
 *   ANALOG_I_3V3     PB14   +3V3_ASensing        INA138             ADC4_IN4
 *   ANALOG_I_5V      PA3    +5V_ASensing         INA138             ADC1_IN4
 *   ANALOG_I_11V     PA1    +11V_ASensing        OPAMP3 seguidor    ADC2 (VOPAMP3)
 *   ANALOG_I_PYRO1   PB0    ASensing_Pyro1       OPAMP2 seguidor    ADC2 (VOPAMP2)
 *   ANALOG_I_PYRO2   PB13   ASensing_Pyro2       OPAMP4 seguidor    ADC5 (VOPAMP4)
 *
 * Fuera de CubeMX (sobreviven a una regeneración, pero no hay que usar estos
 * recursos en CubeMX para otra cosa sin revisar este módulo):
 *  - (*) PA0 y PA2 no están en el .ioc: Analog_Init() los pone en analógico.
 *  - ADC5 no está activado en CubeMX: lo inicializa Analog_Init() (handle y
 *    reloj propios en analog.c).
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  ANALOG_V_3V3 = 0,
  ANALOG_V_5V,
  ANALOG_V_11V,
  ANALOG_V_PYRO1,
  ANALOG_V_PYRO2,
  ANALOG_I_3V3,
  ANALOG_I_5V,
  ANALOG_I_11V,
  ANALOG_I_PYRO1,
  ANALOG_I_PYRO2,
  ANALOG_COUNT
} Analog_Channel_t;

/* Pone PA0 y PA2 en analógico, calibra ADC2, inicializa y calibra ADC5 y
 * arranca los OPAMP 2, 3 y 4.
 * Llamar una vez después de CurrentSense_Init() (que calibra ADC1 y ADC4).
 * Llama a Error_Handler() si algo falla. */
void Analog_Init(void);

/* Lee `channel` (media de varias conversiones) y escribe en `*millivolts` la
 * tensión en el pin, convertida contra la referencia MEDIDA con VREFINT (no
 * supone ninguna). Bloqueante: unos pocos ms.
 * Devuelve false si el ADC falla, si el canal esta SATURADO (pin por encima
 * de la referencia) o si la propia referencia no es de fiar. */
bool Analog_ReadPinMillivolts(Analog_Channel_t channel, uint16_t *millivolts);

/* Referencia del ADC medida en la ultima lectura (ver analog.c), y cuantas
 * lecturas se han descartado por no tener una referencia de fiar. Si ese
 * contador sube, VREF+ vuelve a fallar en la placa. */
uint16_t Analog_ReferenceMillivolts(void);
uint32_t Analog_BadReferenceCount(void);

/* Diagnostico (App/adc_diag.c): cuentas crudas (0..4095) de `samples`
 * conversiones, media, minimo y maximo. */
bool Analog_ReadRaw(Analog_Channel_t channel, uint8_t samples, uint16_t *mean,
                    uint16_t *min, uint16_t *max);

/* Diagnostico: VREFINT (~1,21 V interna). Con su valor de fabrica dice cual es
 * de verdad la referencia del ADC: VREF+ = 3000 mV * VREFINT_CAL / lectura. */
bool Analog_ReadVrefintRaw(uint8_t samples, uint16_t *mean, uint16_t *min,
                           uint16_t *max);

/* Diagnostico: VBAT/3. En esta placa VBAT va unido al 3V3, asi que es una
 * segunda medida del 3V3, independiente de su divisor: sirve para comprobarlo
 * (medido: 3311 mV por VBAT, 3319 por el divisor). */
bool Analog_ReadVbatRaw(uint8_t samples, uint16_t *mean, uint16_t *min,
                        uint16_t *max);

#ifdef __cplusplus
}
#endif

#endif /* ANALOG_H */
