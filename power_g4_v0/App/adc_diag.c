#include "adc_diag.h"

#include "analog.h"
#include "main.h"

/* Valor de fabrica de VREFINT, medido con VREF+ = 3,0 V a 30 grados
 * (STM32G4, manual de referencia, "Embedded internal voltage reference") */
#define VREFINT_CAL_ADDR ((const uint16_t *)0x1FFF75AAU)
#define VREFINT_CAL_MV   3000U

#define ADC_DIAG_PERIODO_MS 500U
#define ADC_DIAG_MUESTRAS   16U

volatile adc_diag_t g_adc_diag;

static const Analog_Channel_t canales[ADC_DIAG_CANALES] = {
    ANALOG_V_3V3, ANALOG_V_5V, ANALOG_V_11V, ANALOG_V_PYRO1, ANALOG_V_PYRO2,
};

void AdcDiag_Update(void)
{
#if ADC_DIAG_ENABLE
    static uint32_t ultimo;
    const uint32_t ahora = HAL_GetTick();
    if ((ahora - ultimo) < ADC_DIAG_PERIODO_MS)
    {
        return;
    }
    ultimo = ahora;

    g_adc_diag.magic = ADC_DIAG_MAGIC;
    g_adc_diag.vrefint_cal = *VREFINT_CAL_ADDR;

    uint16_t m, lo, hi;
    if (Analog_ReadVrefintRaw(ADC_DIAG_MUESTRAS, &m, &lo, &hi))
    {
        g_adc_diag.vrefint[0] = m;
        g_adc_diag.vrefint[1] = lo;
        g_adc_diag.vrefint[2] = hi;
        /* La referencia real del ADC: si VREFINT se lee alto, la referencia
         * esta baja, y al reves */
        g_adc_diag.vref_mv = (m > 0U)
            ? (uint16_t)(((uint32_t)VREFINT_CAL_MV * g_adc_diag.vrefint_cal) / m)
            : 0U;
    }
    else
    {
        g_adc_diag.errores++;
    }

    for (uint32_t i = 0U; i < ADC_DIAG_CANALES; i++)
    {
        if (Analog_ReadRaw(canales[i], ADC_DIAG_MUESTRAS, &m, &lo, &hi))
        {
            g_adc_diag.raw[i][0] = m;
            g_adc_diag.raw[i][1] = lo;
            g_adc_diag.raw[i][2] = hi;
        }
        else
        {
            g_adc_diag.errores++;
        }
    }
    if (Analog_ReadVbatRaw(ADC_DIAG_MUESTRAS, &m, &lo, &hi))
    {
        g_adc_diag.vbat[0] = m;
        g_adc_diag.vbat[1] = lo;
        g_adc_diag.vbat[2] = hi;
    }
    const uint32_t malas = Analog_BadReferenceCount();
    g_adc_diag.ref_malas = (malas > 0xFFFFU) ? 0xFFFFU : (uint16_t)malas;
    for (uint32_t i = 0U; i < ADC_DIAG_CANALES; i++)
    {
        uint16_t mv = 0U;
        g_adc_diag.pin_mv[i] = Analog_ReadPinMillivolts(canales[i], &mv)
                                   ? mv : 0xFFFFU;
    }
    g_adc_diag.vueltas++;
#endif
}
