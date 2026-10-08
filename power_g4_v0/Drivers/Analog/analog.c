#include "analog.h"

#include "adc.h"
#include "main.h"
#include "opamp.h"

#define ANALOG_ADC_MAX_COUNT  4095U

/*
 * REFERENCIA DEL ADC
 *
 * VREF+ va unido a VDDA (3,3 V) en la placa. Hasta el 2026-10-01 NO lo
 * estaba: el pin flotaba y todos los canales de todos los ADC leian 4095,
 * con lo que el core recibia 3V3 3630, 5V 5500 y 11V 17050 mV clavados (que
 * eran 3300 mV saturados por el divisor de cada rail). Se arreglo en la placa.
 *
 * Aun asi, las lecturas NO suponen que la referencia valga 3300 mV: cada una
 * se convierte contra VREFINT medida en ese momento, con el valor de fabrica
 * grabado en el chip. Asi se corrige la tolerancia del regulador (medido:
 * 3309 mV) y, sobre todo, si la referencia vuelve a fallar las medidas salen
 * como NO VALIDAS en vez de como un numero creible. Fue un numero creible
 * (17 V) lo que tuvo el fallo escondido.
 *
 * El VREFBUF interno se deja APAGADO y en alta impedancia de forma explicita
 * en Analog_Init(). Con VREF+ unido a VDDA, si se encendiera, VREFBUF y VDDA
 * se pelearian por el pin.
 */
#define VREFINT_CAL_ADDR      ((const uint16_t *)0x1FFF75AAU)
#define VREFINT_CAL_VREF_MV   3000U

/* VREF+ = VDDA tiene que estar aqui; fuera, la referencia no es de fiar */
#define VREF_MIN_MV           3100U
#define VREF_MAX_MV           3500U

/* Una media por encima de esto es SATURACION: la tension del pin es mayor
 * que la referencia y el numero no significa nada */
#define ANALOG_SATURADO       4090U
#define ANALOG_SAMPLES        8U
#define ANALOG_TIMEOUT_MS     10U

/* Tiempo de muestreo largo: las salidas de los divisores y de los OPAMP
 * internos necesitan tiempo para cargar el condensador de muestreo. */
#define ANALOG_SAMPLING_TIME  ADC_SAMPLETIME_640CYCLES_5

/* ADC5 no está en CubeMX: se gestiona aquí (ver analog.h) */
static ADC_HandleTypeDef hadc5;

typedef struct {
  ADC_HandleTypeDef *hadc;
  uint32_t channel;
} Analog_Source_t;

static const Analog_Source_t sources[ANALOG_COUNT] = {
    [ANALOG_V_3V3] = {&hadc2, ADC_CHANNEL_15},
    [ANALOG_V_5V] = {&hadc1, ADC_CHANNEL_1},
    [ANALOG_V_11V] = {&hadc1, ADC_CHANNEL_3},
    [ANALOG_V_PYRO1] = {&hadc2, ADC_CHANNEL_13},
    [ANALOG_V_PYRO2] = {&hadc1, ADC_CHANNEL_11},
    [ANALOG_I_3V3] = {&hadc4, ADC_CHANNEL_4},
    [ANALOG_I_5V] = {&hadc1, ADC_CHANNEL_4},
    [ANALOG_I_11V] = {&hadc2, ADC_CHANNEL_VOPAMP3_ADC2},
    [ANALOG_I_PYRO1] = {&hadc2, ADC_CHANNEL_VOPAMP2},
    [ANALOG_I_PYRO2] = {&hadc5, ADC_CHANNEL_VOPAMP4},
};

/* VREFINT y VBAT van por ADC1; ConfigChannel activa VREFEN / VBATSEL */
static const Analog_Source_t s_vrefint = {&hadc1, ADC_CHANNEL_VREFINT};
static const Analog_Source_t s_vbat = {&hadc1, ADC_CHANNEL_VBAT};

static uint16_t s_vref_mv;     /* ultima referencia medida */
static uint32_t s_ref_malas;   /* lecturas descartadas por referencia mala */

static void ADC5_Init(void)
{
    /* Reloj ADC345: la fuente (SYSCLK) ya la fija HAL_ADC_MspInit() de ADC4.
     * HAL_ADC_MspInit() no tiene rama para ADC5, así que el reloj se activa
     * aquí antes de HAL_ADC_Init(). */
    __HAL_RCC_ADC345_CLK_ENABLE();

    /* Misma configuración que ADC1/2/4 en adc.c */
    hadc5.Instance = ADC5;
    hadc5.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
    hadc5.Init.Resolution = ADC_RESOLUTION_12B;
    hadc5.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc5.Init.GainCompensation = 0;
    hadc5.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc5.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc5.Init.LowPowerAutoWait = DISABLE;
    hadc5.Init.ContinuousConvMode = DISABLE;
    hadc5.Init.NbrOfConversion = 1;
    hadc5.Init.DiscontinuousConvMode = DISABLE;
    hadc5.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc5.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc5.Init.DMAContinuousRequests = DISABLE;
    hadc5.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    hadc5.Init.OversamplingMode = DISABLE;
    if (HAL_ADC_Init(&hadc5) != HAL_OK)
    {
        Error_Handler();
    }
}

/* Una conversión: selecciona el canal, convierte y devuelve la cuenta */
static bool ConvertOnce(const Analog_Source_t *src, uint32_t *raw)
{
    if (HAL_ADC_Start(src->hadc) != HAL_OK)
    {
        return false;
    }

    bool ok = (HAL_ADC_PollForConversion(src->hadc, ANALOG_TIMEOUT_MS) == HAL_OK);
    if (ok)
    {
        *raw = HAL_ADC_GetValue(src->hadc);
    }

    (void)HAL_ADC_Stop(src->hadc);
    return ok;
}

/*
 * Configura el canal, descarta la primera conversión (el condensador de
 * muestreo aún guarda la carga del canal anterior) y hace `samples` más:
 * media, mínimo y máximo.
 */
static bool SampleRaw(const Analog_Source_t *src, uint8_t samples,
                      uint16_t *mean, uint16_t *min, uint16_t *max)
{
    ADC_ChannelConfTypeDef config = {0};
    config.Channel = src->channel;
    config.Rank = ADC_REGULAR_RANK_1;
    config.SamplingTime = ANALOG_SAMPLING_TIME;
    config.SingleDiff = ADC_SINGLE_ENDED;
    config.OffsetNumber = ADC_OFFSET_NONE;
    config.Offset = 0;
    if ((samples == 0U) || (HAL_ADC_ConfigChannel(src->hadc, &config) != HAL_OK))
    {
        return false;
    }

    uint32_t raw = 0U;
    if (!ConvertOnce(src, &raw))
    {
        return false;
    }

    uint32_t sum = 0U, lo = 0xFFFFU, hi = 0U;
    for (uint8_t i = 0U; i < samples; i++)
    {
        if (!ConvertOnce(src, &raw))
        {
            return false;
        }
        sum += raw;
        lo = (raw < lo) ? raw : lo;
        hi = (raw > hi) ? raw : hi;
    }
    *mean = (uint16_t)(sum / samples);
    *min = (uint16_t)lo;
    *max = (uint16_t)hi;
    return true;
}

/*
 * Mide la referencia del ADC con VREFINT. false si no es de fiar: VREFINT
 * saturada (referencia por debajo de 1,21 V, el fallo que tuvo la placa) o
 * fuera de VREF_MIN_MV..VREF_MAX_MV.
 */
static bool MedirReferencia(uint16_t *vref_mv)
{
    uint16_t m = 0U, lo = 0U, hi = 0U;
    if (!SampleRaw(&s_vrefint, ANALOG_SAMPLES, &m, &lo, &hi) ||
        (m == 0U) || (m >= ANALOG_SATURADO))
    {
        return false;
    }
    const uint32_t mv = ((uint32_t)VREFINT_CAL_VREF_MV * (*VREFINT_CAL_ADDR)) / m;
    s_vref_mv = (uint16_t)mv;
    if ((mv < VREF_MIN_MV) || (mv > VREF_MAX_MV))
    {
        return false;
    }
    *vref_mv = (uint16_t)mv;
    return true;
}

void Analog_Init(void)
{
    /* +5V_VSensing (PA0) y +11V_VSensing (PA2) no están en el .ioc */
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_2;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* VREFBUF apagado y en alta impedancia, de forma explicita: VREF+ lo da
     * VDDA por la placa (ver la nota de arriba) */
    HAL_SYSCFG_DisableVREFBUF();
    HAL_SYSCFG_VREFBUF_HighImpedanceConfig(SYSCFG_VREFBUF_HIGH_IMPEDANCE_ENABLE);

    ADC5_Init();

    /* ADC1 y ADC4 los calibra CurrentSense_Init() */
    if ((HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) != HAL_OK) ||
        (HAL_ADCEx_Calibration_Start(&hadc5, ADC_SINGLE_ENDED) != HAL_OK))
    {
        Error_Handler();
    }

    /* MX_OPAMPx_Init() los configura pero no los arranca: sin esto las
     * medidas de corriente de 11 V, Pyro1 y Pyro2 no llegan al ADC. */
    if ((HAL_OPAMP_Start(&hopamp2) != HAL_OK) ||
        (HAL_OPAMP_Start(&hopamp3) != HAL_OK) ||
        (HAL_OPAMP_Start(&hopamp4) != HAL_OK))
    {
        Error_Handler();
    }
}

bool Analog_ReadPinMillivolts(Analog_Channel_t channel, uint16_t *millivolts)
{
    if ((channel >= ANALOG_COUNT) || (millivolts == NULL))
    {
        return false;
    }

    uint16_t m = 0U, lo = 0U, hi = 0U;
    if (!SampleRaw(&sources[channel], ANALOG_SAMPLES, &m, &lo, &hi))
    {
        return false;
    }
    if (m >= ANALOG_SATURADO)
    {
        /* Solo se sabe que el pin esta por encima de la referencia */
        return false;
    }

    uint16_t vref_mv = 0U;
    if (!MedirReferencia(&vref_mv))
    {
        s_ref_malas++;
        return false; /* sin referencia de fiar no hay medida de fiar */
    }

    *millivolts = (uint16_t)(((uint32_t)m * vref_mv) / ANALOG_ADC_MAX_COUNT);
    return true;
}

uint16_t Analog_ReferenceMillivolts(void) { return s_vref_mv; }

uint32_t Analog_BadReferenceCount(void) { return s_ref_malas; }

bool Analog_ReadRaw(Analog_Channel_t channel, uint8_t samples, uint16_t *mean,
                    uint16_t *min, uint16_t *max)
{
    if (channel >= ANALOG_COUNT)
    {
        return false;
    }
    return SampleRaw(&sources[channel], samples, mean, min, max);
}

bool Analog_ReadVrefintRaw(uint8_t samples, uint16_t *mean, uint16_t *min,
                           uint16_t *max)
{
    return SampleRaw(&s_vrefint, samples, mean, min, max);
}

bool Analog_ReadVbatRaw(uint8_t samples, uint16_t *mean, uint16_t *min,
                        uint16_t *max)
{
    return SampleRaw(&s_vbat, samples, mean, min, max);
}
