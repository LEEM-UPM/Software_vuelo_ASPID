/*
 * Parámetros comunes del bus CAN1 (ver ICD.md, sección 2).
 *
 * Aquí va lo que TODAS las placas deben cumplir. El prescaler y los segmentos
 * no están definidos aquí porque dependen del reloj FDCAN de cada micro: cada
 * placa elige los suyos y comprueba con CAN_BUS_TIMING_IS_VALID() que dan la
 * velocidad y el punto de muestreo del bus.
 *
 * Sin dependencias de ningún fabricante: los valores de este archivo son
 * genéricos y cada placa los traduce a las constantes de su HAL.
 */

#ifndef CAN_BUS_CONFIG_H
#define CAN_BUS_CONFIG_H

/* ------------------------------------------------------------------------
 * Temporización
 * ------------------------------------------------------------------------ */
#define CAN_BUS_BITRATE_BPS 1000000U /* velocidad nominal */

/* Punto de muestreo en milésimas: 875 = 87,5 % */
#define CAN_BUS_SAMPLE_POINT_PERMILLE 875U

/*
 * Desviación admitida del punto de muestreo, en milésimas (10 = ±1 %).
 * Con un número entero de tq por bit no siempre se puede dar el valor exacto
 * (p. ej. 25 tq dan un 88 %).
 */
#define CAN_BUS_SAMPLE_POINT_TOLERANCE_PERMILLE 10U

/* ------------------------------------------------------------------------
 * Formato de trama
 * ------------------------------------------------------------------------ */
#define CAN_BUS_FD_ENABLED 1U     /* tramas CAN FD (FDF = 1) */
#define CAN_BUS_FD_ISO 1U         /* ISO CAN FD, no "non-ISO" */
#define CAN_BUS_BRS_ENABLED 0U    /* sin bit rate switching */
#define CAN_BUS_EXTENDED_IDS 0U   /* solo IDs estándar de 11 bits */
#define CAN_BUS_REMOTE_FRAMES 0U  /* no se usan tramas remotas */
#define CAN_BUS_MAX_DATA_BYTES 32U /* longitud máxima de datos por trama */

/* ------------------------------------------------------------------------
 * Comprobación de tiempos de bit de una placa
 *
 * clk_hz: reloj del periférico FDCAN
 * presc:  prescaler nominal
 * seg1:   segmento 1 nominal (PROP_SEG + PHASE_SEG1; en la HAL de ST es
 *         NominalTimeSeg1)
 * seg2:   segmento 2 nominal (PHASE_SEG2)
 *
 * Son expresiones constantes, así que se pueden usar en un _Static_assert
 * para que el proyecto no compile si los tiempos no cumplen el bus:
 *
 *   _Static_assert(CAN_BUS_TIMING_IS_VALID(100000000UL, 4U, 21U, 3U),
 *                  "Los tiempos de CAN1 no cumplen el bus");
 *
 * O en tiempo de ejecución, con los valores reales del periférico tras
 * inicializarlo (así también detecta cambios al regenerar con CubeMX):
 *
 *   if (!CAN_BUS_TIMING_IS_VALID(HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN),
 *                                hfdcan1.Init.NominalPrescaler,
 *                                hfdcan1.Init.NominalTimeSeg1,
 *                                hfdcan1.Init.NominalTimeSeg2)) {
 *     Error_Handler();
 *   }
 * ------------------------------------------------------------------------ */
#define CAN_BUS_TQ_PER_BIT(seg1, seg2) (1UL + (unsigned long)(seg1) + (unsigned long)(seg2))

/* Velocidad exacta: se multiplica en vez de dividir para no perder decimales */
#define CAN_BUS_BITRATE_IS_VALID(clk_hz, presc, seg1, seg2)                    \
  ((unsigned long long)(clk_hz) ==                                             \
   (unsigned long long)CAN_BUS_BITRATE_BPS * (unsigned long long)(presc) *     \
       (unsigned long long)CAN_BUS_TQ_PER_BIT(seg1, seg2))

/* Punto de muestreo de una configuración, en milésimas */
#define CAN_BUS_SAMPLE_POINT_OF(seg1, seg2)                                    \
  ((1000UL * (1UL + (unsigned long)(seg1))) / CAN_BUS_TQ_PER_BIT(seg1, seg2))

#define CAN_BUS_SAMPLE_POINT_IS_VALID(seg1, seg2)                              \
  ((CAN_BUS_SAMPLE_POINT_OF(seg1, seg2) + CAN_BUS_SAMPLE_POINT_TOLERANCE_PERMILLE >= \
    (unsigned long)CAN_BUS_SAMPLE_POINT_PERMILLE) &&                           \
   (CAN_BUS_SAMPLE_POINT_OF(seg1, seg2) <=                                     \
    (unsigned long)CAN_BUS_SAMPLE_POINT_PERMILLE + CAN_BUS_SAMPLE_POINT_TOLERANCE_PERMILLE))

#define CAN_BUS_TIMING_IS_VALID(clk_hz, presc, seg1, seg2)                     \
  (CAN_BUS_BITRATE_IS_VALID(clk_hz, presc, seg1, seg2) &&                      \
   CAN_BUS_SAMPLE_POINT_IS_VALID(seg1, seg2))

#endif /* CAN_BUS_CONFIG_H */
