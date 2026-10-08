#ifndef PYRO_CONTROL_H
#define PYRO_CONTROL_H

/*
 * Pirotecnia mandada por el core por CAN1 (ICD, 6.3 a 6.5):
 *  - CMD_PYRO_FIRE: enciende el canal pedido y lo apaga solo pasado
 *    PYRO_ON_TIME_MS.
 *  - CMD_PYRO_CONTINUITY: mide los cuatro canales y responde con
 *    PYRO_CONTINUITY.
 */

#ifdef __cplusplus
extern "C" {
#endif

#define PYRO_ON_TIME_MS 1000U /* tiempo que los pirotécnicos permanecen encendidos */

/*
 * El canal 2B NO es un pirotécnico: alimenta el SERVO DEL AEROFRENO (decidido
 * el 2026-10-04). Se enciende al arrancar (main.c, con la alimentación de los
 * pares) y se queda encendido SIEMPRE: un CMD_PYRO_FIRE del canal 4 se ignora
 * (no lo enciende ni, sobre todo, lo apaga al segundo con el AutoOff). Su
 * continuidad se sigue midiendo y mandando como la de los demás.
 * A 0 vuelve a ser un pirotécnico normal.
 */
#ifndef PYRO_2B_ES_SERVO
#define PYRO_2B_ES_SERVO 1
#endif

/* Llamar en cada vuelta del bucle principal, después de CanNode_Poll(). */
void PyroControl_Update(void);

#ifdef __cplusplus
}
#endif

#endif /* PYRO_CONTROL_H */
