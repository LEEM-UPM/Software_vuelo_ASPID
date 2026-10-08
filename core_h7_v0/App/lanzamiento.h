/*
 * La rampa de lanzamiento: lo manda tierra por radio y aquí se guarda.
 *
 *   ÁNGULO  medido DESDE LA VERTICAL (0 = rampa vertical). UPLINK_CMD_ANGULO.
 *   RUMBO   hacia dónde se inclina el morro, en grados desde el NORTE
 *           GEOGRÁFICO, en sentido horario (0 N, 90 E, 180 S, 270 O).
 *           UPLINK_CMD_RUMBO. Con la rampa vertical no significa nada.
 *
 * Quien los pasa al GNC es quien los cambia (uplink.c) y flight_software.c
 * al arrancar, con gnc_set_rampa(), que hace la conversión al convenio del
 * modelo (cabeceo = 90 - ángulo). OJO: el EKF solo se alinea una vez, al
 * arrancar; un valor que llegue después se guarda y se aplica en la siguiente
 * alineación (ver gnc_set_rampa).
 *
 * SE GUARDA TAMBIÉN EN LAS DOS SD (ANGULO.TXT), y se lee al arrancar. Si el
 * core se reinicia en la rampa, no puede volver a la vertical por defecto sin
 * que nadie se entere. Sólo se escribe si el valor CAMBIA: tierra reintenta
 * el comando hasta ver la confirmación, y cada repetición no tiene por qué
 * costar una escritura en la tarjeta.
 *
 * Unidades: centésimas de grado, entero (como viaja por radio). 0 a 9000.
 */

#ifndef LANZAMIENTO_H
#define LANZAMIENTO_H

#include <stdbool.h>
#include <stdint.h>

#define LANZAMIENTO_MAX_CENTESIMAS 9000U /* 90 grados: horizontal */
#define LANZAMIENTO_RUMBO_MAX_CENTESIMAS 35999U /* 359,99 grados */
#define LANZAMIENTO_ARCHIVO "ANGULO.TXT"

/* Lee el último guardado de la SD1 (o de la SD2 si la 1 no lo tiene).
 * Llamar después de storage_init(). Sin archivo: 0 (vertical), "sin definir". */
void lanzamiento_init(void);

/*
 * Guarda un ángulo nuevo. Devuelve false si está fuera de 0..90 grados (no
 * cambia nada). Que el cohete esté en un momento en el que se pueda cambiar
 * lo decide quien llama (uplink.c).
 */
bool lanzamiento_definir(uint16_t centesimas);

uint16_t lanzamiento_centesimas(void);
float lanzamiento_grados(void);

/* true si el valor salió de un comando o de la SD; false si es el de defecto */
bool lanzamiento_definido(void);

/* El rumbo, igual: false si se sale de 0..359,99 grados */
bool lanzamiento_definir_rumbo(uint16_t centesimas);
uint16_t lanzamiento_rumbo_centesimas(void);
float lanzamiento_rumbo_grados(void);
bool lanzamiento_rumbo_definido(void);

/* true si la última escritura en la SD salió bien en al menos una tarjeta */
bool lanzamiento_guardado(void);

/* De dónde salió el valor actual, para la consola */
const char *lanzamiento_origen(void);

#endif /* LANZAMIENTO_H */
