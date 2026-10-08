/*
 * Reproducción de vuelo: le da de comer al GNC un vuelo grabado desde la SD y
 * deja que mueva el aerofreno de verdad.
 *
 * Es una prueba distinta del banco PIL (pil_test.h), y conviene no
 * confundirlas:
 *
 *   PIL          compara los modelos contra unas salidas de referencia
 *                generadas en el PC. Responde "¿el M7 calcula lo mismo que
 *                Simulink?". En cuanto se cambie el GNC, esas referencias
 *                dejan de valer y hay que regenerarlas con MATLAB.
 *
 *   Reproducción  no compara contra nada: pasa los sensores por la cadena
 *                COMPLETA tal y como vuela (gnc.c, con su promediado y su
 *                giro de ejes), y apunta lo que ha pasado.
 *
 * EL MISMO CODIGO QUE EN VUELO. Lo unico propio de la reproduccion es de
 * DONDE salen las muestras (el archivo en vez de CAN2) y el informe. Todo lo
 * demas es el bucle de vuelo sin tocar: el paso lo da gnc_update(), el
 * paracaidas lo enciende despliegue_update() y el servo lo mueve
 * airbrake_update(). Aqui no se manda nada a ninguna placa. Si algo solo
 * funcionase simulando, es que se ha colado un camino aparte: no deberia
 * existir. Responde "¿qué hace nuestro cohete con este
 *                vuelo?". Sigue valiendo aunque el GNC cambie, que es de lo
 *                que se trata.
 *
 * ENTRADA: VUELO.BIN en la raíz de la SD1, con las muestras EN EJES DE PLACA,
 * es decir tal y como las manda la aviónica por CAN2. Así la reproducción
 * pasa también por el giro de montaje y no se salta nada. Se prepara con
 * tools/vuelo_from_pil.py; el día que se grabe un vuelo real de la aviónica,
 * el archivo entra aquí sin tocar nada.
 *
 * SALIDA, en la misma tarjeta:
 *   VUELOLOG.CSV   una fila por paso: navegación, fase, mando de aerofreno y
 *                  el byte que salió al servo
 *   VUELORES.TXT   el resumen: cambios de fase, apogeo, recorrido del
 *                  aerofreno y cuánto tiempo estuvo saturado
 *
 * Al terminar, el GNC se reinicia (POWER_ON) y airbrake_update() recoge el
 * aerofreno en el paso siguiente.
 */

#ifndef GNC_PLAYBACK_H
#define GNC_PLAYBACK_H

#include <stdbool.h>
#include <stdint.h>

/*
 * A 1, flight_software lanza la reproducción al arrancar SI ADEMÁS existe
 * VUELO.BIN en la tarjeta. Las dos condiciones a la vez es a propósito: que
 * un archivo olvidado en la SD no pueda hacer que el cohete reproduzca un
 * vuelo en vez de volar. Es lo mismo que pulsar SIMULAR RUTA nada mas
 * arrancar: corre dentro del bucle, con la radio funcionando.
 */
#ifndef GNC_PLAYBACK_ENABLE
#define GNC_PLAYBACK_ENABLE 0
#endif

typedef struct {
  bool ran;
  uint32_t steps;
  uint32_t duration_ms;

  /* Lo que ha hecho el vuelo */
  float max_actuator;      /* 0..1 */
  uint32_t steps_actuated; /* pasos con el aerofreno fuera */
  uint32_t steps_saturated;/* pasos al 100 % */
  float apogee_pred_m;     /* última predicción con el aerofreno activo */
  float max_altitude_m;    /* altura máxima alcanzada (-pos_ned[2]) */
  int32_t last_phase;
  uint32_t can_failures;   /* tramas de aerofreno que no se pudieron encolar */
} gnc_playback_result_t;

/*
 * Avanza dentro del bucle principal, a ritmo real (10 ms por registro): el
 * resto sigue funcionando, y en particular la RADIO, que manda a tierra el
 * paquete de vuelo de lo que va saliendo como si el cohete volara de verdad.
 *
 * Mientras dura, el GNC está en modo reproducción (gnc_set_replay): los
 * sensores reales no entran y el GNC CREE QUE VUELA. Ver el aviso en gnc.h.
 */

/* Abre VUELO.BIN, reinicia el GNC y empieza. false si no hay archivo válido
 * o ya había una en marcha. */
bool gnc_playback_start(void);

/* Llamar en cada vuelta del bucle, ANTES de gnc_update() (es una fuente de
 * sensores). Termina sola al acabar el archivo. */
void gnc_playback_update(void);

/* Corta la reproducción: cierra los archivos y deja el GNC reiniciado y otra
 * vez con los sensores reales (con eso el aerofreno se recoge solo). */
void gnc_playback_stop(void);

bool gnc_playback_active(void);

/* 0..100 */
uint8_t gnc_playback_progress_pct(void);

const gnc_playback_result_t *gnc_playback_get_result(void);

#endif /* GNC_PLAYBACK_H */
