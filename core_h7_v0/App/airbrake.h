/*
 * Aerofreno: lleva el mando que calcula el GNC hasta el servo de verdad.
 *
 * El servo está en la placa de aviónica, en PB10, y se manda por CAN2 con la
 * trama 0x200: un solo byte, 0 a 255, que la aviónica traduce a un pulso de
 * 0,5 a 2,5 ms (Drivers/Servo/servo.h de sensores_avionica_g4_v0). No hay
 * realimentación: no se sabe si el servo llegó a la posición, solo que la
 * orden salió al bus.
 *
 * UN SOLO CAMINO, igual que despliegue.c: airbrake_update() se llama en el
 * bucle principal justo después del GNC, y es lo ÚNICO que manda la trama
 * 0x200. Vuele de verdad o se esté simulando (gnc_playback.c), el mando sale
 * de aquí con las mismas reglas: por cada paso nuevo del guiado (52 Hz), una
 * trama.
 *
 *   - Se manda SIEMPRE lo que pida el GNC, en cualquier estado (desde la
 *     v1.6, 2026-10-11). Hasta la v1.5 solo EN VUELO y en tierra un 0, pero
 *     la v1.6 usa el aerofreno también fuera de EN VUELO y lo da por hecho:
 *       - al terminar de alinearse, un BARRIDO (todo fuera y a ras de piel,
 *         ~0,6 s, una vez por encendido), con el cohete EN LA RAMPA;
 *       - en ERROR habiendo volado, todo fuera (fallo en vuelo).
 *     En tierra sin nada de eso el guiado da 0 (a ras de piel). Las pruebas
 *     de tierra (comandos 0x14 y 0x15) siguen mandando sobre él.
 *   - Tras un gnc_init() (vuelo o simulación nueva, o fin de simulación) el
 *     GNC vuelve a POWER_ON y su mando a 0: el aerofreno se recoge solo al
 *     acabar una simulación, sin código aparte.
 *
 * EL MANDO DEL GNC NO ES EXTENSIÓN (v1.6): es despliegue aerodinámico de 0
 * (paleta a ras de piel) a GNC_ACTUADOR_MAX = 0,90 (todo fuera). Aquí se
 * pasa a la extensión de abajo dividiendo por 0,90, porque nuestra
 * extensión ya empieza a ras de piel (AIRBRAKE_CERO_MM) y acaba en el tope.
 * La entrega propone otra cuenta (recorrido = mando + 0,10 y un acos, con
 * una manivela de 90 grados y 40 mm de carrera): es su modelo simplificado
 * del mecanismo. Aquí se usa el de verdad (biela-manivela del PDF, 31,49 mm,
 * 109 grados), con los mismos extremos: 0 a ras de piel, 0,90 al tope. El
 * modo de emergencia publica su mando en la misma escala.
 *
 * Se manda a 52 Hz siempre, también en tierra (con 0): si la aviónica se
 * reinicia, el servo recibe su posición en 20 ms.
 *
 * === De la EXTENSIÓN al byte del servo ===
 *
 * Todo lo que entra aquí (el mando del GNC y las pruebas de tierra) es una
 * EXTENSIÓN: 0 recogido, 1 todo fuera, como fracción de la carrera ÚTIL del
 * actuador.
 *
 * EL 0 % NO ES EL TOPE DE DENTRO DEL MECANISMO (cambiado el 2026-10-07): el
 * aerofreno empieza con AIRBRAKE_CERO_MM (4 mm) de extensión, y el 100 %
 * sigue en el tope de fuera. La carrera útil son 27,49 mm de los 31,49 del
 * mecanismo, y el tope de dentro ya no se manda nunca.
 *
 * El mecanismo es una biela-manivela centrada (CinematicaBielaManivela.pdf),
 * así que la extensión NO es proporcional al ángulo:
 *
 *            extensión     manivela    servo     byte
 *   tope      0,00 mm        0,0 g     90,0 g    128   (ya no se manda)
 *   0 %      +4,00 mm       24,4 g     72,1 g    102
 *   50 %    +17,75 mm       68,8 g     39,5 g     56
 *   100 %   +31,49 mm      109,0 g     10,0 g     14
 *
 * La cuenta va en airbrake_extension_a_grados(), y de ahí al servo con los
 * dos topes medidos de abajo.
 */

#ifndef AIRBRAKE_H
#define AIRBRAKE_H

#include <stdbool.h>
#include <stdint.h>

/* Trama de mando del servo en CAN2 (canCommand.h de la aviónica) */
#define AIRBRAKE_CAN_ID 0x200U

/* --- Mecanismo biela-manivela (CinematicaBielaManivela.pdf), en mm --- */
#define AIRBRAKE_MANIVELA_MM 18.928f  /* r */
#define AIRBRAKE_BIELA_MM 34.704f     /* l */
#define AIRBRAKE_X_MIN_MM 17.968f     /* extensión en el tope de dentro */
#define AIRBRAKE_X_MAX_MM 49.458f     /* extensión con el aerofreno todo fuera */
#define AIRBRAKE_MANIVELA_MAX_GRADOS 109.0f /* giro de tope a tope (PDF) */

/* El 0 % del aerofreno: esto más allá del tope de dentro. El 100 % no cambia
 * (AIRBRAKE_X_MAX_MM): la carrera útil es la de los topes menos esto. */
#define AIRBRAKE_CERO_MM 5.0f

/*
 * --- Servo: los dos extremos, MEDIDOS en el banco el 2026-10-04 ---
 *
 * En la escala de mando del servo, 0..180 grados = byte 0..255 = 500..2500 us
 * (servo.c de la aviónica), el aerofreno está:
 *   - RECOGIDO del todo con el servo a 90 grados (byte 128). Es el tope de
 *     dentro, la referencia de la manivela; el 0 % del aerofreno queda 4 mm
 *     más fuera, a 72,1 grados (byte 102)
 *   - FUERA del todo con el servo a 10 grados (byte 14)
 *
 * Son 80 grados de mando para los 109 de giro de la manivela del PDF: el
 * servo no da 180 grados reales entre 500 y 2500 us (daría unos 245), así que
 * se usan los dos extremos medidos y entre ellos se reparte la cinemática:
 * el ángulo de manivela (0..109) se lleva lineal a 90..10. Así los extremos
 * son exactos y en medio se sigue la curva de la biela-manivela.
 * Si se vuelve a montar el brazo, medir de nuevo estos dos valores.
 */
#define AIRBRAKE_SERVO_GRADOS 180.0f           /* escala de mando: byte 255 */
#define AIRBRAKE_SERVO_GRADOS_RECOGIDO 90.0f   /* medido */
#define AIRBRAKE_SERVO_GRADOS_FUERA 10.0f      /* medido */

/* --- Prueba de tierra (comando 0x14, ver uplink.h) --- */
#define AIRBRAKE_PRUEBA_CICLOS 3U        /* todo fuera y recogido, 3 veces */
#define AIRBRAKE_PRUEBA_FUERA_MS 500U    /* cuánto se queda fuera cada vez */
#define AIRBRAKE_PRUEBA_DENTRO_MS 500U   /* y recogido antes del siguiente */
#define AIRBRAKE_PRUEBA_RAMPA_MS 2000U   /* rampa de salida y de entrada */

/* Llamar en cada vuelta del bucle, después de gnc_update(). Solo hace algo
 * cuando el guiado ha dado un paso nuevo. */
void airbrake_update(void);

/* Extensión (0..1, se acota) -> ángulo de MANIVELA desde la posición
 * recogida (0..109 grados), por la cinemática de la biela-manivela. */
float airbrake_extension_a_grados(float extension);

/* Ángulo de manivela (0..109) -> ángulo de MANDO del servo, en la escala
 * 0..180 (90 recogido, 10 fuera) */
float airbrake_manivela_a_servo(float grados_manivela);

/* Ángulo de mando del servo (0..180) -> byte de la trama 0x200 */
uint8_t airbrake_grados_a_byte(float grados_servo);

/*
 * Pruebas de tierra. SOLO tienen efecto con el cohete fuera de EN VUELO: en
 * cuanto el GNC pasa a EN VUELO, o se reinicia (simulación), se cancelan
 * solas y manda el GNC. Quien las pide (uplink.c) comprueba además el modo
 * test y que el cohete esté en tierra, y las para al salir del modo test.
 */
void airbrake_prueba_secuencia(void);        /* 3 x (fuera/dentro) y rampas */
void airbrake_prueba_extension(float extension); /* se queda en ese valor */
void airbrake_prueba_parar(void);            /* vuelve a mandar el GNC (0) */
bool airbrake_prueba_activa(void);
bool airbrake_prueba_secuencia_activa(void);

float airbrake_last_command(void);  /* 0..1, extensión que se mandó */
float airbrake_last_grados(void);   /* ángulo de mando del servo, 0..180 */
uint8_t airbrake_last_byte(void);   /* 0..255, lo que salió al bus */
bool airbrake_last_ok(void);        /* si la última trama se pudo encolar */
uint32_t airbrake_sent(void);
uint32_t airbrake_failed(void);

#endif /* AIRBRAKE_H */
