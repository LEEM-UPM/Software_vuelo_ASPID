/*
 * GNC del cohete (ASPID v1.5), generado con Simulink y copiado tal cual en
 * third_party/aspid_gnc. Aquí solo está el pegamento: recoger los sensores
 * que llegan por CAN2 y ejecutar los tres modelos a su ritmo.
 *
 * Los tres modelos, en este orden (third_party/aspid_gnc/LEEME_INTEGRACION):
 *  1. nav            400 Hz  INS + EKF de 15 estados con las dos IMU, el
 *                            barómetro y el GNSS: posición y velocidad NED,
 *                            actitud, sesgos y altura sobre el suelo.
 *  2. state_machine  400 Hz  estado del cohete y detección del despegue.
 *  3. gc              50 Hz  (uno de cada 8 ticks) fase de vuelo, apogeo
 *                            previsto, mando del aerofreno y pirotecnia.
 *
 * Las salidas que mueven cosas NO salen de aquí: las recogen, en el bucle y
 * justo después de gnc_update(), despliegue.c (banderas de drogue y main ->
 * pirotécnicos) y airbrake.c (mando del aerofreno -> servo). Este módulo no
 * manda nada a ninguna placa.
 *
 * Los sensores llegan a su ritmo por CAN2 y se acumulan entre ticks.
 *
 * MEDIDAS (barómetro y GNSS): data_valid = 1 SOLO en el tick en que hay dato
 * nuevo. Con el nivel fijo a 1 el filtro inyecta la misma medida muchas
 * veces, la covarianza colapsa y deja de aceptar medidas (lo dice la
 * entrega, y ya nos pasó con el GPS en la versión 1.1).
 *   barómetro    uno de cada GNC_BARO_DECIMATION ticks (66,7 Hz), si hay
 *                muestra nueva; la presión se mantiene SIEMPRE con la última
 *                porque el alineamiento la promedia en todos los ticks
 *   GNSS         cuando llega (10 Hz)
 *
 * IMU: AQUÍ NOS APARTAMOS DE LA GUÍA DE LA ENTREGA, a propósito. La IMU no
 * es una medida del filtro sino la entrada de la integración, y nav.c hace
 * con un tick sin IMU válida dos cosas: integra con la muestra anterior (lo
 * mismo que pasarle la anterior) y PONE A CERO EL CONTADOR DEL ALINEAMIENTO.
 * Las nuestras llegan a ~193 Hz (la mitad de los ticks sin muestra nueva), y
 * aunque llegaran a 416 Hz el jitter dejaría algún tick vacío entre los 200
 * seguidos que pide el alineamiento: no se alinearía nunca (pasó al
 * integrarla, 2026-10-04). Así que la IMU entra con data_valid = 1 mientras
 * esté VIVA, con la media de lo llegado o la última muestra, y a 0 solo si
 * lleva más de GNC_IMU_MAX_HOLD_TICKS sin mandar nada. Comentarlo con GNC.
 */

#ifndef GNC_H
#define GNC_H

#include <stdbool.h>
#include <stdint.h>

/* Ritmos del código generado: nav y máquina de estados a 400 Hz (dt fijo de
 * 2,5 ms dentro del modelo), guiado uno de cada 8 ticks (50 Hz) */
#define GNC_RATE_HZ 400U
#define GNC_PERIOD_US 2500U
#define GNC_GC_DECIMATION 8U
#define GNC_BARO_DECIMATION 6U /* 66,7 Hz, como el dataset de la entrega */

/*
 * Si el bucle se atrasa, se dan los ticks que faltan uno por vuelta, hasta
 * este retraso. Más atrás se da por perdido, se cuenta como overrun y se
 * vuelve a sincronizar con el reloj. Recuperar importa: el modelo integra con
 * dt fijo, así que un tick saltado es tiempo que el EKF no ve pasar.
 */
#define GNC_MAX_RETRASO_TICKS 8U /* 20 ms */

/* Una IMU que lleva más de esto sin mandar muestras se da por caída
 * (data_valid = 0, ver arriba). Hoy llegan cada ~5 ms. */
#define GNC_IMU_MAX_HOLD_TICKS 8U /* 20 ms */

/* Estados del cohete (state_machine v1.2 y v1.5). OJO: la v1.1 tenía READY en el 2
 * y EN VUELO en el 3. Usar siempre estas constantes, nunca el número. */
#define GNC_ESTADO_POWER_ON 0
#define GNC_ESTADO_MANTENIMIENTO 1
#define GNC_ESTADO_EN_VUELO 2
#define GNC_ESTADO_ERROR 3

/* Comandos de telemetría (gnc_command) que la v1.5 añade a la máquina de
 * estados: los pasa a la navegación (nav_cmd) en el tick siguiente */
#define GNC_CMD_REALINEAR 10U /* solo en POWER ON o MANTENIMIENTO */
#define GNC_CMD_REINICIAR_COVARIANZA 11U

/* Fases de vuelo (gc) */
#define GNC_FASE_IDLE 0
#define GNC_FASE_BOOST 1
#define GNC_FASE_COAST 2
#define GNC_FASE_CONTROL 3
#define GNC_FASE_APOGEO 4
#define GNC_FASE_DROGUE 5
#define GNC_FASE_MAIN 6
#define GNC_FASE_EN_TIERRA 7

/*
 * Ejes.
 *
 * La placa de sensores va de mamparo: su eje Z corre a lo largo del cohete,
 * hacia el morro, y X e Y quedan en el plano transversal. Las muestras que
 * llegan por CAN2 están en esos ejes de placa. (La IMU1, WSEN-ISDS, va
 * además girada 180° en Z respecto a la otra, pero eso ya lo corrige la
 * propia aviónica en su driver, Drivers/Sensors/imu.c, IMU_MOUNT_X/Y.)
 *
 * El modelo generado, en cambio, trabaja en el convenio aeronáutico de
 * cuerpo: X al morro, Y a la derecha, Z abajo. Eso está metido en la
 * mecanización y en el EKF y no se puede cambiar desde fuera, así que lo que
 * se hace es girar las medidas al entrar:
 *
 *   cuerpo X (morro)   =   placa Z
 *   cuerpo Y (derecha) =   placa Y
 *   cuerpo Z (abajo)   = - placa X
 *
 * Es un giro puro (determinante +1), así que sirve igual para la aceleración
 * y para la velocidad angular. Qué eje transversal hace de "derecha" solo
 * mueve la referencia del alabeo; si se cambia, hay que mantener la terna a
 * derechas.
 *
 * La navegación (posición y velocidad) sigue saliendo en NED, que es
 * independiente de cómo esté montada la placa: Z abajo, o sea que la altura
 * sobre el punto de partida es -pos_ned_m[2].
 */

/* A 0 las medidas entran tal cual, sin girar (placa alineada con el
 * convenio del modelo). */
#define GNC_BOARD_Z_ALONG_ROCKET 1

typedef struct {
  /* Navegación */
  float pos_ned_m[3];   /* respecto al origen, que es donde se fijó el GPS */
  float vel_ned_mps[3];
  float quat_nb[4];       /* w, x, y, z. Es lo que hay que MANDAR por radio:
                           * los angulos de Euler de abajo son degenerados a
                           * 90 grados de cabeceo, que es la actitud del cohete
                           * en la rampa (ver el comentario de euler_rpy_deg). */
  float euler_rpy_deg[3]; /* roll, pitch, yaw. Comodos de leer en la consola,
                           * pero OJO: a +/-90 de cabeceo hay bloqueo de cardan
                           * y alabeo y guiñada dejan de ser independientes. */
  float accel_body_mps2[3];
  float gyro_body_rps[3];
  float accel_bias_mps2[3]; /* sesgos que estima el EKF */
  float gyro_bias_rps[3];
  float alt_agl_m;          /* altura sobre el suelo (barómetro + EKF) */
  float gnss_age_s;
  bool gnss_only;           /* en el descenso: solo GNSS, sin inercial */
  bool nav_valid;

  /* Máquina de estados y guiado */
  int32_t rocket_state; /* GNC_ESTADO_* */
  int32_t flight_phase; /* GNC_FASE_* */
  uint8_t error_code;
  float apogee_pred_m;
  float apogee_error_m;
  float time_to_apogee_s;
  float actuator_cmd; /* 0..1, extensión que pide el guiado */
  bool pyro_armed;    /* el guiado ha pasado de 100 m: pirotecnia armada */
  bool drogue_flag;   /* 1 s al entrar en DROGUE (ver despliegue.h) */
  bool main_flag;     /* 1 s al entrar en MAIN */

  /* Diagnóstico */
  uint32_t steps;           /* ticks de nav + máquina de estados (400 Hz) */
  uint32_t gc_steps;        /* pasos del guiado (50 Hz) */
  uint32_t last_step_us;    /* lo que tardó el último tick (con guiado o no) */
  uint32_t max_step_us;     /* el peor hasta ahora */
  uint32_t max_gc_step_us;  /* el peor tick CON paso de guiado */
  uint32_t overruns;        /* veces que se perdieron ticks (> 20 ms tarde) */
  uint32_t ticks_recuperados; /* ticks dados con retraso, recuperando */
  uint32_t imu1_samples;    /* muestras promediadas en el último tick */
  uint32_t imu2_samples;
  uint32_t imu1_sin_dato;   /* ticks sin muestra nueva de la IMU1 */
  uint32_t imu1_caida;      /* ticks con la IMU1 dada por caída (> 20 ms) */
  uint32_t imu2_caida;
  uint32_t baro_updates;    /* medidas de barómetro dadas al EKF */
  uint32_t gps_updates;     /* tramas de GPS usadas desde el arranque */
  uint8_t gps_fix_type;     /* fix que ve el EKF: UBX, 3 = 3D; 0 = ninguno */

  /*
   * Alineamiento inicial del EKF. Hasta que no termina, nav_valid es 0 y a
   * los 60 s la máquina de estados declara ERROR 1.
   *
   * Para alinearse necesita GNC_ALIGN_STEPS ticks SEGUIDOS con el cohete
   * quieto (giro por debajo de GNC_ALIGN_GYRO_MAX_DPS y aceleración a menos
   * de GNC_ALIGN_ACCEL_TOL_MPS2 de la gravedad) Y que la gravedad medida
   * esté a menos de GNC_ALIGN_ATT_MAX_DEG de la actitud de la rampa. Solo en
   * POWER_ON. No hace falta GPS.
   */
  uint32_t align_count;   /* ticks acumulados, de GNC_ALIGN_STEPS */
  float rampa_vertical_deg;   /* rampa en uso: desde la vertical */
  float rampa_rumbo_deg;      /* y rumbo */
  float rampa_alabeo_deg;     /* alabeo medido o dado */
  bool rampa_alabeo_medido;   /* true si se mide (GNC_ALABEO_MEDIR) */
  float inclinacion_medida_deg; /* la de la gravedad medida, mientras alinea */
  float rampa_error_deg;      /* ángulo entre la gravedad medida y la que
                                 espera la rampa: lo que nav.c exige < 10 */
  bool rampa_cambiada;        /* la de vuelo ya no es con la que se alineó */
  uint32_t realineados;   /* veces que se ha mandado realinear (v1.5) */
  float accel_norm_mps2;  /* |a| de la IMU1 en el último tick */
  float gyro_norm_dps;    /* |w| de la IMU1 en el último tick */
} gnc_data_t;

/* Lo que exige nav.c para darse por alineado */
#define GNC_ALIGN_STEPS 200U          /* 0,5 s a 400 Hz */
#define GNC_ALIGN_GYRO_MAX_DPS 4.0f   /* 0,0698 rad/s */
#define GNC_ALIGN_ACCEL_TOL_MPS2 1.5f /* alrededor de 9,80665 */
#define GNC_ALIGN_ATT_MAX_DEG 10.0f   /* gravedad medida vs actitud de rampa */

/* Inicializa los tres modelos y sus variables configurables. Llamar una vez,
 * antes del bucle principal. */
void gnc_init(void);

/* Da los ticks que toquen por el reloj (GNC_RATE_HZ). Llamar en cada vuelta
 * del bucle principal. */
void gnc_update(void);

/*
 * LA RAMPA, que es la actitud con la que se alinea el EKF.
 *
 *   desde_vertical_deg  inclinación de la rampa desde la vertical (0 = vertical)
 *   rumbo_deg           hacia dónde se inclina el morro, desde el norte
 *                       geográfico en sentido horario (0 N, 90 E...)
 *
 * Conversión al convenio del modelo (v1.5: nav_tun.rail_elev_deg y
 * rail_azim_deg; el modelo se alinea SIEMPRE con alabeo 0):
 *   elevación = 90 - desde_vertical,   azimut = rumbo,
 *   alabeo  = el que se dé, o si es GNC_ALABEO_MEDIR (lo normal en vuelo),
 *             MEDIDO con el acelerómetro mientras se alinea: con el cohete
 *             inclinado la gravedad tiene una componente lateral, y hacia qué
 *             lado de la placa cae dice cómo está girada sobre su eje. Sin eso
 *             habría que saber cómo se montó la placa, y si no cuadra la
 *             gravedad esperada cae hasta el doble del ángulo de rampa lejos
 *             de la medida y el EKF no se alinea nunca. Con la rampa casi
 *             vertical (< 2 grados de inclinación medida) no se puede medir y
 *             no hace falta: alabeo 0. OJO: la medida es sensible al sesgo
 *             del acelerómetro con poca inclinación (con 5,5 grados, 0,37
 *             m/s2 de sesgo son 21 grados de alabeo); el error de alabeo es
 *             un giro sobre el eje del cohete y apenas afecta, pero si se
 *             conoce (p. ej. el vuelo grabado) mejor darlo.
 *             Como la v1.5 no admite alabeo, gnc.c GIRA las medidas ese
 *             ángulo sobre el eje del cohete antes de dárselas y deshace el
 *             giro en las salidas (ver _quitar_alabeo() en gnc.c).
 *
 * El alineamiento solo termina si la gravedad medida está a menos de 10
 * grados de esta actitud, y lo que quede de diferencia lo toma como SESGO del
 * acelerómetro. Ojo en el banco: con 17 grados guardados y la placa plana en
 * la mesa no se alinea nunca (la consola lo dice).
 *
 * gnc_set_rampa() es la de VUELO: se guarda y la aplican todos los
 * gnc_init() (también el del final de una simulación). En tierra (POWER ON
 * o MANTENIMIENTO) se aplica además EN EL ACTO: si el EKF ya estaba
 * alineado, lo REALINEA (comando 10 de la v1.5; en la placa tarda ~0,5 s).
 * En vuelo, en ERROR o simulando solo se guarda.
 * gnc_set_rampa_solo_este_vuelo() es para la reproducción: la
 * rampa del vuelo grabado, hasta el próximo gnc_init(). Las dos van por el
 * mismo camino; llamar a la segunda justo después de gnc_init().
 */
#define GNC_ALABEO_MEDIR (__builtin_nanf(""))
void gnc_set_rampa(float desde_vertical_deg, float rumbo_deg,
                   float alabeo_deg);
void gnc_set_rampa_solo_este_vuelo(float desde_vertical_deg, float rumbo_deg,
                                   float alabeo_deg);

/* Volver a alinear el EKF con el cohete quieto y la rampa que ya tiene
 * (comando 10, v1.5). Solo en POWER ON o MANTENIMIENTO y sin simular; false
 * si no se puede. Hoy no lo llama nadie: gnc_set_rampa() realinea sola. */
bool gnc_realinear(void);

/*
 * Fuente de los datos del GNC: los sensores de verdad o una reproduccion.
 *
 * Con la reproduccion activa (gnc_playback.c):
 *   - sensor_board.c deja de pasar las medidas de CAN2 al GNC. Si no, las
 *     medidas reales de la placa quieta encima de la mesa se promediarian con
 *     las del archivo y el EKF veria un cohete que vuela y no vuela a la vez.
 *   - gnc_update() no mira el reloj: da un tick por registro, cuando la
 *     reproduccion avisa con gnc_replay_muestra_lista().
 *
 * El GNC CREE QUE ESTA VOLANDO mientras dura, y A PROPOSITO todo lo que
 * cuelga de el se ejecuta igual que en vuelo: ninguna de esas funciones
 * sabe si se esta simulando. En particular despliegue.c ENCIENDE los
 * pirotecnicos cuando el GNC da las banderas de drogue y main, y airbrake.c
 * MUEVE EL SERVO: la simulacion existe para validar ese flujo, asi que no
 * lleva excepciones.
 */
void gnc_set_replay(bool activa);
bool gnc_replay_activa(void);

/*
 * Cuantas veces se ha llamado a gnc_init(). Cada llamada es un vuelo nuevo
 * (o una simulacion nueva): quien guarde estado de "esto ya se hizo en este
 * vuelo" lo usa para saber cuando olvidarlo, sin que gnc.c tenga que
 * conocerle.
 */
uint32_t gnc_init_count(void);

/*
 * Solo para la reproduccion: "ya he metido las muestras de este registro".
 * El siguiente gnc_update() da el tick, uno y solo uno. Asi la simulacion es
 * determinista (un registro = un tick, como en el vuelo grabado) pero el tick
 * lo sigue dando gnc_update() desde el bucle, igual que en vuelo.
 */
void gnc_replay_muestra_lista(void);

/* Estado más reciente del GNC. Nunca es NULL. */
const gnc_data_t *gnc_get_data(void);

/*
 * Comando de telemetria para la maquina de estados y el guiado. Lo ve el
 * siguiente tick de la maquina de estados y el siguiente paso del guiado,
 * una vez cada uno (van a ritmos distintos).
 *
 * Los que entiende la v1.5:
 *   maquina de estados  1 a MANTENIMIENTO   2 salir de MANTENIMIENTO
 *                       3 salir de ERROR    255 forzar ERROR
 *                       10 realinear la navegacion (solo en POWER ON o
 *                          MANTENIMIENTO)   11 reiniciar su covarianza
 *                          (los dos se los pasa a nav en el tick siguiente)
 *   guiado (solo en MANTENIMIENTO)
 *                       4 aerofreno manual (param 0..1)
 *                       5 / 6 pulso de prueba de drogue / main
 * Ya no hay ARMAR: de POWER_ON se pasa a EN VUELO detectando el despegue.
 * Hoy solo lo llama la reproduccion (los comandos del vuelo grabado, que no
 * trae ninguno); el 10 lo manda gnc_set_rampa() por su cuenta. No hay
 * comando de tierra que lo use.
 */
void gnc_command(uint8_t cmd_id, float param);

/* --- Entrada de sensores: lo llama sensor_board.c al decodificar CAN2 --- */

void gnc_on_imu1(const float accel_mps2[3], const float gyro_rps[3],
                 uint64_t timestamp_us);
void gnc_on_imu2(const float accel_mps2[3], uint64_t timestamp_us);

/* Barómetro (el BARO1 de la aviónica). La altura que pide el bus del modelo
 * (alt_msl_m) se calcula con la atmósfera estándar; nav.c solo usa la
 * presión. */
void gnc_on_baro(float pressure_kpa, float temperature_c,
                 uint64_t timestamp_us);

/*
 * Del GPS, en las unidades enteras del ICD de CAN2. La conversión a lo que
 * pide el modelo (latitud y longitud en RADIANES, altura en metros) se hace
 * dentro. Dos tramas, cada una con lo suyo:
 *
 *   GGA (0x106)     -> gnc_on_gps_position(): solo la posicion.
 *   NAV-PVT (0x108) -> gnc_on_gps_pvt(): velocidad NED y CALIDAD DEL FIX.
 *
 * La calidad sale de NAV-PVT y no del GGA a proposito: el modelo la quiere
 * en la convencion UBX (fix_type 3 = 3D) y el fix_quality del GGA es NMEA
 * (1 = fix normal). Con el del GGA el EKF no usaba nunca el GPS.
 */
void gnc_on_gps_position(int32_t latitude_e7, int32_t longitude_e7,
                         int32_t altitude_mm);

/*
 * NAV-PVT. fix_type en convencion UBX (0 ninguno, 2 2D, 3 3D...), valid es
 * el bit gnssFixOK. Sin valid no se toca la velocidad y el fix cuenta como
 * 0. vd_mmps = INT32_MIN si no se sabe la vertical. Si NAV-PVT deja de
 * llegar, el fix caduca a 0 en GNC_GPS_FIX_MAX_AGE_MS (gnc.c).
 */
void gnc_on_gps_pvt(int32_t vn_mmps, int32_t ve_mmps, int32_t vd_mmps,
                    uint8_t fix_type, uint8_t satellites, bool valid);

#endif /* GNC_H */
