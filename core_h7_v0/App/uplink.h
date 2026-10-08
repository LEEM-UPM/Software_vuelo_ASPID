/*
 * Comandos que llegan de tierra por radio (el camino de subida).
 *
 *   tierra -> puente -> LoRa -> placa RF -> CAN1 0x241 -> aquí
 *
 * La RF ya ha filtrado por CRC: lo que llega aquí pasó el CRC de LoRa. Este
 * módulo sólo tiene que decidir si además es un comando nuestro, si está
 * PERMITIDO ahora mismo, y ejecutarlo.
 *
 * FORMATO, 4 bytes (parámetro de 8 bits):
 *
 *   0   0x5A   marca de comando
 *   1   id del comando (UPLINK_CMD_*)
 *   2   parámetro, 0 si el comando no lleva
 *   3   XOR de los tres anteriores
 *
 * O 5 bytes, para los comandos que necesitan un parámetro de 16 bits (hoy
 * sólo el ángulo de lanzamiento, que en centésimas de grado no cabe en 8):
 *
 *   0   0x5A
 *   1   id
 *   2-3 parámetro, uint16 little endian
 *   4   XOR de los cuatro anteriores
 *
 * La longitud la da LoRa, así que los dos formatos no se confunden.
 *
 * === ACUSE (paquete de bajada, 8 bytes) ===
 *
 * Algunos comandos se contestan con un acuse propio, porque tierra necesita
 * saber no sólo que llegaron sino QUÉ ha quedado guardado (el ángulo) o a
 * QUÉ petición contestan (el ping). Va en un tercer tipo de paquete porque el
 * de estado está lleno:
 *
 *   0   0xC3   marca de acuse
 *   1   id del comando al que contesta
 *   2   resultado: 0 aceptado, o el motivo (uplink_denegado_t)
 *   3-4 ángulo: el valor que tiene AHORA el cohete (no el que se pidió)
 *       ping:   el número de ping, devuelto tal cual
 *   5   contador de acuses, +1 por acuse
 *   6   sólo para el ángulo: b0 definido (no es el de defecto)
 *                               b1 guardado en la SD
 *   7   reservado, a cero
 *
 * Se manda un acuse POR CADA COMANDO RECIBIDO, también por las repeticiones:
 * tierra reintenta hasta ver uno, y el que llegue le vale.
 *
 * POR QUÉ UN XOR SI LORA YA TRAE CRC. Porque el CRC de LoRa dice que los bits
 * llegaron como salieron, no que lo que salió fuera un comando. En el aire
 * hay más cosas: otro equipo con la misma palabra de sincronismo, una prueba
 * de alcance, un paquete de otro cohete. La marca y el XOR hacen falsísimo
 * que un paquete ajeno se ejecute como si fuera nuestro.
 *
 * LA SUBIDA ES EL LADO DÉBIL DEL ENLACE y no tiene acuse: la RF está sorda
 * mientras transmite. Medido a 15 Hz de telemetría: llega el 30 % de los
 * comandos (en tierra ahora va a 3 Hz y deberían llegar muchos más, ver
 * rf_telemetry.h). Por eso tierra los REPITE y por eso todo comando tiene que poder
 * recibirse varias veces sin hacer daño:
 *
 *   pedir estado    manda otro paquete de estado, nada más
 *   modo test       renueva el plazo, no hace nada más
 *   pirotécnico     power mantiene el canal 1 s y lo apaga solo; repetirlo
 *                   sólo reinicia ese segundo (ICD 6.3), y un iniciador ya
 *                   quemado no se puede volver a quemar
 *
 * === MODO TEST ===
 *
 * Los pirotécnicos SÓLO se pueden encender desde aquí en modo test, y el modo
 * test SÓLO se puede activar en tierra. Es el primer camino del firmware que
 * puede disparar un pirotécnico por una orden de fuera, así que las
 * protecciones van todas en este archivo:
 *
 *   1. Se entra sólo si el cohete está en tierra: estado POWER ON,
 *      MANTENIMIENTO o ERROR, y fase IDLE o EN TIERRA.
 *      OJO, CAMBIÓ CON EL GNC v1.2: antes también se negaba en READY (el
 *      cohete en la rampa, con el detector de despegue armado y sin saber qué
 *      hay en cada canal, que podría ser el encendido del motor). La v1.2 ya
 *      no tiene READY: en la rampa el cohete está en POWER ON hasta que
 *      despega, así que AHORA EL MODO TEST SE ACEPTA EN LA RAMPA. Pendiente
 *      de decidir si hace falta otro cerrojo (ver PENDIENTES).
 *
 *   2. Si estando en modo test el GNC deja de estar en tierra, el modo test
 *      se cancela EN ESE MISMO PASO, sin esperar a nada.
 *
 *   3. Caduca solo a los UPLINK_TEST_TIMEOUT_MS si nadie lo renueva. Tierra
 *      lo renueva mientras tenga el panel de pruebas abierto, así que si se
 *      cierra la estación o se cae el enlace, el cohete sale solo.
 *
 *   4. Mientras dura, el cohete manda un paquete de estado por segundo, para
 *      que tierra vea que sigue dentro, cuánto le queda y qué ha disparado.
 *      Tierra no habilita los botones de disparo hasta verlo.
 *
 * === SIMULAR RUTA ===
 *
 * Reproduce VUELO.BIN de la SD1 sin bloquear el bucle (gnc_playback.h): el
 * GNC vuela el archivo y la radio manda a tierra lo que va saliendo, como en
 * un vuelo de verdad. El aerofreno se mueve DE VERDAD.
 *
 *   5. Sólo se empieza desde el modo test, y por tanto sólo en tierra.
 *      Al empezar, el modo test SE CIERRA: durante la simulación no se acepta
 *      ningún pirotécnico MANDADO DESDE TIERRA, ni se puede volver a entrar en
 *      modo test. OJO: los del despliegue automático (despliegue.c) SÍ se
 *      encienden, a propósito, porque la simulación sirve para validar ese
 *      flujo de vuelo. Ver despliegue.h. El GNC
 *      cree que vuela y la fase dejaría de ser "en tierra" a los pocos
 *      segundos, pero no se espera a eso: se bloquea desde el primer paso.
 *
 *   6. Parar se acepta siempre. Al terminar (sola o parada) el aerofreno se
 *      recoge, el GNC se reinicia y vuelve a los sensores reales.
 *
 *   7. Mientras dura, un paquete de estado por segundo con el avance, para
 *      que tierra pueda marcar en pantalla que los datos son SIMULADOS.
 */

#ifndef UPLINK_H
#define UPLINK_H

#include <stdbool.h>
#include <stdint.h>

#include "sys_utilities.h"

#define UPLINK_MARCA 0x5AU
#define UPLINK_CMD_BYTES 4U

/* Comandos */
#define UPLINK_CMD_PEDIR_ESTADO 0x01U
#define UPLINK_CMD_PING 0x02U /* 16 bits: numero de ping. Contesta con un
                                 acuse que lo devuelve tal cual, para medir
                                 la ida y vuelta y saber a que ping contesta */
#define UPLINK_CMD_MODO_TEST 0x10U /* parámetro: 1 entrar/renovar, 0 salir */
#define UPLINK_CMD_PIRO 0x11U      /* parámetro: canal 1=1A 2=1B 3=2A 4=2B */
#define UPLINK_CMD_SIMULACION 0x12U /* parámetro: 1 empezar, 0 parar */
#define UPLINK_CMD_ANGULO 0x13U     /* 16 bits: centésimas de grado desde la
                                       vertical, 0..9000. Ver lanzamiento.h */

/*
 * Pruebas del aerofreno (ver airbrake.h). Las dos piden MODO TEST, cohete en
 * tierra y no estar simulando, igual que un pirotécnico, y las dos contestan
 * SIEMPRE con un acuse (también al denegar). Al salir del modo test, el
 * aerofreno vuelve a recogido.
 *
 * PRUEBA: parámetro = número de prueba, 1..255, que elige tierra; 0 la para.
 *   3 veces todo fuera 0,5 s y recogido 0,5 s, luego sale en 2 s y entra en
 *   2 s: 7 s en total. NO ES IDEMPOTENTE, por eso lleva número: el cohete
 *   recuerda el último que ejecutó y, si le llega otra vez el mismo (un
 *   reintento de tierra que se cruzó con el acuse), solo contesta, no la
 *   repite. Para otra prueba, otro número. Acuse: valor = el número.
 *
 * EXTENSION: parámetro = 0..100 % de la carrera. Se queda ahí hasta otra
 *   orden o hasta salir del modo test. Idempotente. Acuse: valor = el % que
 *   tiene ahora el aerofreno.
 */
#define UPLINK_CMD_PRUEBA_AEROFRENO 0x14U
#define UPLINK_CMD_EXTENSION_AEROFRENO 0x15U

/* 16 bits: rumbo de la rampa (hacia dónde se inclina el morro), centésimas de
 * grado desde el norte geográfico en sentido horario, 0..35999. 0xFFFF
 * consulta. Mismas reglas y mismo acuse que el ángulo. Ver lanzamiento.h. */
#define UPLINK_CMD_RUMBO 0x16U

/* Alimentación de las cámaras (camaras.h). Parámetro: 1 encender (siempre
 * permitido; si se estaban apagando, vuelven a grabar), 0 apagar (solo en
 * tierra; primero las para y luego corta, unos segundos). Contesta SIEMPRE
 * con un acuse, valor = estado de verdad: b0 alimentación, b1 apagando
 * (paradas o parando, aún con corriente). Tierra reintenta hasta verlo.
 * Idempotente. */
#define UPLINK_CMD_CAMARAS 0x17U

/* Valor especial del ángulo: no cambia nada, sólo pide el acuse con el valor
 * que tiene el cohete. Así tierra puede saberlo sin tener que mandar uno. */
#define UPLINK_ANGULO_CONSULTA 0xFFFFU

#define UPLINK_CMD_BYTES_16 5U

/*
 * Acuse, 12 bytes:
 *   [0] 0xC3  [1] comando  [2] resultado  [3..4] valor  [5] contador
 *   [6] ángulo/rumbo: b0 definido, b1 guardado
 *   [7] b0: los bytes 8..11 son válidos (la RF manda cómo oyó el comando)
 *   [8] RSSI con el que el COHETE oyó el comando, int8 dBm
 *   [9] SNR, int8 dB   [10..11] error de frecuencia, int16 Hz LE
 * Hasta el 2026-10-04 eran 8 (sin los bytes 7..11).
 */
#define UPLINK_ACUSE_MARCA 0xC3U
#define UPLINK_ACUSE_BYTES 12U

/* Plazo del modo test sin renovar */
#define UPLINK_TEST_TIMEOUT_MS 120000U

/* Paquetes de estado mientras dura el modo test */
#define UPLINK_TEST_ESTADO_PERIODO_MS 1000U

/* Por qué se denegó el último comando bien formado */
typedef enum {
  UPLINK_DENEGADO_NADA = 0,
  UPLINK_DENEGADO_FUERA_DE_TIERRA, /* modo test pedido en vuelo */
  UPLINK_DENEGADO_SIN_MODO_TEST,   /* pirotécnico sin estar en modo test */
  UPLINK_DENEGADO_CANAL,           /* canal de pirotécnico fuera de 1..4 */
  UPLINK_DENEGADO_POWER,           /* power no respondió o CAN lleno */
  UPLINK_DENEGADO_SIMULANDO,       /* pirotécnico o modo test en simulación */
  UPLINK_DENEGADO_SIN_ARCHIVO,     /* no hay VUELO.BIN válido en la SD1 */
  UPLINK_DENEGADO_FUERA_DE_RANGO,  /* ángulo fuera de 0..90 grados o
                                      extensión fuera de 0..100 % */
} uplink_denegado_t;

/* Llamar en cada vuelta del bucle principal: mira si la RF ha pasado algo,
 * lo ejecuta, y vigila el plazo y las condiciones del modo test. */
void uplink_update(sys_state *state);

/* Modo test */
bool uplink_modo_test(void);
uint32_t uplink_test_restante_s(void);
uint8_t uplink_ultimo_piro(void);     /* 0 ninguno, 1..4 */
uint8_t uplink_ultima_prueba_aerofreno(void); /* número; 0 ninguna */
uint32_t uplink_piros_aceptados(void);

/* Contadores para la consola */
uint32_t uplink_recibidos(void);   /* paquetes llegados del aire */
uint32_t uplink_validos(void);     /* comandos ejecutados */
uint32_t uplink_rechazados(void);  /* marca, longitud o XOR mal */
uint32_t uplink_denegados(void);   /* bien formados pero no permitidos ahora */
uint32_t uplink_desconocidos(void); /* bien formados, id sin implementar */
uplink_denegado_t uplink_ultimo_denegado(void);
uint8_t uplink_ultimo_cmd(void);
uint32_t uplink_ultimo_ms(void);
/* Cómo oyó el cohete el último paquete de tierra (valida = false sin datos) */
sys_rf_rx_meta_t uplink_ultima_recepcion(void);

#endif /* UPLINK_H */
