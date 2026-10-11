#include <stdio.h>

#include "buzzer.h"
#include "can_driver.h"
#include "can_protocol/can_ids.h"
#include "can_protocol/can_pack.h"
#include "debug_console.h"
#include "debug_out.h"
#include "fdcan.h"
#include "gnc.h"
#include "airbrake.h"
#include "aspid_data.h"
#include "boton_memoria.h"
#include "camaras.h"
#include "gnc_playback.h"
#include "pil_test.h"
#include "sd_bench.h"
#include "gpio.h"
#include "led_show.h"
#include "main.h"
#include "rf_telemetry.h"
#include "sensor_board.h"
#include "sounds.h"
#include "storage.h"
#include "despliegue.h"
#include "emergencia.h"
#include "lanzamiento.h"
#include "memoria_vuelo.h"
#include "watchdog.h"
#include "status_packet.h"
#include "sys_utilities.h"
#include "uplink.h"

extern void SystemClock_Config(void);
extern void PeriphCommonClock_Config(void);

/* Periodo del LED de actividad: parpadea mientras lleguen tramas válidas */
#define LED_ACTIVITY_PERIOD_MS 250U

sys_state state;

/*
 * MPU: zona de memoria externa (0x60000000-0xDFFFFFFF: FMC, QSPI/OCTOSPI)
 * SIN ACCESO. No hay nada conectado ahí.
 *
 * IMPRESCINDIBLE CON LAS CACHÉS ENCENDIDAS. En el mapa por defecto esa zona es
 * memoria "normal", y el Cortex-M7 puede leer de ella ESPECULATIVAMENTE (sin
 * que el programa lo pida, por un valor que haya en un registro). Con nada
 * detrás, esa lectura no termina nunca y el bus se queda bloqueado: el micro
 * entero parado, sin USB y SIN DEPURADOR, que no puede ni leer el ID. Es la
 * configuración que recomienda ST siempre que se encienden las cachés
 * (AN4838/4839). (El cuelgue que se vio el 2026-10-04 al empezar SIMULAR
 * RUTA NO era esto, sino un HardFault por un float desalineado en
 * gnc_playback.c; el "sin depurador" era un cable mal puesto.)
 * y la que pone CubeMX por defecto: una región de fondo de 4 GB sin acceso, con
 * las subregiones 0, 1, 2 y 7 desactivadas para que la flash, las RAM, los
 * periféricos y los registros del sistema sigan con el mapa normal.
 */
static void _mpu_config(void) {
  MPU_Region_InitTypeDef r = {0};

  HAL_MPU_Disable();
  r.Enable = MPU_REGION_ENABLE;
  r.Number = MPU_REGION_NUMBER0;
  r.BaseAddress = 0x00000000U;
  r.Size = MPU_REGION_SIZE_4GB;
  r.SubRegionDisable = 0x87U; /* activas solo 0x60000000-0xDFFFFFFF */
  r.TypeExtField = MPU_TEX_LEVEL0;
  r.AccessPermission = MPU_REGION_NO_ACCESS;
  r.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  r.IsShareable = MPU_ACCESS_SHAREABLE;
  r.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  r.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  HAL_MPU_ConfigRegion(&r);
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/*
 * ¿Pitido de vuelo? (sounds.h) Cualquier estado de vuelo que no sea la espera
 * en tierra:
 *   - reiniciada en el aire (emergencia) o tras un vuelo (POSTVUELO): siempre;
 *   - arranque normal: desde que el GNC sale de la fase IDLE (despegue,
 *     motor, planeo, apogeo, paracaídas y EN TIERRA tras aterrizar), o desde
 *     que las IMU vieron el lanzamiento (3 g durante 100 ms) aunque el GNC no
 *     lo diera. SIMULAR RUTA también pita mientras dura: recorre el mismo
 *     camino que un vuelo, y así se oye que la core lo está viviendo como
 *     tal. En POWER ON, MANTENIMIENTO o ERROR en tierra el GNC está en IDLE:
 *     no pita.
 */
static bool _pitido_de_vuelo(emergencia_modo_t modo) {
  if (modo != EMERGENCIA_INICIO) {
    return true;
  }
  return (gnc_get_data()->flight_phase != GNC_FASE_IDLE) ||
         rf_telemetry_lanzamiento_visto();
}

/*
 * ¿Se puede borrar la memoria de vuelo con el botón (boton_memoria.h)? En
 * tierra sin vuelo en marcha (lo contrario del pitido de vuelo), o tras un
 * vuelo (POSTVUELO), que es justo cuando hace falta para empezar de cero.
 * Nunca en el modo de emergencia.
 */
static bool _boton_permitido(emergencia_modo_t modo) {
  return (modo == EMERGENCIA_POSTVUELO) || !_pitido_de_vuelo(modo);
}

int main(void) {
  /*
   * Cachés del Cortex-M7. Sin ellas el EKF del GNC v1.2 no cabía en su tick
   * de 2,5 ms. La de instrucciones no tiene pegas. La de DATOS sí: los DMA
   * leen y escriben la RAM sin pasar por ella, así que cada buffer de DMA se
   * limpia o invalida a mano alrededor de la transferencia. Hoy hay dos:
   * la SD (Drivers/Sd_card/sd_card.c) y los LEDs (Drivers/Leds/leds.c).
   * EL QUE AÑADA OTRO DMA TIENE QUE HACER LO MISMO, o leerá basura sin
   * ningún error. El USB va sin DMA y la RAM de mensajes del FDCAN es zona de
   * periférico, que no se cachea.
   */
  _mpu_config();

  /* La memoria de vuelo, lo primero: el último estado guardado en la flash y
   * por qué se ha reiniciado (memoria_vuelo.h). Antes de las cachés: lee la
   * flash ignorando los errores de ECC de una escritura cortada. */
  memoria_vuelo_arrancar();

  SCB_EnableICache();
  SCB_EnableDCache();

  HAL_Init();
  SystemClock_Config();
  PeriphCommonClock_Config();
  MX_GPIO_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();

  /* Consola de debug por USB (puerto COM virtual). Lo que se escriba antes de
   * que el PC abra el puerto queda en el buffer. */
  DebugOut_Init();
  DebugConsole_Banner();

  /* ¿Se ha reiniciado en mitad de un vuelo? ANTES de la música y las luces
   * (emergencia.h). Fuera del arranque normal no hay ni una ni otras, ni SD,
   * ni GNC: hay que decidir en menos de un segundo. */
  emergencia_modo_t modo = emergencia_arranque();

  /* Secuencia de LEDs y melodía de arranque, antes de despertar al resto de
   * placas */
  buzzer_init();
  if (modo == EMERGENCIA_INICIO) {
    led_show_startup();
    sounds_startup();
  }

  DebugConsole_WakeUpStart();
  can_driver_init();

  sys_init(&state, emergencia_arranque_rapido());

  /* Vuelo sin terminar: ¿está en el aire? (como mucho 1,5 s) */
  modo = emergencia_confirmar(&state);
  DebugConsole_Startup(&state);

  if (modo == EMERGENCIA_INICIO) {
    /* Si la memoria de vuelo se está llenando, se borra ahora, en tierra
     * (bloquea 1-2 s) */
    memoria_vuelo_mantener();

    /* Tarjetas SD: montaje y prueba de escritura/lectura (resultado en la
     * consola, sección TARJETAS SD). Bloquea mientras dura (~1 s por tarjeta). */
    storage_init();
    lanzamiento_init(); /* el angulo de la rampa guardado, si lo hay */

    /* La calibración de la aviónica (CMD_AVIONICS_CALIBRATE) ya NO se pide
     * (2026-10-04): los sesgos de las IMU los estima el EKF del GNC al
     * alinearse, y calibrar además en la aviónica restaba dos veces (y con el
     * ángulo de rampa mal, siempre 0). La aviónica sigue sabiendo calibrar,
     * pero sin la orden manda las IMU en bruto. Ver PENDIENTES. */

    /* GNC (navegación, máquina de estados y guiado). Se ejecuta a 416 Hz desde
     * el bucle, con las medias de los sensores que hayan llegado por CAN2. */
    /* La rampa guardada (ángulo y rumbo), en el convenio del GNC: ver
     * gnc_set_rampa(). Antes de gnc_init(), que es quien la aplica. */
    gnc_set_rampa(lanzamiento_grados(), lanzamiento_rumbo_grados(),
                  GNC_ALABEO_MEDIR);
    gnc_init();
  }

  /* Cámaras: los UART de las dos (RunCam, camaras.h). En emergencia se
   * encienden en la primera vuelta (el vuelo sigue); tras un vuelo, no. */
  camaras_init();

  if (modo == EMERGENCIA_INICIO) {
    /* Registro de la sesión en la SD1: carpeta aspid_data_..., telemetría
     * desde ya y datos de vuelo a 100 Hz desde el despegue (aspid_data.h) */
    aspid_data_init(&state);

#if GNC_PLAYBACK_ENABLE
    /* Reproduccion de vuelo al arrancar: si hay VUELO.BIN en la SD1, empieza
     * como un SIMULAR RUTA y avanza en el bucle. Sin ese archivo no hace nada. */
    (void)gnc_playback_start();
#endif

#if SD_BENCH_ENABLE
    /* Banco de velocidad de la SD1: barrido del reloj del bus, escritura
     * sostenida y simulación del registro de vuelo. Bloquea 1-2 minutos y deja
     * el informe en SDBENCH.TXT. Poner SD_BENCH_ENABLE a 0 para volar. */
    sd_bench_run();
#endif

#if PIL_TEST_ENABLE
    /* Banco PIL: inyecta el vuelo simulado desde la SD y compara con las
     * salidas esperadas. Bloquea el bucle alrededor de un minuto y deja el
     * informe en PILRES.TXT. Poner PIL_TEST_ENABLE a 0 para volar. */
    pil_test_run();
#endif
  }

  /* Desde aquí, un bucle colgado más de 2 s reinicia la core (watchdog.h) */
  watchdog_iniciar();

  can_rx_frame_t frame;

  uint32_t valid_frames = 0U;
  uint32_t led_last_frames = 0U;
  uint32_t led_last_tick = HAL_GetTick();

  while (1) {
    watchdog_refrescar();

    /* Un bus que cae en bus-off no vuelve solo (can_driver.h) */
    can_driver_poll_bus_off();
#if CAN_DRIVER_PRUEBA_BUS_OFF
    {
      static uint32_t hecho;
      const uint32_t t = HAL_GetTick();
      if ((hecho < 2U) && (t >= (40000U + (hecho * 10000U)))) {
        const can_port_t p = (hecho == 0U) ? CAN_PORT_1 : CAN_PORT_2;
        const int32_t ms = can_driver_prueba_bus_off(p);
        DebugOut_Printf("\r\n>>> PRUEBA BUS-OFF CAN%lu: %s %ld ms\r\n",
                        (unsigned long)(hecho + 1U),
                        (ms >= 0) ? "en bus-off en" : "NO LLEGO en 200",
                        (long)((ms >= 0) ? ms : 200));
        hecho++;
      }
    }
#endif

    /* ---- Recepción ---- */
    while (can_driver_receive(CAN_PORT_1, &frame)) {
      sys_process_can1_frame(&state, &frame);
    }

    while (can_driver_receive(CAN_PORT_2, &frame)) {
      valid_frames += sensor_board_process_frame(frame.id, frame.data,
                                                 frame.len, frame.rx_tick_ms);
    }

    /* TODO: usar sensor_board_get_data() según lo que haya que hacer con los
     * datos (log, reenvío...) */

    /* ---- Comandos de tierra (RF_RECEIVED por CAN1) ---- */
    uplink_update(&state);

    /* ---- LEDs: modo test (rojo respirando) y camaras (el primero en
     * violeta), ver led_show.h ---- */
    led_show_update(uplink_modo_test(), camaras_encendidas());

    /* ---- SIMULAR RUTA: el archivo de la SD como fuente de sensores ----
     * Va aqui, en el sitio de los sensores y antes del GNC. No hace nada si
     * no se simula. Lo que viene detras es el MISMO codigo en vuelo y
     * simulando: ninguna de esas funciones sabe si se esta simulando. */
    /* ---- GNC a 416 Hz, guiado a 52 Hz (simulando, un tick por registro:
     * ver gnc.h). Reiniciado en vuelo, en su sitio va el modo de emergencia,
     * que da los mismos datos con el barómetro (emergencia.h); tras un
     * vuelo, nada. ---- */
    if (modo == EMERGENCIA_INICIO) {
      gnc_playback_update();
      gnc_update();
    } else if (modo == EMERGENCIA_EN_VUELO) {
      emergencia_update();
    }

    /* ---- Paracaidas: drogue -> 1A y 1B a la vez, main -> 2A ---- */
    despliegue_update(&state);

    /* ---- Aerofreno: mando del GNC -> servo (CAN2 0x200) ---- */
    airbrake_update();

    /* ---- Cámaras: se encienden solas al despegar (camaras.h) ---- */
    camaras_update();

    /* ---- Actividad de los buses, para el paquete de estado ---- */
    status_packet_update();

    /* ---- Telemetría por radio: navegación -> CAN1 -> placa RF ---- */
    rf_telemetry_update(&state);

    /* ---- Registro en la SD1: telemetría y vuelo (aspid_data.h) ---- */
    aspid_data_update(&state);

    /* ---- Memoria de vuelo: lo que cambie, a la flash (emergencia.h) ---- */
    emergencia_vigilar();

    /* ---- Botón BUTTON1: borrar la memoria de vuelo, solo en tierra ---- */
    boton_memoria_update(_boton_permitido(modo));

    /* ---- Gestión del resto de placas (continuidad de power a 1 Hz...) ---- */
    sys_update(&state);

    /* ---- Pitido de vuelo: uno por segundo fuera de la espera en tierra
     * (sounds.h) ---- */
    sounds_pitido_vuelo(_pitido_de_vuelo(modo));

    /* ---- Consola de debug por USB (informe cada segundo) ---- */
    DebugConsole_Update(&state);

    const uint32_t now = HAL_GetTick();

    /* ---- LED de actividad (placa de sensores) ---- */
    if ((now - led_last_tick) >= LED_ACTIVITY_PERIOD_MS) {
      led_last_tick = now;
      if (valid_frames != led_last_frames) {
        led_last_frames = valid_frames;
        HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin);
      } else {
        HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
      }
    }
  }
}
