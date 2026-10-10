# core_h7_v0

Ordenador de vuelo (STM32H723).

## Estructura

| Carpeta / archivo | Qué es |
|-------------------|--------|
| `Core/` | Código generado por CubeMX (solo se toca dentro de `USER CODE`) |
| `App/flight_software.c` | `main()`: arranque y bucle principal. Antes de la melodía y las luces mira la memoria de vuelo: si se ha reiniciado en el aire, arranca en menos de 1 s en modo de emergencia |
| `App/emergencia.c` | Reinicio en vuelo: decide al arrancar (normal, EMERGENCIA o POSTVUELO), sigue el vuelo solo con los barómetros (aerofreno fuera hasta el apogeo, drogue a 10 m por debajo de la máxima, main a 400 m) y guarda en la flash cada cambio del vuelo |
| `App/baro_alt.c` | Altura de cada barómetro respecto a su presión del suelo, con descarte de medidas ruidosas o con saltos (transónico) y altura máxima |
| `App/memoria_vuelo.c` | Memoria de vuelo en el sector 3 de la flash interna (0x08060000): registros de 32 bytes con CRC, a prueba de cortes |
| `App/watchdog.c` | IWDG de 2 s: un bucle colgado reinicia la core (y en el aire sigue la emergencia) |
| `App/sys_utilities.c` | Estado del resto de placas (`sys_state`), wake up y reparto de las tramas de CAN1 |
| `App/sys_power.c` | Placa de potencia: continuidad, encendido de pirotécnicos y raíles |
| `App/sys_avionics.c` | Placa de aviónica: calibración de IMUs y barómetros |
| `App/sys_rf.c` | Placa de RF: manda bytes para transmitir por radio (`sys_rf_transmit`) |
| `App/rf_telemetry.c` | Telemetría por radio (paquetes binarios de vuelo, estado y cámaras, y acuses): 3 Hz en tierra hasta ver el lanzamiento, 15 Hz desde entonces |
| `App/sensor_board.c` | Telemetría de la placa de sensores (CAN2) |
| `App/gnc.c` | GNC v1.6: nav y máquina de estados a 416 Hz, guiado a 52 Hz; alimenta los modelos de `third_party/aspid_gnc` con los sensores de CAN2 (IMU, barómetro, GNSS). Se alinea con la rampa guardada (elevación y azimut; el alabeo lo mide y lo quita girando las medidas) y se realinea en tierra en cuanto cambia |
| `App/debug_console.c` | Consola de debug por USB (puerto COM virtual): banner al arrancar e informe cada segundo |
| `App/storage.c` | Archivos en las dos microSD (FileX standalone): montaje, leer/escribir/borrar, prueba al arrancar |
| `App/aspid_data.c` | Registro de cada sesión en la SD1: carpeta `aspid_data_<fecha>` con la telemetría desde el arranque, el vuelo a 100 Hz y la trust curve a 400 Hz, escrito por DMA sin bloquear (ver abajo) |
| `App/camaras.c` | Las dos cámaras (RunCam Split 4): una alimentación (PA2, CAM_EN), apagadas al arrancar, se encienden solas al despegar y se apagan solas a los 50 m de bajada (como el registro de la SD), o por radio (`UPLINK_CMD_CAMARAS`). Apagar es parar la grabación, confirmar que ha parado y luego cortar. Hora del GPS para su reloj. Manda a tierra el paquete de cámaras (0xCA). Encendidas, el primer LED RGB se pone violeta |
| `App/camara.c` | Una cámara por UART (la 1 en UART8, la 2 en UART9): lee sus ajustes cada segundo, sabe si graba porque baja el espacio libre de su tarjeta, pulsa el botón solo cuando eso confirma que hace falta, y le pone la hora (aprendiendo el huso que la cámara suma) |
| `App/sd_bench.c` | Banco de velocidad de la SD1 (reloj del bus, bloqueo por escritura, simulación del registro de vuelo). Solo con `SD_BENCH_ENABLE` a 1; informe en `SDBENCH.TXT` |
| `App/sounds.c` | Melodías con el zumbador (Bella Ciao al arrancar) y pitido de vuelo: uno por segundo en cualquier estado de vuelo que no sea la espera en tierra (también tras aterrizar, para encontrar el cohete) |
| `App/led_show.c` | Animaciones en los 6 LEDs RGB (secuencia de arranque) |
| `Drivers/Buzzer/buzzer.c` | Zumbador en PE4 (TIM15_CH1N): tono de frecuencia variable |
| `Drivers/Leds/leds.c` | Los 6 LEDs RGB en cadena (WL-ICLED, tipo WS2812) en PD12: TIM4_CH1 + DMA |
| `Drivers/Runcam/runcam.c` | Cámaras RunCam por UART (RunCam Device Protocol, 115200): envío y recepción por interrupción, sin esperar nunca; información, botones, lectura y escritura de ajustes |
| `Drivers/Sd_card/` | Acceso por bloques a SD1 (SDMMC1) y SD2 (SDMMC2) con DMA, y driver de FileX |
| `Drivers/Debug_out/debug_out.c` | Salida de texto de debug con buffer circular (no bloquea) sobre el USB |
| `Drivers/Usb_cdc/` | Puerto serie virtual USB (CDC) en USB_OTG_HS full speed, PA11/PA12, reloj HSI48 + CRS |
| `third_party/stm32_usb_device_library/` | Librería USB Device de ST (núcleo + clase CDC), copiada de STM32Cube H7 1.12.1 |
| `Drivers/Can_driver/can_driver.c` | Driver de FDCAN1 y FDCAN2: filtros, envío y recepción con ring buffer, y salida de bus-off (`can_driver_poll_bus_off`, en cada vuelta del bucle) |
| `config/` | Configuración de ThreadX/FileX (FileX en modo standalone) |
| `bsp/` | Driver de SD para FileX con ThreadX (solo la PoC `poc/usd_filex_dma`, no se compila) |
| `third_party/aspid_gnc/` | GNC del cohete generado con Simulink: EKF de navegación, máquina de estados y guiado (licencia académica de MathWorks) |
| `third_party/` | ThreadX, FileX y `can_protocol` (submodule con el ICD del bus CAN1) |
| `poc/` | Pruebas de concepto (no se compilan por defecto) |

Cada placa nueva que se gestione desde el core va en su propio archivo
`App/sys_<placa>.c`, igual que `sys_power.c`.

## Registro en la SD1 (`App/aspid_data.c`)

Cada arranque crea una carpeta en la SD1:

```
aspid_data_2026-10-05_14-30-12/      fecha y hora UTC del arranque (del GPS)
  sesion.txt                         configuracion al arrancar, sesgos del EKF,
                                     eventos importantes y donde cayo
  eventos/eventos.csv                todos los eventos, uno por linea
  salud/salud.csv                    1 Hz desde el arranque
  telemetria/telemetria.csv          cada paquete enviado por radio y cada uno recibido
  vuelo/vuelo_01.csv                 100 Hz, del despegue a los 50 m de bajada
  trust_curve/trust_curve_01.csv     las dos IMU a 400 Hz durante el motor
  simulaciones/sim_01.csv            lo mismo durante un SIMULAR RUTA
  simulaciones/sim_01_trust_curve.csv
```

- **Fecha:** la core no tiene reloj; la fecha sale del GPS. Hasta que llega,
  la carpeta es `aspid_data_sin_fecha_NNNN` (número de arranque de la SD1) y
  se renombra en cuanto el GPS da fecha, solo en tierra. Todas las filas
  llevan `t_ms` (ms desde el arranque) y, si ya se sabe, `hora_utc`.
- **Vuelo:** empieza cuando el GNC pasa a EN VUELO o una IMU mide más de 3 g
  durante 100 ms, con el segundo de antes. Termina al bajar de 50 m tras el
  apogeo, con el GNC EN TIERRA o a los 20 min. Si pasó de 100 m **se cierra
  todo y la SD no se vuelve a tocar** en ese arranque.
- **Trust curve:** una fila por tick del GNC (400 Hz) con lo último de las
  dos IMU, su marca de tiempo de la aviónica y cuántas muestras nuevas trae
  (hoy las IMU van a ~200 Hz y media fila de cada dos repite). Desde 0,5 s
  antes del despegue hasta 5 s después de que el GNC dé el motor por apagado
  (como mucho 30 s).
- **`sesion.txt`:** la ficha del arranque (firmware, protocolo, SD, placas,
  qué sensores arrancaron, rampa), los sesgos del EKF al alinearse y al
  despegar, los eventos importantes y, al terminar un vuelo, **dónde cayó**:
  la última posición GPS buena con un enlace al mapa (se apunta a 50 m de
  altura bajando, así que el cohete acaba unos metros más allá) y la
  posición del GNC respecto a su origen.
- **`eventos/eventos.csv`** (`t_ms,hora_utc,tipo,valor,detalle`): cambios de
  estado, fase, error y navegación del GNC; pirotecnia armada; peticiones de
  drogue y main del GNC y si power aceptó cada orden; continuidad; modo test;
  pirotécnicos manuales; simulación; pruebas de aerofreno; rampa; cámaras;
  placas que dejan de responder; resets de la aviónica; primer fix del GPS; y
  cada segundo en que se pierden ticks del GNC, hay errores de CAN, fallos de
  radio o datos que no caben en la SD.
- **`salud/salud.csv`** (1 Hz): vueltas del bucle y **la más larga**
  (`bucle_max_ms`: si algo bloquea, sea lo que sea, sale aquí); ticks del
  GNC, el más lento, perdidos y recuperados; Hz de cada sensor; tramas y
  errores de los dos CAN; contadores de radio y de subida; placas; raíles de
  power; continuidad; escrituras y pérdidas de la SD; cámaras.
- **Telemetría:** tipo y bytes de cada paquete que sale (vuelo, estado,
  acuse); de cada uno que llega, los bytes, RSSI/SNR/error de frecuencia con
  que lo oyó el cohete y qué se hizo con él (ejecutado, denegado y por qué...).
- **Nada bloquea, ni con la tarjeta muerta:** en tierra los archivos se crean
  y se **reservan** (vuelo 64 MB, trust curve 8 MB, telemetría por tramos).
  Luego los datos van por DMA directamente a esos sectores, sin FileX ni el
  HAL: `sd_card_write_start()` / `sd_card_write_poll()` llevan la escritura a
  mano sobre los registros del SDMMC y nunca esperan (el HAL espera cada
  respuesta hasta ~14 s si la tarjeta no contesta: medido). Cada segundo se
  actualiza también el tamaño de cada archivo en su carpeta. Si una escritura
  falla o tarda más de lo que permite la norma, la tarjeta se da por perdida y
  no se toca más; el bucle sigue. Probado desconectando la SD1 en pleno vuelo
  de prueba: se detecta en el acto y el GNC no pierde ni un tick.
- **Reservas:** lo que sobra se libera al cerrar en tierra. Al terminar un
  vuelo de verdad no (bloquearía ~50 ms), y si se corta la corriente antes de
  cerrar tampoco: lo libera el arranque siguiente (`aspid_reservas.txt`).
- **Pruebas en la mesa** (`aspid_data.h`, dejar a 0 para volar):
  `ASPID_DATA_PRUEBA` (fecha falsa y despegue forzado, relee todo al sellar),
  `ASPID_DATA_RELEER` (relee cada archivo al cerrarlo) y
  `ASPID_DATA_PRUEBA_MUERTE` (desconecta la SD1 en pleno vuelo de prueba).
