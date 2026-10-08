# rf_g4_v0

Firmware de la placa de telemetría RF para el cohete de sondeo universitario LEEM (UPM).

## El sistema
 
La aviónica del cohete está compuesta por cuatro placas independientes que se comunican mediante un bus CAN:
 
| Placa | Responsabilidad |
|---|---|
| **RF** (este repositorio) | Telemetría RF en tiempo real a estación de tierra |
| **Core** | Cerebro del sistema, almacenamiento en µSD, toma de decisiones |
| **Sensórica** | IMU, GNSS, temperatura, humedad, giróscopo |
| **Power** | Gestión de alimentación, potencia y pirotécnicos |

### Hardware de la placa RF
 
- **MCU:** STM32G473CET6 (Cortex-M4 @ 170 MHz, 512 KB Flash, 128 KB RAM)
- **Radio sub-GHz:** Semtech SX1262 — LoRa 868 MHz, hasta +22 dBm
- **Radio 2.4 GHz:** Semtech SX1280 — LoRa/FLRC/BLE
- **Bus:** FDCAN × 2 (nodo de la red CAN del cohete)
- **Toolchain:** arm-none-eabi-gcc · STM32CubeMX · CMake · Ninja
## Firmware

Al arrancar, la placa entra en el bus CAN1 común, configura el SX1262 y espera
el `CMD_WAKE_UP` del core (`services/can_node`). Responde a **todos** los wake
ups que le llegan, no solo al primero, así que si el core se reinicia vuelve a
verla.

Ya despierta, el bucle principal de `app/main.c`:

| Qué | Dónde | Detalle |
|---|---|---|
| Atiende el CAN1 | `can_node_poll()` | Wake up y comandos de este nodo (0x140–0x14F) |
| Transmite lo que manda el core | `radio_transmit()` | `CMD_RF_TRANSMIT` (0x140, ICD 6.10): los bytes de la trama salen tal cual en un paquete LoRa, sin cabeceras ni interpretación |
| Baliza de prueba | `radio_beacon_update()` | Paquete de 12 bytes cada `RADIO_BEACON_PERIOD_MS` |
| Watchdog | `board_watchdog_refresh()` | IWDG de 1 s (`bsp/board.c`): una vuelta del bucle más larga reinicia la placa, que vuelve a esperar el wake up |
| Bus-off | `can_node_poll()` | Si el CAN1 cae en bus-off lo saca (`bsp_can1_recover_bus_off()`, ICD 2.1); también mientras espera el wake up. La consola lo dice |

Al arrancar, la consola dice por qué (`Arranque por: watchdog / software /
pin de reset / encendido...`). Un fallo (HardFault...) o un `Error_Handler()`
reinician la placa en vez de dejarla colgada.

De `CMD_RF_TRANSMIT` solo se guarda el último comando: si llega otro mientras
el anterior espera, se descarta el viejo (`can_node_radio_dropped_count()`).
Cada paquete de 12 bytes ocupa la radio unos 41 ms y `sx1262_transmit()`
bloquea ese tiempo.

**Cristal del SX1262:** la placa venía con dos condensadores de carga de 10 pF
(C37 y C38) junto al cristal. **Hay que quitarlos**: el SX1262 lleva los suyos
dentro, y con los dos juegos en paralelo el oscilador trabajaba al límite. El
resultado era que ni transmitía ni recibía bien: los paquetes llegaban con la
cabecera correcta pero el payload corrupto, con SNR de 0 dB a cualquier
potencia. Sin ellos: SNR +13 dB y ningún error de CRC.

El ajuste fino se hace con los condensadores internos (`radio.xtal_trim` en el
BSP, ~5,4 ppm por paso). Medido contra la estación de tierra, **0x0F** deja el
error en +0,1 ppm. Ese ajuste se pierde cada vez que el cristal arranca, así
que el driver deja el chip en reposo con el oscilador en marcha
(`STDBY_XOSC`) en vez de pararlo.

**Banda y ciclo de trabajo:** la radio trabaja en **869,525 MHz** (BW125, es
decir 869,4625–869,5875 MHz), dentro de la sub-banda **869,4–869,65 MHz**, que
permite **500 mW e.r.p. y 10 % de ciclo de trabajo** (ERC 70-03 / EN 300 220),
frente a los 25 mW y 1 % de 868,0–868,6 MHz. La baliza a 1 Hz gasta un 4,1 %;
lo que pida el core se suma a esa cuenta.

### Receptor (otro extremo del enlace)

`poc/radio_receiver/main.c` es el firmware de recepción: se programa en una
segunda placa RF, no toca el bus CAN y saca por la consola (LPUART1, 115200)
todo lo que recibe, con número de paquete, longitud, RSSI, SNR, el contenido
en texto y en hexadecimal. Usa la misma configuración de radio del BSP, así
que emisor y receptor no se pueden descuadrar.

Para compilarlo, en `CMakeLists.txt` comenta las líneas de `app/` y
descomenta `poc/radio_receiver/main.c`.

La recepción en la placa de vuelo (tierra → cohete, con su mensaje CAN hacia
el core) todavía no está implementada.

### Estación de tierra: puente USB ↔ radio

La estación de tierra **no es una `rf_g4_v0`**: es una DX-PJ26 v1.2 (Blue Pill
STM32F103C8T6 con un módulo DX-LR30) programada desde el Arduino IDE con
RadioLib. Su código vive fuera de los repos, en
`C:\Users\User\Documents\Arduino\LEEM_Puente_Radio\`.

Hace de **módem transparente**: no interpreta nada, sólo pasa bytes. Quien
decodifica el paquete de vuelo (`core_h7_v0/App/flight_packet.h`) es el PC.

```
radio -> serie   AA 55 | len | payload[len] | rssi | snr | df_lo df_hi | flags | crc8
serie -> radio   AA 55 | len | payload[len] | crc8
```

| campo | tipo | unidad |
|---|---|---|
| `rssi` | int8 | dBm |
| `snr` | int8 | pasos de 0,25 dB |
| `df` | int16 LE | Hz, error de frecuencia con signo |
| `crc8` | — | polinomio 0x07, sobre `len` + payload + metadatos |

`flags`: bit 0, el CRC del paquete LoRa venía mal; bit 1, **acuse local** (la
trama la genera la propia placa, no viene del aire); bit 2, junto al bit 1, la
transmisión falló.

Tres decisiones que conviene entender antes de tocarlo:

- **Va enmarcado y no en crudo** porque la serie es un flujo de bytes y la
  radio son paquetes. Sin una longitud explícita, un solo byte perdido
  desincroniza el flujo para siempre; con el sincronismo se reengancha en el
  paquete siguiente.
- **Los paquetes con CRC malo también se reenvían**, marcados en `flags`. En
  una prueba de alcance, distinguir «llegó corrupto» de «no llegó nada» es
  justo la señal que marca el borde de la cobertura.
- **Cada transmisión devuelve un acuse** (`len = 0`, bit 1), porque si no el
  operador manda un comando y no sabe si llegó siquiera a radiarse. El enlace
  de subida es el lado débil. Un decodificador que sólo quiera telemetría lo
  ignora solo: longitud cero, sin payload.

Mientras transmite, la radio está sorda unos 15 ms (payload de 4 bytes a
SF7/BW250), así que un envío desde tierra puede costar un paquete de
telemetría. Con el tráfico previsto no se nota.

Todo lo que el PC mande sale al aire sin filtro: **el firmware no vigila el
ciclo de trabajo**, y eso depende de quien escriba el lado del PC.
