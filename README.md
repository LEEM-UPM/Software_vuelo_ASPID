# Software de vuelo ASPID

Firmware de las cuatro placas del cohete ASPID (LEEM-UPM), del puente de radio
de tierra y de la estación de tierra.

```
                         CAN2 (sensores)
   aviónica G4 ────────────────────────────── core H7 ── SD1 / SD2, USB (consola)
       │                                        │
       └───────────── CAN1 (1 Mbit/s, CAN FD) ──┼── power G4 (pirotecnia, raíles)
                                                │
                                                └── RF G4 ))) LoRa 869,525 MHz )))
                                                                       │
                                     estación (Python) ── USB ── puente DX-PJ26
```

| Carpeta | Qué es | Herramientas |
|---|---|---|
| [`core_h7_v0/`](core_h7_v0) | Ordenador de vuelo (STM32H723): GNC ASPID v1.5 a 400 Hz, despliegue, aerofreno, modo de emergencia tras un reinicio en vuelo, telemetría, registro en SD y cámaras | CMake + STM32CubeCLT |
| [`power_g4_v0/`](power_g4_v0) | Placa de potencia (STM32G473): pirotécnicos, continuidad y raíles | CMake + STM32CubeCLT |
| [`sensores_avionica_g4_v0/`](sensores_avionica_g4_v0) | Aviónica (STM32G473): barómetros, IMU, magnetómetro, GPS, VN-100 y servo del aerofreno | CMake + STM32CubeCLT |
| [`rf_g4_v0/`](rf_g4_v0) | Placa de radio (STM32G473 + SX1262): pone en el aire lo que manda el core y le pasa los comandos de tierra | CMake + STM32CubeCLT |
| [`can_protocol/`](can_protocol) | Protocolo del bus CAN1 común: [`ICD.md`](can_protocol/ICD.md) y cabeceras | — |
| [`puente_radio_tierra/`](puente_radio_tierra) | Puente USB ↔ LoRa de la estación (DX-PJ26: Blue Pill + DX-LR30) | Arduino IDE + RadioLib |
| [`estacion_tierra/`](estacion_tierra) | Interfaz de la estación de tierra: telemetría, mando, registro | Python 3 |

## Compilar y grabar

**Placas STM32** (cada carpeta compila sola):

```
cd core_h7_v0
cmake --preset Debug
cmake --build build/Debug
STM32_Programmer_CLI -c port=SWD mode=UR -w build/Debug/core_h7_v0.elf -v -rst
```

Hace falta [STM32CubeCLT](https://www.st.com/en/development-tools/stm32cubeclt.html)
(arm-none-eabi-gcc, CMake, Ninja y STM32CubeProgrammer). Las placas G4 igual,
cada una con su `.elf`. Antes de grabar, comprobar el Device ID que dice el
programador: 0x483 es el core (H7), 0x469 una G4.

**Puente de tierra:** abrir `puente_radio_tierra/LEEM_Puente_Radio` en el
Arduino IDE (núcleo STM32 de STM32duino, RadioLib 7.7.1). Ver su README.

**Estación:** `python estacion_tierra/estacion.py` (o `--demo` sin
hardware). Necesita tkinter y `pip install pyserial`.

## El protocolo CAN1

Lo comparten las cuatro placas. La referencia es [`can_protocol/`](can_protocol)
(repositorio [LEEM-UPM/can_protocol](https://github.com/LEEM-UPM/can_protocol));
cada placa lleva una copia idéntica en `third_party/can_protocol` para
compilar sola. Si se cambia el protocolo, cambiarlo en las cinco.

## Antes de volar

Los modos de prueba se activan al compilar y algunos **disparan pirotecnia o
bloquean el arranque**. Todos tienen que estar a 0 (en esta versión lo están):

| Placa | Define | Dónde |
|---|---|---|
| core | `GNC_PLAYBACK_ENABLE` | `App/gnc_playback.h` |
| core | `PIL_TEST_ENABLE` | `App/pil_test.h` |
| core | `EMERGENCIA_PRUEBA` | `App/emergencia.h` |
| core | `CAN_DRIVER_PRUEBA_BUS_OFF` | `Drivers/Can_driver/can_driver.h` |
| core | `SD_BENCH_ENABLE`, `ASPID_DATA_PRUEBA`, `CAMARAS_PRUEBA_UART` | `App/` |
| power | `PYRO_PRUEBA_ARRANQUE` | `Drivers/Pyro/pyro.h` |

Lo pendiente de todas las placas está en
[`core_h7_v0/PENDIENTES.txt`](core_h7_v0/PENDIENTES.txt).

## De dónde sale

Copia del 2026-10-08 de los repositorios de trabajo de cada placa (core
`Recepcion-datos-bus-avionica`, power `flight-software-develop`, aviónica
`flight_software`, RF `aplication`), con lo que aún no estaba commiteado en
ellos. Diferencias con esos repositorios:

- La estación de tierra está en `estacion_tierra/`; allí vivía en
  `core_h7_v0/tools/estacion`, y la documentación del core la sigue citando
  con esa ruta.
- `third_party/can_protocol` es una copia, no un submódulo.
- De power se ha quitado `Drivers/CMSIS/DSP` (46 MB que no se usan).

El código de terceros mantiene su licencia: HAL y CMSIS de ST, ThreadX y
FileX (Eclipse), WE Sensors SDK, driver de ST del LSM6DSV32X, minmea,
RadioLib, y el GNC, generado con MATLAB/Simulink (`core_h7_v0/third_party/aspid_gnc`).
