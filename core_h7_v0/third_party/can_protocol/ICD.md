# ICD — Protocolo CAN1

Documento de control de interfaz (*Interface Control Document*) del bus CAN1
entre las placas de la aviónica modular. **Es la referencia:** si el código de
una placa y este documento no coinciden, el error está en el código.

## 1. Versión

Versión actual: **0.2** (ver historial al final).

La versión se define en `include/can_protocol/can_protocol_version.h`.

| Número | Cuándo sube | Ejemplos |
|--------|-------------|----------|
| MAJOR | Cambio incompatible: una placa con la versión anterior interpretaría mal los datos. Al subir MAJOR, MINOR vuelve a 0. | Mover o quitar un campo, cambiar un tipo o una unidad, reutilizar un ID con otro significado |
| MINOR | Cambio compatible: una placa antigua puede ignorarlo sin problema. | Añadir un mensaje nuevo, usar un byte que antes era relleno |

- Todo cambio de versión va en el mismo PR que el cambio del protocolo y se
  anota en el historial (sección 7).
- Codificación en las tramas: `uint16` con MAJOR en el byte alto y MINOR en el
  bajo (`CAN_PROTOCOL_VERSION`). Se enviará en el heartbeat de cada nodo.
- Pendiente de decidir: regla de compatibilidad mientras MAJOR = 0 (exigir
  también el mismo MINOR, o pasar a 1.0 con el primer mensaje real probado).

## 2. Capa física

### 2.1 Parámetros del bus

| Parámetro | Valor |
|-----------|-------|
| Estándar | CAN FD ISO 11898-1:2015 (ISO CAN FD, no "non-ISO") |
| Formato de trama | CAN FD **sin BRS** (toda la trama a velocidad nominal) |
| Velocidad nominal | **1 Mbit/s** |
| Punto de muestreo | **87,5 %** (tolerancia ±1 %) |
| Tipo de identificador | Estándar, 11 bits |
| Tramas remotas | No se usan |
| Longitud máxima de datos | 32 bytes (pendiente de confirmar) |
| Retransmisión automática | TODO: definir por nodo (ver estado actual en 2.2) |
| Bus-off | **Cada nodo sale solo** (ver abajo) |

Todos los nodos deben usar exactamente la misma velocidad y el mismo punto de
muestreo, con una desviación máxima de ±1 % en este último (con un número
entero de tq por bit no siempre se puede dar el valor exacto). Estos valores
están en `include/can_protocol/can_bus_config.h`.

Motivo de la elección (2026-09-14):

- **1 Mbit/s:** es el máximo que contempla ISO 11898 para la fase de
  arbitraje. Sin BRS toda la trama va a esa velocidad, así que ir más rápido
  exigiría un bus muy corto y transceptores muy rápidos.
- **87,5 %:** todas las placas lo alcanzan con su reloj FDCAN actual (16, 32 y
  100 MHz), sin tocar los PLL que comparten otros periféricos. Un 80 % no es
  alcanzable con 16 ni 32 MHz dentro de la tolerancia.

**Bus-off (decidido 2026-10-07).** Un nodo cuyo contador de errores de
transmisión pasa de 255 (unas 32 tramas fallidas seguidas: un mal contacto,
CANH y CANL en corto, ruido) se desconecta solo del bus. En el FDCAN de los
STM32 el periférico se queda entonces en INIT y **no vuelve por su cuenta**:
sin hacer nada, una placa se quedaría sorda hasta apagar el cohete (power sin
pirotecnia, RF sin telemetría, aviónica sin sensores). Por eso **todos los
nodos miran `PSR.BO` en cada vuelta de su bucle principal, también mientras
esperan el wake up, y si está a 1 con `CCCR.INIT` a 1, quitan `CCCR.INIT`**. El
FDCAN espera entonces 128 ratos de bus libre (~1,4 ms a 1 Mbit/s) y se
reincorpora. Cada placa cuenta las veces. Probado en el core tirando el bus
a bus-off con el transceptor en standby: vuelve en el acto y el tráfico
sigue.

Un nodo con el controlador en modo CAN clásico (no FD) no puede conectarse a
este bus: vería las tramas FD como error y generaría error frames. Un nodo en
modo FD sí puede enviar y recibir tramas en formato clásico.

### 2.2 Tiempos de bit por nodo

El prescaler y los segmentos dependen del reloj FDCAN de cada micro, así que no
son iguales en todas las placas. Cada una debe dar 1 Mbit/s y un muestreo del
87,5 % ±1 %. Los tiempos de la fase de datos no se usan (sin BRS).

| Nodo | MCU | Periférico | Reloj FDCAN | Prescaler | Sync + Seg1 + Seg2 | tq/bit | SJW | Muestreo | Retransmisión |
|------|-----|------------|-------------|-----------|--------------------|--------|-----|----------|---------------|
| Core | STM32H723 | FDCAN1 | 100 MHz (PLL2Q) | 4 | 1 + 21 + 3 | 25 | 3 | 88 % | Activada |
| Power | STM32G473 | FDCAN1 | 32 MHz (PLLQ) | 2 | 1 + 13 + 2 | 16 | 2 | 87,5 % | Activada |
| Aviónica | STM32G473 | FDCAN1 | 100 MHz (PCLK1) | 4 | 1 + 21 + 3 | 25 | 3 | 88 % | Desactivada |
| RF | STM32G473 | FDCAN1 | 16 MHz (PCLK1) | 1 | 1 + 13 + 2 | 16 | 2 | 87,5 % | Activada |

Comprobación: velocidad = reloj / (prescaler × tq/bit);
muestreo = (1 + Seg1) / tq/bit.

Cada placa lo verifica con `CAN_BUS_TIMING_IS_VALID()` de `can_bus_config.h`.
La forma recomendada es en tiempo de ejecución, justo después de inicializar
el periférico (en el bloque `USER CODE BEGIN FDCAN1_Init 2` de `fdcan.c`),
porque usa el reloj y los valores reales y detecta también un cambio al
regenerar con CubeMX:

```c
if ((hfdcan1.Init.FrameFormat != FDCAN_FRAME_FD_NO_BRS) ||
    !CAN_BUS_TIMING_IS_VALID(HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN),
                             hfdcan1.Init.NominalPrescaler,
                             hfdcan1.Init.NominalTimeSeg1,
                             hfdcan1.Init.NominalTimeSeg2)) {
  Error_Handler();
}
```

Un nodo mal configurado puede tirar el bus entero, así que es preferible que
se detenga al arrancar.

### 2.3 Conexión en cada nodo

| Nodo | Periférico | RX | TX | Transceptor | Pin standby |
|------|------------|----|----|-------------|-------------|
| Core | FDCAN1 | PB8 | PB9 | TCAN1044 | PD4 `CAN1_STB` (LOW = normal, HIGH = standby) |
| Power | FDCAN1 | PA11 | PA12 | TODO | No tiene pin de standby en el `.ioc` |
| Aviónica | FDCAN1 | PA11 | PA12 | TODO | PB1 `CAN1_STB` |
| RF | FDCAN1 | PA11 | PA12 | TODO | PA9 `FDCAN1_STB` |

### 2.4 Cableado y terminación

- Topología en bus lineal (daisy chain), sin estrellas.
- Terminación de **120 Ω en los dos extremos físicos** del bus y en ningún otro
  nodo. Con el bus sin alimentar, entre CANH y CANL deben medirse ~60 Ω.
- Derivaciones (stubs) desde el bus a cada placa lo más cortas posible.
- Cable de par trenzado, impedancia característica ~120 Ω.
- TODO: conector y asignación de pines (CANH, CANL, GND).
- TODO: qué nodos están en los extremos y llevan la terminación.
- TODO: longitud total aproximada del bus.

## 3. Codificación de datos

Reglas comunes a todos los mensajes, pensadas para que el protocolo funcione
igual en cualquier micro (STM32, ESP32, PC...):

| Regla | Motivo |
|-------|--------|
| Todos los campos multibyte en **little-endian** | El orden de bytes en memoria depende del micro |
| Solo tipos de **tamaño fijo**: `uint8_t`, `int16_t`, `uint32_t`... (`<stdint.h>`, estándar C99) | `int` puede ser de 16 o 32 bits según el micro |
| Números reales como **`float32` IEEE-754** | Formato común a prácticamente todos los micros actuales |
| Serializar **campo a campo** con `can_pack.h`; nunca enviar un struct con `memcpy` ni usar `packed` | Cada compilador puede meter padding distinto entre campos |
| Booleanos como `uint8_t`: 0 = falso, 1 = verdadero | `bool` no tiene tamaño garantizado |
| Bytes de relleno a **0** al transmitir e **ignorados** al recibir | La longitud CAN FD salta a 12, 16, 20, 24, 32... |
| El receptor saca el número de campos del **ID del mensaje**, no de la longitud de la trama | Por el relleno, la longitud no indica cuántos campos hay |

Tipos disponibles en `include/can_protocol/can_pack.h` (funciones
`can_put_<tipo>` / `can_get_<tipo>`):

| Tipo | Bytes | Función |
|------|-------|---------|
| `uint8_t` / `int8_t` | 1 | `u8` / `i8` |
| `bool` | 1 | `bool` (0 = falso, 1 = verdadero; al leer, ≠ 0 es verdadero) |
| `uint16_t` / `int16_t` | 2 | `u16_le` / `i16_le` |
| `uint32_t` / `int32_t` | 4 | `u32_le` / `i32_le` |
| `uint64_t` | 8 | `u64_le` |
| `float` (IEEE-754) | 4 | `f32_le` |

Excepción conocida: los micros sin tipo de 8 bits exactos (p. ej. DSP TI C2000,
con `char` de 16 bits) no definen `uint8_t` y necesitarían una adaptación.

## 4. Nodos

Número de nodo de 4 bits, definido en `include/can_protocol/can_ids.h`.

| Node ID | Macro | Placa | Función |
|---------|-------|-------|---------|
| 0x0 | `CAN_NODE_BROADCAST` | — | Solo como destino de comandos: todos los nodos |
| 0x1 | `CAN_NODE_CORE` | Core H7 | Ordenador de vuelo, orquesta el bus |
| 0x2 | `CAN_NODE_POWER` | Potencia | TODO |
| 0x3 | `CAN_NODE_AVIONICS` | Aviónica | TODO |
| 0x4 | `CAN_NODE_RF` | Radiofrecuencia | TODO |
| 0x5–0xF | — | Libres | — |

## 5. Mapa de identificadores

Reglas generales:

- En CAN el ID es también la prioridad: **el ID más bajo gana el arbitraje**.
  Los rangos se reparten por importancia, no por orden de creación.
- **Cada ID tiene un único nodo transmisor.** Si dos nodos envían el mismo ID a
  la vez, ambos ganan el arbitraje y colisionan en el campo de datos.
- Ninguna placa escribe IDs a mano: todos se construyen con `CAN_ID()` de
  `include/can_protocol/can_ids.h`.

### 5.1 Estructura del ID

```
 bit  10  9  8 | 7  6  5  4 | 3  2  1  0
     [ clase  ] [   nodo   ] [ mensaje  ]
```

| Campo | Bits | Significado |
|-------|------|-------------|
| Clase | 10–8 | Prioridad del mensaje. Manda sobre el resto en el arbitraje |
| Nodo | 7–4 | Placa **origen**, salvo en comandos, donde es la placa **destino** |
| Mensaje | 3–0 | Número de mensaje dentro de esa clase y ese nodo (0–15) |

Así cada ID tiene un único transmisor: el nodo indicado, o el core en los
comandos (el único que envía comandos). Por eso no existen comandos con
nodo = `CAN_NODE_CORE`.

Prioridad resultante:

1. Primero decide la **clase**: cualquier mensaje crítico gana a cualquier
   comando, cualquier comando a cualquier respuesta, etc.
2. Dentro de una misma clase decide el **nodo**: a menor número, más prioridad
   (core > power > avionics > rf). En comandos, los de broadcast (nodo 0) ganan
   a los dirigidos a una placa.
3. Por último, el **número de mensaje**: el 0 es el más prioritario.

### 5.2 Clases

| Clase | Rango de IDs | Macro | Uso |
|-------|--------------|-------|-----|
| 0 | 0x000–0x0FF | `CAN_CLASS_CRITICAL` | Emergencias y abort |
| 1 | 0x100–0x1FF | `CAN_CLASS_COMMAND` | Core → nodo (nodo = destino) |
| 2 | 0x200–0x2FF | `CAN_CLASS_RESPONSE` | Nodo → core, respuesta a un comando |
| 3 | 0x300–0x3FF | `CAN_CLASS_TELEMETRY` | Datos periódicos de cada nodo |
| 4–5 | 0x400–0x5FF | — | Reservadas |
| 6 | 0x600–0x6FF | `CAN_CLASS_HEARTBEAT` | Estado y versión de cada nodo |
| 7 | 0x700–0x7FF | `CAN_CLASS_DEBUG` | Diagnóstico, no necesario en vuelo |

### 5.3 Filtros por hardware

Con filtros de tipo "ID + máscara":

| Qué aceptar | ID | Máscara |
|-------------|----|---------|
| Todos los mensajes de una clase | `CAN_ID(clase, 0, 0)` | `CAN_ID_CLASS_MASK` (0x700) |
| Todo lo que lleva un nodo en el campo nodo | `CAN_ID(0, nodo, 0)` | `CAN_ID_NODE_MASK` (0x0F0) |
| Una clase de un nodo concreto | `CAN_ID(clase, nodo, 0)` | 0x7F0 |

Ojo con el filtro por nodo: como en los comandos el campo nodo es el
**destino**, `CAN_ID(0, nodo, 0)` con máscara 0x0F0 acepta lo que envía ese
nodo **y también** los comandos que el core le manda.

Filtros típicos de una placa (que no sea el core) para recibir sus comandos:

| Filtro | ID | Máscara |
|--------|----|---------|
| Comandos dirigidos a esta placa | `CAN_ID(CAN_CLASS_COMMAND, mi_nodo, 0)` | 0x7F0 |
| Comandos de broadcast | `CAN_ID(CAN_CLASS_COMMAND, CAN_NODE_BROADCAST, 0)` | 0x7F0 |
| Mensajes críticos de cualquier nodo | `CAN_ID(CAN_CLASS_CRITICAL, 0, 0)` | 0x700 |

Si la placa tiene un solo filtro libre y su número de nodo tiene **un único
bit a 1** (power 0x2, rf 0x4), los comandos de broadcast y los dirigidos a
ella caben en un filtro: se quita ese bit de la máscara.

| Placa | ID | Máscara | Acepta |
|-------|----|---------|--------|
| Power | 0x100 | 0x7D0 | 0x100–0x10F y 0x120–0x12F |
| RF | 0x100 | 0x7B0 | 0x100–0x10F y 0x140–0x14F |

Aviónica (0x3, dos bits a 1) no puede: con un solo filtro aceptaría también
los comandos a otros nodos, y necesita dos.

## 6. Mensajes

### 6.1 0x100 — CMD_WAKE_UP

- Transmisor: core / Receptor(es): todas las placas (broadcast)
- Periodo: 10 Hz al arrancar el core, hasta que respondan todas las placas
  (6.2) o pasen 5 s; despues 1 Hz mientras falte alguna por responder, y
  **0,5 Hz como latido aunque esten todas** / Longitud: 2 bytes
- Macro: `CAN_ID_CMD_WAKE_UP`

| Byte | Campo | Tipo | Unidad | Descripción |
|------|-------|------|--------|-------------|
| 0–1 | protocol_version | `uint16` | — | `CAN_PROTOCOL_VERSION` del core |

Cada placa espera al arrancar en un bucle hasta recibir este mensaje, y
entonces empieza a ejecutar su código.

Una placa que arranca tarde, o que **se reinicia en cualquier momento**, se
recupera sola: el core no deja nunca de mandar el mensaje.

**El latido no es un adorno.** Hasta el 2026-10-01 el core dejaba de mandar
wake ups en cuanto las tres placas habían contestado una vez, y una placa que
se reiniciara después se quedaba esperando uno que no iba a llegar nunca. Se
vio al regrabar la placa de RF: el core seguía encolando paquetes con cero
errores y diciendo `rf OK` mientras la RF los tiraba todos por no estar
despierta. En vuelo, una RF que se reinicie por un bajón de tensión se
llevaría la telemetría para siempre.

Repetirlo es barato. Las tres placas **contestan a todos** los wake ups, no
sólo al primero: el WAKE_UP_ACK (6.2) es incondicional. Además la aviónica
reenvía su INIT_STATUS (6.11) con cada uno; power y rf no, porque todavía no
lo implementan. Ninguna reinicia sensores ni recalibra.

Coste: **5 tramas cada 2 s** (1 wake up + 3 ACK + 1 INIT_STATUS), es decir 2,5
tramas/s en un bus que mueve 2 por segundo. Cuando power y rf implementen
INIT_STATUS serán 7 cada 2 s.

Pendiente de decidir:

- Qué hace una placa si `CAN_PROTOCOL_IS_COMPATIBLE(protocol_version)` es
  falso (¿no responde y no arranca?).
- Cómo detecta el core que una placa que ya había respondido se ha reiniciado
  (haría falta un heartbeat o un timeout por placa): mientras todas figuren
  como despiertas, el core deja de reenviar el wake up.

### 6.2 0x220 / 0x230 / 0x240 — WAKE_UP_ACK

- Transmisor: power (0x220), avionics (0x230), rf (0x240) / Receptor(es): core
- Periodo: una respuesta por cada CMD_WAKE_UP recibido, también si la placa ya
  estaba despierta (así, si el core se resetea, vuelve a ver a todas) /
  Longitud: 0 bytes
- Macro: `CAN_ID_WAKE_UP_ACK(nodo)`; número de mensaje `CAN_MSG_WAKE_UP_ACK`

Sin datos. El core marca como despierta la placa del campo nodo del ID.

### 6.3 0x120 — CMD_PYRO_FIRE

- Transmisor: core / Receptor(es): power
- Periodo: bajo demanda / Longitud: 1 byte
- Macro: `CAN_ID_CMD_PYRO_FIRE`

| Byte | Campo | Tipo | Unidad | Descripción |
|------|-------|------|--------|-------------|
| 0 | channel | `uint8` | — | Canal a encender: 0x01 = 1A, 0x02 = 1B, 0x03 = 2A, 0x04 = 2B |

Comportamiento de power:

- Un mensaje enciende un solo canal. Para encender varios se envía un mensaje
  por canal.
- El canal se mantiene encendido 1 s y se apaga solo. Si llega otro
  CMD_PYRO_FIRE del mismo canal mientras está encendido, el tiempo vuelve a
  empezar.
- Se ignora si power todavía no ha recibido CMD_WAKE_UP (6.1), si el valor de
  `channel` no es uno de los cuatro de la tabla o si la trama trae 0 bytes.
- No hay respuesta de confirmación.
- **Canal 2B (0x04): hoy NO es un pirotécnico.** Alimenta el servo del
  aerofreno: power lo enciende al arrancar y lo deja encendido siempre, y
  un CMD_PYRO_FIRE con el canal 0x04 se ignora (`PYRO_2B_ES_SERVO` en
  `App/pyro_control.h` de power). Su continuidad se sigue mandando en 6.5.

### 6.4 0x121 — CMD_PYRO_CONTINUITY

- Transmisor: core / Receptor(es): power
- Periodo: 1 Hz, solo si power respondió al wake up (6.2) / Longitud: 0 bytes
- Macro: `CAN_ID_CMD_PYRO_CONTINUITY`

Sin datos. Pide a power que mida la continuidad de los cuatro canales y
responda con PYRO_CONTINUITY (6.5). Se ignora si power todavía no ha
recibido CMD_WAKE_UP (6.1).

### 6.5 0x221 — PYRO_CONTINUITY

- Transmisor: power / Receptor(es): core
- Periodo: una respuesta por cada CMD_PYRO_CONTINUITY (6.4) / Longitud: 1 byte
- Macro: `CAN_ID_PYRO_CONTINUITY`

| Byte | Campo | Tipo | Unidad | Descripción |
|------|-------|------|--------|-------------|
| 0 | continuity | `uint8` | — | Un bit por canal, 1 = hay continuidad: bit 0 = 1A, bit 1 = 1B, bit 2 = 2A, bit 3 = 2B. Bits 4–7 a 0 |

Que haya continuidad no garantiza que la resistencia de la línea sea la
correcta.

### 6.6 0x122 — CMD_POWER_RAILS

- Transmisor: core / Receptor(es): power
- Periodo: 1 Hz, solo si power respondió al wake up (6.2) / Longitud: 0 bytes
- Macro: `CAN_ID_CMD_POWER_RAILS`

Sin datos. Pide a power las tensiones y corrientes de sus raíles; responde
con POWER_RAILS (6.7). Se ignora si power todavía no ha recibido
CMD_WAKE_UP (6.1).

### 6.7 0x222 — POWER_RAILS

- Transmisor: power / Receptor(es): core
- Periodo: una respuesta por cada CMD_POWER_RAILS (6.6) / Longitud: 12 bytes
- Macro: `CAN_ID_POWER_RAILS`

| Byte | Campo | Tipo | Unidad | Descripción |
|------|-------|------|--------|-------------|
| 0–1 | current_3v3 | `uint16` | mA | Corriente del raíl de 3,3 V |
| 2–3 | current_5v | `uint16` | mA | Corriente del raíl de 5 V |
| 4–5 | current_11v | `uint16` | mA | Corriente del raíl de 11 V |
| 6–7 | voltage_3v3 | `uint16` | mV | Tensión del raíl de 3,3 V |
| 8–9 | voltage_5v | `uint16` | mV | Tensión del raíl de 5 V |
| 10–11 | voltage_11v | `uint16` | mV | Tensión del raíl de 11 V |

`0xFFFF` en un campo = lectura no válida (fallo del ADC en esa medida). Los
valores válidos van de 0 a 65 534.

### 6.8 0x131 — CMD_AVIONICS_CALIBRATE

- Transmisor: core / Receptor(es): avionics
- Periodo: bajo demanda, solo en tierra / Longitud: 4 bytes
- Macro: `CAN_ID_CMD_AVIONICS_CALIBRATE`

| Byte | Campo | Tipo | Unidad | Descripción |
|------|-------|------|--------|-------------|
| 0–3 | tilt_angle | `float32` | grados | Ángulo al que está inclinado el cohete respecto a la vertical durante la calibración |

Rutina (una sola, bloqueante, con todos los sensores muestreados a la vez
en el mismo bucle, no uno detrás de otro):

- Barómetros: presión media durante 1 s = presión en la altura base. Se
  envía al core; no se resta a las medidas.
- Giroscopios: media durante 2 s = offset, que se resta a las medidas.
- Acelerómetros: media durante 2 s menos la proyección de g esperada en cada
  eje para `tilt_angle` = offset, que se resta a las medidas. Hacia qué lado
  de la placa se inclina el cohete lo fija aviónica (`CALIBRATION_TILT_TOWARD`
  en `App/calibration.c`).

Comportamiento:

- Responde con AVIONICS_CALIBRATION (6.9) al terminar (~2 s).
- Mientras dura, aviónica no hace nada más (no envía telemetría ni atiende
  CAN).
- Aviónica aplica los offsets a sus lecturas. No se guardan en memoria no
  volátil: se pierden al resetear aviónica.
- Se ignora si aviónica todavía no ha recibido CMD_WAKE_UP (6.1).
- Es responsabilidad del core enviarlo solo con el cohete en tierra.
- El core espera la respuesta como máximo 5 s.

Pendiente de confirmar: el eje de la placa que apunta a la ogiva (se supone
+Z) y hacia qué lado se inclina el cohete.

### 6.9 0x231 — AVIONICS_CALIBRATION

- Transmisor: avionics / Receptor(es): core
- Periodo: una respuesta por cada CMD_AVIONICS_CALIBRATE (6.8), al terminar
  la rutina / Longitud: 32 bytes
- Macro: `CAN_ID_AVIONICS_CALIBRATION`

| Byte | Campo | Tipo | Unidad | Descripción |
|------|-------|------|--------|-------------|
| 0–1 | imu1_accel_x | `int16` | 0,001 m/s² | Offset del acelerómetro de la IMU 1 (WSEN-ISDS), eje X |
| 2–3 | imu1_accel_y | `int16` | 0,001 m/s² | Ídem, eje Y |
| 4–5 | imu1_accel_z | `int16` | 0,001 m/s² | Ídem, eje Z |
| 6–7 | imu1_gyro_x | `int16` | 0,0001 rad/s | Offset del giroscopio de la IMU 1, eje X |
| 8–9 | imu1_gyro_y | `int16` | 0,0001 rad/s | Ídem, eje Y |
| 10–11 | imu1_gyro_z | `int16` | 0,0001 rad/s | Ídem, eje Z |
| 12–13 | imu2_accel_x | `int16` | 0,001 m/s² | Offset del acelerómetro de la IMU 2 (LSM6DSV32X), eje X |
| 14–15 | imu2_accel_y | `int16` | 0,001 m/s² | Ídem, eje Y |
| 16–17 | imu2_accel_z | `int16` | 0,001 m/s² | Ídem, eje Z |
| 18–19 | imu2_gyro_x | `int16` | 0,0001 rad/s | Offset del giroscopio de la IMU 2, eje X |
| 20–21 | imu2_gyro_y | `int16` | 0,0001 rad/s | Ídem, eje Y |
| 22–23 | imu2_gyro_z | `int16` | 0,0001 rad/s | Ídem, eje Z |
| 24–27 | baro1_pressure | `float32` | kPa | Presión media en la altura base, barómetro 1 |
| 28–31 | baro2_pressure | `float32` | kPa | Presión media en la altura base, barómetro 2 |

Valor no calibrado: `0x7FFF` en los campos `int16` y NaN en los `float32`.
Rangos válidos: ±32,767 m/s² en acelerómetro y ±3,2767 rad/s en giroscopio
(hasta 0x7FFE).

Cada mensaje nuevo se añade con esta plantilla:

### Plantilla: 0xXXX — NOMBRE_MENSAJE

- Transmisor: / Receptor(es):
- Periodo: / Longitud:

| Byte | Campo | Tipo | Unidad | Descripción |
|------|-------|------|--------|-------------|

### 6.10 0x140 — CMD_RF_TRANSMIT

- Transmisor: core / Receptor(es): rf
- Periodo: bajo demanda / Longitud: 1 a 32 bytes
- Macro: `CAN_ID_CMD_RF_TRANSMIT`

| Byte | Campo | Tipo | Unidad | Descripción |
|------|-------|------|--------|-------------|
| 0–n | payload | `uint8[]` | — | Bytes a transmitir por radio, tal cual |

Los datos de la trama son el paquete de radio: la placa de RF los copia sin
interpretarlos ni añadir cabeceras, y los transmite en un único paquete LoRa
(868,0 MHz, SF7, BW125; ver el firmware de rf). El contenido y su formato los
decide el core.

La RF solo guarda el último comando: si llega otro antes de haber transmitido
el anterior, el viejo se descarta (el paquete de 12 bytes tarda unos 41 ms en
salir). Un comando recibido antes del wake up se ignora. No hay confirmación.

Pendiente de decidir:

- Ciclo de trabajo: en 868,0–868,6 MHz el límite es del 1 %. Con paquetes de
  ~41 ms, el core no debería mandar más de uno cada 4,1 s.
- ~~Recepción por radio (tierra → cohete)~~: definida en 6.12 (RF_RECEIVED, 0x241).

### 6.12 0x241 — RF_RECEIVED

- Transmisor: rf / Receptor(es): core
- Periodo: bajo demanda / Longitud: 6 a 32 bytes
- Macro: `CAN_ID_RF_RECEIVED`

| Byte | Campo | Tipo | Unidad | Descripción |
|------|-------|------|--------|-------------|
| 0 | length | `uint8` | bytes | Largo del payload, 1 a 27 |
| 1 | rssi | `int8` | dBm | RSSI con el que la RF oyó el paquete |
| 2 | snr | `int8` | dB | SNR del paquete |
| 3–4 | freq_error | `int16` | Hz | Error de frecuencia (+ = tierra transmite por encima), saturado |
| 5–… | payload | `uint8[]` | — | Bytes recibidos por radio, tal cual |

La cabecera (desde el 2026-10-04) sirve para que tierra vea el enlace de
subida **desde el lado del cohete**: el core la devuelve en el acuse del
comando. Lleva la longitud porque CAN FD rellena la trama hasta el siguiente
tamaño válido y sin ella no se sabría dónde acaba el paquete.

**Formato anterior** (sin cabecera, el payload pelado desde el byte 0): el core
lo sigue aceptando mientras dure la transición. Se distinguen por el byte 0:
en el nuevo es la longitud (< 0x40); en el anterior es el primer byte del
paquete, que en un comando de tierra es 0x5A.

Es el **camino de subida** tierra → cohete, el simétrico de CMD_RF_TRANSMIT.
La placa de RF está en recepción continua siempre que no esté transmitiendo, y
cada paquete LoRa que pasa el CRC lo reenvía por aquí sin interpretarlo. Los
que no pasan el CRC se descartan y no llegan al core: subir bytes corruptos al
core sólo le daría trabajo para acabar tirándolos.

Paquetes de más de 27 bytes se descartan (con la cabecera no caben en el
límite de 32 del ICD). Hoy los comandos son de 4 y 5 bytes.

**MIENTRAS TRANSMITE, LA RADIO ESTÁ SORDA.** Un paquete de 24 bytes ocupa el
aire 30,9 ms y en ese rato no se oye nada. A 15 Hz de telemetría eso es casi
la mitad del tiempo, así que un comando puede perderse sin más. Quien mande
comandos tiene que repetirlos hasta ver el efecto; no hay acuse en este
camino.

### 6.11 0x22F / 0x23F / 0x24F — INIT_STATUS

- Transmisor: power (0x22F), avionics (0x23F), rf (0x24F) / Receptor(es): core
- Periodo: una vez tras cada CMD_WAKE_UP atendido, cuando la placa ya sabe
  que ha arrancado / Longitud: 8 bytes
- Macro: `CAN_ID_INIT_STATUS(nodo)`; número de mensaje `CAN_MSG_INIT_STATUS`

| Byte | Campo | Tipo | Unidad | Descripción |
|------|-------|------|--------|-------------|
| 0–3 | ok | `uint32` | — | Componentes que han arrancado bien, un bit cada uno |
| 4–7 | supported | `uint32` | — | Componentes que esa placa tiene, un bit cada uno |

Cada placa dice con qué cuenta sin que el core pregunte. `supported` marca los
bits que significan algo en esa placa: un bit a 0 en `ok` solo es un fallo si
el mismo bit está a 1 en `supported`.

Bits de avionics (0x23F):

| Bit | Componente |
|-----|------------|
| 0 | Barómetro 1 (WSEN-PADS, I2C1) |
| 1 | Barómetro 2 (WSEN-PADS, I2C2) |
| 2 | IMU 1 (WSEN-ISDS, I2C1) |
| 3 | IMU 2 (LSM6DSV32X, I2C2) |
| 4 | Magnetómetro (MLX90393, I2C1) |
| 5 | GPS (MAX-M10S, LPUART1) |
| 6 | VN-100 (USART2) |

Los bits de power (0x22F) y rf (0x24F) están sin definir: esas placas todavía
no envían el mensaje.

Un componente que se cae más tarde no genera otro INIT_STATUS: el mensaje es
del arranque. Lo que pase después se ve en su telemetría (si deja de llegar) y,
cuando exista, en el comando que pregunte por el estado actual.

## 7. Historial de cambios

| Versión | Fecha | Cambio |
|---------|-------|--------|
| 0.1 | 2026-09-14 | Estructura inicial: versionado, capa física (1 Mbit/s, CAN FD sin BRS, muestreo 87,5 % ±1 %) con tiempos de bit de cada placa y su verificación, reglas de codificación de datos y tipos de `can_pack.h`, nodos (core, power, avionics, rf) y estructura de IDs (clase 3 bits + nodo 4 bits + mensaje 4 bits) |
| — | 2026-10-07 | Requisito de nodo, sin cambios en las tramas (misma versión): todos los nodos salen solos de bus-off (2.1) |
| 0.2 | 2026-09-14 | Añade CMD_WAKE_UP (0x100) y WAKE_UP_ACK (0x220, 0x230, 0x240) para el arranque de las placas; comandos de pirotecnia de power CMD_PYRO_FIRE (0x120), CMD_PYRO_CONTINUITY (0x121) y su respuesta PYRO_CONTINUITY (0x221); CMD_POWER_RAILS (0x122) y POWER_RAILS (0x222) con tensión y corriente de los raíles de 3,3, 5 y 11 V; CMD_AVIONICS_CALIBRATE (0x131) y AVIONICS_CALIBRATION (0x231) para la calibración en tierra de IMUs y barómetros; filtro único para broadcast + comandos propios en nodos con un solo bit a 1; CMD_RF_TRANSMIT (0x140) para transmitir por radio bytes que manda el core; INIT_STATUS (0x22F, 0x23F, 0x24F) con los componentes que ha arrancado cada placa |
