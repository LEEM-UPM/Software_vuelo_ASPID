# Protocolo de radio del cohete

Cómo hablar con el cohete desde tierra: qué se emite por radio, cómo llega al
PC y qué comandos entiende. Está pensado para quien quiera escribir **una
estación de tierra nueva** sin tener que leer el firmware.

La implementación de referencia es la de esta carpeta:
[`paquete.py`](paquete.py) (paquetes y comandos) y [`puente.py`](puente.py)
(el puerto serie). Si algo de este documento no cuadra con esos dos archivos
o con el firmware, **manda el código**, y este documento hay que corregirlo.

---

## 1. La cadena

```
           BAJADA (telemetría)                       SUBIDA (comandos)

  core ──CAN1 0x140──► placa RF ──LoRa──►      core ◄──CAN1 0x241── placa RF ◄──LoRa──
                                       │                                          │
                         puente de tierra (Blue Pill + SX1262)  ◄──────────────────┘
                                       │
                                  USB serie
                                       │
                                     PC (tu estación)
```

- El **core** decide qué se emite y cuándo. La **placa RF** sólo transmite lo
  que le manda el core y le pasa lo que oye; no interpreta nada.
- El **puente de tierra** es un módem transparente: lo que llega por radio lo
  saca por USB añadiéndole la calidad del enlace, y lo que le llega por USB lo
  emite tal cual. Tampoco interpreta nada.
- **Tu estación sólo tiene que hablar con el puente por el puerto serie.**

---

## 2. La radio (LoRa)

| parámetro | valor |
|---|---|
| frecuencia | **869,525 MHz** |
| ancho de banda | 250 kHz |
| factor de ensanchado | SF7 |
| tasa de código | 4/5 |
| palabra de sincronismo | 0x34 |
| cabecera | explícita (la longitud viaja en el paquete) |
| CRC | activado |
| preámbulo | 8 símbolos |
| LDRO | desactivado |
| IQ | normal (sin invertir) |
| potencia | +22 dBm, en los dos extremos |

El puente de tierra lleva además una **corrección de −2000 Hz** en la
frecuencia por la desviación del cristal de *esa* placa. Si haces otro puente
con otra placa, mide su desviación (el puente la da en cada paquete, ver §3)
y ajústala.

### Tiempo en el aire y ciclo de trabajo

La banda de 869,4–869,65 MHz permite emitir el **10 % del tiempo medido sobre
una hora**: 360 s de emisión por hora entre todos los que transmitan.

| paquete | bytes | tiempo en el aire |
|---|---|---|
| vuelo | 24 | 30,9 ms |
| estado | 32 | 36,0 ms |
| acuse | 12 | 20,6 ms |
| comando | 4 o 5 | 15,5 ms |

LoRa cobra el aire por bloques, no por byte: a SF7 y 4/5, cualquier tamaño
entre 23 y 26 bytes ocupa exactamente lo mismo.

---

## 3. El puente: puerto serie

**115200 baudios, 8N1**, por un conversor USB‑serie CH340 (en el PC de
pruebas sale como COM11). Al arrancar, el puente escribe unas líneas de texto
que empiezan por `#`; un lector que busque el sincronismo `AA 55` las ignora
sin más.

### Trama radio → PC

```
AA 55 | len | payload[len] | rssi | snr | df_lo df_hi | flags | crc8
```

| campo | tipo | significado |
|---|---|---|
| `AA 55` | — | sincronismo |
| `len` | uint8 | longitud del payload (lo que llegó por radio) |
| `payload` | `len` bytes | el paquete del cohete, tal cual |
| `rssi` | int8 | potencia recibida, dBm |
| `snr` | int8 | relación señal/ruido, en pasos de **0,25 dB** |
| `df` | int16 LE | desviación de frecuencia medida, Hz, con signo |
| `flags` | uint8 | ver abajo |
| `crc8` | uint8 | sobre `len`, `payload` y los 5 bytes de metadatos |

`flags`:

| bit | significado |
|---|---|
| 0 | el **CRC de LoRa venía mal**: el payload es basura |
| 1 | **acuse local**: no viene del aire, lo genera el puente (`len` = 0) |
| 2 | sólo con el bit 1: el puente **no pudo transmitir** |

- **Los paquetes con CRC de LoRa malo también se reenvían** (bit 0). No los
  decodifiques, pero cuéntalos: saber que llegó algo corrupto es lo que marca
  el borde de la cobertura.
- **El acuse local** sale tras cada transmisión que le pidas al puente. Sólo
  dice que el puente **emitió**, no que el cohete lo recibiera.

### Trama PC → radio

```
AA 55 | len | payload[len] | crc8
```

`len` de 1 a 255; `crc8` sobre `len` y `payload`. Con `len` = 0 el puente la
descarta (la longitud cero está reservada para su acuse).

### CRC‑8

Polinomio 0x07, valor inicial 0, sin reflejar ni XOR final:

```python
def crc8(datos):
    c = 0
    for b in datos:
        c ^= b
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c
```

### Al leer el puerto

- **Reconstruye la trama byte a byte** con una máquina de estados (sincronismo
  → longitud → datos → CRC). El puerto no garantiza que una trama llegue de
  una pieza.
- **Con pyserial, lee con `read(in_waiting or 1)`, no con `read(512)`.**
  Medido con el CH340 en Windows: `read(512)` con `timeout=0.1` se salta el
  timeout y espera a juntar los 512 bytes (~1 s). Los paquetes llegan a
  ráfagas, la cadencia medida sale al doble y la pantalla va un segundo por
  detrás.
- **Pon la hora de llegada en el hilo que lee**, no al procesar. Si la
  interfaz se atasca un momento y procesa una ráfaga de golpe, todas las
  tramas tendrían la misma hora.

---

## 4. Paquetes de bajada (cohete → tierra)

Por la misma radio llegan **cuatro paquetes distintos**. Se reconocen así, y
**en este orden**:

| si… | es |
|---|---|
| mide **24 bytes** | paquete de **vuelo** |
| mide **32 bytes** y el byte 0 es **0xA5** | paquete de **estado** |
| mide **32 bytes** y el byte 0 es **0xCA** | paquete de **cámaras** |
| mide **12 bytes** (u 8, firmware anterior) y el byte 0 es **0xC3** | **acuse** |
| cualquier otra cosa | desconocido: ignóralo |

> **Trampa.** El byte 0 del paquete de vuelo es un contador y vale 0xA5 una
> vez de cada 256. Si identificaras el de estado sólo por su primer byte,
> uno de cada 256 paquetes de vuelo se leería como si fuera de estado. Por eso
> el de vuelo se reconoce por la longitud y los demás por longitud **y**
> marca.

Todo en **little endian**.

### 4.1 Vuelo — 24 bytes

Dónde está el cohete y qué hace. Firmware: `App/flight_packet.h` y `.c`.

| byte | campo | tipo | escala |
|---|---|---|---|
| 0 | contador | uint8 | +1 por paquete de vuelo, da la vuelta en 256 |
| 1 | b0‑3 continuidad pirotecnia 1A, 1B, 2A, 2B · b4‑6 fase de vuelo · b7 navegación válida | bits | |
| 2 | b0‑2 estado del cohete · b3‑7 código de error (satura en 31) | bits | |
| 3‑4 | altura sobre el origen, **+ hacia arriba** | int16 | 0,25 m |
| 5‑6 | posición norte | int16 | 1 m |
| 7‑8 | posición este | int16 | 1 m |
| 9‑10 | velocidad vertical, **+ subiendo** | int16 | 0,05 m/s |
| 11 | velocidad norte | int8 | 1 m/s |
| 12 | velocidad este | int8 | 1 m/s |
| 13 | aceleración X de cuerpo (morro) | int8 | 0,25 g |
| 14 | aceleración Y de cuerpo (derecha) | int8 | 0,25 g |
| 15 | aceleración Z de cuerpo (abajo) | int8 | 0,25 g |
| 16‑19 | cuaternión w, x, y, z | 4 × int8 | 1/127 |
| 20 | apogeo previsto | uint8 | 25 m |
| 21 | aerofreno: extensión que **se manda al servo** | uint8 | 0..250 = 0..100 % de la carrera |
| 22 | tensión de la batería (raíl de 11 V) | uint8 | 80 mV · 0 = sin dato |
| 23 | temperatura del barómetro 1 | int8 | 1 °C · 0 = sin dato |

**Fases de vuelo** (byte 1, b4‑6): 0 IDLE · 1 BOOST · 2 COAST sin control ·
3 COAST con control · 4 APOGEO · 5 DROGUE · 6 MAIN · 7 EN TIERRA.

**Estados del cohete** (byte 2, b0‑2), GNC v1.2 y v1.5: 0 POWER ON ·
1 MANTENIMIENTO · 2 EN VUELO · 3 ERROR. **Cambió con la v1.2** (antes:
2 READY, 3 EN VUELO, 4 ERROR). Ya no hay READY: el cohete está en POWER ON
hasta que detecta el despegue.

**4 EMERGENCIA** no es del GNC: la core se ha **reiniciado en vuelo** y
sigue solo con los barómetros (`App/emergencia.h`). La altura y la velocidad
vertical son del barómetro, `nav válida` dice si el barómetro es fiable, la
posición horizontal y el cuaternión van a cero (no hay actitud) y el apogeo
previsto es la altura máxima medida. La fase sigue el vuelo: la que tenía
hasta el apogeo, luego DROGUE, MAIN (400 m) y EN TIERRA.

**Códigos de error** (byte 2, b3‑7): 0 ninguno · 1 navegación inválida
demasiado tiempo · 2 estancamiento · 3 entradas no finitas · 4 parada
ordenada desde tierra. En EN VUELO, un error **no se borra** hasta el final
del vuelo.

Cosas que conviene saber:

- **Ejes.** Cuerpo: X morro, Y derecha, Z abajo. Navegación: NED (norte,
  este, abajo). La altura y la velocidad vertical ya vienen con el signo
  cambiado para que + sea hacia arriba; la posición y velocidad horizontales
  no.
- **Un NaN del filtro se manda como 0**, y el bit de navegación válida dice si
  hay que fiarse. Un 0 no es necesariamente un dato.
- **Navegación válida significa que el filtro se ha alineado, no que acierte.**
  Sin GPS deriva y lo sigue marcando como válido.
- **El cuaternión es la actitud del cuerpo respecto a NED.** Convenio
  verificado contra la salida Euler del propio modelo (menos de 0,1° de
  diferencia):

  ```
  alabeo  = atan2(2(wx + yz), 1 − 2(x² + y²))
  cabeceo = asin (2(wy − zx))
  guiñada = atan2(2(wz + xy), 1 − 2(y² + z²))
  morro en NED = (1 − 2(y² + z²),  2(xy + wz),  2(xz − wy))
  elevación del morro = asin(−morro_D)
  ```

  Cuidado: **la matriz transpuesta da resultados igual de creíbles y
  falsos**. Con la placa plana, la transpuesta ponía el morro 32° bajo la
  horizontal.
- **Usa la elevación del morro, no el cabeceo de Euler**, para enseñar la
  inclinación: Euler tiene bloqueo de cardán a 90°, justo la actitud del
  cohete en la rampa. Ojo también con la resolución: a 8 bits por
  componente, cerca de la vertical la elevación pierde precisión (a 88° reales
  puede salir 84,9°).
- **Este paquete no lleva número de versión.** Si cambia una escala en el
  firmware y no en tu estación, verás números creíbles y equivocados.

### 4.2 Estado — 32 bytes

Si el cohete está entero. Firmware: `App/status_packet.h` y `.c`.
**Versión actual: 5.**

| byte | campo |
|---|---|
| 0 | `0xA5`, marca |
| 1 | versión del formato |
| 2 | contador de paquetes de estado |
| 3 | placas y validez: b0 power responde · b1 aviónica responde · b2 RF responde · b3 INIT_STATUS de aviónica recibido · b4 de power · b5 de RF · b6 calibración de aviónica hecha · b7 navegación válida |
| 4 | sensores de aviónica **arrancados**: b0 baro1 · b1 baro2 · b2 imu1 · b3 imu2 · b4 mag · b5 gps · b6 vn100 |
| 5 | sensores que la placa **lleva montados** (mismos bits) |
| 6 | tarjetas: b0‑1 estado SD1 · b2‑3 estado SD2 · b4 autotest SD1 OK · b5 autotest SD2 OK · b6 pin CD SD1 · b7 pin CD SD2 |
| 7 | arranques de la SD1 (uint8, satura) |
| 8 | arranques de la SD2 |
| 9‑10 | MB libres en la SD1 ÷ 16 (uint16) |
| 11‑12 | MB libres en la SD2 ÷ 16 |
| 13‑14 | tramas por segundo en CAN1 (uint16, media de 4 s) |
| 15‑16 | tramas por segundo en CAN2 |
| 17 | errores de CAN1 acumulados (uint8, satura) |
| 18 | errores de CAN2 |
| 19‑20 | segundos encendido (uint16, satura a las 18 h) |
| 21 | wake ups reenviados (uint8) |
| 22‑23 | paquetes de radio que no se pudieron encolar (uint16) |
| 24 | b0‑2 fase de vuelo · b3‑5 estado del cohete · b6 GNC alineado · b7 0 AGL enclavado (v7) |
| 25 | b0 **modo test activo** · b1‑3 último pirotécnico aceptado (0 ninguno, 1 1A, 2 1B, 3 2A, 4 2B) · b4 **SIMULANDO** · b5 **POSTVUELO** (v6) · b6 origen GNSS enclavado · b7 navegando sin haber alineado (v7). Con 24.b6, 24.b7 y 25.b6, el GNC v1.6 está **LISTO PARA VOLAR** (`nav_ready`): es lo que hay que ver antes de armar el iniciador |
| 26 | segundos que le quedan al modo test |
| 27 | pirotécnicos aceptados desde el arranque (uint8) |
| 28 | comandos de subida denegados (uint8) |
| 29 | avance de la simulación, 0..100 % |
| 30 | **motivo de la última denegación** (tabla en §5) |
| 31 | despliegue en este vuelo: b0 se ha mandado el drogue (1A y/o 1B) · b1 se ha mandado el main (2A) · b2 la power rechazó alguna orden · **b3 cámaras encendidas** (versión 5) · b4 **EMERGENCIA** · b5 apogeo detectado (barómetro) · b6 altura del main · b7 barómetro no fiable (v6) |

Estado de cada SD (byte 6): 0 sin iniciar · 1 no responde · 2 no monta ·
3 montada.

- **Un sensor tiene tres estados, no dos**: arrancado (bit a 1 en el byte 4),
  falla (a 0 en el 4 y a 1 en el 5) o no lo lleva la placa (a 0 en el 5).
  Sin el byte 5 no se distingue un fallo de un sensor que no existe.
- **Cada versión sólo ha añadido campos en bytes que antes iban a cero.** Una
  estación de una versión anterior lee esos bytes como «inactivo». Mira el
  byte 1 antes de usar un campo nuevo: modo test desde la 2, simulación y
  motivo desde la 3, despliegue desde la 4, cámaras desde la 5 (con una
  versión anterior, el estado de las cámaras **no se sabe**: no es
  «apagadas»), reinicio en vuelo desde la 6.
- **Versión 6 (reinicio en vuelo, `App/emergencia.h`):** byte 25 **b5
  POSTVUELO** (reiniciado después de un vuelo terminado: no toca la SD, no
  enciende las cámaras ni manda pirotecnia de vuelo) · byte 31 **b4
  EMERGENCIA** (reiniciado en vuelo, decide con el barómetro) · **b5 apogeo
  detectado** por el barómetro · **b6 altura del main** (400 m) alcanzada ·
  **b7 barómetro NO fiable** ahora (descartando medidas por ruido o saltos).
  En emergencia, b0 y b1 cuentan también lo mandado antes del reinicio.
- **El byte 31 ya está lleno.**

### 4.3 Cámaras — 32 bytes

Las dos cámaras RunCam Split 4: si graban, con qué, y su tarjeta. El cohete
sabe si una cámara graba porque le lee por UART, cada segundo, el espacio
libre de su tarjeta: si baja, graba (la cámara no tiene otra forma de
decirlo).

- **Cuándo llega:** cada 5 s con ellas encendidas, en cada cambio (como mucho
  uno por segundo), una vez al arrancar y cuando se pide el estado (`0x01`).
- **Firmware:** `App/camaras.h` (el paquete y cuándo graban y se apagan),
  `App/camara.h` (cada cámara) y `Drivers/Runcam` (el protocolo de la
  cámara).

| byte | campo |
|---|---|
| 0 | `0xCA`, marca |
| 1 | versión (1) |
| 2 | contador, +1 por paquete |
| 3 | b0 alimentación (PA2) · b1 **apagando**: parando la grabación antes de cortar · b2‑3 quién las encendió o apagó (0 arranque, 1 despegue, 2 tierra, 3 fin del vuelo) · b4 en vuelo · b5 el cohete sabe la hora (GPS) · b6 el último apagado se cortó **por plazo**, sin confirmar que pararan |
| 4‑17 | cámara 1 |
| 18‑31 | cámara 2 |

Cada cámara, 14 bytes (desde su primer byte):

| byte | campo |
|---|---|
| +0 | estado: **0** sin corriente · **1** arrancando · **2** no contesta (con corriente 6 s y nada por UART) · **3** comprobando · **4 GRABANDO** · **5** parada · **6** parando |
| +1 | b0 quiere grabar · b1 **NO GRABA** (pulsó para grabar y sigue parada) · b2 reloj puesto con la hora del GPS · b3 tarjeta leída en este encendido · b4 PAL (si no, NTSC) · b5 resolución leída · b6 sin tarjeta |
| +2‑3 | líneas de la resolución (1080) |
| +4 | fotogramas por segundo (60) |
| +5‑6 | MB libres en su tarjeta / 16 |
| +7‑8 | MB totales / 16 |
| +9‑10 | segundos de grabación que caben |
| +11‑12 | segundos que lleva grabando |
| +13 | pulsaciones del botón desde el arranque |

- Los campos +2 a +12 son **lo último que dijo** la cámara; los bits b3 y b5
  dicen si es de este encendido.
- «Apagar» (fin del vuelo o `0x17` con 0) es **parar y luego cortar**: la
  cámara pasa a «parando» y a «parada», y la alimentación se corta 1,5 s
  después (o a los 12 s pase lo que pase). Unos 5,5 s en la mesa.

### 4.4 Acuse — 12 bytes

Respuesta a un comando concreto (ping, ángulo de lanzamiento y las dos
pruebas de aerofreno).
Firmware: `App/uplink.h`.

| byte | campo |
|---|---|
| 0 | `0xC3`, marca |
| 1 | id del comando al que contesta |
| 2 | resultado: 0 aceptado, si no el motivo (tabla en §5) |
| 3‑4 | **ping**: el número de ping, devuelto tal cual · **ángulo** y **rumbo**: el valor que tiene **ahora** el cohete, en centésimas de grado · **prueba de aerofreno**: el número de prueba · **extensión de aerofreno**: el % que tiene **ahora** el aerofreno |
| 5 | contador de acuses |
| 6 | sólo ángulo y rumbo: b0 definido (no es el de por defecto) · b1 guardado en la SD |
| 7 | b0: los bytes 8‑11 son válidos (la RF del cohete los mandó) |
| 8 | RSSI con el que **el cohete** oyó el comando, int8 dBm |
| 9 | SNR con el que lo oyó, int8 dB |
| 10‑11 | error de frecuencia que midió el cohete, int16 LE, Hz (+ = tierra transmite por encima) |

Los bytes 8‑11 son el enlace de subida visto **desde el cohete** (lo que el
puente da en cada trama es el de bajada visto desde tierra). Un cohete con
firmware anterior manda acuses de **8 bytes**, sin estos cuatro: un decodificador
debería aceptar los dos largos.

El cohete manda un acuse **por cada comando que recibe**, también por las
repeticiones.

---

## 5. Comandos de subida (tierra → cohete)

Firmware: `App/uplink.h` y `.c`.

### Formato

Parámetro de 8 bits, **4 bytes**:

```
5A | id | parámetro | XOR(byte0..byte2)
```

Parámetro de 16 bits, **5 bytes**:

```
5A | id | param_lo | param_hi | XOR(byte0..byte3)
```

El cohete distingue los dos por la longitud. Cualquier otra longitud, otra
marca o un XOR que no cuadre se **descarta sin respuesta**. El XOR va
*además* del CRC de LoRa porque el CRC dice que los bits llegaron como
salieron, no que lo que salió fuera un comando nuestro.

### Comandos

| id | comando | formato | parámetro | qué contesta |
|---|---|---|---|---|
| `0x01` | pedir estado | 4 B | — | un paquete de estado y uno de cámaras |
| `0x02` | ping | 5 B | número de ping (16 bits) | acuse con el mismo número |
| `0x10` | modo test | 4 B | 1 entrar o renovar · 0 salir | estado cada segundo mientras dure |
| `0x11` | pirotécnico | 4 B | canal: **1** 1A · **2** 1B · **3** 2A · **4** 2B | estado (sube «aceptados») |
| `0x12` | simular ruta | 4 B | 1 empezar · 0 parar | estado cada segundo mientras dure |
| `0x13` | ángulo de lanzamiento | 5 B | centésimas de grado **desde la vertical**, 0..9000 · `0xFFFF` = sólo consultar | acuse con el valor guardado |
| `0x14` | prueba de aerofreno | 4 B | **número de prueba** 1..255 · 0 = parar | acuse con el número |
| `0x15` | extensión de aerofreno | 4 B | 0..100 % de la carrera | acuse con el % que tiene ahora |
| `0x16` | rumbo de la rampa | 5 B | centésimas de grado desde el **norte geográfico**, sentido horario, 0..35999 · `0xFFFF` = sólo consultar | acuse con el valor guardado |
| `0x17` | cámaras | 4 B | 1 encender · 0 apagar (primero paran de grabar, luego se corta la corriente) | acuse con el estado de verdad: b0 alimentación, b1 apagando (aún con corriente); un paquete de estado y uno de cámaras |

Un id que el cohete no conoce se cuenta y no tiene respuesta. Cuando un
comando se deniega, el cohete manda un paquete de estado con el motivo en el
byte 30 (o, en el caso del ángulo, en el propio acuse).

### Cuándo se acepta cada uno

- **Pedir estado, ping, salir del modo test, parar la simulación y consultar
  el ángulo**: siempre.
- **Entrar en modo test**: sólo en tierra, es decir, estado POWER ON,
  MANTENIMIENTO o ERROR **y** fase IDLE o EN TIERRA. Con el GNC v1.2 ya no
  hay READY, así que **en la rampa también se acepta** hasta el despegue.
  **Nunca en
  vuelo.** Tampoco mientras se simula. Caduca solo a los **120 s** si no se
  renueva, y se cancela en ese mismo paso si el cohete sale de tierra.
- **Pirotécnico**: sólo en modo test y sin simular. La power mantiene el
  canal encendido **1 s** y lo apaga sola; si llega otra orden mientras está
  encendido, el segundo vuelve a empezar.
- **Simular ruta**: sólo en modo test y en tierra. **Al empezar, el cohete
  cierra el modo test.** Si ya está simulando, repetirlo no hace nada.
- **Prueba y extensión de aerofreno**: como un pirotécnico, sólo en modo
  test, en tierra y sin simular. Contestan **siempre** con acuse, también al
  denegar. La prueba hace 3 veces todo fuera 0,5 s / recogido 0,5 s y luego
  sale en 2 s y entra en 2 s (7 s). **No es idempotente, por eso lleva
  número**: el cohete recuerda el último que ejecutó y, si vuelve a llegar el
  mismo (un reintento cruzado con el acuse), sólo contesta. Para otra prueba,
  otro número. La extensión se queda puesta hasta otra orden o hasta salir
  del modo test; al salir, el aerofreno vuelve a recogido. En cuanto el
  cohete pasa a EN VUELO, manda el GNC y cualquier prueba se cancela.
- **Ángulo y rumbo de la rampa** (no consulta): en tierra y sin simular. El
  cohete los guarda en las dos SD y se los pasa al GNC (elevación = 90 −
  ángulo, azimut = rumbo; el alabeo lo mide él con el acelerómetro). Si
  llegan antes de que el EKF se alinee se usan ya; si llegan después, con el
  GNC v1.5 el cohete **se realinea en el acto** con los nuevos (~0,5 s sin
  navegación válida).
- **Cámaras** (pin PA2 de la core): encender, **siempre**; apagar, sólo en
  tierra (en vuelo se deniega). Contesta **siempre** con acuse, también al
  denegar, con el estado de verdad: tierra reintenta hasta verlo. Arrancan
  **apagadas** y el primer paquete de estado del arranque ya lo dice. **Se
  encienden solas al despegar** (el GNC pasa a EN VUELO o una IMU mide más de
  3 g durante 100 ms; nunca por una simulación) **y se apagan solas al
  terminar el vuelo**, con las mismas condiciones que cierran el registro de
  la SD: bajando de 50 m tras el apogeo, el GNC en EN TIERRA o 20 min. Cada
  cambio va con un paquete de estado para que tierra lo vea. Tras un vuelo de
  verdad (más de 100 m) ya no se encienden solas en ese arranque: el golpe
  del aterrizaje no cuenta como otro despegue.

### Motivos de denegación

| código | motivo |
|---|---|
| 0 | ninguno (aceptado) |
| 1 | el cohete no está en tierra |
| 2 | no está en modo test |
| 3 | canal de pirotécnico inválido |
| 4 | la power no responde |
| 5 | está simulando una ruta |
| 6 | no hay `VUELO.BIN` en la SD1 |
| 7 | valor fuera de rango: ángulo fuera de 0 a 90° o extensión mayor de 100 % |

---

## 6. Cómo hablar con el cohete sin equivocarse

Son lecciones medidas contra el cohete, no teoría. Cada una costó un fallo.

**1. La subida pierde comandos, y es lo normal.** La RF está sorda mientras
transmite. A 15 Hz de telemetría transmite 30,9 ms de cada 66,7, y un comando
de 15,5 ms se pierde si se solapa: **llega ~30 %** (medido 9 de 30). En tierra
la telemetría va a 3 Hz (30,9 ms de cada 333) y deberían llegar ~85 % (sin
medir todavía); a 15 Hz solo desde el lanzamiento. **No hay acuse
de que el cohete recibiera un comando** salvo la respuesta que da (tabla de §5).

**2. Reintenta hasta ver la respuesta, con variación aleatoria.** Nunca a un
periodo fijo: **1,000 s son justo 15 periodos de la telemetría**, así que todos
los reintentos caen en la misma fase y, si el primero coincide con la ventana
sorda, coinciden todos. Medido: **0 de 40** peticiones llegaron con espaciado
fijo de 1 s y 7 de 40 con espaciado al azar. Lo mismo con 400 ms (6 periodos)
o 5 s (75). La estación de referencia usa ±25 % sobre cada periodo
(`_con_jitter` en `estacion.py`).

**3. Da algo por hecho sólo cuando lo diga el cohete.** Pulsar no es
conseguir. Habilita los botones de pirotecnia sólo si un paquete de estado
**de hace menos de 3 s** dice que está en modo test (en modo test manda uno
por segundo, así que si el enlace se cae se deshabilitan solos). Con el
ángulo, compara el valor que devuelve el acuse con el que mandaste: un acuse
de un envío anterior puede llegar mientras mandas uno nuevo.

**4. Todos los comandos se pueden repetir sin daño.** Por eso se puede
reintentar a ciegas: pedir estado dos veces manda dos paquetes, renovar el
modo test sólo amplía el plazo, un pirotécnico repetido sólo prolonga su
segundo. **Un comando nuevo que no cumpla esto necesita número de secuencia**,
y el formato aún no lo tiene.

**5. Hombre muerto en las dos direcciones con el modo test.** Mientras quieras
estar dentro, renuévalo (la estación de referencia lo hace cada ~5 s). Si tu
estación se cierra o se cae el enlace, el cohete sale solo a los 120 s. Y si
al arrancar te encuentras el cohete en modo test sin haberlo pedido, sácalo:
el estado seguro por defecto es fuera.

**6. Pirotecnia: dos clics y una ráfaga.** La estación de referencia pide un
clic para armar el botón (3 s) y otro para confirmar, y entonces manda el
comando **5 veces cada 150 ms** (150 ms son 2,25 periodos, así que la fase ya
varía). Con el 30 % de acierto, cinco llegan el 83 % de las veces. No
reintenta sola un disparo: eso lo decide una persona.

**7. Simular una ruta enciende 1A, 1B y 2A DE VERDAD.** La simulación reproduce
`VUELO.BIN` de la SD del cohete por el mismo código que un vuelo real, y ese
código enciende **1A y 1B a la vez al entrar en DROGUE y 2A al entrar en
MAIN** (cada uno
~1,8 s). Es a propósito, para validarlo en el banco. **Se simula con cargas
de prueba, nunca con cargas reales.** El aerofreno también se mueve. Mientras
dura, todos los datos de vuelo son simulados: déjalo claro en pantalla (byte
25, bit 4 del paquete de estado).

**8. Un hueco grande en el contador no son paquetes perdidos.** El paquete de
estado y el acuse ocupan el hueco de un paquete de vuelo, pero **el contador
de vuelo sólo avanza con los de vuelo**, así que no dejan hueco. Si el cohete
se reinicia, el contador vuelve a cero: un salto de cientos no son cientos de
paquetes perdidos. La estación de referencia cuenta como «corte» cualquier
hueco de más de 32 paquetes (~2 s a 15 Hz, ~11 s a 3 Hz).

---

## 7. Cadencias

| qué | cuándo |
|---|---|
| paquete de vuelo | **3 Hz** en tierra hasta el lanzamiento (9,3 % de aire: se puede sostener sin límite) · **15 Hz** desde el lanzamiento (GNC EN VUELO, 3 g durante 100 ms o arranque en emergencia) hasta el siguiente arranque · SIMULAR RUTA: 15 Hz mientras dura, luego 3 (`App/rf_telemetry.h`) |
| paquete de estado | 3 veces al arrancar (cuando contestan las tres placas o a los 5 s; luego cada 2 s) · al pedirlo · cada segundo en modo test o simulando · al denegar un comando · al aceptar un pirotécnico |
| paquete de cámaras | 1 al arrancar · cada 5 s con las cámaras encendidas · en cada cambio (máx. 1/s) · al pedir el estado |
| acuse | en el siguiente hueco tras recibir ping o ángulo |

Un paquete de estado, de cámaras o un acuse **sustituyen** al de vuelo de ese
hueco (no se añaden), así que el ciclo de trabajo no cambia. Si coinciden, va
primero el acuse, luego el estado y luego el de cámaras.

---

## 8. Ejemplos

### Paquete de vuelo real, de punta a punta

Lo que llega por el puerto serie (`len` 0x18 = 24, RSSI −34 dBm, SNR 11,25 dB,
desviación +155 Hz):

```
aa 55 18 | 90 80 00 86 fa 41 ff 7b ff fc ff 00 00 04 00 00 a2 00 bd cb 00 00 d5 1f | de 2d 9b 00 00 | 39
```

El payload:

| bytes | valor | significa |
|---|---|---|
| `90` | 144 | contador |
| `80` | b7 = 1 | navegación válida, fase IDLE, sin continuidad |
| `00` | | POWER ON, sin error |
| `86 fa` | −1402 × 0,25 | altura −350,50 m |
| `41 ff` / `7b ff` | −191 / −133 | norte −191 m, este −133 m |
| `fc ff` | −4 × 0,05 | velocidad vertical −0,20 m/s |
| `04 00 00` | | aceleración X +1,00 g (placa plana, morro arriba) |
| `a2 00 bd cb` | −94, 0, −67, −53 ÷ 127 | alabeo 44,8°, cabeceo 51,4°, guiñada 81,3° |
| `d5` | 213 × 80 mV | batería 17,04 V |
| `1f` | 31 | barómetro 31 °C |

(Capturado con la placa en el banco y sin GPS: la navegación había derivado.)

### Comandos

| comando | payload | trama completa PC → puente |
|---|---|---|
| pedir estado | `5a 01 00 5b` | `aa 55 04 5a 01 00 5b 02` |
| entrar en modo test | `5a 10 01 4b` | `aa 55 04 5a 10 01 4b ae` |
| salir del modo test | `5a 10 00 4a` | `aa 55 04 5a 10 00 4a bc` |
| pirotécnico 1A | `5a 11 01 4a` | `aa 55 04 5a 11 01 4a c2` |
| simular ruta | `5a 12 01 49` | `aa 55 04 5a 12 01 49 76` |
| ping nº 7 | `5a 02 07 00 5f` | `aa 55 05 5a 02 07 00 5f 0a` |
| ángulo 5,25° | `5a 13 0d 02 46` | `aa 55 05 5a 13 0d 02 46 99` |
| consultar el ángulo | `5a 13 ff ff 49` | `aa 55 05 5a 13 ff ff 49 f3` |
| prueba de aerofreno nº 1 | `5a 14 01 4f` | `aa 55 04 5a 14 01 4f 19` |
| parar la prueba de aerofreno | `5a 14 00 4e` | `aa 55 04 5a 14 00 4e 0b` |
| aerofreno al 40 % | `5a 15 28 67` | `aa 55 04 5a 15 28 67 b9` |
| aerofreno recogido (0 %) | `5a 15 00 4f` | `aa 55 04 5a 15 00 4f 67` |
| rumbo 90° (este) | `5a 16 28 23 47` | `aa 55 05 5a 16 28 23 47 e8` |
| consultar el rumbo | `5a 16 ff ff 4c` | `aa 55 05 5a 16 ff ff 4c a6` |

Acuse local del puente tras transmitir bien (`len` 0, bit 1 de flags):

```
aa 55 00 00 00 00 00 02 0e
```

---

## 9. Si cambias el protocolo

- **Cambia los dos lados a la vez** y en el mismo commit: el firmware (`App/`
  del core) y [`paquete.py`](paquete.py). El paquete de vuelo no tiene número
  de versión, así que nada avisa de un desajuste.
- **Un paquete nuevo de bajada** necesita marca propia y una longitud que no
  choque con 24, 12 ni 8 (32 solo con una marca distinta de 0xA5 y 0xCA).
  Además tiene que ser **una longitud válida de CAN
  FD** (0‑8, 12, 16, 20, 24, 32, 48, 64), porque viaja del core a la RF por
  CAN: si pides otra, el driver rellena con ceros hasta la siguiente y por el
  aire sale más largo de lo que crees (pasó con el paquete de estado, que se
  diseñó de 26 y salía de 32).
- **Un comando nuevo** tiene que poder repetirse sin daño (§6, punto 4), o
  llevar número de secuencia.
- **Actualiza este documento.**

## 10. Dónde está cada cosa

| qué | dónde |
|---|---|
| paquete de vuelo | `App/flight_packet.h` y `.c` |
| paquete de estado | `App/status_packet.h` y `.c` |
| comandos, acuse, modo test | `App/uplink.h` y `.c` |
| cadencias, cola de transmisión | `App/rf_telemetry.h` y `.c` |
| despliegue automático 1A+1B / 2A | `App/despliegue.h` y `.c` |
| simulación de ruta | `App/gnc_playback.h` y `.c` |
| ángulo de lanzamiento | `App/lanzamiento.h` y `.c` |
| cámaras: paquete, cuándo graban, apagado | `App/camaras.h` y `.c`, `App/camara.h` y `.c` |
| reinicio en vuelo (estado 4, bits de la v6) | `App/emergencia.h`, `App/baro_alt.h`, `App/memoria_vuelo.h` |
| protocolo de las RunCam por UART | `Drivers/Runcam/runcam.h` |
| mensajes CAN entre placas | `can_protocol/ICD.md` (0x140 transmitir, 0x241 recibido) |
| configuración de radio del cohete | `rf_g4_v0/bsp/sx1262/bsp_sx1262.c` |
| firmware del puente | `Documents/Arduino/LEEM_Puente_Radio/` (fuera de los repos) |
| implementación de referencia en Python | [`paquete.py`](paquete.py), [`puente.py`](puente.py) |
