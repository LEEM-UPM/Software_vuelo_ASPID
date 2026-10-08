# sensores_avionica_g4_v0
MCU STM32G473

# Sensores de la placa:
- **IMU Principal (hasta 16Gs)    -> WSEN-ISDS 6 Axis IMU (Wurth Electronik)**

    Esta será la IMU principal de la placa de donde se extraerán los datos de 
    aceleración axial del acelerómetro, y velocidad angular del giroscopio. Para 
    obtener datos más precisos será importante calibrar el sensor antes del vuelo 
    del cohete. Dicho sensor tiene la oción de medir desde +-2g a +-16g, y +-250dps
    hasta +-2000dps. Para más información técnica se puede consultar el manual del 
    fabricante a continuación: 
        
    https://www.we-online.com/components/products/manual/Manual_UM_WSEN-ISDS_2536030320001%20(rev1.3).pdf

    Driver: https://github.com/WurthElektronik/Sensors-SDK_STM32/tree/main/SensorsSDK/WSEN_ISDS_2536030320001

- **IMU de altas Gs (hasta 32Gs)  -> LSM6DSV32XTR 6 Axis IMU**
  (configurada a ±16 g / ±1000 dps, como la ISDS: el EKF de la core la usa
  como IMU de alta g por encima de 12 g y no puede saturarse antes que la
  otra; ver `Drivers/Sensors/imu2.c`)

    Esta será la IMU de altas Gs usada por la placa de avionica cuando se supere el 
    rago de aceleraciones de +-16g. Los valores máximos y mínimos de medición del 
    acelerómetro y giroscopio de este sensor serán desde +-4g a +-32g, y desde 
    +-125dps a +-4000dps. Para más información técnica se puede consultar el manual 
    del fabricante a continuación:

    https://www.st.com/resource/en/datasheet/lsm6dsv32x.pdf

- **2 Barometros                  -> WSEN PADS (Wurth Electronik)**

    Ambos barómetros presentes en la placa son del mismo modelo, sin embargo estan 
    puestos en extremos de ella, y se usarán dos por redundancia y para descartar 
    medidas espurias. Dichos barómetros darán medidas de presión, que mediante su 
    procesamiento podrán servir para calcular la altura teórica del cohete o para 
    modelizar el drag aplicado sobre este en ese instante. Además, dichas medidas se 
    podrán combinar con el procesamiento de datos del resto de senores (ej: las IMUs
    , el magnetómetro y el GPS) mediante un filtro de Kalman, para mejorar el 
    posicionamiento vertical teórico del cohete. Para más información técnica sobre 
    este sensor consutlat el manual del fabricante a continuación:

    https://www.we-online.com/components/products/manual/Manual-um-wsen-pads-2511020213301%20(rev3.0).pdf

    Driver: https://github.com/WurthElektronik/Sensors-SDK_STM32/tree/main/SensorsSDK/WSEN_PADS_2511020213301
        
- **Magnetómetro                  -> MLX90393ELW-ABA-014-SP 3 Axis Magnetometer**

    Este magnetómetro presente en la placa servirá para conocer en todo momento la 
    dirección del campo magnético, la cual, conociendo la orientación teórica del 
    cohete permitirá corregir dicha orientación a una que se aproxime más a la real.
    El campo magnético máximo que podrá medir dicho sensor será de 50 mT, por ello 
    es importante colocarlo lejos de fuentes de intensidad que puedan generar un 
    campo magnético al rededor y afectar a las mediciones del sensor. Para más 
    información, consultar el manual del sensor a continuación:

    https://eu.mouser.com/ProductDetail/Melexis/MLX90393ELW-ABA-014-SP?qs=4pzlT8jjbhc1GB0OqKEoCw%3D%3D
        
- **GPS                           -> MAX-M10S-00B GPS**

    Este sensor permitirá conocer la posición nominal del cohete en los tres ejes,
    es importante recalcar que en su operación nominal puede llegar a tener un error
    de 1.5m, sin embargo a velocidades mayores a 30m/s, este error crece ampliamente
    y no puede ser usado para conocer la posición precisa del cohete. Además, el 
    sensor puede funcionar desde frecuencias de 4Hz hasta 25Hz, dependiendo del 
    número de constelaciones que esté usando. Dicho sensor puede ser usado para la 
    corrección de la posción en 3 dimensiones del cohete sobre todo en su etapa de 
    descenso (ya que en el ascenso se alcanzan velocidades superiores a los 30m/s). 
    Para más información, consultar el manual técnico a continuación:

    https://content.u-blox.com/sites/default/files/MAX-M10S_DataSheet_UBX-20035208.pdf

    Librería NMEA: https://github.com/kosma/minmea

- **Sensor de Humedad             -> WSEN-HIDS (Wurth Electonik)**

    El sensor de humedad presente en la placa permitirá conocer la humedad existente 
    en el interior del cohete. Dichos datos podrán ser usados para conocer el estado
    de la electrónica y apagarla en caso de emergencia (ej: si esta se moja). Además,
    dicho sensor podrá ser usado en experimentos futuros. Para más información 
    consultar el manual técnico del sensor a continuación:

    https://www.we-online.com/components/products/manual/UM_WSEN-HIDS_2525020210002%20(rev1.2).pdf

    Driver: https://github.com/WurthElektronik/Sensors-SDK_STM32/tree/main/SensorsSDK/WSEN_HIDS_2525020210002

# Firmware

## Estructura

- `Core/`: código generado por CubeMX (solo se toca dentro de `USER CODE`).
- `App/`: firmware de vuelo, todos los sensores, la telemetría CAN y el servo.

  | Archivo          | Función                                                              |
  |------------------|----------------------------------------------------------------------|
  | `main.c`         | Arranque, espera del wake up por CAN1, bucle principal y LED         |
  | `sensorsI2c`     | Barómetros, IMUs y magnetómetro: lectura y envío por CAN2            |
  | `sensorIrq`      | Interrupciones data-ready de los sensores I2C                        |
  | `sensorsUart`    | GPS y VectorNav                                                      |

- `Drivers/`: además de la HAL y CMSIS de ST, los drivers propios:

  | Módulo                        | Función                                                 |
  |-------------------------------|---------------------------------------------------------|
  | `Sensors/barometer`           | WSEN-PADS (x2), sobre el WESensorsSDK                   |
  | `Sensors/imu`                 | WSEN-ISDS, sobre el WESensorsSDK. Va montada girada 180° en Z: el driver le cambia el signo a X e Y |
  | `Sensors/imu2`                | LSM6DSV32X, sobre el driver de ST                       |
  | `Sensors/magnetometer`        | MLX90393, comandos I2C directos (sin SDK)               |
  | `Sensors/gps`                 | MAX-M10S, NMEA con minmea, recepción por interrupción (idle line) |
  | `Sensors/vectorNav`           | VN-100, paquete binario con CRC16, recepción por interrupción |
  | `Can_driver/canNode`          | CAN1: bus común entre placas (ICD de `third_party/can_protocol`) |
  | `Can_driver/canTelemetry`     | Envío de las muestras de los sensores por CAN FD        |
  | `Can_driver/canCommand`       | Recepción de la trama de mando de la placa core H7 (ID 0x200) |
  | `Can_driver/canBusOff`        | Saca de bus-off FDCAN1 y FDCAN2 (ICD 2.1); se llama en cada vuelta y mientras se espera el wake up |
  | `Watchdog/watchdog`           | IWDG de 1 s: una vuelta del bucle más larga reinicia la placa. La calibración (2 s) lo refresca ella misma |
  | `Servo/servo`                 | PWM de servo de 50 Hz en PB10 (AIRBRAKE)                |
  | `Timestamp/timestamp`         | Contador libre de microsegundos para marcar las muestras |

- `poc/`: pruebas de concepto de cada sensor, un `main.c` por carpeta.
- `third_party/`: WESensorsSDK (PADS, ISDS), driver ST LSM6DSV32X, minmea y
  `can_protocol` (submodule).

## Compilar

Por defecto se compila el firmware de vuelo (`App/`). Cada PoC tiene su propia
función `main()`, así que para compilar una hay que comentar en `CMakeLists.txt`
las líneas `App/...` y dejar **una sola** línea `poc/.../main.c` sin comentar.
Después: `cmake --preset Debug` y `cmake --build --preset Debug`, o la tarea
"CMake: Build (Debug)" de VS Code.

- Si al depurar se sigue ejecutando el programa anterior, mira la salida de la
  compilación: si pone `FAILED`, el depurador programa el último `.elf` que sí se
  generó.
- El toolchain enlaza con `-fno-use-linker-plugin` (`cmake/gcc-arm-none-eabi.cmake`).
  Sin esa opción, `ld` no puede cargar `liblto_plugin.dll` cuando el toolchain está
  en una ruta con caracteres no ASCII, como el que instala la extensión STM32 de
  VS Code en `C:\Users\<usuario con tilde>\AppData\...`. El proyecto no usa LTO.

## Direcciones I2C (7 bits)

Comprobadas con `poc/i2c_scan`:

| Bus  | Dispositivo              | Dirección |
|------|--------------------------|-----------|
| I2C1 | MLX90393 (magnetómetro)  | 0x18      |
| I2C1 | WSEN-PADS 1              | 0x5D      |
| I2C1 | WSEN-ISDS                | 0x6B      |
| I2C2 | WSEN-HIDS (sin driver)   | 0x44      |
| I2C2 | WSEN-PADS 2              | 0x5D      |
| I2C2 | LSM6DSV32X               | 0x6B      |

## Buses CAN

FDCAN1 (PA11 RX / PA12 TX) y FDCAN2 (PB12 RX / PB13 TX) tienen la misma
configuración: CAN FD **sin BRS** a **2 Mbit/s** (reloj 100 MHz, prescaler 1,
Seg1 = 43, Seg2 = 6, SJW = 6, punto de muestreo 88 %) y **sin retransmisión
automática**. El filtro global rechaza todo lo que no acepte un filtro explícito.

- **FDCAN1:** bus CAN1 común entre placas, a 1 Mbit/s según el ICD de
  `third_party/can_protocol` (no la configuración de 2 Mbit/s de arriba). Solo
  acepta comandos broadcast del core; ver `Drivers/Can_driver/canNode.h`.
- **FDCAN2:** telemetría de sensores (envío) y trama de mando del H7 (recepción).

### Telemetría de sensores (FDCAN2, envío)

Una trama por muestra leída, con ID estándar `0x100 + id_sensor`. Payload en
little-endian:

| Bytes  | Contenido                                                          |
|--------|--------------------------------------------------------------------|
| 0      | id_sensor                                                          |
| 1..4   | timestamp `uint32` en µs (da la vuelta cada ~71,6 min)             |
| 5..    | valores `float32`                                                  |
| resto  | relleno con ceros hasta una longitud CAN FD válida                 |

| ID    | Sensor       | Valores (en orden)                              | Longitud |
|-------|--------------|-------------------------------------------------|----------|
| 0x101 | WSEN-PADS 1  | presión [kPa], temperatura [°C]                 | 16 bytes |
| 0x102 | WSEN-PADS 2  | presión [kPa], temperatura [°C]                 | 16 bytes |
| 0x103 | WSEN-ISDS    | accel X, Y, Z [m/s²], gyro X, Y, Z [rad/s]      | 32 bytes |
| 0x104 | LSM6DSV32X   | accel X, Y, Z [m/s²], gyro X, Y, Z [rad/s]      | 32 bytes |
| 0x105 | MLX90393     | campo X, Y, Z [T]                               | 20 bytes |
| 0x106 | MAX-M10S GGA | enteros, ver abajo                              | 32 bytes |
| 0x107 | MAX-M10S RMC | enteros, ver abajo                              | 32 bytes |

**GPS (0x106 y 0x107):** misma cabecera de 5 bytes, pero los datos son enteros
(sin pérdida de precisión en latitud y longitud). Posición desde el byte 0:

| Byte | GGA (0x106)                     | RMC (0x107)                      |
|------|---------------------------------|----------------------------------|
| 5    | `uint32` hora UTC [ms del día]  | `uint32` hora UTC [ms del día]   |
| 9    | `int32` latitud [1e-7 grados]   | `int32` latitud [1e-7 grados]    |
| 13   | `int32` longitud [1e-7 grados]  | `int32` longitud [1e-7 grados]   |
| 17   | `int32` altitud MSL [mm]        | `int32` velocidad [mm/s]         |
| 21   | `uint8` calidad del fix (0 = no)| `uint16` rumbo [0,01 grados]     |
| 22   | `uint8` satélites usados        |                                  |
| 23   | `uint16` HDOP ×100              | `uint16` año (0 = desconocido)   |
| 25   |                                 | `uint8` mes                      |
| 26   |                                 | `uint8` día                      |
| 27   |                                 | `uint8` estado (1 = 'A' válido)  |

Campos vacíos (sin fix): `int32` 0x80000000, `uint16` 0xFFFF, hora 0xFFFFFFFF.

- `MX_GPIO_Init()` deja **GPS_RST (PA6) a nivel bajo, con el módulo en reset**.
  `GPS_Init()` da un pulso de reset, suelta la línea (nivel alto) y espera 1 s a
  que arranque.
- La antena de J1 es activa: `SensorsUart_Init()` pone **ACT_ANTENNA (PA4) a nivel
  alto** para alimentarla desde `VCC_RF`.
- El LED D11 va a TIMEPULSE: apagado sin fix, un destello de 100 ms por segundo
  (1PPS) cuando el módulo tiene hora GNSS válida.
- El GPS se configura al arrancar por UBX (`GPS_Init()`, solo en RAM): 10 Hz,
  solo GGA y RMC, UART a 115200. Como no se sabe a qué velocidad está el módulo
  (fábrica o ya configurado), la configuración se envía a 9600, 38400 y 115200.
  LPUART1 usa prescaler /8 (con PCLK1 = 100 MHz no llega a 9600 sin él).
- El timestamp del GPS es el momento en que termina de llegar la ráfaga NMEA
  (línea en reposo), no el instante de la medida; la hora UTC va en los datos.
- El timestamp se toma dentro de la interrupción data-ready de cada sensor, es
  decir, cuando la muestra estuvo disponible, no cuando se envió. Es el mismo
  reloj para todos los sensores. Si se ha perdido el flanco de la interrupción,
  o el magnetómetro entra por timeout, se usa el momento de lectura.
- Tasas nominales: ISDS ~416 Hz, LSM6DSV32X ~480 Hz, PADS hasta ~200 Hz,
  MLX90393 ~65 Hz. Sin retransmisión: pueden faltar tramas.
- La VN-100 no se envía por CAN.
- **Ningún sensor puede parar la placa:** el que no responde al arrancar, o el
  que deja de responder, se marca como caído y se salta, y se reintenta
  inicializar cada 5 s por si vuelve. Si se caen todos los de un bus se
  reinicia el periférico I2C, que es lo que saca a un bus bloqueado.
  `SensorsI2c_ActiveMask()` dice cuáles están funcionando.

### Trama de mando del H7 (FDCAN2, recepción)

- ID estándar **0x200**, 1 byte de datos (`uint8`, 0–255), enviada a 50 Hz por la
  placa core (STM32H723).
- Filtro hardware solo para 0x200 → RX FIFO 0 → interrupción `FDCAN2_IT0`.
  La interrupción solo guarda el último valor; el bucle principal lo aplica al servo.
- El enlace se considera caído si pasan más de 100 ms sin tramas
  (`canCommandLinkAlive` en el depurador).

## Servo (PB10, AIRBRAKE)

- PWM de 50 Hz con TIM2_CH3, el único timer al que se puede llevar PB10.
- El valor recibido por CAN se convierte linealmente en el ancho del pulso:
  0 → 0,5 ms (2,5 %), 255 → 2,5 ms (12,5 %).
- No se envían pulsos hasta que llega la primera trama. Si se cae el enlace, el
  servo mantiene la última posición.

## Configuración fuera de CubeMX

Hay periféricos configurados a mano, en secciones `USER CODE` o en módulos
propios. Sobreviven a una regeneración, pero **no hay que activar estos
periféricos en CubeMX para otra cosa** sin revisar estos módulos:

- **TIM2:** PWM del servo (`servo.c`). En el `.ioc`, PB10 sigue como salida GPIO
  `AIRBRAKE`; `Servo_Init()` toma el pin después de `MX_GPIO_Init()`.
- **TIM5:** timestamp en µs (`timestamp.c`).
- **Interrupciones `FDCAN1_IT0` y `FDCAN2_IT0`:** activadas en `fdcan.c`, con los
  handlers en `stm32g4xx_it.c`.
- **Interrupción de LPUART1** (GPS): activada en `usart.c`, con el handler en
  `stm32g4xx_it.c`.
- **EXTI de PA15 (PADS1_INT1):** comparte la línea 15 con PC15 (IMU1_INT1), que
  queda como entrada normal (ver `main.h` y `gpio.c`).