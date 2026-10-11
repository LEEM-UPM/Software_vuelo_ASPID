# Estación de tierra

Interfaz en Python para ver la telemetría del cohete y mandarle órdenes.

```
python estacion.py                 busca solo el puerto del puente
python estacion.py --puerto COM11  lo fuerza
python estacion.py --demo          vuelo simulado, sin tocar hardware
python estacion.py --demo-tierra   igual, pero sin despegar: para probar el modo test
python estacion.py --demo-emergencia  la core "se reinicia" a los 20 s de vuelo
python estacion.py --listar        enumera los puertos serie y sale
python estacion.py --sin-guardar   no guarda nada en disco (para pruebas)
python estacion.py --carpeta-datos RUTA   guarda ahí en vez de en el escritorio
python estacion.py --cats COM12    fuerza el puerto de la estación de CATS
python estacion.py --sin-cats      no lee CATS
```

**No hace falta enchufar nada antes de abrirla.** Si el puente o la estación de CATS
no están al arrancar (o se desenchufan y vuelven con otro COM), se siguen buscando
cada 2 s y se conectan solos. Con `--puerto` / `--cats` se usa ese puerto y ningún otro.

**Todo lo que llega se guarda** en `Escritorio/LEEM_datos/<fecha_hora>/`,
una carpeta por cada vez que se abre la estación. Ver
[Datos guardados](#datos-guardados).

**¿Vas a escribir otra estación?** Todo lo que se emite por radio, el
formato del puerto serie del puente y los comandos que entiende el cohete
están en [PROTOCOLO.md](PROTOCOLO.md).

Sólo necesita **Python 3 con Tkinter** (viene de serie) y **pyserial**. Nada
más: ni Qt, ni matplotlib, ni un navegador. En el campo, una dependencia que
no hay que instalar es una que no puede fallar.

## De dónde salen los datos

```
cohete --(LoRa 869,525 MHz)--> Blue Pill con el puente --(USB serie)--> esta interfaz
```

La Blue Pill (`Documents/Arduino/LEEM_Puente_Radio/`) es un módem
transparente: entrega el payload tal cual llegó por el aire y le añade al
final el RSSI, el SNR y la desviación de frecuencia. El formato de trama está
en `rf_g4_v0/README.md`.

Por esa radio llegan **dos paquetes distintos**:

| paquete | tamaño | cuándo | qué dice |
|---|---|---|---|
| vuelo (`App/flight_packet.h`) | 24 B | 3 Hz en tierra, 15 Hz desde el lanzamiento | dónde está y qué hace |
| estado (`App/status_packet.h`) | 32 B | 3 veces al arrancar, y cuando se pide | si está todo entero |
| acuse (`App/uplink.h`) | 12 B (8 con firmware anterior) | en respuesta a algunos comandos | qué valor ha quedado guardado y cómo oyó el cohete el comando |

**Cómo se distinguen, que tiene trampa.** El de vuelo se reconoce por *medir
24 bytes*, no por su primer byte: ese byte es un contador y vale 0xA5 una vez
cada 256 paquetes. El de estado se reconoce por medir 32 **y además** empezar
por 0xA5. Las dos condiciones a la vez, y en ese orden (`decodificar_cualquiera`).

El de estado mide 32 y no los 26 que ocupan sus campos porque **CAN FD sólo
admite ciertas longitudes** (0–8, 12, 16, 20, 24, 32, 48, 64): al pedir 26 el
driver rellena con ceros hasta 32 y eso es lo que sale al aire. Mejor decirlo
y quedarse los 6 bytes de sobra.

## Datos guardados

Cada vez que se abre, la estación crea una carpeta nueva y va guardando todo
lo que recibe (`grabador.py`). La ruta sale en la primera línea del registro.

```
Escritorio/LEEM_datos/2026-10-04_17-30-12/      (_DEMO si es --demo)
  LEEME.txt                     qué es cada archivo y cada columna
  registro.txt                  el registro de la pantalla
  crudo/tramas.csv              TODAS las tramas del puente, en hex
  vuelo/tramo_01_17-30-12.csv   paquetes de vuelo reales, por tramos
  estado/estado.csv             paquetes de estado
  acuses/acuses.csv             respuestas a comandos (ping, ángulo...)
  camaras/camaras.csv           paquetes de cámaras (si graban, tarjeta...)
  comandos/enviados.csv         cada comando que salió al aire
  simulaciones/sim_01_17-41-05/ vuelo.csv y estado.csv de esa simulación
```

- **CSV** con coma y punto decimal, cabecera en la primera fila. Cada fila
  lleva la hora del PC al llegar la trama (con ms) y los segundos de sesión.
  Los paquetes van decodificados a unidades, con las tuplas abiertas en
  columnas (`accel_g_x`, `quat_w`, `sd_estado_sd1`...), y además con el RSSI,
  SNR y error de frecuencia con los que los oyó tierra y sus bytes en hex.
- **Lo simulado nunca va a `vuelo/`.** Cuenta como simulación desde que se
  pide hasta que el cohete dice que ha terminado; la columna `sim_confirmada`
  dice si el cohete ya la había confirmado. Lo que dice el cohete se recuerda
  hasta el siguiente paquete de estado (no caduca a los 3 s como en la
  pantalla), para que perder unos estados no parta una simulación.
- **Tramos:** un archivo de vuelo nuevo tras más de 20 s sin paquetes de vuelo
  o si el cohete se reinicia (su tiempo encendido va hacia atrás).
- **`crudo/tramas.csv` es la referencia**: si se cambia un decodificador, todo
  lo demás se puede regenerar desde ahí.
- Se vacía al disco cada segundo. Un error al guardar **no para la
  estación**: se avisa una vez en rojo en el registro y se sigue.

## Los módulos

| archivo | qué hace |
|---|---|
| `paquete.py` | interpreta los dos paquetes y monta los comandos. **Espejo de `flight_packet.c` y `status_packet.c`** |
| `puente.py` | lee el puerto serie en un hilo, valida CRC, encola tramas |
| `widgets.py` | paleta, fuentes, primitivas de dibujo y el `Panel` base |
| `paneles.py` | cada recuadro de la pantalla |
| `estacion.py` | el estado compartido, la distribución y el bucle |
| `grabador.py` | guarda en disco todo lo que llega, ordenado por tipo |
| `cats.py` | lee la estación de tierra de CATS (el otro ordenador de vuelo) |

La separación no es capricho: **toda la interpretación del paquete está en
`paquete.py` y toda la acumulación en `Estado`**. Los paneles sólo presentan.
En cuanto un panel empieza a hacer cuentas, ese cálculo acaba duplicado en
otro sitio y los dos se separan sin que nadie se entere.

## Colores

Los **datos** usan los colores con significado: verde lo normal, ámbar aviso,
rojo alarma, rosa lo que viene de CATS. Los **marcos** de los paneles van por
familias, solo para agrupar (`widgets.ACENTOS`): verde vuelo, azul
navegación, cian radio, violeta sistema. La decoración nunca usa rojo, ámbar
ni rosa, para que no se confunda con un aviso. Un panel nuevo se pone en su
familia añadiendo su título a `ACENTOS`.

## Cómo añadir cosas

La pantalla está pensada con hueco de sobra:

- La fila **RANURAS RESERVADAS**, con 24 casillas ya rotuladas (GPS, satélites,
  barómetros, IMU, raíles de la power, servo, Mach…).
- Los **6 bytes reservados** del paquete de estado (25 a 31), que van a cero
  para que una estación antigua lea cero y no basura.

Para meter un indicador nuevo:

1. Si el dato no viene aún en ningún paquete, añádelo primero en
   `App/flight_packet.c` o `App/status_packet.c` **y en `paquete.py`**, con la
   misma escala. Lo que cambie poco va al de estado, no al de vuelo: el de
   vuelo paga aire 15 veces por segundo.
2. Si hace falta historia, añade la cola en `Estado` y rellénala en
   `Estado.encajar()`.
3. Escribe una clase `Panel` en `paneles.py` con su `pintar()`.
4. Cuélgala del grid en `Estacion._construir()`.

Nada más se entera. Los paneles no se conocen entre ellos.

## Avisos que conviene leer antes de fiarse de la pantalla

**El paquete de vuelo no lleva número de versión.** Si cambia una escala en
`flight_packet.c` y no se cambia aquí, esta interfaz enseña números creíbles y
equivocados, sin avisar de nada. No hay forma automática de detectarlo. El de
estado sí lo lleva (byte 1), que para eso se hizo después.

**`nav_valida` dice que el filtro se cree sus números, no que sean ciertos.**
En el banco del 2026-09-28, con la aviónica sin GPS, el paquete llegaba con
`nav OK`, la altura por −350 m y la actitud cayendo 1,7 °/s, todo con la placa
quieta encima de la mesa. La pantalla lo pinta tal cual porque eso es lo que
manda el cohete; el criterio lo pones tú.

**La elevación pierde resolución cerca de la vertical.** El cuaternión va a 8
bits por componente (~0,9° en cada una), pero la elevación sale de un arcoseno
y ese se aplana en los 90°: medido en simulación, a 88° reales el paquete
devuelve 84,9°. A 45° el error baja a décimas de grado. Es del formato del
paquete, no de esta interfaz, y en la rampa da igual porque ahí ya sabemos que
está vertical.

**Los paquetes con el CRC de LoRa malo se cuentan pero NO se decodifican.**
Sus bytes son basura. Aparecen en el registro y en el contador «aire» del
panel ENLACE porque saber que llegó algo corrupto es justo la señal que marca
el borde de la cobertura, pero no se pintan como si fueran datos.

**El ciclo de trabajo se estima, no se mide.** Se cuentan los paquetes
recibidos por 30,9 ms de aire cada uno, sobre una ventana de una hora. Los que
se pierdan por el camino no se cuentan, así que la barra se queda **corta**:
es una cota inferior de lo que el cohete está radiando de verdad.

## El mando y el enlace de subida

El camino de subida **ya existe de punta a punta**:

```
estacion.py --(serie)--> puente --(LoRa)--> placa RF --(CAN1 0x241)--> core/uplink.c
```

Un comando son 4 bytes: `0x5A | id | parámetro | XOR`. El XOR va *además* del
CRC de LoRa porque son cosas distintas: el CRC dice que los bits llegaron como
salieron, no que lo que salió fuera un comando nuestro. En el aire hay más
cosas, y la marca más el XOR hacen falsísimo que un paquete ajeno se ejecute.

Comandos implementados en el cohete (`App/uplink.h`):

| id | comando | parámetro |
|---|---|---|
| `0x01` | pedir estado | — |
| `0x10` | modo test | 1 entrar o renovar, 0 salir |
| `0x11` | pirotécnico | canal 1=1A 2=1B 3=2A 4=2B (ojo: empieza en 1) |
| `0x12` | simular ruta | 1 empezar, 0 parar |
| `0x13` | ángulo de lanzamiento | **16 bits**: centésimas de grado desde la vertical; `0xFFFF` consulta |

Los comandos con parámetro de 16 bits van en un formato de 5 bytes:
`0x5A | id | param_lo | param_hi | XOR`. `paquete.comando_para()` elige el
formato que toca.

ARMAR, DESARMAR y ABORTAR siguen dibujados y apagados hasta que existan allí.

## CATS

El cohete lleva además un **CATS Vega** ([catsystems.io](https://catsystems.io)),
un ordenador de vuelo comercial con su propia radio (2,4 GHz) y su propia
estación de tierra. Esa estación se enchufa al PC por USB y la interfaz la
lee a la vez que nuestro puente (`cats.py`).

**Conexión.** Se busca sola: es un ESP32-S2 por USB, con VID 0x303A (Espressif) o 0x239A (Adafruit, el de nuestra estación); ver `cats.VIDS_CATS`.
Si no la encuentra lo dice en el registro y sigue sin ella; `--cats COMx` la
fuerza. Es un puerto USB CDC: la velocidad da igual, **pero no abrirlo nunca a
1200 baudios**, que la reinicia en su bootloader. La búsqueda del puente
nunca coge el puerto de CATS.

**Qué manda.** Una línea de texto por paquete recibido (firmware
`cats-embedded`, `Utils::streamUsb`):

```
Link 1: Ts: 12.3, State: 3, Lat: 47.12345, Lon: 8.12345, Alt: 512, Vel: 87, V: 8.1
```

link del receptor, tiempo del CATS, estado (INVALID, CALIBRATING, READY,
THRUSTING, COASTING, DROGUE, MAIN, TOUCHDOWN), latitud y longitud, altura y
velocidad vertical **enteras** y su batería. Ojo con los negativos: el
firmware imprime la parte entera y los decimales por separado, así que
-3,73 sale `-3.-73000` (y -0,0007, `0.-0070`); `cats.py` lo junta bien.

**Botón DATOS.** Cambia de quién se pintan los datos que tienen los dos:
altura, velocidad vertical, fase y recorrido (los medidores ALT y VEL, los
paneles de perfil, velocidad, fase, VUELO y RECORRIDO). Con CATS se ven en
rosa y con la marca **CATS** en cada panel, para que no se confundan con los
nuestros. Todo lo demás (actitud, aerofreno, salud, enlace, la batería del
cohete) sigue siendo nuestro. Si CATS lleva más de 3 s sin mandar, el botón
dice `CATS (sin datos)`.

- **Fases:** THRUSTING es BOOST, COASTING es COAST y DROGUE, MAIN y
  TOUCHDOWN coinciden. CATS no distingue el control del aerofreno ni el
  apogeo.
- **Recorrido:** CATS da latitud y longitud, y el nuestro va en metros desde
  donde se alineó el GNC. El de CATS se pasa a metros desde su primer fix de
  GPS.

Todo lo que llega queda en `cats/cats.csv`, con la línea original. Los
cambios de estado de CATS salen en el registro. En `--demo` hay un CATS
fingido (10 Hz, con su propio error de altura) para probar el botón.

## Cerrojo de la rampa

El botón **RAMPA** del mando es un seguro contra un clic sin querer con el
cohete cargado en la rampa. **La estación arranca siempre con el cerrojo
puesto** (`RAMPA: BLOQUEADO`, en verde) y para quitarlo pide una contraseña
(`CLAVE_DESBLOQUEO` en `estacion.py`). Ponerlo otra vez es un clic, sin
contraseña.

Con el cerrojo puesto **no sale al aire nada que pueda poner en peligro el
cohete**:

| bloqueado | por qué |
|---|---|
| entrar en modo test | de él cuelgan la pirotecnia y las pruebas de aerofreno |
| pirotécnicos | |
| SIMULAR RUTA | dispara los pirotécnicos de verdad |
| prueba y extensión del aerofreno | mueven el servo |

Sigue permitido todo lo que **para** algo (salir del modo test, parar la
simulación o la prueba, recoger el aerofreno al 0 %), y también el ping, pedir
el estado y las cámaras. Así nunca se queda nada peligroso en marcha sin
poder pararlo. **El ángulo y el rumbo de la rampa no se bloquean**: se fijan
con el cohete ya en la rampa, y el GNC se realinea con ellos al momento.

**Al ponerlo** la estación saca al cohete del modo test, para una simulación
y olvida los envíos de aerofreno que estuvieran reintentándose. El registro
dice cada orden que el cerrojo para.

Es un seguro **de la estación**: el cohete no lo sabe. Otra estación, o una
prueba por consola (`prueba_radio.py`), no lo tiene.

## Modo test y pirotécnicos

El botón **MODO TEST** de abajo cambia el panel RECORRIDO por el de pruebas,
desde donde se encienden los cuatro pirotécnicos. Hay casillas libres en
OTRAS PRUEBAS para lo que vaya haciendo falta: un botón en
`pruebas.py::_construir_otras()` y su comando en `App/uplink.c`.

Las protecciones, en el orden en que actúan:

1. **El cohete sólo entra en modo test en tierra**: estado POWER ON,
   MANTENIMIENTO o ERROR y fase IDLE o EN TIERRA. Nunca en vuelo. Con el GNC
   v1.2 ya no hay READY: en la rampa el cohete está en POWER ON hasta que
   despega, así que **el modo test se acepta en la rampa**. Por eso la
   estación lleva el cerrojo de la rampa (arriba).
   Si se deniega, el panel dice por qué en vez de quedarse esperando.
2. **La pantalla enseña lo que dice el cohete, no lo que se ha pulsado.** Los
   botones de disparo sólo se habilitan cuando un paquete de estado de hace
   menos de 3 s confirma el modo test. En modo test el cohete manda uno por
   segundo, así que si el enlace se cae, los botones se apagan solos.
3. **Dos clics por disparo.** El primero arma el botón (rojo, CONFIRMAR) 3 s;
   el segundo dispara. Armar otro desarma el anterior.
4. **Hombre muerto en las dos direcciones.** Mientras el panel esté abierto la
   estación renueva el modo test cada 5 s; si se cierra la estación o se cae
   el enlace, el cohete sale solo a los 120 s. Y si al arrancar la estación se
   encuentra el cohete en modo test sin haberlo pedido, insiste en sacarlo.
5. **Si el cohete sale de tierra en modo test, el modo test cae** en ese mismo
   paso del bucle.

Cada disparo manda una **ráfaga de 5**: con el 30 % de acierto medido en la
subida, uno solo falla dos de cada tres veces, y cinco llegan el 83 %.
Repetirlo no hace daño: power mantiene el canal 1 s y lo apaga solo. La
estación **no reintenta sola** un disparo que no se confirma; eso lo decide
una persona.

La única confirmación de que un disparo llegó es el contador **aceptados por
el cohete** del panel (y la línea roja del registro), que sale del paquete de
estado. «Enviados desde aquí» sólo dice lo que se ha mandado.

**La petición automática.** Mientras no haya llegado ningún paquete de estado,
la estación lo pide cada `PERIODO_PETICION_S` (5 s), y deja de pedirlo en
cuanto llega uno: el estado no cambia solo. Sólo lo pide **si el enlace está
vivo**; si no entra telemetría, el cohete no está ahí o no nos oye, y radiar
peticiones al vacío sólo gasta aire en la dirección que menos sobra.

**Los comandos tienen que ser idempotentes.** La subida no tiene acuse y la RF
está sorda mientras transmite (a 15 Hz en vuelo, casi la mitad del tiempo; a
3 Hz en tierra, un 9 %), así que un
comando se pierde con facilidad y tierra lo repite. «Mándame el estado» dos
veces no hace daño; un comando de pirotecnia sí lo haría, y necesitará número
de secuencia, que todavía no está en el formato.

**PING ENLACE** sí funciona: manda cinco bytes por el aire y espera el acuse
del puente. No es una orden para el cohete, sirve para comprobar que el camino
físico PC → puente → radio está vivo. Verificado en banco: 4 de 4 acuses.


## Simular ruta

En OTRAS PRUEBAS del panel de pruebas está **SIMULAR RUTA** (dos clics). El
cohete reproduce `VUELO.BIN` de su SD1 y manda por radio lo que va saliendo,
a ritmo real, igual que en un vuelo. Sirve para ver en la estación lo que
llegaría durante un vuelo de verdad sin sacar el cohete.

- **Todo lo que se ve mientras tanto es simulado.** La cabecera lo dice en
  cian (`SIM 45 %`) en lugar de ENLACE OK, el registro avisa al empezar y al
  terminar, y las gráficas se limpian al empezar.
- **El aerofreno se mueve de verdad**, por eso pide dos clics. Lo mueve el
  mismo código que en vuelo (`App/airbrake.c`), así que solo sale mientras
  el GNC está EN VUELO; en tierra, en la rampa y bajando se queda recogido.
- **1A, 1B Y 2A SE ENCIENDEN DE VERDAD.** La simulación pasa por el mismo
  flujo de despliegue que el vuelo (`App/despliegue.c`): al llegar a DROGUE
  se encienden 1A y 1B a la vez y al llegar a MAIN se enciende 2A. Es a propósito, para validar
  ese código en el banco. **Simular con cargas de prueba, nunca con cargas
  reales.** El botón lo avisa al armarse y el registro lo marca en rojo.
- **Lo que sí se bloquea son los pirotécnicos mandados desde tierra**: el
  cohete cierra el modo test al empezar y deniega pirotécnicos manuales y modo
  test hasta que acabe.
- Termina sola al acabar el archivo, o con **PARAR SIMULACION** en la barra de
  abajo. Al terminar el cohete recoge el aerofreno, reinicia el GNC y vuelve a
  sus sensores.
- Igual que con el modo test, mientras el cohete no confirme se reintenta
  cada segundo. Si no hay `VUELO.BIN`, el registro lo dice con esas palabras.

Para probarlo sin hardware: `python estacion.py --demo-tierra`.

## Cámaras

El botón **CAMARAS** del mando es a la vez indicador y orden: dice lo que
dice el cohete (`CAMARAS: ON`, `CAMARAS: OFF`, o `CAMARAS ?` si aún no ha
llegado un paquete de estado de la versión 5) y al pulsarlo pide lo
contrario. La orden (`0x17`) se manda en ráfaga y se **reintenta cada
segundo hasta que el cohete la confirma**, con su acuse o con un paquete de
estado; mientras tanto el botón dice `ENCENDIENDO CAM.` / `APAGANDO CAM.`. A
los 30 s sin confirmación se rinde y lo avisa en rojo. El panel SISTEMA
también lo muestra.

Las cámaras arrancan apagadas, **se encienden solas al despegar** y **se
apagan solas al bajar de 50 m**, igual que el registro de la SD. En vuelo no
se pueden apagar desde aquí (el cohete lo deniega y se ve en el registro).

**Apagar es parar primero.** Al apagarlas (desde aquí o a los 50 m) el
cohete para la grabación de cada cámara, espera a confirmar que han parado y
solo entonces les corta la corriente, para que el vídeo se cierre bien. Son
unos 5 s; mientras tanto el botón y el panel SISTEMA dicen `CERRANDO VIDEOS`.

**Si graban de verdad.** Son dos cámaras (RunCam Split 4) y el cohete manda
un paquete de cámaras cada 5 s con ellas encendidas. El panel SISTEMA dice
cuántas graban (`CAMARAS ON  graban 1 de 2`: verde si todas las que
contestan graban, ámbar si alguna no contesta, rojo si una NO GRABA) y el
panel SALUD, debajo de las tarjetas, una línea por cámara: estado
(`GRABANDO 2:31`, `PARADA`, `NO CONTESTA`, `NO GRABA`...), resolución
(`1080p60 NTSC`), su tarjeta (libre/total y minutos que caben) y `reloj` si
ya tiene la hora del GPS. Los cambios importantes salen en el registro y
todo se guarda en `camaras/camaras.csv`. `NO GRABA` quiere decir que el
cohete pulsó para grabar y la tarjeta sigue sin llenarse: tarjeta llena,
rota o demasiado lenta (la microSD de 4 GB de las primeras pruebas no
grababa). Detalle en `PROTOCOLO.md` (§4.3).

## Reinicio en vuelo (EMERGENCIA)

Si la core se reinicia en el aire (un fallo, el watchdog o un corte), vuelve
en menos de un segundo y sigue el vuelo solo con los barómetros
(`App/emergencia.h` en el core). Aquí se ve así:

- El panel SISTEMA pone el estado **EMERGENCIA** en rojo y debajo un recuadro
  rojo: `SOLO BARO  apogeo SI/no  main SI/no  baro fiable/NO FIABLE`.
- En el registro: el reinicio, cuándo el barómetro deja de ser fiable (ruido o
  saltos, p. ej. cerca de Mach 1) y cuándo vuelve, el **apogeo** detectado
  (10 m por debajo de la máxima: drogue, 1A y 1B) y la **altura del main**
  (400 m: 2A).
- La altura y la velocidad vertical son del barómetro; el mapa y la actitud
  no valen (no hay GNC).
- Si se reinicia **después** del vuelo, sale `POSTVUELO`: no toca la SD (la
  del vuelo queda como se cerró), no enciende las cámaras ni dispara nada.

Para verlo sin hardware: `python estacion.py --demo-emergencia`.

## Pruebas de aerofreno

Al lado de SIMULAR RUTA, en OTRAS PRUEBAS (solo con el modo test confirmado):

- **PROBAR AEROFRENO** (dos clics): el cohete saca el aerofreno del todo,
  espera 0,5 s y lo recoge, tres veces, y luego lo saca poco a poco en 2 s y
  lo recoge poco a poco en otros 2 s. 7 s en total.
- **Casilla de % y ENVIAR EXTENSION** (un clic, o Intro en la casilla): deja
  el aerofreno en ese % de la carrera hasta otra orden o hasta salir del modo
  test. 0 lo recoge. Fuera de 0..100 la casilla lo dice y no manda nada.

Las dos se reintentan cada ~0,5 s hasta que el cohete contesta, y la línea
AEROFRENO de debajo dice en qué están. El medidor y el paquete de vuelo
enseñan ahora la extensión que **de verdad** se manda al servo, así que la
prueba se ve moverse en pantalla.

Cada pulsación de PROBAR AEROFRENO lleva un número de prueba nuevo: el cohete
ejecuta cada número una sola vez, así que un reintento que llegue tarde no
la repite. Al salir del modo test el aerofreno vuelve a recogido.

El % es de la CARRERA del actuador, no del ángulo del servo: el mecanismo es
una biela-manivela y el cohete hace la cuenta (`App/airbrake.h`).

## Ángulo de lanzamiento

El botón **ANGULO LANZ.** cambia el RECORRIDO por el panel del ángulo de la
rampa, **medido desde la vertical** (0 = vertical). Se elige arrastrando el
cohete de la ruleta, con la rueda del ratón encima (0,5°; 0,1° con
Mayúsculas) o escribiéndolo en la casilla. **ENVIAR AL COHETE** lo manda.

- Al abrir el panel se le pregunta al cohete qué tiene, para que la ruleta
  arranque ahí.
- Se reintenta cada ~0,5 s hasta que el cohete **contesta con ese mismo
  valor**. El acuse trae lo que el cohete tiene guardado; si coincide, está
  confirmado. Si no contesta en 60 s, el panel lo dice.
- En la ruleta, el cohete verde es lo elegido aquí y la línea cian
  discontinua es lo que tiene el cohete. Si no coinciden, el panel avisa de
  que el valor está **sin enviar**.
- El cohete lo guarda en las dos SD y lo recuerda tras un reinicio.
- En vuelo y simulando se deniega.
- **Todavía no lo usa el GNC**: está en PENDIENTES.


**Rumbo.** Debajo de la casilla del ángulo está la del **rumbo**: hacia dónde
se inclina el morro, en grados desde el **norte geográfico** en sentido
horario (0 N, 90 E, 180 S, 270 O). Con la rampa vertical da igual. ENVIAR
manda los dos, cada uno con sus reintentos.

El cohete los pasa al GNC (cabeceo = 90 − ángulo, guiñada = rumbo) y **mide
él mismo el alabeo** (cómo está girada la placa) con el acelerómetro. El EKF
se alinea una vez, al arrancar y quieto: lo que llegue antes se usa en esa
alineación; lo que llegue después, al reiniciar. Si el ángulo no cuadra con
cómo está el cohete (más de 10° entre la gravedad medida y la esperada), el
EKF **no se alinea** y la consola del core lo dice. Ojo en el banco: con la
placa plana, el ángulo tiene que ser ~0.
