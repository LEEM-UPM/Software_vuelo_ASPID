# ASPID GNC (código generado) — versión 1.6

Copiado sin tocar de `GNC_V1_6/code` (entrega generada el 2026-10-10), con
Simulink / Embedded Coder para Cortex-M. **No editar a mano**: cualquier
cambio se hace en el modelo y se vuelve a generar. Las instrucciones de la
entrega están en `LEEME_INTEGRACION.txt`.

| Carpeta | Modelo | Ritmo | Qué hace |
|---|---|---|---|
| `nav/` | `nav` | 416 Hz | INS + EKF de 15 estados con las dos IMU, el barómetro y el GNSS |
| `state_machine/` | `state_machine` | 416 Hz | Estado del cohete, detección del despegue y comandos para nav |
| `gc/` | `gc` | 52 Hz (1 de cada 8) | Fase de vuelo, apogeo previsto, aerofreno y pirotecnia |
| `shared/` | — | — | Utilidades comunes (`rt_nonfinite`, `rtwtypes`...). **Una sola vez** |

Quien los ejecuta y los alimenta es `App/gnc.c`.

Estados del cohete (`state_machine`): 0 POWER_ON, 1 MANTENIMIENTO,
2 EN VUELO, 3 ERROR. Fases (`gc`): 0 IDLE, 1 BOOST, 2 COAST sin control,
3 COAST con control, 4 APOGEO, 5 DROGUE, 6 MAIN, 7 EN TIERRA. En el
firmware se usan siempre las constantes `GNC_ESTADO_*` y `GNC_FASE_*`.

Cada carpeta trae su `rtmodel.h`, distinto en cada una. No chocan: cada `.c`
incluye el de su propia carpeta.

## Qué cambió de la 1.5 a la 1.6 (lo que toca al firmware)

- **416 Hz y 52 Hz** (antes 400 y 50). El barómetro, uno de cada 6 ticks
  (69,3 Hz). El alineado promedia 832 muestras (2 s quieto).
- **Aerofreno:** `actuator_cmd` es despliegue aerodinámico de 0 (paleta a
  ras de piel) a 0,90 (tope), no extensión. `App/airbrake.c` lo pasa a su
  extensión (0 = a ras, 1 = tope) dividiendo por 0,90, con la cinemática de
  verdad del mecanismo (ver `App/airbrake.h`: la entrega propone un acos
  sobre un mecanismo simplificado, con los mismos extremos).
- **El aerofreno se mueve fuera de EN VUELO** y el firmware ya deja pasar el
  mando en cualquier estado: un barrido al terminar de alinear (una vez por
  encendido) y todo fuera en ERROR habiendo volado (fallo en vuelo).
- **`nav_ready` y `nav_init_flags`:** listo para volar = alineado + 0 AGL +
  origen GNSS. **Sin fix de GPS no está listo** (flags = 3). Es lo que hay
  que mirar antes de armar el iniciador, no `nav_valid`. Sale por la consola,
  la SD y el paquete de estado (versión 7).
- **`status_word`:** todo el GNC en un uint32 (mapa en
  `LEEME_INTEGRACION.txt`). En la consola y en la SD (`palabra_gnc`).
- **`restore`** (reinyectar estado tras un reinicio): **no se usa**, `valid`
  a 0. Reiniciada en el aire, la core pasa al modo de emergencia, solo
  barómetro (`App/emergencia.h`), porque el filtro no se puede alinear con
  el cohete volando.
- **Ajustables nuevos del guiado:** control desde 1500 m (antes 1200),
  drogue no antes de 15 s de vuelo y forzado a los 30 s sin apogeo.
- **Pirotecnia armada solo pasados 100 m de verdad.**

**Fallo de la entrega, que sigue en la 1.6:** `nav.c` y `gc.c` definen los
dos `rt_powd_snf()` sin `static` (idénticas), y el enlazado da *multiple
definition*. `CMakeLists.txt` compila `gc/gc.c` con
`-Drt_powd_snf=gc_rt_powd_snf`. Hay que decírselo a GNC.

El banco PIL de esta versión está en la entrega original, en `pil/`: 76487
filas a 416 Hz (el vuelo entero, 183,9 s) y 9561 del guiado. Pasa entero,
sin una discrepancia, en el PC. Ver `tools/pil_csv_to_bin.py` y
`App/pil_test.h`; con `tools/vuelo_from_pil.py` sale el `VUELO.BIN` para
SIMULAR RUTA.

Versiones anteriores: `Downloads/GNC_V1_5`, `GNC_V1_2`, `Code_V_1_1`.

**Licencia:** los archivos llevan la cabecera *Academic License — for use in
teaching, academic research, and meeting course requirements at degree
granting institutions only.*
