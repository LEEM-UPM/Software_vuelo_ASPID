# ASPID GNC (código generado) — versión 1.5

Copiado sin tocar de `GNC_V1_5/code` (entrega generada el 2026-10-06), con
Simulink / Embedded Coder para Cortex-M. **No editar a mano**: cualquier
cambio se hace en el modelo y se vuelve a generar. Las instrucciones de la
entrega están en `LEEME_INTEGRACION.txt`.

| Carpeta | Modelo | Ritmo | Qué hace |
|---|---|---|---|
| `nav/` | `nav` | 400 Hz | INS + EKF de 15 estados con las dos IMU, el barómetro y el GNSS |
| `state_machine/` | `state_machine` | 400 Hz | Estado del cohete, detección del despegue y comandos para nav |
| `gc/` | `gc` | 50 Hz (1 de cada 8) | Fase de vuelo, apogeo previsto, aerofreno y pirotecnia |
| `shared/` | — | — | Utilidades comunes (`rt_nonfinite`, `rtwtypes`...). **Una sola vez** |

Quien los ejecuta y los alimenta es `App/gnc.c`.

Estados del cohete (`state_machine`): 0 POWER_ON, 1 MANTENIMIENTO,
2 EN VUELO, 3 ERROR. **No es la numeración de la versión 1.1** (que tenía
READY en el 2 y EN VUELO en el 3): en el firmware se usan siempre las
constantes `GNC_ESTADO_*` de `App/gnc.h`.

Cada carpeta trae su `rtmodel.h`, distinto en cada una. No chocan: cada `.c`
incluye el de su propia carpeta, que es donde el compilador mira primero.

## Qué cambió de la 1.2 a la 1.5 (lo que toca a `App/gnc.c`)

- **Rampa:** `nav_tun.init_euler_rad` desaparece. Ahora son
  `nav_tun.rail_elev_deg` y `nav_tun.rail_azim_deg`, y el modelo se alinea
  **siempre con alabeo 0**. Como el alabeo de la placa en la rampa no se
  conoce, `gnc.c` lo mide con la gravedad (como antes) y gira las medidas
  sobre el eje del cohete para que nav lo vea a 0; las salidas se devuelven a
  los ejes de verdad.
- **Comandos para nav:** la máquina de estados saca `sm_out.nav_cmd` y hay
  que dárselo a nav en `nav_in.nav_cmd` en el tick siguiente. 10 = realinear
  (solo en POWER ON o MANTENIMIENTO), 11 = reiniciar la covarianza. El
  firmware usa el 10 para realinear en tierra en cuanto cambia la rampa.
- **Barómetro:** nav deja de usarlo por encima de Mach 0,75
  (`baro_mach_max`), por el ruido transónico.
- **Ajustables del guiado (`gc_tun`) de fábrica:** apogeo objetivo 3000 m,
  altura mínima de control 1200 m, **main a 450 m** y drogue 0,5 s después
  del apogeo.

**Fallo de la entrega, que sigue en la 1.5:** `nav.c` y `gc.c` definen los
dos `rt_powd_snf()` sin `static` (idénticas), y el enlazado da *multiple
definition*. Para no editar el código generado, `CMakeLists.txt` compila
`gc/gc.c` con `-Drt_powd_snf=gc_rt_powd_snf`. Hay que decírselo a GNC para
que la próxima entrega la lleve a `shared/`.

El banco PIL de esta versión está en la entrega original, en `pil/`: 72609
filas a 400 Hz (el vuelo entero, 181,5 s, con drogue y main) y 9077 del
guiado. Pasa entero, sin una discrepancia, compilado para el PC con el
`App/pil_test.c` de la placa. Ver `tools/pil_csv_to_bin.py` y
`App/pil_test.h`; con `tools/vuelo_from_pil.py` sale de ahí el `VUELO.BIN`
para SIMULAR RUTA.

Las versiones anteriores están en `Downloads/GNC_V1_2` (400 Hz, 35 s de
banco PIL) y `Downloads/Code_V_1_1` (todo a 100 Hz).

**Licencia:** los archivos llevan la cabecera *Academic License — for use in
teaching, academic research, and meeting course requirements at degree
granting institutions only.*
