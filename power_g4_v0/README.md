# power_g4_v0

## Robustez (2026-10-07)

- **Watchdog** (`Drivers/Watchdog`): IWDG de 1 s, arrancado justo antes del
  bucle principal (después de `AllPyroCheck()`, que bloquea 5 s si está
  activado). Si el bucle se cuelga, la placa se reinicia, vuelve a esperar el
  wake up del core (lo repite cada 2 s) y sigue aceptando pirotecnia.
  `Watchdog_LastResetWasWatchdog()` lo dice en el depurador.
- **Bus-off** (`Drivers/Can_driver/canBusOff`): FDCAN1 y FDCAN2 salen solos de
  bus-off (ICD 2.1), también mientras se espera el wake up. Sin esto, un mal
  contacto dejaba a power sorda a la pirotecnia hasta apagar el cohete.
- Un fallo (HardFault...) o un `Error_Handler()` reinician la placa en vez de
  dejarla colgada.

## Prueba de los pirotécnicos sin CAN (solo banco, sin cargas)

`PYRO_PRUEBA_SIN_CAN` (`Drivers/Pyro/pyro.h`): nada más arrancar enciende los
dos pares y los cuatro canales a la vez y los deja encendidos, sin CAN y sin
apagado automático. Se compila aparte para no tocar el firmware de vuelo:

```
cmake --preset Debug -B build/PruebaPiros -DCMAKE_C_FLAGS=-DPYRO_PRUEBA_SIN_CAN=1
cmake --build build/PruebaPiros
```

Para volver a volar, grabar el de `build/Debug`.
