# Puente de radio de tierra

Firmware de la placa de la estación de tierra: una **DX-PJ26 v1.2** (Blue Pill,
STM32F103C8T6) con un módulo **DX-LR30** (SX1262). Hace de módem transparente
entre el puerto USB del PC y la radio: no interpreta nada, quien decodifica es
la estación (`estacion_tierra/puente.py`).

## Compilar

Arduino IDE con el núcleo STM32 de STM32duino (placa Blue Pill F103C8) y la
librería **RadioLib 7.7.1**. Abrir la carpeta `LEEM_Puente_Radio`.

## Radio

Tiene que coincidir con la placa RF del cohete (`rf_g4_v0`):

| | |
|---|---|
| Frecuencia | 869,525 MHz (sub-banda 869,4–869,65 MHz: 10 % de ciclo de trabajo) |
| LoRa | BW 250 kHz, SF7, CR 4/5, preámbulo 8, sync word 0x34 |
| Potencia | 22 dBm |
| Corrección | `FREQ_OFFSET_HZ` -2000 Hz (el cristal de esta placa) |

## Tramas por el puerto serie (115200)

```
radio -> serie   AA 55 | len | payload[len] | rssi | snr | df_lo df_hi | flags | crc8
serie -> radio   AA 55 | len | payload[len] | crc8
```

`rssi` en dBm, `snr` en pasos de 0,25 dB, `df` error de frecuencia en Hz,
`crc8` polinomio 0x07 sobre `len`, payload y metadatos. `flags`: bit 0 CRC de
LoRa malo, bit 1 acuse local (lo genera la placa tras cada transmisión, con
`len` = 0), bit 2 la transmisión falló.

La explicación de cada decisión está en la cabecera del `.ino` y en el README
de `rf_g4_v0`. **No vigila el ciclo de trabajo**: todo lo que mande el PC sale
al aire.
