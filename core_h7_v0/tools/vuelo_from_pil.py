#!/usr/bin/env python3
"""Genera VUELO.BIN para la reproduccion de vuelo (App/gnc_playback.c).

Para el GNC v1.6 (y las anteriores): un registro por tick del GNC (416 Hz
con la v1.6, 400 hasta la v1.5),
sacado de pil/nav_inputs.csv de la entrega. La v1.5 trae el vuelo entero:
181,5 s, despegue a ~1,1 s, apogeo a ~22 s, drogue a ~26 s, main a 450 m
(~120 s) y suelo.

El archivo guarda las muestras TAL Y COMO LAS MANDA LA AVIONICA POR CAN2, no
como las quiere el modelo. Esa es la gracia: al reproducirlo, los datos pasan
por toda la cadena real (el giro de ejes de montaje, el promediado, la
decimacion del barometro, la conversion de unidades del GPS), en vez de
saltarsela. El dia que se grabe un vuelo de verdad desde la avionica, el
archivo entra sin tocar nada mas.

Conversiones respecto a los vectores PIL:

1. EJES. Los vectores vienen en ejes del modelo (X morro, Y derecha, Z abajo)
   y la placa los manda en los suyos (Z a lo largo del cohete). Se aplica la
   inversa de _board_to_body() de gnc.c:

       placa = (-cuerpo_z, cuerpo_y, cuerpo_x)

2. GPS. El modelo quiere latitud y longitud en radianes; el ICD de CAN2 manda
   enteros (1e-7 grados, mm, mm/s). La velocidad va en NED entera, como la
   da NAV-PVT.

3. VALIDEZ. Cada sensor lleva su data_valid del vector: el registro solo
   "entrega" ese sensor en los ticks en que el vector lo marca nuevo, igual
   que llegaria por CAN2.

Los comandos de telemetria del dataset (no hay ninguno en la v1.2 ni en la
v1.5) van en el registro por si algun dia los hay. La columna nav_cmd de la
v1.5 no se guarda: la saca la maquina de estados.

Uso:
    python tools/vuelo_from_pil.py <carpeta_pil> <carpeta_salida>
"""

import math
import os
import struct
import sys

MAGIC = b"LEEMVL12"
HEADER_BYTES = 32
NAME = "VUELO v1.5"

# imu1: accel[3] gyro[3] ts valid | imu2: accel[3] ts valid
# baro: press_pa temp ts valid
# gps:  lat_e7 lon_e7 alt_mm vn ve vd (mm/s) fix sats valid
# cmd:  id param valid
RECORD_FMT = "<3f3fdB3fdBffdBiiiiiiBBBBfB"
RECORD_BYTES = struct.calcsize(RECORD_FMT)
# Un tick del GNC: 416 Hz con la v1.6 (hasta la v1.5, 400 Hz)
HZ = 416.0
TICK_S = 1.0 / HZ


def main():
    if len(sys.argv) != 3:
        raise SystemExit(__doc__)
    src_dir, dst_dir = sys.argv[1], sys.argv[2]
    os.makedirs(dst_dir, exist_ok=True)

    import csv
    nav = list(csv.DictReader(open(os.path.join(src_dir, "nav_inputs.csv"),
                                   encoding="utf-8")))
    sm = list(csv.DictReader(open(os.path.join(src_dir,
                                               "state_machine_inputs.csv"),
                                  encoding="utf-8")))
    assert len(nav) == len(sm), "nav y state_machine con distinto numero de filas"

    packer = struct.Struct(RECORD_FMT).pack
    body = bytearray()
    cuenta = {"imu1": 0, "imu2": 0, "baro": 0, "gps": 0, "cmd": 0}

    for r, s in zip(nav, sm):
        f = lambda k: float(r[k])
        i = lambda k: int(float(r[k]))

        # --- IMU1 (baja g) y IMU2 (alta g), cuerpo -> placa ---
        ax, ay, az = f("imu_lowg.accel_mps2_1"), f("imu_lowg.accel_mps2_2"), \
            f("imu_lowg.accel_mps2_3")
        gx, gy, gz = f("imu_lowg.gyro_rps_1"), f("imu_lowg.gyro_rps_2"), \
            f("imu_lowg.gyro_rps_3")
        hx, hy, hz = f("imu_highg.accel_mps2_1"), f("imu_highg.accel_mps2_2"), \
            f("imu_highg.accel_mps2_3")
        imu1_v = i("imu_lowg.data_valid")
        imu2_v = i("imu_highg.data_valid")

        # --- Barometro ---
        baro_v = i("baro.data_valid")

        # --- GPS ---
        lat_e7 = int(round(math.degrees(f("gnss.pos_lla_1")) * 1e7))
        lon_e7 = int(round(math.degrees(f("gnss.pos_lla_2")) * 1e7))
        alt_mm = int(round(f("gnss.pos_lla_3") * 1000.0))
        vn = int(round(f("gnss.vel_ned_1") * 1000.0))
        ve = int(round(f("gnss.vel_ned_2") * 1000.0))
        vd = int(round(f("gnss.vel_ned_3") * 1000.0))
        gps_v = i("gnss.data_valid")

        # --- Comando de telemetria (de los vectores de la maquina de estados)
        cmd_v = int(float(s["telemetry_cmd.cmd_valid"]))

        for k, v in (("imu1", imu1_v), ("imu2", imu2_v), ("baro", baro_v),
                     ("gps", gps_v), ("cmd", cmd_v)):
            cuenta[k] += 1 if v else 0

        body += packer(
            -az, ay, ax, -gz, gy, gx, f("imu_lowg.timestamp_s"), imu1_v,
            -hz, hy, hx, f("imu_highg.timestamp_s"), imu2_v,
            f("baro.press_pa"), f("baro.temp_C"), f("baro.timestamp_s"), baro_v,
            lat_e7, lon_e7, alt_mm, vn, ve, vd,
            i("gnss.fix_type"), i("gnss.num_sats"), gps_v,
            int(float(s["telemetry_cmd.cmd_id"])),
            float(s["telemetry_cmd.cmd_param"]), cmd_v)

    rows = len(nav)
    head = struct.pack("<8sIII12s", MAGIC, RECORD_BYTES, rows, 27,
                       NAME.encode("ascii"))
    assert len(head) == HEADER_BYTES
    dst = os.path.join(dst_dir, "VUELO.BIN")
    with open(dst, "wb") as fo:
        fo.write(head)
        fo.write(body)

    duration = rows * TICK_S
    print("VUELO.BIN  %d B/registro  %d ticks de %.0f Hz  %.1f s  %.2f MB"
          % (RECORD_BYTES, rows, HZ, duration,
             (HEADER_BYTES + len(body)) / 1048576.0))
    for k in ("imu1", "imu2", "baro", "gps"):
        print("  %-5s %6d muestras = %6.1f Hz" % (k, cuenta[k],
                                                 cuenta[k] / duration))
    print("  comandos de telemetria: %d" % cuenta["cmd"])
    print("\nCopiar %s a la raiz de la SD1." % dst)


if __name__ == "__main__":
    main()
