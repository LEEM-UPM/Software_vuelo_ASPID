#!/usr/bin/env python3
"""Convierte los vectores PIL del GNC (v1.5; vale para la v1.2) a binario.

Con la v1.5 son 72609 filas a 400 Hz (el vuelo entero, 181,5 s) y unos 28 MB
de .BIN entre los seis.

Los CSV de la entrega obligarian a la placa a parsear texto y convertir
flotantes 72609 veces por modelo, que es lento y mete su propio redondeo.
Aqui se hace esa conversion una sola vez en el PC y la placa se limita a leer
registros y volcarlos sobre la estructura de entrada (App/pil_test.c).

Cada .BIN lleva una cabecera de 32 bytes para que la placa pueda comprobar que
el archivo es el que espera:

    char     magic[8]     "LEEMPI12"
    uint32   record_bytes
    uint32   record_count
    uint32   field_count
    char     name[12]     p.ej. "nav in"

y detras los registros, empaquetados sin relleno y en little endian.

EL ORDEN Y LOS TIPOS DE LOS CAMPOS SALEN DE PIL_manifest.txt, no de una lista
escrita a mano: el script lee las secciones ENTRADAS y SALIDAS de cada modelo
y comprueba que el CSV tiene todas esas columnas. La columna t_s (la primera)
no se escribe: solo sirve para alinear los archivos. Las estructuras de
App/pil_test.c tienen que seguir ese mismo orden; si el modelo se regenera
con otra interfaz, el tamaño de registro deja de cuadrar y la placa lo dice.

Uso:
    python tools/pil_csv_to_bin.py <carpeta_pil> <carpeta_salida>
"""

import csv
import os
import re
import struct
import sys

MAGIC = b"LEEMPI12"
# "uint32" sin _T: asi viene status_word en el manifiesto de la v1.6
TIPOS = {"real32_T": "f", "real_T": "d", "uint8_T": "B", "int32_T": "i",
         "uint32_T": "I", "uint32": "I"}

# modelo del manifiesto -> (prefijo del CSV, archivo de entradas, de salidas)
MODELOS = {
    "nav": ("nav", "PILNAVI.BIN", "PILNAVE.BIN"),
    "state_machine": ("state_machine", "PILSMI.BIN", "PILSME.BIN"),
    "gc": ("gc", "PILGCI.BIN", "PILGCE.BIN"),
}


def leer_manifiesto(ruta):
    """{modelo: {"ENTRADAS": [(campo, tipo)], "SALIDAS": [...]}}"""
    out = {}
    modelo = seccion = None
    for linea in open(ruta, encoding="utf-8"):
        m = re.match(r"=+ (\w+) +\(", linea)
        if m:
            modelo, seccion = m.group(1), None
            out[modelo] = {"ENTRADAS": [], "SALIDAS": []}
            continue
        if modelo is None:
            continue
        if linea.startswith("ENTRADAS"):
            seccion = "ENTRADAS"
            continue
        if linea.startswith("SALIDAS"):
            seccion = "SALIDAS"
            continue
        m = re.match(r"\s+(\S+)\s+(real32_T|real_T|uint8_T|int32_T|"
                     r"uint32_T|uint32)\s", linea)
        if m and seccion:
            out[modelo][seccion].append((m.group(1), TIPOS[m.group(2)]))
    return out


def convertir(csv_ruta, campos, bin_ruta, nombre):
    fmt = "<" + "".join(t for _, t in campos)
    packer = struct.Struct(fmt)
    filas = list(csv.DictReader(open(csv_ruta, encoding="utf-8")))
    falta = [c for c, _ in campos if c not in filas[0]]
    if falta:
        raise SystemExit("%s: faltan columnas %s" % (csv_ruta, falta))
    with open(bin_ruta, "wb") as f:
        f.write(struct.pack("<8sIII12s", MAGIC, packer.size, len(filas),
                            len(campos), nombre.encode("ascii")[:12]))
        for r in filas:
            vals = []
            for c, t in campos:
                v = float(r[c])
                vals.append(v if t in "fd" else int(round(v)))
            f.write(packer.pack(*vals))
    print("%-12s %4d B/registro  %6d registros  %2d campos  (%s)"
          % (os.path.basename(bin_ruta), packer.size, len(filas), len(campos),
             nombre))


def main():
    if len(sys.argv) != 3:
        raise SystemExit(__doc__)
    src, dst = sys.argv[1], sys.argv[2]
    os.makedirs(dst, exist_ok=True)
    man = leer_manifiesto(os.path.join(src, "PIL_manifest.txt"))

    for modelo, (prefijo, f_in, f_out) in MODELOS.items():
        if modelo not in man:
            raise SystemExit("el manifiesto no tiene el modelo %s" % modelo)
        convertir(os.path.join(src, prefijo + "_inputs.csv"),
                  man[modelo]["ENTRADAS"], os.path.join(dst, f_in),
                  modelo[:5] + " in")
        convertir(os.path.join(src, prefijo + "_expected.csv"),
                  man[modelo]["SALIDAS"], os.path.join(dst, f_out),
                  modelo[:5] + " exp")

    print("\nCopiar los seis .BIN a la raiz de la SD1 y poner PIL_TEST_ENABLE"
          " a 1 (App/pil_test.h).")


if __name__ == "__main__":
    main()
