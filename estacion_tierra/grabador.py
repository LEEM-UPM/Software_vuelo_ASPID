# -*- coding: utf-8 -*-
"""
Guardado de todo lo que llega del cohete, desde que se abre la estación.

Cada ejecución crea su propia carpeta en el escritorio:

    Escritorio/LEEM_datos/AAAA-MM-DD_HH-MM-SS/      (_DEMO en modo demo)
      LEEME.txt                    qué hay en cada sitio y qué es cada columna
      registro.txt                 el registro de la interfaz, tal cual
      crudo/tramas.csv             TODO lo que llega del puente, sin interpretar
      vuelo/tramo_NN_HH-MM-SS.csv  paquetes de vuelo REALES, por tramos
      estado/estado.csv            paquetes de estado (salud del cohete)
      acuses/acuses.csv            respuestas a comandos (ping, ángulo...)
      camaras/camaras.csv          paquetes de cámaras (graban, tarjeta...)
      comandos/enviados.csv        lo que se ha mandado al cohete
      simulaciones/sim_NN_HH-MM-SS/vuelo.csv, estado.csv

LO SIMULADO VA APARTE. Los paquetes de vuelo de una simulación van a la
carpeta de esa simulación y no a vuelo/: si no, un análisis posterior podría
tomar una simulación por un vuelo. Se considera que hay simulación desde que
tierra la pide hasta que el cohete dice que ha terminado. Se empieza a contar
al PEDIRLA y no cuando el cohete la confirma porque los paquetes simulados
empiezan a llegar antes que el paquete de estado que lo confirma; si el
cohete la deniega, esa carpeta se queda con unos pocos paquetes de tierra, y
la columna sim_confirmada lo dice.

Lo que dice el cohete se recuerda hasta el siguiente paquete de estado, no
caduca a los pocos segundos como en la pantalla: perder tres paquetes de
estado seguidos en mitad de una simulación no debe partirla en dos ni mandar
la mitad a vuelo/.

TRAMOS. La telemetría de vuelo se parte en archivos cuando hay un hueco de
más de TRAMO_HUECO_S sin paquetes de vuelo, o cuando el cohete se reinicia
(su tiempo encendido vuelve hacia atrás en el paquete de estado). Así cada
archivo es un trozo continuo, que es como se quiere analizar.

crudo/tramas.csv es la referencia: si alguna vez se cambia un decodificador,
todo lo demás se puede volver a sacar de ahí.

NUNCA DEBE TIRAR LA ESTACIÓN. Cualquier error al escribir se guarda en
self.fallo (la estación lo anota una vez) y se sigue: perder el guardado es
malo, perder la pantalla en mitad de un vuelo es peor.
"""

import csv
import os
import time

import paquete as pq

# Hueco sin paquetes de vuelo a partir del cual empieza un tramo nuevo
TRAMO_HUECO_S = 20.0

# Cada cuánto se vacían los archivos al disco (si se cuelga el PC, se pierde
# como mucho esto)
VACIAR_CADA_S = 1.0

# Nombres de las columnas de los campos que son tuplas
_PARTES = {
    "pirotecnia": ("1A", "1B", "2A", "2B"),
    "accel_g": ("x", "y", "z"),
    "quat": ("w", "x", "y", "z"),
    "sd_estado": ("sd1", "sd2"),
    "sd_test_ok": ("sd1", "sd2"),
    "sd_cd": ("sd1", "sd2"),
    "sd_arranques": ("sd1", "sd2"),
    "sd_libres_mb": ("sd1", "sd2"),
    "can_hz": ("can1", "can2"),
    "can_errores": ("can1", "can2"),
}

# Del paquete de vuelo, además de los campos, lo que la estación calcula
_DERIVADOS_VUELO = ("estado_nombre", "fase_nombre", "error_nombre",
                    "elevacion_deg", "rumbo_deg", "accel_modulo_g",
                    "velocidad_horizontal_mps", "norma_quat", "bateria_pct")


def carpeta_escritorio():
    """El escritorio de verdad, también si Windows lo ha movido a OneDrive o
    tiene otro nombre por el idioma."""
    try:
        import ctypes
        from ctypes import wintypes
        buf = ctypes.create_unicode_buffer(wintypes.MAX_PATH)
        # CSIDL_DESKTOPDIRECTORY = 0x10
        if ctypes.windll.shell32.SHGetFolderPathW(None, 0x10, None, 0,
                                                  buf) == 0 and \
                os.path.isdir(buf.value):
            return buf.value
    except Exception:
        pass
    for nombre in ("Desktop", "Escritorio"):
        p = os.path.join(os.path.expanduser("~"), nombre)
        if os.path.isdir(p):
            return p
    return os.path.expanduser("~")


def _hora_pc(t_mono):
    """La hora del PC en un instante de time.monotonic(), con milisegundos."""
    t = time.time() - (time.monotonic() - t_mono)
    return time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(t)) + \
        (".%03d" % int((t % 1.0) * 1000))


def _num(v):
    if isinstance(v, bool):
        return int(v)
    if isinstance(v, float):
        return "%.7g" % v
    return "" if v is None else v


def _plano(obj, campos):
    """Cabecera y valores de un paquete decodificado, con las tuplas
    abiertas en una columna por elemento."""
    cab, val = [], []
    for c in campos:
        if c == "crudo":
            continue
        v = getattr(obj, c, None)
        if isinstance(v, (tuple, list)):
            partes = _PARTES.get(c) or [str(i + 1) for i in range(len(v))]
            for nombre, x in zip(partes, v):
                cab.append("%s_%s" % (c, nombre))
                val.append(_num(x))
        else:
            cab.append(c)
            val.append(_num(v))
    return cab, val


class _Csv(object):
    """Un CSV que se crea, con su cabecera, al escribir la primera fila."""

    def __init__(self, ruta):
        self.ruta = ruta
        self._f = None
        self._w = None

    def fila(self, cabecera, valores):
        if self._f is None:
            os.makedirs(os.path.dirname(self.ruta), exist_ok=True)
            self._f = open(self.ruta, "w", newline="", encoding="utf-8")
            self._w = csv.writer(self._f)
            self._w.writerow(cabecera)
        self._w.writerow(valores)

    def vaciar(self):
        if self._f is not None:
            self._f.flush()

    def cerrar(self):
        if self._f is not None:
            self._f.close()
            self._f = None


class Grabador(object):

    def __init__(self, demo=False, base=None):
        base = base or os.path.join(carpeta_escritorio(), "LEEM_datos")
        nombre = time.strftime("%Y-%m-%d_%H-%M-%S") + ("_DEMO" if demo else "")
        self.dir = os.path.join(base, nombre)
        os.makedirs(self.dir, exist_ok=True)
        self.t0 = time.monotonic()
        self.fallo = None          # texto del primer error, si lo hubo
        self._vaciado = 0.0

        self._crudo = _Csv(os.path.join(self.dir, "crudo", "tramas.csv"))
        self._estado = _Csv(os.path.join(self.dir, "estado", "estado.csv"))
        self._acuses = _Csv(os.path.join(self.dir, "acuses", "acuses.csv"))
        self._camaras = _Csv(os.path.join(self.dir, "camaras",
                                          "camaras.csv"))
        self._comandos = _Csv(os.path.join(self.dir, "comandos",
                                           "enviados.csv"))
        self._registro = open(os.path.join(self.dir, "registro.txt"), "w",
                              encoding="utf-8")

        # Tramos de vuelo real
        self._n_tramo = 0
        self._tramo = None
        self._t_ultimo_vuelo = None
        self._ultimo_encendido = None

        # Simulaciones. _sim_cohete es lo que dijo el ultimo paquete de
        # estado, sin caducidad (ver la cabecera).
        self._sim_cohete = False
        self._n_sim = 0
        self._sim_vuelo = None
        self._sim_estado = None

        self._escribir_leeme(demo)

    # --- Entrada ---

    def trama(self, trama, sim_pedida):
        """Una trama del puente. sim_pedida: tierra ha pedido simular y el
        cohete todavia no ha dicho que ha terminado."""
        try:
            self._trama(trama, sim_pedida)
        except Exception as e:   # nunca tirar la estación
            self._fallar(e)

    def comando(self, cmd, parametro, automatico):
        try:
            t = time.monotonic()
            self._comandos.fila(
                ["hora_pc", "t_sesion_s", "cmd", "nombre", "parametro",
                 "bytes_hex", "automatico"],
                [_hora_pc(t), "%.3f" % (t - self.t0), "0x%02X" % cmd,
                 pq.UPLINK_NOMBRES.get(cmd, "?"), parametro,
                 pq.comando_para(cmd, parametro).hex(" "), int(automatico)])
        except Exception as e:
            self._fallar(e)

    def evento(self, hora, texto):
        try:
            self._registro.write("%s  %s\n" % (hora, texto))
        except Exception as e:
            self._fallar(e)

    def tic(self):
        """Llamar a menudo: vacía al disco cada VACIAR_CADA_S."""
        ahora = time.monotonic()
        if ahora - self._vaciado < VACIAR_CADA_S:
            return
        self._vaciado = ahora
        try:
            for c in self._todos():
                c.vaciar()
            self._registro.flush()
        except Exception as e:
            self._fallar(e)

    def cerrar(self):
        for c in self._todos():
            try:
                c.cerrar()
            except Exception:
                pass
        try:
            self._registro.close()
        except Exception:
            pass

    # --- Por dentro ---

    def _todos(self):
        return [c for c in (self._crudo, self._estado, self._acuses,
                            self._comandos, self._tramo, self._sim_vuelo,
                            self._sim_estado) if c is not None]

    def _fallar(self, e):
        if self.fallo is None:
            self.fallo = "%s: %s" % (type(e).__name__, e)

    def _trama(self, tr, sim_pedida):
        t = tr.t
        hora = _hora_pc(t)
        ts = "%.3f" % (t - self.t0)
        radio = [tr.rssi_dbm, "%.2f" % tr.snr_db, tr.df_hz]
        crudo_hex = bytes(tr.payload).hex(" ")

        if tr.es_acuse:
            tipo, p = "acuse_local_puente", None
        elif tr.crc_lora_malo:
            tipo, p = "crc_lora_malo", None
        else:
            tipo, p = pq.decodificar_cualquiera(tr.payload)
            if p is None:
                tipo = "desconocido"

        self._crudo.fila(
            ["hora_pc", "t_sesion_s", "tipo", "largo", "rssi_dbm", "snr_db",
             "df_hz", "flags", "payload_hex"],
            [hora, ts, tipo, len(tr.payload)] + radio +
            ["0x%02X" % tr.flags, crudo_hex])
        if p is None:
            return

        cab0 = ["hora_pc", "t_sesion_s"]
        val0 = [hora, ts]
        cab1 = ["rssi_tierra_dbm", "snr_tierra_db", "df_tierra_hz",
                "crudo_hex"]
        val1 = radio + [crudo_hex]

        if tipo == "vuelo":
            cab, val = _plano(p, pq.Paquete.__slots__)
            der = [getattr(p, d) for d in _DERIVADOS_VUELO]
            cab = cab0 + cab + list(_DERIVADOS_VUELO) + cab1
            val = val0 + val + [_num(x) for x in der] + val1
            if self._en_sim(sim_pedida):
                self._sim_vuelo.fila(["sim_confirmada"] + cab,
                                     [int(self._sim_cohete)] + val)
            else:
                self._tramo_para(t).fila(cab, val)
                self._t_ultimo_vuelo = t

        elif tipo == "estado":
            self._reinicio(p)
            self._sim_cohete = p.version >= 3 and p.simulando
            cab, val = _plano(p, pq.PaqueteEstado.__slots__)
            cab = cab0 + cab + ["motivo_denegacion_texto"] + cab1
            val = val0 + val + [p.motivo_denegacion_texto] + val1
            self._estado.fila(cab, val)
            # El que cierra la simulacion tambien va en su carpeta: dice
            # como acabo
            if self._en_sim(sim_pedida) or self._sim_estado is not None:
                self._sim_estado.fila(cab, val)
            self._en_sim(sim_pedida, cerrar=True)

        elif tipo == "camaras":
            cab, val = _plano(p, [c for c in pq.PaqueteCamaras.__slots__
                                  if c != "camaras"])
            cab = cab0 + cab + ["origen_texto"]
            val = val0 + val + [p.origen_texto]
            for i, c in enumerate(p.camaras):
                cc, vc = _plano(c, pq.Camara.__slots__)
                cab += ["cam%d_%s" % (i + 1, x) for x in cc] + \
                    ["cam%d_estado_nombre" % (i + 1)]
                val += vc + [c.estado_nombre]
            self._camaras.fila(cab + cab1, val + val1)

        elif tipo == "acuse":
            cab, val = _plano(p, pq.PaqueteAcuse.__slots__)
            cab = cab0 + ["nombre"] + cab + ["motivo_texto"] + cab1
            val = val0 + [pq.UPLINK_NOMBRES.get(p.cmd, "?")] + val + \
                [p.motivo_texto] + val1
            self._acuses.fila(cab, val)

    def _en_sim(self, sim_pedida, cerrar=False):
        """Abre la carpeta de una simulacion nueva cuando empieza y, si
        cerrar, la cierra cuando ha terminado. Devuelve si hay simulacion."""
        activa = self._sim_cohete or sim_pedida
        if activa and self._sim_vuelo is None:
            self._n_sim += 1
            d = os.path.join(self.dir, "simulaciones", "sim_%02d_%s"
                             % (self._n_sim, time.strftime("%H-%M-%S")))
            self._sim_vuelo = _Csv(os.path.join(d, "vuelo.csv"))
            self._sim_estado = _Csv(os.path.join(d, "estado.csv"))
        elif not activa and cerrar and self._sim_vuelo is not None:
            self._sim_vuelo.cerrar()
            self._sim_estado.cerrar()
            self._sim_vuelo = self._sim_estado = None
        return activa

    def _reinicio(self, p):
        """El cohete se ha reiniciado si su tiempo encendido va hacia atras:
        lo que venga despues es otro tramo."""
        enc = p.segundos_encendido
        if self._ultimo_encendido is not None and \
                enc + 2 < self._ultimo_encendido and self._tramo is not None:
            self._tramo.cerrar()
            self._tramo = None
        self._ultimo_encendido = enc

    def _tramo_para(self, t):
        if self._tramo is not None and self._t_ultimo_vuelo is not None and \
                t - self._t_ultimo_vuelo > TRAMO_HUECO_S:
            self._tramo.cerrar()
            self._tramo = None
        if self._tramo is None:
            self._n_tramo += 1
            self._tramo = _Csv(os.path.join(
                self.dir, "vuelo", "tramo_%02d_%s.csv"
                % (self._n_tramo, time.strftime("%H-%M-%S"))))
        return self._tramo

    def _escribir_leeme(self, demo):
        with open(os.path.join(self.dir, "LEEME.txt"), "w",
                  encoding="utf-8") as f:
            f.write(LEEME % {
                "inicio": time.strftime("%Y-%m-%d %H:%M:%S"),
                "demo": ("\nMODO DEMOSTRACION: todo lo de esta carpeta lo ha "
                         "inventado el simulador\nde la estacion. NO son "
                         "datos del cohete.\n" if demo else ""),
                "hueco": TRAMO_HUECO_S,
            })


LEEME = """DATOS RECIBIDOS POR LA ESTACION DE TIERRA LEEM
Sesion abierta: %(inicio)s
%(demo)s
Todo lo que ha llegado del cohete desde que se abrio la estacion. Formato:
CSV con coma de separador y punto decimal (en Excel: Datos > Desde texto/CSV,
delimitador coma). La primera fila de cada archivo es la cabecera.
Todos los archivos llevan hora_pc (la hora del PC al llegar la trama, con
milisegundos) y t_sesion_s (segundos desde que se abrio la estacion).
Los archivos se crean al llegar el primer dato de su tipo: si falta alguno,
es que no llego nada de eso.

crudo/tramas.csv
    TODO lo que ha dado el puente, sin interpretar: tambien los acuses
    locales del puente, los paquetes con el CRC de LoRa malo y los que no
    se sabe que son. payload_hex son los bytes tal cual. Es la referencia:
    todo lo demas se puede volver a sacar de aqui.

vuelo/tramo_NN_HH-MM-SS.csv
    Paquetes de vuelo (24 B) REALES, decodificados a unidades (ver
    tools/estacion/PROTOCOLO.md). Tambien los de tierra: el cohete manda
    telemetria todo el rato, no solo volando. Un tramo nuevo empieza si
    pasan mas de %(hueco).0f s sin paquetes de vuelo o si el cohete se reinicia.
    pirotecnia_1A..2B es la CONTINUIDAD de cada canal (1 = hay carga
    conectada), no si se ha disparado.
    Detras de los campos van los que calcula la estacion (nombres de estado,
    fase y error, elevacion y rumbo del morro, modulo de la aceleracion...).

simulaciones/sim_NN_HH-MM-SS/
    Desde que tierra pide simular hasta que el cohete dice que ha
    terminado: vuelo.csv (paquetes de vuelo SIMULADOS, de VUELO.BIN) y
    estado.csv. NO son vuelos. sim_confirmada = 0 en las primeras filas es
    normal (aun no habia llegado el paquete de estado que la confirma); si
    vale 0 en TODA la simulacion, el cohete la denego y esas filas son de
    tierra (el motivo esta en estado.csv).

estado/estado.csv
    Todos los paquetes de estado (salud: placas, SD, buses, modo test,
    simulacion, despliegue), tambien los de las simulaciones.

camaras/camaras.csv
    Paquetes de camaras: alimentacion, apagando (parando la grabacion antes
    de cortar la corriente) y, de cada camara (cam1_, cam2_): estado
    (0 sin corriente, 1 arrancando, 2 no contesta, 3 comprobando,
    4 GRABANDO, 5 parada, 6 parando), no_graba, reloj_puesto, resolucion
    (lineas, fps, pal), tarjeta (libre_mb, total_mb), queda_s (segundos de
    grabacion que caben), grabando_s y pulsaciones del boton.

acuses/acuses.csv
    Respuestas del cohete a comandos (ping, angulo, rumbo, aerofreno...).
    rx_rssi_dbm, rx_snr_db y rx_df_hz son como OYO EL COHETE el comando
    (vacios si la RF del cohete tiene el firmware anterior).

comandos/enviados.csv
    Cada comando que ha salido al aire, uno por fila: las rafagas y los
    reintentos tambien (por eso hay muchos mas que lineas "->" en
    registro.txt). automatico = 1 son las peticiones de estado que hace la
    estacion sola. En modo demo no sale nada al aire.

registro.txt
    El registro de la interfaz, linea a linea.

En los archivos de paquetes, rssi_tierra_dbm, snr_tierra_db y df_tierra_hz
son como oyo TIERRA ese paquete (lo mide el puente), y crudo_hex sus bytes.
"""
