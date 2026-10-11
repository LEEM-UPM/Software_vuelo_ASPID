"""
Recepción de un CATS (catsystems.io): un ordenador de vuelo comercial (Vega)
que el cohete lleva además de nuestra aviónica, con su propia estación de
tierra y su propia radio (2,4 GHz, nada que ver con la nuestra).

CÓMO SE CONECTA. La estación de tierra de CATS (ESP32-S2) se enchufa al PC por
USB y aparece como un puerto serie virtual (USB CDC: la velocidad da igual,
pero NUNCA abrirlo a 1200 baudios, que la reinicia en su bootloader). Por ahí
saca UNA LÍNEA DE TEXTO por cada paquete que recibe del cohete. Del firmware
(cats-embedded, ground_station/src/utils.cpp, Utils::streamUsb):

    Link 1: Ts: 12.3, State: 3, Lat: 47.12345, Lon: 8.12345, Alt: 512, Vel: 87, V: 8.1

    Link    receptor de la estación (1 o 2)
    Ts      tiempo del ordenador de vuelo desde que arrancó [s, 0,1]
    State   0 INVALID 1 CALIBRATING 2 READY 3 THRUSTING 4 COASTING
            5 DROGUE 6 MAIN 7 TOUCHDOWN
    Lat/Lon grados (vienen con 4 decimales de verdad)
    Alt     altura estimada [m, entera]
    Vel     velocidad vertical estimada [m/s, entera]
    V       batería del CATS [V, 0,1]

TRAMPA DEL SIGNO. El firmware imprime parte entera y decimales por separado,
así que con coordenadas negativas el signo cae en los decimales: -3,7 sale
"-3.-70000", y -0,5 sale "0.-50000". Aquí se junta el signo de las dos partes.

Además del dato, por el mismo puerto salen otros mensajes de su consola: se
ignora cualquier línea que no tenga esa forma.
"""

import math
import queue
import re
import threading
import time

import paquete as pq

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None

# Así se encuentra el puerto solo. La estación es un ESP32-S2 por USB nativo,
# pero el VID depende de la placa: 0x303A (Espressif) en las genéricas, y la
# nuestra (2026-10-11, COM11) sale con 0x239A (Adafruit, PID 0x80AB).
VID_ESPRESSIF = 0x303A
VID_ADAFRUIT = 0x239A
VIDS_CATS = (VID_ESPRESSIF, VID_ADAFRUIT)

# USB CDC: cualquier valor sirve MENOS 1200 (reinicia en el bootloader)
BAUDIOS = 115200

# Sin recibir nada este tiempo, se vuelve a despertar (LectorCats._despertar)
SILENCIO_S = 5.0

ESTADOS = ("INVALID", "CALIBRATING", "READY", "THRUSTING", "COASTING",
           "DROGUE", "MAIN", "TOUCHDOWN")

# Batería del CATS: LiPo 3S (manda 11,3-11,5 V, que solo cuadra con 3 celdas).
# El porcentaje sale de la misma tabla que la nuestra (paquete.porcentaje_lipo).
CELDAS_BATERIA = 3

# Estado de CATS -> nuestra fase (paquete.FASES). CATS no distingue el control
# del aerofreno ni el apogeo: COASTING es nuestro COAST y del DROGUE en
# adelante coinciden.
FASE_PROPIA = {0: 0, 1: 0, 2: 0, 3: 1, 4: 2, 5: 5, 6: 6, 7: 7}

_ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]")
_NUM = r"(-?\d+)\.(-?\d+)"
_LINEA = re.compile(
    r"Link\s*(\d+):\s*Ts:\s*" + _NUM + r",\s*State:\s*(\d+),\s*"
    r"Lat:\s*" + _NUM + r",\s*Lon:\s*" + _NUM + r",\s*"
    r"Alt:\s*(-?\d+),\s*Vel:\s*(-?\d+),\s*V:\s*" + _NUM)


def _decimal(entera, decimales, escala):
    """'-3', '-70000', 100000 -> -3,7. El signo puede venir en cualquiera de
    las dos partes (ver TRAMPA DEL SIGNO). La escala es FIJA y no sale del
    número de cifras: el firmware imprime con %05ld y el signo ocupa una de
    las cinco, así que -0,0007 sale "0.-0070" (cuatro cifras)."""
    negativo = entera.startswith("-") or decimales.startswith("-")
    v = int(entera.lstrip("-")) + int(decimales.lstrip("-")) / float(escala)
    return -v if negativo else v


class PaqueteCats(object):
    """Un paquete de CATS, con los nombres de los nuestros donde se solapan
    (altura_m, vel_vertical_mps, fase) para poder pintarlos igual."""

    def __init__(self, link, ts, estado, lat, lon, alt, vel, bateria, t,
                 linea):
        self.link = link
        self.ts_s = ts
        self.estado = estado
        self.lat = lat
        self.lon = lon
        self.altura_m = float(alt)
        self.vel_vertical_mps = float(vel)
        self.bateria_v = bateria
        self.t = t            # time.monotonic() al recibirla
        self.linea = linea    # tal cual llegó, para el registro

    @property
    def estado_nombre(self):
        return ESTADOS[self.estado] if 0 <= self.estado < len(ESTADOS) else \
            "?%d" % self.estado

    @property
    def fase(self):
        return FASE_PROPIA.get(self.estado, -1)

    @property
    def bateria_pct(self):
        """Carga de SU batería (no la del cohete), 0 a 100; None sin dato."""
        if self.bateria_v <= 0.0:
            return None
        return pq.porcentaje_lipo(self.bateria_v, CELDAS_BATERIA)

    @property
    def gps_valido(self):
        # El firmware de CATS manda 0/0 sin fix (y así lo trata él)
        return not (self.lat == 0.0 and self.lon == 0.0)


def decodificar(linea, t=None):
    """Una línea del puerto -> PaqueteCats, o None si no es un dato."""
    limpia = _ANSI.sub("", linea).strip()
    m = _LINEA.search(limpia)
    if m is None:
        return None
    g = m.groups()
    return PaqueteCats(
        link=int(g[0]), ts=_decimal(g[1], g[2], 10), estado=int(g[3]),
        lat=_decimal(g[4], g[5], 100000), lon=_decimal(g[6], g[7], 100000),
        alt=int(g[8]), vel=int(g[9]), bateria=_decimal(g[10], g[11], 10),
        t=time.monotonic() if t is None else t, linea=limpia)


def a_norte_este(lat, lon, lat0, lon0):
    """Metros al norte y al este de (lat0, lon0). Plano: de sobra para los
    pocos km de un vuelo."""
    norte = (lat - lat0) * 111320.0
    este = (lon - lon0) * 111320.0 * math.cos(math.radians(lat0))
    return norte, este


def buscar_puerto():
    """El primer puerto con un VID de VIDS_CATS (la estación de CATS), o
    None."""
    if list_ports is None:
        return None
    for p in list_ports.comports():
        if getattr(p, "vid", None) in VIDS_CATS:
            return p.device
    return None


def es_puerto_cats(dispositivo):
    """Para que la búsqueda del puente no se quede con el de CATS."""
    if list_ports is None:
        return False
    for p in list_ports.comports():
        if p.device == dispositivo:
            return getattr(p, "vid", None) in VIDS_CATS
    return False


class LectorCats(object):
    """
    Lee la estación de CATS en un hilo y deja los paquetes en una cola, igual
    que puente.Puente (ver allí por qué). Si el puerto desaparece, reintenta.
    Solo lee: a CATS no se le manda nada.
    """

    def __init__(self, puerto, cola_max=2000):
        self.puerto = puerto
        self.cola = queue.Queue(maxsize=cola_max)
        self.conectado = False
        self.ultimo_error = ""
        self.lineas = 0        # todas, sean datos o no
        self.paquetes = 0
        self.descartados = 0   # cola llena
        self.despertares = 0   # bajadas y subidas de DTR (_despertar)
        self._sp = None
        self._hilo = None
        self._parar = threading.Event()

    def arrancar(self):
        if serial is None:
            self.ultimo_error = "falta pyserial (pip install pyserial)"
            return False
        self._parar.clear()
        self._hilo = threading.Thread(target=self._bucle, name="cats",
                                      daemon=True)
        self._hilo.start()
        return True

    def parar(self, esperar=True):
        """esperar=False: no bloquea, como en puente.Puente.parar()."""
        self._parar.set()
        if not esperar:
            return
        if self._hilo is not None:
            self._hilo.join(timeout=2.0)
        self._cerrar()

    def _cerrar(self):
        if self._sp is not None:
            try:
                self._sp.close()
            except Exception:
                pass
            self._sp = None
        self.conectado = False

    def _bucle(self):
        while not self._parar.is_set():
            if self._sp is None:
                try:
                    self._sp = serial.Serial(self.puerto, BAUDIOS, timeout=0.1)
                    self.conectado = True
                    self.ultimo_error = ""
                except Exception as e:
                    self.ultimo_error = str(e)
                    self.conectado = False
                    time.sleep(1.0)
                    continue
            try:
                self._leer()
            except Exception as e:
                self.ultimo_error = str(e)
                self._cerrar()
                time.sleep(1.0)
        self._cerrar()

    def _despertar(self):
        """
        La estación solo manda mientras ve subir DTR: así sabe que hay alguien
        al otro lado. Al cerrar y volver a abrir el puerto, Windows no
        siempre da esa subida, y se queda callada sin ningún error (medido el
        2026-10-11 en COM11: 5 aperturas de 6 mudas, y las 5 despertaron
        bajando y subiendo DTR). Saca su banner, que se ignora.
        """
        self._sp.dtr = False
        time.sleep(0.1)
        self._sp.dtr = True
        self.despertares += 1

    def _leer(self):
        pendiente = b""
        self._despertar()
        ultimo = time.monotonic()
        while not self._parar.is_set():
            datos = self._sp.read(self._sp.in_waiting or 1)
            if not datos:
                # Callada: o no le llega nada del cohete, o se ha dormido.
                # Despertarla de más no hace daño.
                if time.monotonic() - ultimo > SILENCIO_S:
                    self._despertar()
                    ultimo = time.monotonic()
                continue
            ultimo = time.monotonic()
            pendiente += datos
            while b"\n" in pendiente:
                linea, pendiente = pendiente.split(b"\n", 1)
                self.lineas += 1
                p = decodificar(linea.decode("utf-8", "replace"))
                if p is None:
                    continue
                self.paquetes += 1
                try:
                    self.cola.put_nowait(p)
                except queue.Full:
                    self.descartados += 1
            if len(pendiente) > 4096:   # basura sin fin de línea
                pendiente = b""

    def vaciar(self, maximo=200):
        salida = []
        for _ in range(maximo):
            try:
                salida.append(self.cola.get_nowait())
            except queue.Empty:
                break
        return salida


class SimuladorCats(object):
    """
    Para --demo: líneas como las de la estación de CATS a partir del mismo
    vuelo que finge estacion.Simulador, con sus propios errores (otra
    altura, velocidad entera, GPS a 4 decimales) para que al cambiar de
    fuente se note. Pasa por decodificar(), como las de verdad.
    """

    LAT0, LON0 = 40.45, -3.73   # Madrid: longitud NEGATIVA (la trampa)

    def __init__(self, simulador):
        self.sim = simulador
        self._proximo = 0.0

    def lineas(self):
        """Las que tocaban desde la última llamada, a 10 Hz."""
        ahora = time.monotonic()
        if self._proximo == 0.0:
            self._proximo = ahora
        salida = []
        while self._proximo <= ahora:
            salida.append(self._una(self._proximo - self.sim.t0))
            self._proximo += 0.1
        return salida

    def _una(self, t):
        fase, h, v, _ = self.sim._perfil(t)
        estado = {0: 2, 1: 3, 2: 4, 3: 4, 4: 4, 5: 5, 6: 6, 7: 7}.get(fase, 2)
        h = h * 1.03 + 2.0                       # su barómetro, su error
        norte, este = 0.05 * h, 0.02 * h         # deriva con la altura
        lat = self.LAT0 + norte / 111320.0
        lon = self.LON0 + este / (111320.0 * math.cos(math.radians(self.LAT0)))
        return _linea_como_cats(1, t, estado, lat, lon, int(h), int(v), 11.4)


def _linea_como_cats(link, ts, estado, lat, lon, alt, vel, bateria):
    """Como Utils::streamUsb, CON su forma de imprimir los negativos."""
    def par(x, dec):
        e = int(x)                     # trunca hacia cero, como en C
        d = int((x - e) * (10 ** dec))
        return "%d.%0*d" % (e, dec, d)
    lat = round(lat, 4)
    lon = round(lon, 4)
    return ("Link %d: Ts: %s, State: %d, Lat: %s, Lon: %s, Alt: %d, Vel: %d, "
            "V: %s" % (link, par(ts, 1), estado, par(lat, 5), par(lon, 5),
                       alt, vel, par(bateria, 1)))
