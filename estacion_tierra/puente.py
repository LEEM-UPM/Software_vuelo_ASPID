# -*- coding: utf-8 -*-
"""
Lado PC del puente USB <-> radio de la estación de tierra.

El firmware del otro lado vive fuera de los repos, en
Documents/Arduino/LEEM_Puente_Radio/, y es un módem transparente: no
interpreta nada, sólo pasa bytes. El formato de trama está documentado en
rf_g4_v0/README.md.

    radio -> serie   AA 55 | len | payload[len] | rssi | snr | df_lo df_hi | flags | crc8
    serie -> radio   AA 55 | len | payload[len] | crc8

Aquí se hacen tres cosas: leer el puerto en un hilo aparte, validar el CRC y
dejar las tramas en una cola que la interfaz vacía a su ritmo.

POR QUÉ UN HILO Y UNA COLA. Tkinter es de un solo hilo y no se puede llamar
desde otro. Y leer del puerto bloquea. Si se leyera dentro del bucle de la
interfaz, cada lectura congelaría la ventana; y si no se leyera bastante
rápido, el buffer del sistema se llenaría y se perderían paquetes. El hilo lee
a su aire, la cola absorbe los picos y la interfaz la vacía cuando puede.
"""

import queue
import threading
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None

SYNC0 = 0xAA
SYNC1 = 0x55
META = 5  # rssi, snr, df (2), flags

FLAG_CRC_MALO = 0x01
FLAG_LOCAL = 0x02
FLAG_TX_FALLO = 0x04

BAUDIOS = 115200


def crc8(datos):
    """CRC-8 con polinomio 0x07, el mismo que calcula el sketch."""
    c = 0
    for b in datos:
        c ^= b
        for _ in range(8):
            c = ((c * 2) ^ 0x07) & 0xFF if c & 0x80 else (c * 2) & 0xFF
    return c


def enmarcar(payload):
    """payload -> trama lista para mandar al puente."""
    n = len(payload)
    if not 1 <= n <= 255:
        raise ValueError("el payload debe medir entre 1 y 255 bytes")
    cuerpo = bytes([n]) + bytes(payload)
    return bytes([SYNC0, SYNC1]) + cuerpo + bytes([crc8(cuerpo)])


def puertos_disponibles():
    if list_ports is None:
        return []
    return [(p.device, p.description or "") for p in list_ports.comports()]


class Trama(object):
    """Una trama válida recibida del puente."""

    __slots__ = ("payload", "rssi_dbm", "snr_db", "df_hz", "flags", "t")

    def __init__(self, payload, rssi, snr, df, flags, t):
        self.payload = payload
        self.rssi_dbm = rssi
        self.snr_db = snr
        self.df_hz = df
        self.flags = flags
        self.t = t

    @property
    def es_acuse(self):
        """Trama generada por la propia placa, no recibida del aire."""
        return bool(self.flags & FLAG_LOCAL)

    @property
    def tx_ok(self):
        return not (self.flags & FLAG_TX_FALLO)

    @property
    def crc_lora_malo(self):
        return bool(self.flags & FLAG_CRC_MALO)


class Puente(object):
    """
    Lector del puerto serie. Se abre con arrancar() y se para con parar().

    Reintenta solo si el puerto desaparece (desenchufar el USB, un reset de la
    placa). En campo eso pasa, y no tiene sentido que haya que reiniciar la
    interfaz entera por ello.
    """

    def __init__(self, puerto, baudios=BAUDIOS, cola_max=2000):
        self.puerto = puerto
        self.baudios = baudios
        self.cola = queue.Queue(maxsize=cola_max)
        self.conectado = False
        self.ultimo_error = ""
        self.bytes_leidos = 0
        self.tramas_ok = 0
        self.tramas_crc_malo = 0
        self.tramas_descartadas = 0  # desechadas por cola llena
        self._sp = None
        self._hilo = None
        self._parar = threading.Event()
        self._candado_tx = threading.Lock()

    def arrancar(self):
        if serial is None:
            self.ultimo_error = "falta pyserial (pip install pyserial)"
            return False
        self._parar.clear()
        self._hilo = threading.Thread(target=self._bucle, name="puente",
                                      daemon=True)
        self._hilo.start()
        return True

    def parar(self, esperar=True):
        """esperar=False: no bloquea (para cambiar de puerto desde la
        interfaz); el hilo cierra el puerto él solo al salir del bucle."""
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

    def enviar(self, payload):
        """
        Pone un payload en el aire. Devuelve True si salió por el PUERTO; que
        la radio lo transmitiera de verdad lo confirma después el acuse
        (Trama.es_acuse).
        """
        if not self.conectado or self._sp is None:
            return False
        try:
            with self._candado_tx:
                self._sp.write(enmarcar(payload))
            return True
        except Exception as e:
            self.ultimo_error = str(e)
            return False

    def _bucle(self):
        while not self._parar.is_set():
            if self._sp is None:
                try:
                    self._sp = serial.Serial(self.puerto, self.baudios,
                                             timeout=0.1)
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

    def _leer(self):
        """
        Máquina de estados idéntica a la del sketch, pero al revés. Va byte a
        byte porque el sistema puede partir una trama en varias lecturas: el
        puerto serie no garantiza que llegue de una pieza.
        """
        buf = bytearray()
        estado = 0
        largo = 0

        while not self._parar.is_set():
            # in_waiting or 1: devuelve lo que haya YA, y si no hay nada espera
            # a un solo byte. NO usar read(512): medido con el CH340 en
            # Windows (2026-09-28), read(512) con timeout=0.1 se salta el
            # timeout y espera a juntar los 512 bytes, o sea 1030 ms. Los
            # paquetes entraban a rafagas de 16 con un segundo de silencio
            # entre medias: la cadencia medida salia al doble de la real y la
            # pantalla iba hasta un segundo por detras del cohete.
            datos = self._sp.read(self._sp.in_waiting or 1)
            if not datos:
                continue
            self.bytes_leidos += len(datos)

            for b in datos:
                if estado == 0:
                    if b == SYNC0:
                        estado = 1
                elif estado == 1:
                    if b == SYNC1:
                        estado = 2
                    elif b != SYNC0:
                        estado = 0
                elif estado == 2:
                    largo = b
                    buf = bytearray()
                    estado = 3
                elif estado == 3:
                    # Recoge payload y metadatos de un tirón. El acuse tiene
                    # largo 0 y cae aquí igual: son los 5 de metadatos.
                    buf.append(b)
                    if len(buf) >= largo + META:
                        estado = 4
                else:
                    if crc8(bytes([largo]) + bytes(buf)) == b:
                        self._entregar(largo, buf)
                    else:
                        self.tramas_crc_malo += 1
                    estado = 0

    def _entregar(self, largo, buf):
        payload = bytes(buf[:largo])
        m = largo
        rssi = buf[m] - 256 if buf[m] > 127 else buf[m]
        snr_b = buf[m + 1] - 256 if buf[m + 1] > 127 else buf[m + 1]
        df = buf[m + 2] | (buf[m + 3] * 256)
        if df > 32767:
            df -= 65536
        trama = Trama(payload, rssi, snr_b / 4.0, df, buf[m + 4],
                      time.monotonic())
        self.tramas_ok += 1
        try:
            self.cola.put_nowait(trama)
        except queue.Full:
            self.tramas_descartadas += 1

    def vaciar(self, maximo=200):
        """Saca de la cola lo que haya, sin bloquear nunca."""
        salida = []
        for _ in range(maximo):
            try:
                salida.append(self.cola.get_nowait())
            except queue.Empty:
                break
        return salida
