# -*- coding: utf-8 -*-
"""
Estación de tierra del cohete: ventana principal.

    python estacion.py                 busca el puerto del puente solo
    python estacion.py --puerto COM11  lo fuerza
    python estacion.py --demo          vuelo simulado, sin hardware

QUÉ ENSEÑA. Dos paquetes distintos que llegan por la misma radio:

  vuelo   24 bytes a 15 Hz (flight_packet.h): dónde está y qué hace
  estado  32 bytes (status_packet.h): si está todo entero. Lo manda el cohete
          tres veces al arrancar, y si no ha llegado ninguno esta estación lo
          pide cada PERIODO_PETICION_S por el enlace de subida.

EL MANDO ESTÁ A MEDIAS. El camino de subida ya existe de punta a punta (tierra
-> puente -> LoRa -> placa RF -> CAN1 0x241 -> uplink.c), pero el único
comando implementado en el cohete es pedir el estado. ARMAR, DESARMAR,
ABORTAR y REARMAR están dibujados y apagados hasta que existan allí.

ESTÁ HECHA PARA CRECER. Hay huecos a propósito: la fila de ranuras V01..V24 y
los 6 bytes reservados del paquete de estado. Añadir algo es escribir un panel
en paneles.py y colgarlo del grid de abajo; nada más se entera.
"""

import argparse
import math
import random
import sys
import time
from collections import deque

import tkinter as tk
from tkinter import simpledialog

import angulo
import cats as ct
import grabador as grab
import paneles
import paquete as pq
import pruebas
import puente as pte
from widgets import (AMBAR, APAGADO, CIAN, FONDO, MAGENTA, REJILLA, ROJO,
                     VERDE, VERDE_MED, VERDE_OSC, PanelLibre, fuente)

# Aire que ocupa un paquete de 24 bytes a SF7/BW250 (ver flight_packet.h).
AIRE_POR_PAQUETE_S = 0.0309
# El 10 % de una hora. El límite es de la banda 869,4-869,65 MHz y se mide
# sobre una ventana de UNA HORA, no instantánea.
PRESUPUESTO_AIRE_S = 360.0
VENTANA_DUTY_S = 3600.0

# Sin paquete durante esto, el enlace se da por caído.
SILENCIO_ENLACE_S = 2.0

# Un salto del contador mayor que esto NO se cuenta como paquetes perdidos,
# se cuenta como un CORTE. A 15 Hz (en vuelo) son unos 2 s seguidos sin nada,
# a 3 Hz (en tierra) unos 11, y eso ya no
# es "se han perdido unos cuantos": o el enlace se cayo un rato, o el cohete
# se ha reiniciado y su contador ha vuelto a empezar. Metido en la misma cuenta
# que las perdidas sueltas, un simple reinicio enseñaba "PERDIDOS 248" y dejaba
# la cifra sin valor.
UMBRAL_CORTE = 32

REFRESCO_MS = 80  # 12,5 Hz: más no aporta y come CPU
CATS_LINK_S = 2.0  # sin paquetes por un link, su lámpara pasa a roja
CATS_BATERIA_S = 30.0  # media de la tensión de CATS (Estado.cats_bateria_pct)
PERIODO_BUSQUEDA_S = 2.0  # buscar el puente y CATS si no están (_vigilar_puertos)

# Cerrojo de la rampa (boton RAMPA en el MANDO). La estacion arranca SIEMPRE
# con el cerrojo puesto y hace falta esta contrasena para quitarlo. No es
# seguridad informatica: es un seguro contra un clic sin querer con el cohete
# cargado en la rampa. Ver Estacion._peligroso().
CLAVE_DESBLOQUEO = "1234"

# Cada cuánto se le pide al cohete el paquete de estado MIENTRAS NO HAYA
# LLEGADO NINGUNO. En cuanto llega uno se deja de pedir: el estado no cambia
# solo, y cada petición son ~15 ms de aire en la dirección que menos sobra.
PERIODO_PETICION_S = 5.0

# Modo test. El cohete lo cancela solo a los 120 s si no se renueva
# (UPLINK_TEST_TIMEOUT_MS); mientras el panel de pruebas este abierto se
# renueva cada RENOVAR_TEST_S. Con el 30 % de acierto medido en la subida,
# que fallen todas las renovaciones de 120 s seguidos es un 0,02 %.
RENOVAR_TEST_S = 5.0

# Mientras el cohete NO haya confirmado lo que se le ha pedido (entrar o
# salir), se reintenta cada esto. Con el 30 % de acierto de la subida, la
# confirmacion llega en ~3 s de media en vez de ~15 s con la renovacion lenta.
# En cuanto hay confirmacion se pasa a RENOVAR_TEST_S, que no hace falta
# gastar aire de subida en algo que ya esta hecho.
REINTENTO_TEST_S = 1.0

# Mientras el cohete siga en modo test y aqui ya se haya pedido salir, se
# insiste cada esto.
INSISTIR_SALIDA_S = REINTENTO_TEST_S

# Un paquete de estado mas viejo que esto ya no vale para decir que el cohete
# esta en modo test. En modo test manda uno por segundo, asi que tres seguidos
# perdidos significan que no se sabe en que esta, y entonces los botones de
# disparo se apagan.
FRESCURA_TEST_S = 3.0

def _con_jitter(periodo):
    """
    El periodo con una variacion aleatoria de +/-25 %. TODOS los reintentos de
    la subida tienen que pasar por aqui.

    POR QUE. La RF esta sorda mientras transmite, y transmite a 15 Hz: cada
    66,67 ms. Un reintento cada 1,000 s exactos son 15 periodos justos, asi
    que todos caen en la MISMA fase de la telemetria: si el primero cae en la
    ventana sorda, caen ahi todos. Medido el 2026-10-01 contra el cohete:
    0 de 40 peticiones con espaciado fijo de 1 s; 7 de 40 con espaciado al
    azar. La peticion automatica cada 5 s tenia el mismo problema (75
    periodos). Es la misma trampa que ya se vio con el ping cada 400 ms.
    """
    return periodo * random.uniform(0.75, 1.25)


# Angulo de lanzamiento: reintento hasta ver el acuse con el MISMO valor, y
# plazo tras el que se da por perdido. Con jitter, como todos (_con_jitter).
REINTENTO_ANGULO_S = 0.5
PLAZO_ANGULO_S = 60.0

# Pruebas de aerofreno: igual que el angulo, hasta ver el acuse
REINTENTO_AEROFRENO_S = 0.5
PLAZO_AEROFRENO_S = 30.0

# PING y PEDIR ESTADO: lo mismo, reintento hasta tener respuesta y plazo
REINTENTO_PETICION_S = 0.5
PLAZO_PETICION_S = 30.0

# Camaras: hasta cuando se reintenta encenderlas o apagarlas sin confirmacion
PLAZO_CAMARAS_S = 30.0

# Aceptaciones del mismo pirotecnico dentro de este rato son la misma rafaga
AGRUPAR_ACEPTADOS_S = 3.0


class Estado(object):
    """
    Todo lo que la interfaz sabe. Los paneles sólo leen de aquí.

    Guarda historia además del último valor porque casi ningún dato de vuelo
    se entiende suelto: lo que dice si el cohete sube es la pendiente, no la
    altura de este instante.
    """

    def __init__(self):
        self.paquete = None
        self.trama = None
        self.t_paquete = 0.0
        self.t_inicio = time.monotonic()

        # Cerrojo de la rampa: arranca PUESTO (ver CLAVE_DESBLOQUEO)
        self.bloqueado = True

        # CATS (cats.py), aparte de lo nuestro. 'fuente' dice de quien se
        # pintan los datos que se SOLAPAN (boton DATOS del mando).
        self.fuente = "ASPID"
        self.cats = None
        self.cats_recibidos = 0
        self.cats_hist_altura = deque(maxlen=400)
        self.cats_hist_vv = deque(maxlen=90)
        self.cats_pista = deque(maxlen=900)
        self.cats_origen = None   # (lat, lon) del primer fix: el 0 del mapa
        self.cats_bateria = deque()  # (t, V) de los últimos CATS_BATERIA_S
        self.cats_link_t = {}  # link (1, 2) -> t del último paquete por él
        self.cats_puerto = ""

        self.hist_altura = deque(maxlen=400)
        self.hist_vv = deque(maxlen=90)
        self.hist_rssi = deque(maxlen=160)
        self.hist_snr = deque(maxlen=160)
        self.pista = deque(maxlen=900)

        self.recibidos = 0
        self.perdidos = 0   # huecos cortos: perdidas de verdad
        self.cortes = 0     # huecos largos: caida del enlace o reinicio
        self.crc_malos = 0        # corruptos por el AIRE
        self.serie_crc_malos = 0  # corruptos por el CABLE
        self.acuses = 0
        self.puente_descartadas = 0

        self._sellos = deque()        # para la cadencia
        self._aire = deque()          # para el ciclo de trabajo
        self._ultimo_contador = None

        # Paquete de estado (salud). Es otro tipo de paquete, no una parte
        # del de vuelo: llega al arrancar el cohete y cuando se pide.
        self.estado_paquete = None
        self.t_estado = 0.0
        self.peticiones_estado = 0
        self.periodo_peticion_s = PERIODO_PETICION_S

        # Modo test: lo que se ha PEDIDO desde aqui. Lo que el cohete dice
        # que esta haciendo es otra cosa, ver modo_test_confirmado.
        self.modo_test_pedido = False
        self.intentos_test = 0   # envios desde la ultima pulsacion
        # Simulacion de ruta: lo pedido aqui. Lo que hace el cohete, en
        # simulando (sale del paquete de estado).
        self.simulacion_pedida = False
        self.intentos_sim = 0

        # Angulo de lanzamiento. Lo del cohete sale SOLO de sus acuses.
        self.angulo_pedido = None      # grados, lo ultimo mandado
        self.angulo_estado = ""        # consultando/enviando/confirmado/...
        self.angulo_motivo = ""
        self.angulo_cohete = None      # grados que tiene guardados el cohete
        self.angulo_definido_cohete = False
        self.angulo_guardado_sd = False
        self.intentos_angulo = 0
        self.t_angulo = 0.0
        self.angulo_recien_leido = False

        # Como oye el COHETE a tierra: sale de los acuses (RSSI, SNR y error
        # de frecuencia medidos por su radio al recibir el comando)
        self.cohete_oye = None       # (rssi, snr, df, monotonic, cmd, valor)

        # Camaras (App/camaras.h). Lo del cohete sale de sus paquetes de
        # estado (v5) y de los acuses del comando; None = no se sabe todavia.
        # Lo pedido se reintenta hasta verlo confirmado.
        self.camaras_cohete = None
        self.camaras_apagando = False  # parando la grabacion para cortar
        self.camaras_paquete = None    # el ultimo paquete de camaras
        self.t_camaras_paquete = 0.0
        self.camaras_pedidas = None    # True/False mientras se espera
        self.camaras_motivo = ""
        self.intentos_camaras = 0
        self.t_camaras = 0.0

        # Rumbo de la rampa: como el angulo. Lo del cohete, de sus acuses.
        self.rumbo_pedido = None
        self.rumbo_estado = ""         # consultando/enviando/confirmado/...
        self.rumbo_motivo = ""
        self.rumbo_cohete = None       # grados que tiene guardados el cohete
        self.rumbo_definido_cohete = False
        self.intentos_rumbo = 0
        self.t_rumbo = 0.0
        self.rumbo_recien_leido = False

        # Pruebas de aerofreno. Lo del cohete sale SOLO de sus acuses.
        self.aero_prueba_num = 0       # numero de la ultima prueba pedida
        self.aero_prueba_estado = ""   # enviando/en curso/terminada/...
        self.aero_prueba_motivo = ""
        self.aero_prueba_t = 0.0       # envio, o arranque confirmado
        self.intentos_aero_prueba = 0
        self.aero_ext_pedida = None    # %
        self.aero_ext_estado = ""      # enviando/confirmada/denegada/...
        self.aero_ext_motivo = ""
        self.aero_ext_cohete = None    # % que dice el cohete que tiene
        self.aero_ext_t = 0.0
        self.intentos_aero_ext = 0

        # PING (ida y vuelta con el cohete) y PEDIR ESTADO a mano
        self.ping_activo = False
        self.ping_t0 = 0.0
        self.ping_envios = {}      # numero de ping -> monotonic del envio
        self.intentos_ping = 0
        self.ultimo_rtt_ms = None
        self.estado_pedido = False
        self.t_estado_pedido = 0.0
        self.intentos_estado = 0
        self.disparos_enviados = 0
        self._ultimo_aceptado = None   # (canal, hora) para agrupar rafagas

        self.puerto_abierto = False
        self.nombre_puerto = "--"
        self.eventos = deque(maxlen=400)
        # Cuantos se han anotado en total. No vale len(eventos): el deque se
        # queda en 400 y, contando con eso, el registro dejaria de pintarse
        # a partir del evento 400.
        self.n_eventos = 0
        # Lo llama anotar() con cada evento (el guardado en disco)
        self.al_anotar = None

    # --- Derivados ---

    @property
    def edad_s(self):
        return time.monotonic() - self.t_paquete if self.t_paquete else 1e9

    @property
    def enlace_vivo(self):
        return self.edad_s < SILENCIO_ENLACE_S

    @property
    def hz(self):
        return float(len(self._sellos))

    @property
    def duty_fraccion(self):
        """Aire gastado en la última hora, como fracción del presupuesto."""
        return (len(self._aire) * AIRE_POR_PAQUETE_S) / PRESUPUESTO_AIRE_S

    @property
    def edad_estado_s(self):
        return time.monotonic() - self.t_estado if self.t_estado else None

    @property
    def falta_estado(self):
        return self.estado_paquete is None

    @property
    def simulando(self):
        """
        El COHETE dice que esta simulando, con un paquete reciente. Mientras
        sea verdad, TODO lo que se pinta son datos de VUELO.BIN y no de un
        vuelo real, y la pantalla tiene que dejarlo imposible de confundir.
        """
        s = self.estado_paquete
        if s is None or s.version < 3 or not s.simulando:
            return False
        edad = self.edad_estado_s
        return edad is not None and edad < FRESCURA_TEST_S

    # --- CATS ---

    # Lo que tienen los dos y se cambia con el boton DATOS. La bateria NO:
    # la de CATS es la suya, no la del cohete.
    SOLAPADOS = ("altura_m", "vel_vertical_mps", "fase")

    @property
    def ver_cats(self):
        return self.fuente == "CATS"

    def dato(self, attr):
        """Un dato para pintar: de CATS si se ha elegido CATS y es de los
        que se solapan; si no, de nuestro paquete de vuelo."""
        c = self.cats if (self.ver_cats and attr in self.SOLAPADOS) else \
            self.paquete
        return getattr(c, attr) if c is not None else None

    def hist(self, nombre):
        """Historia de 'altura', 'vv' o 'pista' de la fuente elegida."""
        if self.ver_cats:
            return {"altura": self.cats_hist_altura, "vv": self.cats_hist_vv,
                    "pista": self.cats_pista}[nombre]
        return {"altura": self.hist_altura, "vv": self.hist_vv,
                "pista": self.pista}[nombre]

    @property
    def edad_cats_s(self):
        return time.monotonic() - self.cats.t if self.cats else None

    @property
    def cats_bateria_pct(self):
        """
        Carga de la batería de CATS con la MEDIA de los últimos
        CATS_BATERIA_S. La tensión llega cuantizada a ~0,15 V (2026-10-11:
        en 1587 paquetes solo 11,3, 11,5 y 11,6, nunca 11,4) y bailando entre
        dos escalones; en la zona plana de la LiPo cada escalón son 15-20 %,
        así que paquete a paquete la barra saltaba del 29 al 48 %. La media
        de ese baile es estable, y la carga no cambia en 30 s.
        """
        if not self.cats_bateria:
            return None
        v = sum(x for _, x in self.cats_bateria) / len(self.cats_bateria)
        return pq.porcentaje_lipo(v, ct.CELDAS_BATERIA) if v > 0.0 else None

    def cats_link(self, link):
        """Un receptor de la estación de CATS: None si no ha oído nada en
        esta sesión, True si oyó algo hace menos de CATS_LINK_S, False si
        lo perdió."""
        t = self.cats_link_t.get(link)
        if t is None:
            return None
        return time.monotonic() - t < CATS_LINK_S

    def encajar_cats(self, p):
        previo = self.cats
        self.cats = p
        self.cats_recibidos += 1
        self.cats_bateria.append((p.t, p.bateria_v))
        self.cats_link_t[p.link] = p.t
        while p.t - self.cats_bateria[0][0] > CATS_BATERIA_S:
            self.cats_bateria.popleft()
        self.cats_hist_altura.append(p.altura_m)
        self.cats_hist_vv.append(p.vel_vertical_mps)
        if p.gps_valido:
            if self.cats_origen is None:
                self.cats_origen = (p.lat, p.lon)
                self.anotar("CATS: origen de su recorrido en %.5f, %.5f"
                            % (p.lat, p.lon), CIAN)
            self.cats_pista.append(ct.a_norte_este(p.lat, p.lon,
                                                   *self.cats_origen))
        if previo is None:
            self.anotar("CATS: primer paquete (link %d, %s, %.1f V)"
                        % (p.link, p.estado_nombre, p.bateria_v), CIAN)
        elif p.estado != previo.estado:
            self.anotar("CATS: estado -> %s" % p.estado_nombre, CIAN)

    def limpiar_historia(self):
        """Borra las graficas para que la simulacion empiece en blanco."""
        self.hist_altura.clear()
        self.hist_vv.clear()
        self.pista.clear()

    @property
    def modo_test_confirmado(self):
        """
        El COHETE dice que esta en modo test, con un paquete reciente.

        Es lo unico que habilita los botones de disparo. Lo que se haya
        pulsado aqui no cuenta: con la subida perdiendo el 70 % de los
        comandos, dar por hecho que una orden ha llegado es la forma mas
        rapida de creerse en un estado que no es.
        """
        s = self.estado_paquete
        if s is None or s.version < 2 or not s.test_activo:
            return False
        edad = self.edad_estado_s
        return edad is not None and edad < FRESCURA_TEST_S

    def reloj_sesion(self):
        s = int(time.monotonic() - self.t_inicio)
        return "%02d:%02d:%02d" % (s // 3600, (s // 60) % 60, s % 60)

    # --- Entrada ---

    def anotar(self, texto, color=VERDE_MED):
        hora = time.strftime("%H:%M:%S")
        self.eventos.append((hora, texto, color))
        self.n_eventos += 1
        if self.al_anotar is not None:
            self.al_anotar(hora, texto)

    def encajar(self, trama):
        """Mete una trama del puente en el estado. Devuelve True si era vuelo."""
        # La hora la pone el HILO LECTOR cuando la trama llega, no esta linea.
        # Si se usara time.monotonic() de aqui, cualquier atasco de la ventana
        # dejaria la cola acumulada y luego entraria todo de golpe con la misma
        # marca: la cadencia saldria al doble de la real. Medido: 31 Hz con el
        # cohete mandando a 15.
        ahora = trama.t

        if trama.es_acuse:
            self.acuses += 1
            if not trama.tx_ok:
                self.anotar("subida: el puente NO pudo transmitir", ROJO)
            return False

        if trama.crc_lora_malo:
            self.crc_malos += 1
            # Se cuenta pero NO se decodifica: los bytes son basura. Saber que
            # llegó algo corrupto es justo la señal del borde de cobertura.
            self.anotar("paquete con CRC malo (%d B)" % len(trama.payload),
                        AMBAR)
            return False

        tipo, p = pq.decodificar_cualquiera(trama.payload)

        if tipo == "estado":
            self._encajar_estado(p, trama, ahora)
            return False

        if tipo == "camaras":
            if p is not None:
                self._encajar_camaras(p, ahora)
            return False

        if tipo == "acuse":
            if p is not None and p.rx_valida:
                self.cohete_oye = (p.rx_rssi_dbm, p.rx_snr_db, p.rx_df_hz,
                                   ahora, p.cmd, p.valor)
            if p is not None and p.cmd == pq.UPLINK_CMD_PING:
                self._encajar_pong(p, ahora)
            else:
                self._encajar_acuse(p)
            return False

        if tipo != "vuelo" or p is None:
            self.anotar("payload de %d B, no es ningun paquete conocido"
                        % len(trama.payload), AMBAR)
            return False

        # Huecos, por el salto del contador
        if self._ultimo_contador is not None:
            salto = (p.contador - self._ultimo_contador) % 256
            if salto > 1:
                faltan = salto - 1
                if faltan > UMBRAL_CORTE:
                    self.cortes += 1
                    self.anotar("corte: %d paquetes seguidos sin llegar "
                                "(enlace caido o cohete reiniciado)" % faltan,
                                AMBAR)
                else:
                    self.perdidos += faltan
        self._ultimo_contador = p.contador

        fase_previa = self.paquete.fase if self.paquete else None
        error_previo = self.paquete.error if self.paquete else 0
        estado_previo = self.paquete.estado if self.paquete else None

        self.paquete = p
        self.trama = trama
        self.t_paquete = ahora
        self.recibidos += 1

        self.hist_altura.append(p.altura_m)
        self.hist_vv.append(p.vel_vertical_mps)
        self.hist_rssi.append(trama.rssi_dbm)
        self.hist_snr.append(trama.snr_db)
        self.pista.append((p.norte_m, p.este_m))

        self._sellos.append(ahora)
        while self._sellos and ahora - self._sellos[0] > 1.0:
            self._sellos.popleft()
        self._aire.append(ahora)
        while self._aire and ahora - self._aire[0] > VENTANA_DUTY_S:
            self._aire.popleft()

        if p.estado == pq.ESTADO_EMERGENCIA and \
                estado_previo != pq.ESTADO_EMERGENCIA:
            self.anotar("EMERGENCIA: la core se ha REINICIADO EN VUELO y sigue "
                        "solo con el barometro (fase %s)" % p.fase_nombre, ROJO)
        if fase_previa is not None and p.fase != fase_previa:
            self.anotar("FASE -> %s" % p.fase_nombre, CIAN)
        if p.error and p.error != error_previo:
            self.anotar("ERROR %d: %s" % (p.error, p.error_nombre), ROJO)
        return True


    def _encajar_pong(self, a, ahora):
        """
        Respuesta a un PING. Trae el numero de ping que contesta, asi que la
        ida y vuelta se mide desde ESE envio, no desde el ultimo: con los
        reintentos puede contestar uno anterior.
        """
        if not self.ping_activo or a.valor not in self.ping_envios:
            return
        rtt = (ahora - self.ping_envios[a.valor]) * 1000.0
        self.ultimo_rtt_ms = rtt
        self.ping_activo = False
        oye = ("   el cohete lo oyo con RSSI %d dBm, SNR %d dB, error de "
               "frecuencia %+d Hz" % (a.rx_rssi_dbm, a.rx_snr_db, a.rx_df_hz)
               if a.rx_valida else "")
        self.anotar("PONG: el cohete contesta, ida y vuelta %.0f ms "
                    "(intento %d)%s" % (rtt, self.intentos_ping, oye), VERDE)

    def _encajar_acuse(self, a):
        """
        Un acuse trae el valor que el cohete tiene GUARDADO. Lo que se envio
        solo esta confirmado si ese valor coincide: un acuse de una consulta
        o de un envio anterior puede llegar mientras se manda uno nuevo, y no
        vale como confirmacion del nuevo.
        """
        if a is None:
            return
        if a.cmd == pq.UPLINK_CMD_PRUEBA_AEROFRENO:
            self._encajar_acuse_prueba_aero(a)
            return
        if a.cmd == pq.UPLINK_CMD_EXTENSION_AEROFRENO:
            self._encajar_acuse_extension_aero(a)
            return
        if a.cmd == pq.UPLINK_CMD_RUMBO:
            self._encajar_acuse_rumbo(a)
            return
        if a.cmd == pq.UPLINK_CMD_CAMARAS:
            if not a.aceptado and self.camaras_pedidas is not None:
                self.camaras_motivo = a.motivo_texto
                self.anotar("el cohete DENIEGA %s las camaras: %s"
                            % ("encender" if self.camaras_pedidas
                               else "apagar", a.motivo_texto), ROJO)
                self.camaras_pedidas = None
            self.camaras_apagando = bool(a.valor &
                                         pq.ACUSE_CAMARAS_APAGANDO)
            self._camaras_vistas(bool(a.valor &
                                      pq.ACUSE_CAMARAS_ENCENDIDAS))
            return
        if a.cmd != pq.UPLINK_CMD_ANGULO:
            return
        self.angulo_cohete = a.valor / 100.0
        self.angulo_definido_cohete = a.definido
        self.angulo_guardado_sd = a.guardado

        if self.angulo_estado == "consultando" and a.aceptado:
            self.angulo_estado = ""
            self.angulo_recien_leido = True
            self.anotar("el cohete tiene el angulo de lanzamiento en %.2f grados"
                        "%s" % (self.angulo_cohete, "" if a.definido
                                else " (sin definir, vertical)"), CIAN)
        elif self.angulo_estado == "enviando":
            if not a.aceptado:
                self.angulo_estado = "denegado"
                self.angulo_motivo = a.motivo_texto
                self.anotar("el cohete DENIEGA el angulo: %s"
                            % a.motivo_texto, ROJO)
            elif abs(self.angulo_cohete - self.angulo_pedido) < 0.005:
                self.angulo_estado = "confirmado"
                self.anotar("angulo de lanzamiento %.2f grados CONFIRMADO%s"
                            % (self.angulo_cohete, "" if a.guardado else
                               " (pero NO se pudo guardar en la SD)"),
                            VERDE if a.guardado else AMBAR)

    def _encajar_emergencia(self, p, previo):
        """Version 6: reinicio en vuelo (App/emergencia.h)."""
        a = previo if (previo is not None and previo.version >= 6) else None
        ya = self.paquete is not None and             self.paquete.estado == pq.ESTADO_EMERGENCIA
        if p.emergencia and not (a and a.emergencia) and not ya:
            self.anotar("EMERGENCIA: reiniciado en vuelo; decide con el "
                        "barometro (apogeo: 10 m bajo la maxima; main a "
                        "400 m)", ROJO)
        if p.emer_apogeo and not (a and a.emer_apogeo):
            self.anotar("EMERGENCIA: APOGEO detectado por el barometro -> "
                        "drogue (1A y 1B)", ROJO)
        if p.emer_main and not (a and a.emer_main):
            self.anotar("EMERGENCIA: altura del MAIN (400 m) -> 2A", ROJO)
        if p.emergencia and p.baro_no_fiable and not (a and a.baro_no_fiable):
            self.anotar("EMERGENCIA: barometro NO fiable, descartando medidas "
                        "(ruido o saltos)", AMBAR)
        if p.emergencia and not p.baro_no_fiable and a and a.baro_no_fiable:
            self.anotar("EMERGENCIA: barometro fiable otra vez", VERDE)
        if p.postvuelo and not (a and a.postvuelo):
            self.anotar("la core se ha reiniciado TRAS EL VUELO: no toca la "
                        "SD ni enciende las camaras ni dispara nada", CIAN)

    def _encajar_camaras(self, p, ahora):
        """
        Paquete de camaras: si graban, con que y su tarjeta. Se anotan los
        cambios que importan; lo demas se ve en el panel de salud.
        """
        previo = self.camaras_paquete
        self.camaras_paquete = p
        self.t_camaras_paquete = ahora
        self.camaras_apagando = p.apagando
        if p.apagando and not (previo and previo.apagando):
            self.anotar("CAMARAS: parando la grabacion antes de cortar la "
                        "corriente", CIAN)
        if p.ultimo_por_plazo and not p.alimentacion and \
                not (previo and previo.ultimo_por_plazo):
            self.anotar("CAMARAS cortadas POR PLAZO, sin confirmar que "
                        "pararan de grabar (el video puede no cerrarse)",
                        AMBAR)
        for i, c in enumerate(p.camaras):
            a = previo.camaras[i] if previo else None
            n = i + 1
            if a is not None and a.estado == c.estado and \
                    a.no_graba == c.no_graba and \
                    a.reloj_puesto == c.reloj_puesto:
                continue
            if c.no_graba and not (a and a.no_graba):
                self.anotar("CAM%d NO GRABA: %s" % (
                    n, "sin tarjeta" if c.sin_tarjeta else
                    "pulsado y sigue parada (tarjeta llena, rota o lenta)"),
                    ROJO)
            elif c.estado == pq.CAMARA_GRABANDO and \
                    not (a and a.estado == pq.CAMARA_GRABANDO):
                self.anotar("CAM%d GRABANDO  %s  %s de %s libres, caben %d "
                            "min" % (n, c.resolucion or "",
                                     paneles._mb(c.libre_mb),
                                     paneles._mb(c.total_mb),
                                     c.queda_s // 60), VERDE)
            elif c.estado == pq.CAMARA_PARADA and a is not None and \
                    a.estado in (pq.CAMARA_GRABANDO, pq.CAMARA_PARANDO):
                self.anotar("CAM%d parada: video cerrado" % n, CIAN)
            elif c.estado == pq.CAMARA_NO_CONTESTA and \
                    not (a and a.estado == pq.CAMARA_NO_CONTESTA):
                self.anotar("CAM%d no contesta por UART (desconectada?)" % n,
                            AMBAR)
            if c.reloj_puesto and not (a and a.reloj_puesto):
                self.anotar("CAM%d con la hora del GPS" % n, VERDE_MED)
        self._camaras_vistas(p.alimentacion)

    def _camaras_vistas(self, encendidas):
        """Lo que dice el cohete de las camaras: confirma lo pedido."""
        self.camaras_cohete = encendidas
        if self.camaras_pedidas is not None and \
                encendidas == self.camaras_pedidas:
            self.anotar("CAMARAS %s: confirmado por el cohete (%d intentos)"
                        % ("ENCENDIDAS" if encendidas else "APAGADAS",
                           self.intentos_camaras),
                        VERDE if encendidas else CIAN)
            self.camaras_pedidas = None

    def _encajar_acuse_rumbo(self, a):
        """Como el del angulo: confirmado solo si trae ESTE valor."""
        self.rumbo_cohete = a.valor / 100.0
        self.rumbo_definido_cohete = a.definido
        if self.rumbo_estado == "consultando" and a.aceptado:
            self.rumbo_estado = ""
            self.rumbo_recien_leido = True
            self.anotar("el cohete tiene el rumbo de la rampa en %.2f grados%s"
                        % (self.rumbo_cohete, "" if a.definido
                           else " (sin definir)"), CIAN)
        elif self.rumbo_estado == "enviando":
            if not a.aceptado:
                self.rumbo_estado = "denegado"
                self.rumbo_motivo = a.motivo_texto
                self.anotar("el cohete DENIEGA el rumbo: %s" % a.motivo_texto,
                            ROJO)
            elif abs(self.rumbo_cohete - self.rumbo_pedido) < 0.005:
                self.rumbo_estado = "confirmado"
                self.anotar("rumbo de la rampa %.2f grados CONFIRMADO%s"
                            % (self.rumbo_cohete, "" if a.guardado else
                               " (pero NO se pudo guardar en la SD)"),
                            VERDE if a.guardado else AMBAR)

    def _encajar_acuse_prueba_aero(self, a):
        # Solo vale el acuse de ESTA prueba: uno de una anterior que llegue
        # tarde no dice nada de la nueva
        if self.aero_prueba_estado != "enviando" or \
                a.valor != self.aero_prueba_num:
            return
        if a.aceptado:
            self.aero_prueba_estado = "en curso"
            self.aero_prueba_t = time.monotonic()
            self.anotar("PRUEBA DE AEROFRENO #%d en marcha (%.0f s)"
                        % (a.valor, pq.PRUEBA_AEROFRENO_S), VERDE)
        else:
            self.aero_prueba_estado = "denegada"
            self.aero_prueba_motivo = a.motivo_texto
            self.anotar("el cohete DENIEGA la prueba de aerofreno: %s"
                        % a.motivo_texto, ROJO)

    def _encajar_acuse_extension_aero(self, a):
        self.aero_ext_cohete = a.valor
        if self.aero_ext_estado != "enviando":
            return
        if not a.aceptado:
            self.aero_ext_estado = "denegada"
            self.aero_ext_motivo = a.motivo_texto
            self.anotar("el cohete DENIEGA la extension de aerofreno: %s"
                        % a.motivo_texto, ROJO)
        elif a.valor == self.aero_ext_pedida:
            self.aero_ext_estado = "confirmada"
            self.anotar("aerofreno al %d %% CONFIRMADO" % a.valor, VERDE)

    def _encajar_estado(self, p, trama, ahora):
        """Paquete de salud. No toca nada del vuelo: son cosas separadas."""
        if self.estado_pedido and ahora >= self.t_estado_pedido:
            self.estado_pedido = False
            self.anotar("estado recibido (intento %d): placas %d de 3, "
                        "%s, SD1 %s, SD2 %s"
                        % (self.intentos_estado, 3 - len(p.placas_ausentes),
                           ("sensores OK" if not p.sensores_en_fallo else
                            "fallan " + ", ".join(p.sensores_en_fallo)),
                           p.sd_nombre(0), p.sd_nombre(1)), CIAN)
        previo = self.estado_paquete
        primero = previo is None
        self.estado_paquete = p
        self.t_estado = ahora
        self.trama = trama

        if primero:
            self.anotar("paquete de estado recibido (v%d)" % p.version, CIAN)
            for nombre in p.placas_ausentes:
                self.anotar("placa %s NO responde" % nombre.upper(), ROJO)
            for nombre in p.sensores_en_fallo:
                self.anotar("sensor %s ha fallado al arrancar" % nombre, ROJO)
            for i in range(2):
                if p.sd_estado[i] != 3:
                    self.anotar("SD%d: %s" % (i + 1, p.sd_nombre(i)), ROJO)
            if p.todo_bien:
                self.anotar("todo en orden: 3 placas, sensores y tarjetas",
                            VERDE)

        # Cambios del modo test, contados por el cohete. Es la UNICA
        # confirmacion de que un disparo llego: aqui se anota lo que el cohete
        # dice que ha aceptado, no lo que se ha mandado.
        if p.version >= 2:
            antes_test = previo.test_activo if previo is not None else False
            if p.test_activo and not antes_test:
                self.anotar("COHETE EN MODO TEST", ROJO)
            elif antes_test and not p.test_activo:
                self.anotar("el cohete ha salido del modo test", AMBAR)

            # Una rafaga de cinco puede llegar entera y dar cinco aceptaciones
            # seguidas de la misma pulsacion. Se anota la primera; las del
            # mismo canal en los siguientes segundos son la misma orden.
            antes_piros = previo.piros_aceptados if previo is not None else None
            if antes_piros is not None and p.piros_aceptados > antes_piros:
                canal = pq.NOMBRE_CANAL.get(p.ultimo_piro, "?")
                ultimo = self._ultimo_aceptado
                if ultimo is None or ultimo[0] != canal or \
                        ahora - ultimo[1] > AGRUPAR_ACEPTADOS_S:
                    self.anotar("el cohete ACEPTA pirotecnico %s" % canal, ROJO)
                self._ultimo_aceptado = (canal, ahora)

            antes_den = previo.comandos_denegados if previo is not None else None
            if antes_den is not None and p.comandos_denegados > antes_den:
                if p.version >= 3 and p.motivo_denegacion:
                    self.anotar("el cohete DENIEGA: %s"
                                % p.motivo_denegacion_texto, AMBAR)
                else:
                    self.anotar("el cohete DENIEGA un comando (fuera de tierra"
                                " o sin modo test)", AMBAR)
                # Si lo que se denego fue la simulacion por falta de archivo,
                # insistir no va a hacer aparecer el archivo
                if self.simulacion_pedida and p.motivo_denegacion == 6:
                    self.simulacion_pedida = False

        if p.version >= 4:
            sim = " (SIMULACION, pero el canal se enciende DE VERDAD)" \
                if p.simulando else ""
            if p.drogue_mandado and not (previo and previo.drogue_mandado):
                self.anotar("DROGUE: el cohete enciende 1A y 1B" + sim, ROJO)
            if p.main_mandado and not (previo and previo.main_mandado):
                self.anotar("MAIN: el cohete enciende 2A" + sim, ROJO)
            if p.despliegue_rechazado and not (previo and
                                               previo.despliegue_rechazado):
                self.anotar("power RECHAZA una orden de despliegue", ROJO)

        if p.version >= 6:
            self._encajar_emergencia(p, previo)

        if p.camaras is not None:
            antes = self.camaras_cohete
            self._camaras_vistas(p.camaras)
            if antes is not None and antes != p.camaras \
                    and self.camaras_pedidas is None:
                # Sin haberlo pedido: el despegue las enciende solas
                self.anotar("CAMARAS %s (las ha cambiado el cohete)"
                            % ("ENCENDIDAS" if p.camaras else "apagadas"),
                            VERDE if p.camaras else AMBAR)

        if p.version >= 3:
            antes_sim = previo.simulando if previo is not None else False
            if p.simulando and not antes_sim:
                self.limpiar_historia()
                self.anotar("SIMULANDO RUTA: los datos son de VUELO.BIN, NO "
                            "de un vuelo real", CIAN)
            elif antes_sim and not p.simulando:
                self.simulacion_pedida = False
                self.anotar("simulacion terminada, el cohete vuelve a sus "
                            "sensores", CIAN)


# --- Simulación -------------------------------------------------------------


def _sat(v, lo, hi):
    return max(lo, min(hi, v))


class Simulador(object):
    """
    Genera paquetes de 24 bytes de un vuelo inventado, con el MISMO formato
    que el cohete. Sirve para trabajar en la interfaz sin sacar las placas, y
    de paso prueba el decodificador de verdad: si aquí se ve bien, el problema
    no está en paquete.py.
    """

    # --demo-emergencia: a los T_REINICIO_S del vuelo la core "se reinicia" y
    # sigue en modo de emergencia (App/emergencia.h)
    T_REINICIO_S = 20.0

    def __init__(self, despegue=True, emergencia=False):
        self.emergencia = emergencia
        self._emer_vista = (False, False, False, False)
        self.t0 = time.monotonic()
        self.contador = 0
        self.contador_estado = 0
        # Al arrancar manda estado, igual que el cohete de verdad
        self.estado_pendiente = 3
        # Con despegue=False el cohete se queda en tierra en POWER ON, que es
        # como esta en el banco y donde se puede probar el modo test. Con
        # despegue=True despega a los 8 s (con el GNC v1.2 no hay READY: hasta
        # despegar esta en POWER ON, como en el cohete de verdad).
        self.despegue = despegue
        self._lanzado = False
        self.test = False
        self.test_hasta = 0.0
        self.proximo_estado_test = 0.0
        self.ultimo_piro = 0
        self.piros = 0
        self.denegados = 0
        self.continuidad = 0x0F   # los cuatro iniciadores puestos
        self.simulando = False
        self.t_sim = 0.0
        self.motivo = 0
        self.drogue = False
        self.main = False
        self.angulo = 0               # centesimas, como en lanzamiento.c
        self.angulo_definido = False
        self.rumbo = 0
        self.rumbo_definido = False
        self.aero_prueba_num = 0      # como uplink.c: el ultimo ejecutado
        self.aero_prueba_t0 = None    # monotonic del arranque de la prueba
        self.aero_fija = None         # 0..1, extension fijada desde tierra
        self.acuse = None
        self.n_acuses = 0
        self.camaras = False          # arrancan apagadas, como en la core
        # Como camaras.c: la 1 graba y la 2 no esta conectada (como en la
        # mesa). Apagar es parar primero y cortar a los 5,5 s.
        self.t_cam_on = 0.0
        self.cam_apagando = False
        self.t_cam_off = 0.0
        self.cam_origen = 0
        self.cam_terminado = False
        self.cam_libre = 29782.0
        self.cam_pulsos = 0
        self.camaras_pendiente = 1    # uno al arrancar
        self.contador_camaras = 0
        self._t_cam_paquete = 0.0
        self._cam_firma = None

    DURACION_SIM_S = 62.0

    def _estado_cohete(self, fase):
        if 1 <= fase <= 6 and (self.despegue or self.simulando):
            return pq.ESTADO_EN_VUELO
        return pq.ESTADO_POWER_ON

    def _en_tierra(self):
        fase = self._perfil(time.monotonic() - self.t0)[0]
        return fase in (0, 7) and self._estado_cohete(fase) in (
            pq.ESTADO_POWER_ON, pq.ESTADO_MANTENIMIENTO, pq.ESTADO_ERROR)

    def comando(self, cmd, param):
        """Lo que haria App/uplink.c, con las mismas protecciones."""
        ahora = time.monotonic()
        if cmd == pq.UPLINK_CMD_PEDIR_ESTADO:
            self.estado_pendiente += 1
            self.camaras_pendiente += 1
        elif cmd == pq.UPLINK_CMD_MODO_TEST:
            if param == 0:
                if self.test:
                    self.test = False
                    self.aero_prueba_t0 = None
                    self.aero_fija = None
                    self.estado_pendiente += 1
            elif self._en_tierra():
                self.test = True
                self.test_hasta = ahora + 120.0
            else:
                self.denegados += 1
                self.estado_pendiente += 1   # igual que _denegar() del core
        elif cmd == pq.UPLINK_CMD_SIMULACION:
            if param == 0:
                if self.simulando:
                    self.simulando = False
                    self.estado_pendiente += 1
            elif self.simulando:
                pass
            elif not self.test:
                self._denegar(2)
            elif not self._en_tierra():
                self._denegar(1)
            else:
                self.test = False
                self.simulando = True
                self.t_sim = ahora
                self.continuidad = 0x0F
                self.drogue = False
                self.main = False
                self.estado_pendiente += 1
        elif cmd == pq.UPLINK_CMD_PING:
            self.acuse = bytes([pq.ACUSE_MARCA, cmd, 0, param & 0xFF,
                                (param >> 8) & 0xFF, self.n_acuses & 0xFF,
                                0, 0])
            self.n_acuses += 1
        elif cmd == pq.UPLINK_CMD_ANGULO:
            # Lo que hace uplink.c: acuse SIEMPRE, con el valor que hay
            fase = self._perfil(time.monotonic() - self.t0)[0]
            if param == pq.ANGULO_CONSULTA:
                res = 0
            elif self.simulando:
                res = 5
            elif (self._estado_cohete(fase) == pq.ESTADO_EN_VUELO
                  or fase not in (0, 7)):
                res = 1
            elif param > 9000:
                res = 7
            else:
                res = 0
                self.angulo = param
                self.angulo_definido = True
            if res:
                self.denegados += 1
                self.motivo = res
            bits = (0x03 if self.angulo_definido else 0)
            self.acuse = bytes([pq.ACUSE_MARCA, cmd, res, self.angulo & 0xFF,
                                (self.angulo >> 8) & 0xFF,
                                self.n_acuses & 0xFF, bits, 0])
            self.n_acuses += 1
        elif cmd == pq.UPLINK_CMD_RUMBO:
            fase = self._perfil(time.monotonic() - self.t0)[0]
            if param == pq.ANGULO_CONSULTA:
                res = 0
            elif self.simulando:
                res = 5
            elif self._estado_cohete(fase) == pq.ESTADO_EN_VUELO or \
                    fase not in (0, 7):
                res = 1
            elif param > 35999:
                res = 7
            else:
                res = 0
                self.rumbo = param
                self.rumbo_definido = True
            if res:
                self.denegados += 1
                self.motivo = res
            bits = (0x03 if self.rumbo_definido else 0)
            self.acuse = bytes([pq.ACUSE_MARCA, cmd, res, self.rumbo & 0xFF,
                                (self.rumbo >> 8) & 0xFF,
                                self.n_acuses & 0xFF, bits, 0])
            self.n_acuses += 1
        elif cmd in (pq.UPLINK_CMD_PRUEBA_AEROFRENO,
                     pq.UPLINK_CMD_EXTENSION_AEROFRENO):
            # Lo que hace uplink.c: modo test, en tierra y sin simular, y
            # acuse SIEMPRE
            if cmd == pq.UPLINK_CMD_PRUEBA_AEROFRENO and param == 0:
                res = 0
                self.aero_prueba_t0 = None
                self.aero_fija = None
            elif cmd == pq.UPLINK_CMD_EXTENSION_AEROFRENO and param > 100:
                res = 7
            elif self.simulando:
                res = 5
            elif not self.test:
                res = 2
            elif not self._en_tierra():
                res = 1
            else:
                res = 0
            if res == 0 and param != 0:
                if cmd == pq.UPLINK_CMD_PRUEBA_AEROFRENO:
                    if param != self.aero_prueba_num:
                        self.aero_prueba_num = param
                        self.aero_prueba_t0 = ahora
                        self.aero_fija = None
                else:
                    self.aero_fija = param / 100.0
                    self.aero_prueba_t0 = None
            elif cmd == pq.UPLINK_CMD_EXTENSION_AEROFRENO and param == 0 \
                    and res == 0:
                self.aero_fija = 0.0
                self.aero_prueba_t0 = None
            if res:
                self.denegados += 1
                self.motivo = res
            valor = param
            if cmd == pq.UPLINK_CMD_EXTENSION_AEROFRENO and res:
                valor = int(round(100 * (self._aero_extension() or 0.0)))
            self.acuse = bytes([pq.ACUSE_MARCA, cmd, res, valor & 0xFF,
                                (valor >> 8) & 0xFF, self.n_acuses & 0xFF,
                                0, 0])
            self.n_acuses += 1
        elif cmd == pq.UPLINK_CMD_CAMARAS:
            # Lo que hace uplink.c: encender siempre, apagar solo en tierra
            res = 0
            if param == 1:
                self._cam_encender(2)
            elif param != 0:
                res = 7
            elif not self._en_tierra():
                res = 1
            else:
                self._cam_apagar(2)
            if res:
                self.denegados += 1
                self.motivo = res
            valor = (1 if self.camaras else 0) | \
                (2 if self.cam_apagando else 0)
            self.acuse = bytes([pq.ACUSE_MARCA, cmd, res, valor, 0,
                                self.n_acuses & 0xFF, 0, 0])
            self.n_acuses += 1
            self.estado_pendiente += 1
        elif cmd == pq.UPLINK_CMD_PIRO:
            if self.simulando:
                self._denegar(5)
            elif not self.test or not 1 <= param <= 4:
                self._denegar(2 if not self.test else 3)
            else:
                self.piros += 1
                self.ultimo_piro = param
                # El iniciador se quema: deja de haber continuidad
                self.continuidad &= ~(1 << (param - 1))
                self.estado_pendiente += 1

    # --- Camaras, como camaras.c y camara.c ---

    def _cam_encender(self, origen):
        ahora = time.monotonic()
        if self.cam_apagando:
            self.cam_apagando = False
            self.cam_origen = origen
            self.camaras_pendiente += 1
        if not self.camaras:
            self.camaras = True
            self.t_cam_on = ahora
            self.cam_origen = origen
            self.estado_pendiente += 1
            self.camaras_pendiente += 1

    def _cam_apagar(self, origen):
        if not self.camaras or self.cam_apagando:
            return
        self.cam_apagando = True
        self.t_cam_off = time.monotonic()
        self.cam_origen = origen
        self.cam_pulsos += 1
        self.camaras_pendiente += 1

    def _cam_tic(self):
        ahora = time.monotonic()
        if self.cam_apagando and ahora - self.t_cam_off >= 5.5:
            self.cam_apagando = False
            self.camaras = False
            self.estado_pendiente += 1
            self.camaras_pendiente += 1
        if self.camaras and self._cam_estado(0) in (pq.CAMARA_GRABANDO,
                                                    pq.CAMARA_PARANDO):
            self.cam_libre -= 5.7 * min(1.0, ahora - getattr(
                self, "_t_cam_libre", ahora))
        self._t_cam_libre = ahora
        firma = (self.camaras, self.cam_apagando, self._cam_estado(0),
                 self._cam_estado(1))
        if firma != self._cam_firma:
            self._cam_firma = firma
            self.camaras_pendiente += 1
        if self.camaras and ahora - self._t_cam_paquete >= 5.0:
            self.camaras_pendiente += 1
            self._t_cam_paquete = ahora

    def _cam_estado(self, i):
        if not self.camaras:
            return pq.CAMARA_SIN_CORRIENTE
        t = time.monotonic() - self.t_cam_on
        if i == 1:   # sin conectar
            return pq.CAMARA_ARRANCANDO if t < 6.0 else \
                pq.CAMARA_NO_CONTESTA
        if t < 3.0:
            return pq.CAMARA_ARRANCANDO
        if t < 4.0:
            return pq.CAMARA_COMPROBANDO
        if self.cam_apagando:
            d = time.monotonic() - self.t_cam_off
            return pq.CAMARA_PARANDO if d < 4.0 else pq.CAMARA_PARADA
        return pq.CAMARA_GRABANDO

    def paquete_camaras(self):
        ahora = time.monotonic()
        b = bytearray(pq.CAMARAS_BYTES)
        b[0] = pq.CAMARAS_MARCA
        b[1] = 1
        b[2] = self.contador_camaras & 0xFF
        self.contador_camaras += 1
        en_vuelo = self.despegue and not self.cam_terminado and \
            1 <= self._perfil(ahora - self.t0)[0] <= 6
        b[3] = (0x01 if self.camaras else 0) | \
            (0x02 if self.cam_apagando else 0) | \
            ((self.cam_origen & 0x03) << 2) | (0x10 if en_vuelo else 0) | 0x20
        for i in range(2):
            o = 4 + pq.CAMARA_BYTES * i
            e = self._cam_estado(i)
            b[o] = e
            leida = e in (pq.CAMARA_GRABANDO, pq.CAMARA_PARADA,
                          pq.CAMARA_PARANDO, pq.CAMARA_COMPROBANDO) \
                and ahora - self.t_cam_on >= 3.0
            b[o + 1] = (0x00 if self.cam_apagando else 0x01) | \
                ((0x04 | 0x08 | 0x20) if leida and ahora - self.t_cam_on
                 >= 7.0 else (0x08 | 0x20) if leida else 0)
            if i == 0:
                libre = int(self.cam_libre) // 16
                for off, v in ((2, 1080), (5, libre), (7, 30432 // 16),
                               (9, int(self.cam_libre / 5.6)),
                               (11, int(ahora - self.t_cam_on - 3.5)
                                if e in (pq.CAMARA_GRABANDO,
                                         pq.CAMARA_PARANDO) else 0)):
                    v = max(0, min(0xFFFF, v))
                    b[o + off] = v & 0xFF
                    b[o + off + 1] = (v >> 8) & 0xFF
                b[o + 4] = 60
                b[o + 13] = min(255, self.cam_pulsos)
        return bytes(b)

    def _aero_extension(self):
        """La extension de la prueba de tierra (como _secuencia() de
        airbrake.c), o None si no hay ninguna."""
        if self.aero_fija is not None:
            return self.aero_fija
        if self.aero_prueba_t0 is None:
            return None
        t = time.monotonic() - self.aero_prueba_t0
        if t < 3.0:
            return 1.0 if (t % 1.0) < 0.5 else 0.0
        t -= 3.0
        if t < 2.0:
            return t / 2.0
        t -= 2.0
        if t < 2.0:
            return 1.0 - t / 2.0
        self.aero_prueba_t0 = None
        return None

    def _denegar(self, motivo):
        self.denegados += 1
        self.motivo = motivo
        self.estado_pendiente += 1

    def _vigilar_test(self):
        ahora = time.monotonic()
        if self.simulando:
            if ahora - self.t_sim > self.DURACION_SIM_S:
                self.simulando = False
                self.estado_pendiente += 1
            elif ahora >= self.proximo_estado_test:
                self.estado_pendiente += 1
                self.proximo_estado_test = ahora + 1.0
        if not self.test:
            return
        if not self._en_tierra() or ahora >= self.test_hasta:
            self.test = False
            self.aero_prueba_t0 = None   # como _cerrar_test() de uplink.c
            self.aero_fija = None
            self.estado_pendiente += 1
            return
        if ahora >= self.proximo_estado_test:
            self.estado_pendiente += 1
            self.proximo_estado_test = ahora + 1.0

    def paquete_estado(self):
        """
        Estado inventado pero REALISTA: se copia el del banco, con el GPS
        caido, que es como esta la avionica hoy. Un simulador donde todo sale
        perfecto no sirve para comprobar que los avisos se ven.
        """
        t = time.monotonic() - self.t0
        # 32 y no 26: CAN FD solo admite ciertas longitudes y el driver
        # rellena hasta 32, asi que es lo que sale al aire de verdad.
        b = bytearray(pq.ESTADO_BYTES)
        b[0] = 0xA5
        b[1] = 5
        b[2] = self.contador_estado & 0xFF
        self.contador_estado += 1
        b[3] = 0x07 | 0x08 | 0x40 | 0x80   # 3 placas, init, calibracion, nav
        b[4] = 0x5F                        # todos menos el gps (bit 5)
        b[5] = 0x7F                        # la placa lleva los siete
        b[6] = 0x03 | (0x03 << 2) | 0x10 | 0x20   # las dos montadas y con test
        b[7] = 59
        b[8] = 6
        for off, mb in ((9, 3770), (11, 30424)):
            v = mb // 16
            b[off] = v & 0xFF
            b[off + 1] = (v >> 8) & 0xFF
        for off, hz in ((13, 2), (15, 824)):
            b[off] = hz & 0xFF
            b[off + 1] = (hz >> 8) & 0xFF
        b[17] = 0
        b[18] = 0
        seg = int(t) & 0xFFFF
        b[19] = seg & 0xFF
        b[20] = (seg >> 8) & 0xFF
        b[21] = 0
        b[22] = 0
        b[23] = 0
        fase, _, _, _ = self._perfil(t)
        b[24] = (fase & 0x07) | (self._estado_cohete(fase) << 3)
        b[25] = (0x01 if self.test else 0) | ((self.ultimo_piro & 0x07) << 1)
        b[26] = int(max(0.0, self.test_hasta - time.monotonic())) \
            if self.test else 0
        b[27] = min(255, self.piros)
        b[28] = min(255, self.denegados)
        if self.simulando:
            b[25] |= 0x10
            b[29] = min(100, int(100 * (time.monotonic() - self.t_sim)
                                 / self.DURACION_SIM_S))
        b[30] = self.motivo
        b[31] = (0x01 if self.drogue else 0) | (0x02 if self.main else 0) \
            | (0x08 if self.camaras else 0)
        b[1] = 7
        # GNC v1.6: listo a los 2 s de encender (alineado, 0 AGL, GNSS)
        if t >= 2.0:
            b[24] |= 0x40 | 0x80
            b[25] |= 0x40
        emer, apo, mai, mal = self._bits_emergencia(t, fase)
        b[31] |= (0x10 if emer else 0) | (0x20 if apo else 0) | \
            (0x40 if mai else 0) | (0x80 if mal else 0)
        return bytes(b)

    def _en_emergencia(self, t, fase):
        return self.emergencia and self.despegue and \
            t >= self.T_REINICIO_S and 1 <= fase <= 6

    def _baro_mal(self, t):
        """Los primeros 1,5 s tras el reinicio: ruido transonico."""
        return self.T_REINICIO_S <= t < self.T_REINICIO_S + 1.5

    def _bits_emergencia(self, t, fase):
        emer = self._en_emergencia(t, fase)
        bits = (emer, emer and fase >= 4, emer and fase >= 6,
                emer and self._baro_mal(t))
        if bits != self._emer_vista:
            self._emer_vista = bits
            self.estado_pendiente += 1
        return bits

    def cadencia_hz(self):
        """
        La del cohete (App/rf_telemetry.h): 3 Hz hasta el lanzamiento y 15
        desde el. Simulando, 15 solo mientras dura el vuelo simulado.
        """
        fase = self._perfil(time.monotonic() - self.t0)[0]
        if self.simulando:
            return 15.0 if fase >= 1 else 3.0
        if fase >= 1:
            self._lanzado = True
        return 15.0 if self._lanzado else 3.0

    def _perfil(self, t):
        if self.simulando:
            return self._vuelo(time.monotonic() - self.t_sim)
        if not self.despegue:
            return 0, 0.0, 0.0, 1.0
        return self._vuelo(t)

    def _vuelo(self, t):
        if t < 8.0:
            return 0, 0.0, 0.0, 1.0
        t -= 8.0
        if t < 3.2:                       # empuje
            return 1, 110.0 * t, 70.0 * t, 8.0
        t -= 3.2
        v = 224.0 - 9.81 * t
        h = 352.0 + 224.0 * t - 4.9 * t * t
        if v > 0:
            return (3 if t > 1.0 else 2), h, v, -0.2
        t2 = -v / 9.81
        if h > 0:
            fase = 4 if t2 < 1.5 else (6 if h < 400.0 else 5)
            return fase, h, _sat(v, -28.0, 0.0), -0.9
        return 7, 0.0, 0.0, 1.0

    def paquete(self):
        t = time.monotonic() - self.t0
        fase, h, v, acc_x = self._perfil(t)

        n = 0.35 * h
        e = -0.22 * h
        elev = math.radians(_sat(88.0 - 0.06 * max(0.0, t - 8.0) ** 2, 5, 90))
        rumbo = math.radians(30.0)

        # Cuaternión de una elevación y un rumbo, igual que lo construye el
        # modelo: guiñada y luego cabeceo.
        cy, sy = math.cos(rumbo / 2), math.sin(rumbo / 2)
        cp, sp = math.cos(elev / 2), math.sin(elev / 2)
        q = (cy * cp, -sy * sp, cy * sp, sy * cp)

        b = bytearray(24)
        b[0] = self.contador & 0xFF
        self.contador += 1
        # Lo mismo que despliegue.c: en EN VUELO, la fase DROGUE enciende 1A
        # y 1B a la vez y la MAIN enciende 2A. El iniciador se quema y pierde
        # continuidad.
        if self._estado_cohete(fase) == pq.ESTADO_EN_VUELO:
            if fase == 5 and not self.drogue:
                self.drogue = True
                self.continuidad &= ~0x03
                self.estado_pendiente += 1
            if fase == 6 and not self.main:
                self.main = True
                self.continuidad &= ~0x04
                self.estado_pendiente += 1
        piro = self.continuidad
        emer = self._en_emergencia(t, fase)
        b[1] = piro | (fase << 4) | (0x80 if not emer or not
                                    self._baro_mal(t) else 0)
        b[2] = 4 if emer else self._estado_cohete(fase)

        def i16(v, esc):
            x = int(round(v / esc))
            return _sat(x, -32768, 32767) & 0xFFFF

        def i8(v, esc):
            x = int(round(v / esc))
            return _sat(x, -128, 127) & 0xFF

        for off, val in ((3, i16(h, 0.25)), (5, i16(n, 1.0)),
                         (7, i16(e, 1.0)), (9, i16(v, 0.05))):
            b[off] = val & 0xFF
            b[off + 1] = (val >> 8) & 0xFF
        b[11] = i8(0.35 * v, 1.0)
        b[12] = i8(-0.22 * v, 1.0)
        b[13] = i8(acc_x + random.uniform(-0.05, 0.05), 0.25)
        b[14] = i8(random.uniform(-0.2, 0.2), 0.25)
        b[15] = i8(random.uniform(-0.2, 0.2), 0.25)
        for i in range(4):
            b[16 + i] = i8(q[i], 1.0 / 127.0)
        b[20] = _sat(int((h + max(0.0, v) ** 2 / 19.6) / 25.0), 0, 255)
        aero = self._aero_extension()
        if aero is None:
            aero = 0.55 if fase == 3 else 0.0
        b[21] = _sat(int(round(250 * aero)), 0, 250)
        b[22] = _sat(int((12.4 - 0.004 * t) * 1000 / 80), 0, 255)
        b[23] = i8(21 - 0.02 * h, 1.0)
        return bytes(b)

    def trama(self):
        self._vigilar_test()
        # Como camaras.c: al despegar se encienden solas y se avisa; a los
        # 50 m de bajada se paran y se apagan
        fase, h = self._perfil(time.monotonic() - self.t0)[:2]
        if not self.camaras and self.despegue and not self.simulando and \
                not self.cam_terminado and 1 <= fase <= 6:
            self._cam_encender(1)
        if self.camaras and self.despegue and not self.simulando and \
                not self.cam_terminado and fase >= 5 and h <= 50.0:
            self.cam_terminado = True
            self._cam_apagar(3)
        self._cam_tic()
        t_sim = time.monotonic() - self.t0
        self._bits_emergencia(t_sim, self._perfil(t_sim)[0])
        d = 900.0 + 0.0  # distancia ficticia
        rssi = _sat(-42 - 22 * math.log10(max(1.0, d / 50.0))
                    + random.uniform(-3, 3), -128, 0)
        if self.acuse is not None:
            carga, self.acuse = self.acuse, None   # el acuse va el primero
            if len(carga) == pq.ACUSE_BYTES_ANTES:
                # Como lo "oyo" el cohete: valores verosimiles de banco
                rssi = random.randint(-72, -60)
                snr = random.randint(7, 11)
                df = random.randint(1850, 2150)
                carga = carga[:7] + bytes([0x01, rssi & 0xFF, snr & 0xFF,
                                           df & 0xFF, (df >> 8) & 0xFF])
        elif self.estado_pendiente > 0:
            self.estado_pendiente -= 1
            carga = self.paquete_estado()
        elif self.camaras_pendiente > 0:
            self.camaras_pendiente = 0
            carga = self.paquete_camaras()
        else:
            carga = self.paquete()
        return pte.Trama(carga, int(rssi),
                         round(random.uniform(6, 12) * 4) / 4.0,
                         int(random.uniform(-400, 400)), 0, time.monotonic())


# --- Ventana ----------------------------------------------------------------


def boton(padre, texto, orden, color=VERDE, ancho=15, activo=True):
    b = tk.Button(padre, text=texto, command=orden, bg=FONDO,
                  fg=color if activo else VERDE_OSC,
                  activebackground=color, activeforeground=FONDO,
                  disabledforeground=VERDE_OSC, relief="solid", bd=1,
                  highlightthickness=0, font=fuente(9, True), width=ancho,
                  cursor="hand2" if activo else "arrow")
    if not activo:
        b.configure(state="disabled")
    return b


def _es_ch340(descripcion):
    return "CH340" in (descripcion or "").upper()


def _descripcion(dispositivo):
    for d, descripcion in pte.puertos_disponibles():
        if d == dispositivo:
            return descripcion
    return ""


class Estacion(object):

    def __init__(self, raiz, puerto=None, demo=False, demo_tierra=False,
                 guardar=True, carpeta_datos=None, demo_emergencia=False,
                 puerto_cats=None, sin_cats=False):
        self.raiz = raiz
        self.estado = Estado()
        self.lector_cats = None
        self.sim_cats = None
        # Puertos dados con --puerto / --cats: esos y ningún otro. Si no, se
        # buscan solos, también DESPUÉS de arrancar (_vigilar_puertos)
        self._puerto_forzado = puerto
        self._cats_forzado = puerto_cats
        self._sin_cats = sin_cats
        self._proxima_busqueda = 0.0

        # Guardado de todo lo que llega (grabador.py). Lo primero, para que
        # el registro quede entero desde la primera linea.
        self.grabador = None
        self._fallo_grabador_dicho = False
        if guardar:
            try:
                self.grabador = grab.Grabador(demo=demo, base=carpeta_datos)
                self.estado.al_anotar = self.grabador.evento
                self.estado.anotar("guardando los datos en %s"
                                   % self.grabador.dir)
            except Exception as e:
                self.estado.anotar("NO SE GUARDAN LOS DATOS: no se pudo "
                                   "crear la carpeta (%s)" % e, ROJO)
        else:
            self.estado.anotar("los datos NO se guardan (--sin-guardar)",
                               AMBAR)

        self.demo = demo
        self.sim = Simulador(despegue=not demo_tierra,
                             emergencia=demo_emergencia) if demo else None
        self.puente = None
        self._ultimo_demo = 0.0
        self._proxima_peticion = time.monotonic() + 2.0
        self._proximo_test = 0.0
        self._proximo_sim = 0.0
        self._proximo_angulo = 0.0
        self._proximo_rumbo = 0.0
        self._proximo_camaras = 0.0
        self._proximo_aero_prueba = 0.0
        self._proximo_aero_ext = 0.0
        # Numero de prueba de aerofreno: empieza al azar para que, si se
        # reinicia la estacion, no repita el ultimo que ejecuto el cohete
        # (lo tomaria por un reintento y no haria nada)
        self._num_prueba_aero = random.randint(1, 255)
        self._proximo_ping = 0.0
        self._proximo_pedir = 0.0
        self._numero_ping = 0
        self.angulo_abierto = False
        self._panel_visible = "mapa"
        self._test_visible = False

        raiz.title("LEEM - Estacion de tierra")
        raiz.configure(bg=FONDO)
        raiz.geometry("1500x900")
        raiz.minsize(1180, 740)
        # Maximizada: Windows la ajusta a la pantalla MENOS la barra de
        # tareas, que si no tapa la fila de botones de abajo
        try:
            raiz.state("zoomed")              # Windows
        except tk.TclError:
            try:
                raiz.attributes("-zoomed", True)  # Linux
            except tk.TclError:
                pass
        # F11: pantalla completa (sin barra de título ni de tareas); Esc sale
        raiz.bind("<F11>", lambda _e: raiz.attributes(
            "-fullscreen", not raiz.attributes("-fullscreen")))
        raiz.bind("<Escape>", lambda _e: raiz.attributes("-fullscreen",
                                                        False))
        raiz.protocol("WM_DELETE_WINDOW", self.cerrar)

        self._construir()

        if demo:
            self.estado.puerto_abierto = True
            self.estado.nombre_puerto = "SIMULADO"
            self.estado.anotar("modo demostracion: no hay radio", AMBAR)
        else:
            self._abrir_puerto(puerto)

        # CATS: en la demo, fingido a partir del mismo vuelo
        if demo and not sin_cats:
            self.sim_cats = ct.SimuladorCats(self.sim)
            self.estado.cats_puerto = "SIMULADO"
        elif not sin_cats:
            self._abrir_cats(puerto_cats)

        self.raiz.after(REFRESCO_MS, self._tic)

    # --- Distribución ---

    def _construir(self):
        r = self.raiz
        for c, peso, minimo in ((0, 0, 245), (1, 3, 240), (2, 2, 210),
                                (3, 0, 345)):
            r.columnconfigure(c, weight=peso, minsize=minimo)
        # Las filas 4 y 5 crecen con lo que sobre; su mínimo es pequeño para
        # que todo quepa en un portátil de 1080 al 125 % (793 px útiles
        # maximizada): con 210 y 150 la fila de botones quedaba fuera
        for f, peso, minimo in ((0, 0, 54), (1, 0, 150), (2, 0, 92),
                                (3, 0, 58), (4, 2, 160), (5, 1, 110),
                                (6, 0, 46), (7, 0, 62)):
            r.rowconfigure(f, weight=peso, minsize=minimo)

        def poner(w, f, c, fs=1, cs=1):
            w.grid(row=f, column=c, rowspan=fs, columnspan=cs, sticky="nsew",
                   padx=2, pady=2)
            return w

        self.paneles = []

        def registrar(w, *a, **k):
            poner(w, *a, **k)
            self.paneles.append(w)
            return w

        registrar(paneles.PanelCabecera(r), 0, 0, 1, 4)

        registrar(paneles.PanelMedidores(r, titulo="MEDIDORES"), 1, 0, 2)
        registrar(paneles.PanelPerfil(r, titulo="PERFIL DE ALTURA"), 1, 1, 1, 2)
        registrar(paneles.PanelVelocidad(r, titulo="VELOCIDAD VERTICAL"),
                  2, 1, 1, 2)
        registrar(paneles.PanelSistema(r, titulo="SISTEMA"), 1, 3, 2)

        registrar(paneles.PanelFases(r, titulo="FASE DE VUELO"), 3, 0, 1, 4)

        registrar(paneles.PanelActitud(r, titulo="ACTITUD"), 4, 0)
        registrar(paneles.PanelNumeros(r, titulo="VUELO"), 4, 1)
        self.panel_mapa = registrar(paneles.PanelMapa(r, titulo="RECORRIDO"),
                                    4, 2)
        # El panel de pruebas ocupa la MISMA casilla y se alterna con el mapa.
        # No es un Panel de lienzo porque lleva botones de verdad: los
        # paneles se borran y redibujan 12 veces por segundo, y un boton que
        # se recrea cada 80 ms pierde clics.
        self.panel_pruebas = pruebas.PanelPruebas(r, self._rafaga,
                                                  self.salir_modo_test,
                                                  self.iniciar_simulacion,
                                                  self.probar_aerofreno,
                                                  self.enviar_extension_aerofreno)
        poner(self.panel_pruebas, 4, 2)
        self.panel_pruebas.grid_remove()
        # Y el del angulo de lanzamiento, tambien en esa casilla
        self.panel_angulo = angulo.PanelAngulo(r, self.enviar_rampa,
                                               self.alternar_angulo)
        poner(self.panel_angulo, 4, 2)
        self.panel_angulo.grid_remove()
        registrar(paneles.PanelEnlace(r, titulo="ENLACE"), 4, 3)

        registrar(paneles.PanelCrudo(r, titulo="PAQUETE CRUDO"), 5, 0)
        registrar(paneles.PanelAceleracion(r, titulo="ACELERACION"), 5, 1)
        registrar(paneles.PanelSalud(r, titulo="SALUD"), 5, 2)

        self._construir_registro(poner)
        registrar(paneles.PanelRanuras(r, titulo="RANURAS RESERVADAS"),
                  6, 0, 1, 4)
        self._construir_comandos(poner)

    def _construir_registro(self, poner):
        marco = tk.Frame(self.raiz, bg=FONDO, highlightthickness=1,
                         highlightbackground=VERDE_OSC)
        poner(marco, 5, 3)
        tk.Label(marco, text="REGISTRO", bg=FONDO, fg=VERDE_MED,
                 font=fuente(8, True), anchor="w").pack(fill="x", padx=6,
                                                        pady=(3, 0))
        # width y height en caracteres, no en pixeles: si no se ponen, un
        # Text pide 80x24 y estira la fila entera del grid.
        self.texto = tk.Text(marco, bg=FONDO, fg=VERDE_MED, bd=0,
                             highlightthickness=0, font=fuente(8),
                             wrap="word", state="disabled", width=10, height=4,
                             insertbackground=VERDE)
        self.texto.pack(fill="both", expand=True, padx=6, pady=4)
        for nombre, col in (("verde", VERDE), ("medio", VERDE_MED),
                            ("ambar", AMBAR), ("rojo", ROJO), ("cian", CIAN)):
            self.texto.tag_configure(nombre, foreground=col)
        self._eventos_pintados = 0

    def _construir_comandos(self, poner):
        marco = tk.Frame(self.raiz, bg=FONDO, highlightthickness=1,
                         highlightbackground=VERDE_OSC)
        poner(marco, 7, 0, 1, 4)

        tk.Label(marco, text="MANDO", bg=FONDO, fg=VERDE_MED,
                 font=fuente(8, True)).pack(side="left", padx=(10, 14))

        boton(marco, "PING ENLACE", self.ping, CIAN, 13).pack(side="left",
                                                              padx=3)
        boton(marco, "PEDIR ESTADO", self.pedir_estado, CIAN, 13).pack(
            side="left", padx=3)
        self.btn_bloqueo = boton(marco, "RAMPA: BLOQUEADO",
                                 self.alternar_bloqueo, VERDE, 18)
        self.btn_bloqueo.pack(side="left", padx=3)
        self.btn_fuente = boton(marco, "DATOS: ASPID", self.alternar_fuente,
                                VERDE, 21)
        self.btn_fuente.pack(side="left", padx=3)
        self.btn_test = boton(marco, "MODO TEST", self.alternar_modo_test,
                              ROJO, 13)
        self.btn_test.pack(side="left", padx=3)
        self.btn_sim = boton(marco, "PARAR SIMULACION", self.parar_simulacion,
                             CIAN, 17, activo=False)
        self.btn_sim.pack(side="left", padx=3)
        self.btn_angulo = boton(marco, "ANGULO LANZ.", self.alternar_angulo,
                                CIAN, 13)
        self.btn_angulo.pack(side="left", padx=3)
        self.btn_camaras = boton(marco, "CAMARAS ?", self.alternar_camaras,
                                 CIAN, 17)
        self.btn_camaras.pack(side="left", padx=3)
        for etq in ("ARMAR", "DESARMAR", "ABORTAR"):
            col = ROJO if etq == "ABORTAR" else VERDE
            boton(marco, etq, None, col, 11, activo=False).pack(side="left",
                                                                padx=3)

        self.aviso = tk.Label(
            marco, bg=FONDO, fg=AMBAR, font=fuente(8),
            text="MODO TEST solo en tierra; los pirotecnicos se disparan "
                 "desde el panel que sustituye al RECORRIDO")
        self.aviso.pack(side="left", padx=14)

        self.estado_tx = tk.Label(marco, bg=FONDO, fg=VERDE_OSC,
                                  font=fuente(8), text="")
        self.estado_tx.pack(side="right", padx=10)

    # --- Puerto ---

    def _abrir_puerto(self, puerto):
        if puerto is None:
            puerto = self._buscar_puerto()
        if puerto is None:
            self.estado.anotar("puente: no hay ningun puerto serie; se sigue "
                               "buscando (enchufalo cuando quieras)", AMBAR)
            self.estado.nombre_puerto = "NINGUNO"
            return
        self._conectar_puente(puerto)

    def _conectar_puente(self, puerto):
        if self.puente is not None:
            self.puente.parar(esperar=False)
        self.puente = pte.Puente(puerto)
        self.estado.nombre_puerto = puerto
        if self.puente.arrancar():
            self.estado.anotar("abriendo %s a %d baudios"
                               % (puerto, pte.BAUDIOS))
        else:
            self.estado.anotar(self.puente.ultimo_error, ROJO)

    @staticmethod
    def _buscar_puerto():
        """
        El puente va detrás de un CH340, así que se prefiere ese. Si hay
        varios, gana el primero; el usuario siempre puede forzarlo con
        --puerto.
        """
        disponibles = pte.puertos_disponibles()
        for dispositivo, descripcion in disponibles:
            if _es_ch340(descripcion):
                return dispositivo
        # Nunca el de la estacion de CATS (cats.py)
        otros = [d for d, _ in disponibles if not ct.es_puerto_cats(d)]
        return otros[0] if otros else None

    def _abrir_cats(self, puerto):
        if puerto is None:
            puerto = ct.buscar_puerto()
        if puerto is None:
            self.estado.anotar("CATS: no hay estacion de CATS; se sigue "
                               "buscando un ESP32 por USB (--cats COMx para "
                               "forzarla)", VERDE_MED)
            return
        self._conectar_cats(puerto)

    def _conectar_cats(self, puerto):
        if self.lector_cats is not None:
            self.lector_cats.parar(esperar=False)
        self.lector_cats = ct.LectorCats(puerto)
        self.estado.cats_puerto = puerto
        if self.lector_cats.arrancar():
            self.estado.anotar("CATS: leyendo %s" % puerto, CIAN)
        else:
            self.estado.anotar("CATS: %s" % self.lector_cats.ultimo_error,
                               ROJO)

    def _vigilar_puertos(self):
        """
        Cada PERIODO_BUSQUEDA_S, lo que no se encontró al arrancar: el puente
        o la estación de CATS enchufados más tarde, o con otro COM al volver a
        enchufarlos. Solo con los puertos que se buscan solos; uno forzado ya
        lo reintenta su propio lector.

        Del puente se cambia si el que hay está desenchufado, o si se cogió
        uno que no es un CH340 (a falta de otro) y ahora hay un CH340. Uno
        abierto que funciona no se toca.
        """
        if self.demo:
            return
        ahora = time.monotonic()
        if ahora < self._proxima_busqueda:
            return
        self._proxima_busqueda = ahora + PERIODO_BUSQUEDA_S

        actual = self.puente
        if self._puerto_forzado is None and not (
                actual is not None and actual.conectado and
                _es_ch340(_descripcion(actual.puerto))):
            nuevo = self._buscar_puerto()
            if actual is None:
                cambiar = nuevo is not None
            else:
                cambiar = (nuevo is not None and nuevo != actual.puerto and
                           (not actual.conectado or
                            _es_ch340(_descripcion(nuevo))))
            if cambiar:
                self.estado.anotar("puente: encontrado en %s" % nuevo, CIAN)
                self._conectar_puente(nuevo)

        if not self._sin_cats and self._cats_forzado is None:
            lc = self.lector_cats
            if lc is None or not lc.conectado:
                nuevo = ct.buscar_puerto()
                if nuevo is not None and (lc is None or nuevo != lc.puerto):
                    self.estado.anotar("CATS: encontrada en %s" % nuevo, CIAN)
                    self._conectar_cats(nuevo)

    def alternar_fuente(self):
        """Boton DATOS: de quien son la altura, la velocidad vertical, la
        fase y el recorrido que se pintan."""
        e = self.estado
        e.fuente = "ASPID" if e.ver_cats else "CATS"
        e.anotar("altura, velocidad, fase y recorrido: %s"
                 % ("de CATS" if e.ver_cats else "de nuestra avionica"),
                 MAGENTA if e.ver_cats else CIAN)

    def _ingerir_cats(self):
        if self.sim_cats is not None:
            for linea in self.sim_cats.lineas():
                p = ct.decodificar(linea)
                if p is not None:
                    self._encajar_cats(p)
        elif self.lector_cats is not None:
            for p in self.lector_cats.vaciar():
                self._encajar_cats(p)

    def _encajar_cats(self, p):
        self.estado.encajar_cats(p)
        if self.grabador is not None:
            self.grabador.cats(p)

    # --- Mando ---

    def ping(self):
        """
        Ida y vuelta CON EL COHETE: el cohete contesta con un acuse que
        devuelve el numero de ping. Se reintenta cada ~0,5 s hasta la
        respuesta, y cada reintento lleva su numero, para medir la ida y
        vuelta desde el envio que de verdad se contesto.
        """
        e = self.estado
        e.ping_activo = True
        e.ping_t0 = time.monotonic()
        e.ping_envios = {}
        e.intentos_ping = 0
        self._proximo_ping = 0.0
        e.anotar("-> ping al cohete", CIAN)

    def _enviar_comando(self, cmd, parametro=0, automatico=False,
                        silencioso=False):
        """Pone un comando de 4 bytes en el aire. Ver paquete.comando()."""
        nombre = pq.UPLINK_NOMBRES.get(cmd, "cmd 0x%02X" % cmd)

        # Ultima red del cerrojo: las puertas de arriba ya lo paran, pero
        # nada peligroso sale al aire con el cerrojo puesto, venga de donde
        # venga (un reintento que se quedo programado, por ejemplo)
        if self.estado.bloqueado and self._peligroso(cmd, parametro):
            if not silencioso:
                self._bloqueado(nombre)
            return False

        if self.demo:
            self.sim.comando(cmd, parametro)
        else:
            if self.puente is None or not self.puente.conectado:
                if not silencioso:
                    self.estado.anotar("%s: no hay puerto abierto" % nombre,
                                       ROJO)
                return False
            if not self.puente.enviar(pq.comando_para(cmd, parametro)):
                self.estado.anotar("%s: fallo al escribir en el puerto"
                                   % nombre, ROJO)
                return False

        if self.grabador is not None:
            self.grabador.comando(cmd, parametro, automatico)
        if cmd == pq.UPLINK_CMD_PEDIR_ESTADO:
            self.estado.peticiones_estado += 1
        if not silencioso:
            self.estado.anotar("-> %s%s" % (nombre, " (automatico)"
                                            if automatico else ""), CIAN)
        return True

    def _rafaga(self, cmd, parametro, n, espacio_ms, texto):
        """
        Manda el mismo comando n veces seguidas sin bloquear la ventana. Con el
        30 % de acierto medido en la subida, uno suelto falla dos de cada tres
        veces; cinco llegan el 83 %. Solo para comandos que se puedan repetir
        sin dano, que en este cohete son todos (ver App/uplink.h).
        """
        if self.estado.bloqueado and self._peligroso(cmd, parametro):
            self._bloqueado(texto)
            return
        self.estado.anotar("-> %s (x%d)" % (texto, n), ROJO
                           if cmd == pq.UPLINK_CMD_PIRO else CIAN)
        if cmd == pq.UPLINK_CMD_PIRO:
            self.estado.disparos_enviados += 1
        for i in range(n):
            self.raiz.after(i * espacio_ms,
                            lambda: self._enviar_comando(cmd, parametro,
                                                         silencioso=True))

    # --- Cerrojo de la rampa ---

    @staticmethod
    def _peligroso(cmd, parametro):
        """
        Lo que el cerrojo de la rampa no deja mandar: todo lo que puede poner
        en peligro el cohete. Entrar en modo test (y lo que cuelga de el:
        pirotecnia, prueba y extension del aerofreno) y SIMULAR RUTA (dispara
        los pirotecnicos de verdad). Lo que PARA algo sigue permitido (salir
        del modo test, parar la simulacion, la prueba o recoger el
        aerofreno), y tambien el ping, pedir estado y las camaras: con el
        cerrojo puesto nunca se queda nada peligroso en marcha sin poder
        pararlo. El angulo y el rumbo de la rampa NO se bloquean (decidido
        2026-10-08): se fijan con el cohete ya en la rampa.
        """
        if cmd == pq.UPLINK_CMD_PIRO:
            return True
        if cmd in (pq.UPLINK_CMD_MODO_TEST, pq.UPLINK_CMD_SIMULACION,
                   pq.UPLINK_CMD_PRUEBA_AEROFRENO,
                   pq.UPLINK_CMD_EXTENSION_AEROFRENO):
            return parametro != 0
        return False

    def _bloqueado(self, texto):
        """True, y lo dice en el registro, si el cerrojo impide 'texto'."""
        if not self.estado.bloqueado:
            return False
        self.estado.anotar("BLOQUEADO: %s no se manda con el cerrojo de la "
                           "rampa puesto (boton RAMPA y contrasena)" % texto,
                           AMBAR)
        return True

    def alternar_bloqueo(self):
        if self.estado.bloqueado:
            self._desbloquear()
        else:
            self.bloquear()

    def _desbloquear(self):
        clave = simpledialog.askstring(
            "Quitar el cerrojo de la rampa",
            "Contrasena para permitir el modo test, la pirotecnia, SIMULAR "
            "RUTA y el aerofreno:",
            show="*", parent=self.raiz)
        if clave is None:
            return
        if clave != CLAVE_DESBLOQUEO:
            self.estado.anotar("contrasena incorrecta: el cerrojo de la rampa "
                               "sigue puesto", ROJO)
            return
        self.estado.bloqueado = False
        self.estado.anotar("CERROJO DE LA RAMPA QUITADO: ya se pueden mandar "
                           "ordenes peligrosas", ROJO)

    def bloquear(self):
        """
        Pone el cerrojo (sin contrasena) y deshace lo que este en marcha: sale
        del modo test, para la simulacion y olvida los envios pendientes del
        aerofreno, que si no seguirian reintentandose.
        """
        e = self.estado
        e.bloqueado = True
        e.anotar("CERROJO DE LA RAMPA PUESTO: modo test, pirotecnia, "
                 "simulacion y aerofreno bloqueados", VERDE)
        if e.simulacion_pedida or e.simulando:
            self.parar_simulacion()
        if e.modo_test_pedido or e.modo_test_confirmado:
            self.salir_modo_test()
        if e.aero_prueba_estado == "enviando":
            e.aero_prueba_estado = ""
        if e.aero_ext_estado == "enviando" and e.aero_ext_pedida != 0:
            e.aero_ext_estado = ""

    # --- Modo test ---

    def alternar_modo_test(self):
        if self.estado.modo_test_pedido:
            self.salir_modo_test()
        else:
            self.entrar_modo_test()

    def entrar_modo_test(self):
        if self._bloqueado("el modo test"):
            return
        self.estado.modo_test_pedido = True
        self.estado.intentos_test = 1
        self._rafaga(pq.UPLINK_CMD_MODO_TEST, 1, 3, 150, "entrar en modo test")
        self._proximo_test = time.monotonic() + _con_jitter(REINTENTO_TEST_S)

    def salir_modo_test(self):
        self.estado.modo_test_pedido = False
        self.estado.intentos_test = 1
        self._rafaga(pq.UPLINK_CMD_MODO_TEST, 0, 5, 150, "salir del modo test")
        self._proximo_test = time.monotonic() + _con_jitter(INSISTIR_SALIDA_S)

    # --- Angulo de lanzamiento ---

    def alternar_angulo(self):
        self.angulo_abierto = not self.angulo_abierto
        e = self.estado
        # Al abrir, si no se sabe que tiene el cohete, se le pregunta
        if self.angulo_abierto and e.angulo_cohete is None and \
                e.angulo_estado not in ("enviando", "consultando"):
            self.consultar_angulo()
        if self.angulo_abierto and e.rumbo_cohete is None and \
                e.rumbo_estado not in ("enviando", "consultando"):
            self.consultar_rumbo()

    def consultar_angulo(self):
        e = self.estado
        e.angulo_estado = "consultando"
        e.intentos_angulo = 0
        e.t_angulo = time.monotonic()
        self._proximo_angulo = 0.0

    def enviar_angulo(self, grados):
        """
        Lo llama el panel. Se reintenta cada ~REINTENTO_ANGULO_S hasta que el
        cohete conteste con ESTE valor, lo deniegue, o pase PLAZO_ANGULO_S.
        """
        e = self.estado
        e.angulo_pedido = round(float(grados), 2)
        e.angulo_estado = "enviando"
        e.angulo_motivo = ""
        e.intentos_angulo = 0
        e.t_angulo = time.monotonic()
        self._proximo_angulo = 0.0
        e.anotar("-> angulo de lanzamiento %.2f grados" % e.angulo_pedido, CIAN)

    def enviar_rampa(self, grados, rumbo):
        """Lo llama el panel: angulo y rumbo, cada uno con sus reintentos."""
        self.enviar_angulo(grados)
        self.enviar_rumbo(rumbo)

    def consultar_rumbo(self):
        e = self.estado
        e.rumbo_estado = "consultando"
        e.intentos_rumbo = 0
        e.t_rumbo = time.monotonic()
        self._proximo_rumbo = 0.0

    def enviar_rumbo(self, grados):
        e = self.estado
        e.rumbo_pedido = round(float(grados), 2)
        e.rumbo_estado = "enviando"
        e.rumbo_motivo = ""
        e.intentos_rumbo = 0
        e.t_rumbo = time.monotonic()
        self._proximo_rumbo = 0.0
        e.anotar("-> rumbo de la rampa %.2f grados" % e.rumbo_pedido, CIAN)

    # --- Camaras ---

    def alternar_camaras(self):
        """
        Pide lo contrario de lo que dice el cohete (encender si no se sabe).
        Se reintenta cada segundo hasta que un acuse o un paquete de estado
        lo confirma, o hasta PLAZO_CAMARAS_S. Pulsar otra vez mientras se
        espera cambia lo pedido.
        """
        e = self.estado
        if e.camaras_pedidas is not None:
            objetivo = not e.camaras_pedidas
        else:
            objetivo = not bool(e.camaras_cohete)
        e.camaras_pedidas = objetivo
        e.camaras_motivo = ""
        e.intentos_camaras = 1
        e.t_camaras = time.monotonic()
        self._rafaga(pq.UPLINK_CMD_CAMARAS, 1 if objetivo else 0, 3, 150,
                     "ENCENDER CAMARAS" if objetivo else "apagar camaras")
        self._proximo_camaras = time.monotonic() + _con_jitter(
            REINTENTO_TEST_S)

    def _mantener_camaras(self):
        e = self.estado
        if e.camaras_pedidas is None:
            return
        ahora = time.monotonic()
        if ahora - e.t_camaras > PLAZO_CAMARAS_S:
            e.anotar("el cohete NO confirma las camaras en %.0f s (%d "
                     "intentos)" % (PLAZO_CAMARAS_S, e.intentos_camaras), ROJO)
            e.camaras_pedidas = None
            return
        if ahora < self._proximo_camaras:
            return
        self._enviar_comando(pq.UPLINK_CMD_CAMARAS,
                             1 if e.camaras_pedidas else 0, silencioso=True)
        e.intentos_camaras += 1
        self._proximo_camaras = ahora + _con_jitter(REINTENTO_TEST_S)

    def _mantener_rumbo(self):
        e = self.estado
        if e.rumbo_estado not in ("enviando", "consultando"):
            return
        ahora = time.monotonic()
        if ahora - e.t_rumbo > PLAZO_ANGULO_S:
            e.rumbo_estado = "sin respuesta"
            e.anotar("el cohete no ha contestado al rumbo en %.0f s"
                     % PLAZO_ANGULO_S, ROJO)
            return
        if ahora < self._proximo_rumbo:
            return
        if e.rumbo_estado == "consultando":
            valor = pq.ANGULO_CONSULTA
        else:
            valor = int(round(e.rumbo_pedido * 100))
        self._enviar_comando(pq.UPLINK_CMD_RUMBO, valor, silencioso=True)
        e.intentos_rumbo += 1
        self._proximo_rumbo = ahora + _con_jitter(REINTENTO_ANGULO_S)

    def _mantener_angulo(self):
        e = self.estado
        if e.angulo_estado not in ("enviando", "consultando"):
            return
        ahora = time.monotonic()
        if ahora - e.t_angulo > PLAZO_ANGULO_S:
            e.angulo_estado = "sin respuesta"
            e.anotar("el cohete no ha contestado al angulo en %.0f s"
                     % PLAZO_ANGULO_S, ROJO)
            return
        if ahora < self._proximo_angulo:
            return
        if e.angulo_estado == "consultando":
            valor = pq.ANGULO_CONSULTA
        else:
            valor = int(round(e.angulo_pedido * 100))
        self._enviar_comando(pq.UPLINK_CMD_ANGULO, valor, silencioso=True)
        e.intentos_angulo += 1
        self._proximo_angulo = ahora + _con_jitter(REINTENTO_ANGULO_S)

    # --- Pruebas de aerofreno ---

    def probar_aerofreno(self):
        """
        Lo llama el panel tras los dos clics. Cada pulsacion lleva un numero
        nuevo: el cohete ejecuta un numero una sola vez, asi que los
        reintentos de abajo no la repiten aunque lleguen despues del acuse.
        """
        if self._bloqueado("la prueba de aerofreno"):
            return
        e = self.estado
        self._num_prueba_aero = self._num_prueba_aero % 255 + 1
        e.aero_prueba_num = self._num_prueba_aero
        e.aero_prueba_estado = "enviando"
        e.aero_prueba_motivo = ""
        e.intentos_aero_prueba = 0
        e.aero_prueba_t = time.monotonic()
        self._proximo_aero_prueba = 0.0
        e.anotar("-> PRUEBA DE AEROFRENO #%d" % e.aero_prueba_num, CIAN)

    def enviar_extension_aerofreno(self, pct):
        """Lo llama el panel. Se reintenta hasta ver el acuse con ESTE valor."""
        if int(pct) != 0 and self._bloqueado("la extension del aerofreno"):
            return
        e = self.estado
        e.aero_ext_pedida = int(pct)
        e.aero_ext_estado = "enviando"
        e.aero_ext_motivo = ""
        e.intentos_aero_ext = 0
        e.aero_ext_t = time.monotonic()
        self._proximo_aero_ext = 0.0
        e.anotar("-> aerofreno al %d %%" % e.aero_ext_pedida, CIAN)

    def _mantener_aerofreno(self):
        e = self.estado
        ahora = time.monotonic()

        if e.aero_prueba_estado == "en curso" and \
                ahora - e.aero_prueba_t > pq.PRUEBA_AEROFRENO_S:
            e.aero_prueba_estado = "terminada"
        if e.aero_prueba_estado == "enviando":
            if ahora - e.aero_prueba_t > PLAZO_AEROFRENO_S:
                e.aero_prueba_estado = "sin respuesta"
                e.anotar("el cohete no ha contestado a la prueba de aerofreno"
                         " en %.0f s" % PLAZO_AEROFRENO_S, ROJO)
            elif ahora >= self._proximo_aero_prueba:
                self._enviar_comando(pq.UPLINK_CMD_PRUEBA_AEROFRENO,
                                     e.aero_prueba_num, silencioso=True)
                e.intentos_aero_prueba += 1
                self._proximo_aero_prueba = ahora + _con_jitter(
                    REINTENTO_AEROFRENO_S)

        if e.aero_ext_estado == "enviando":
            if ahora - e.aero_ext_t > PLAZO_AEROFRENO_S:
                e.aero_ext_estado = "sin respuesta"
                e.anotar("el cohete no ha contestado a la extension de "
                         "aerofreno en %.0f s" % PLAZO_AEROFRENO_S, ROJO)
            elif ahora >= self._proximo_aero_ext:
                self._enviar_comando(pq.UPLINK_CMD_EXTENSION_AEROFRENO,
                                     e.aero_ext_pedida, silencioso=True)
                e.intentos_aero_ext += 1
                self._proximo_aero_ext = ahora + _con_jitter(
                    REINTENTO_AEROFRENO_S)

    # --- Simulacion de ruta ---

    def iniciar_simulacion(self):
        """
        Lo llama el panel de pruebas tras los dos clics. El cohete cierra el
        modo test al empezar, asi que aqui se deja de renovarlo: si no, la
        renovacion lo volveria a pedir y el cohete lo denegaria por estar
        simulando.
        """
        if self._bloqueado("SIMULAR RUTA"):
            return
        e = self.estado
        e.simulacion_pedida = True
        e.intentos_sim = 1
        e.modo_test_pedido = False
        self._rafaga(pq.UPLINK_CMD_SIMULACION, 1, 3, 150, "SIMULAR RUTA")
        self._proximo_sim = time.monotonic() + _con_jitter(REINTENTO_TEST_S)

    def parar_simulacion(self):
        e = self.estado
        e.simulacion_pedida = False
        e.intentos_sim = 1
        self._rafaga(pq.UPLINK_CMD_SIMULACION, 0, 5, 150, "parar simulacion")
        self._proximo_sim = time.monotonic() + _con_jitter(REINTENTO_TEST_S)

    def _mantener_simulacion(self):
        """
        Igual que con el modo test: hasta que el cohete confirme lo pedido, se
        reintenta cada segundo. Empezar es idempotente en el cohete (si ya
        esta en marcha no hace nada) y parar tambien.
        """
        e = self.estado
        ahora = time.monotonic()
        if ahora < self._proximo_sim:
            return
        if e.simulacion_pedida and not e.simulando:
            self._enviar_comando(pq.UPLINK_CMD_SIMULACION, 1, silencioso=True)
            e.intentos_sim += 1
            self._proximo_sim = ahora + _con_jitter(REINTENTO_TEST_S)
        elif not e.simulacion_pedida and e.simulando:
            self._enviar_comando(pq.UPLINK_CMD_SIMULACION, 0, silencioso=True)
            e.intentos_sim += 1
            self._proximo_sim = ahora + _con_jitter(REINTENTO_TEST_S)

    def _mantener_modo_test(self):
        """
        Hombre muerto en las dos direcciones. Mientras se quiera estar en modo
        test, se renueva; si se ha pedido salir y el cohete sigue dentro, se
        insiste. Y si la estacion arranca y se encuentra el cohete en modo
        test sin haberlo pedido, tambien insiste en salir: el estado seguro
        por defecto es fuera.
        """
        ahora = time.monotonic()
        if ahora < self._proximo_test:
            return
        e = self.estado
        if e.simulacion_pedida or e.simulando:
            # El cohete cierra el modo test al simular. Pedirle que salga
            # mientras la orden de simular viaja podria cerrarlo ANTES de que
            # llegue, y entonces la simulacion se denegaria por no estar en
            # modo test.
            return
        if e.modo_test_pedido:
            self._enviar_comando(pq.UPLINK_CMD_MODO_TEST, 1, silencioso=True)
            s = e.estado_paquete
            if e.modo_test_confirmado:
                # Ya dentro: solo hay que renovar el plazo
                self._proximo_test = ahora + _con_jitter(RENOVAR_TEST_S)
            elif s is not None and not s.admite_modo_test:
                # El cohete lo esta denegando (no esta en tierra). Insistir
                # cada segundo no lo va a cambiar y cada denegacion le roba un
                # hueco a la telemetria; se sigue pidiendo, pero despacio, por
                # si vuelve a tierra.
                self._proximo_test = ahora + _con_jitter(RENOVAR_TEST_S)
            else:
                e.intentos_test += 1
                self._proximo_test = ahora + _con_jitter(REINTENTO_TEST_S)
        elif e.modo_test_confirmado:
            self._enviar_comando(pq.UPLINK_CMD_MODO_TEST, 0, silencioso=True)
            e.intentos_test += 1
            self._proximo_test = ahora + _con_jitter(INSISTIR_SALIDA_S)

    def _pintar_modo_test(self):
        """
        Tres paneles comparten la casilla del RECORRIDO. Manda el de pruebas
        (si el cohete esta o se le ha pedido estar en modo test, eso es lo que
        hay que ver), luego el del angulo si esta abierto, y si no el mapa.
        """
        e = self.estado
        if e.modo_test_pedido or e.modo_test_confirmado:
            quiero = "test"
        elif self.angulo_abierto:
            quiero = "angulo"
        else:
            quiero = "mapa"
        if quiero != self._panel_visible:
            for nombre, w in (("mapa", self.panel_mapa),
                              ("test", self.panel_pruebas),
                              ("angulo", self.panel_angulo)):
                if nombre == quiero:
                    w.grid()
                else:
                    w.grid_remove()
            self._panel_visible = quiero
        self._test_visible = (quiero == "test")

        if quiero == "test":
            self.panel_pruebas.actualizar(e)
        elif quiero == "angulo":
            if e.angulo_recien_leido:
                # La ruleta arranca en lo que tiene el cohete
                self.panel_angulo.poner(e.angulo_cohete)
                e.angulo_recien_leido = False
            if e.rumbo_recien_leido:
                # Y la casilla del rumbo, igual
                self.panel_angulo.poner_rumbo(e.rumbo_cohete)
                e.rumbo_recien_leido = False
            self.panel_angulo.actualizar(e)
        self.btn_angulo.configure(text="CERRAR ANGULO" if self.angulo_abierto
                                  else "ANGULO LANZ.")

        if e.bloqueado:
            self.btn_bloqueo.configure(text="RAMPA: BLOQUEADO", fg=VERDE)
            self.aviso.configure(
                fg=VERDE_MED,
                text="Cerrojo de la rampa PUESTO: modo test, pirotecnia, "
                     "simulacion y aerofreno bloqueados")
        else:
            self.btn_bloqueo.configure(text="RAMPA: LIBRE", fg=ROJO)
            self.aviso.configure(
                fg=AMBAR,
                text="CERROJO QUITADO. MODO TEST solo en tierra; los "
                     "pirotecnicos se disparan desde el panel que sustituye "
                     "al RECORRIDO")

        if e.ver_cats:
            edad = e.edad_cats_s
            self.btn_fuente.configure(
                text="DATOS: CATS" if (edad is not None and edad < 3.0)
                else "DATOS: CATS (sin datos)", fg=MAGENTA)
        else:
            self.btn_fuente.configure(text="DATOS: ASPID", fg=VERDE)

        if e.modo_test_pedido:
            self.btn_test.configure(text="SALIR TEST", fg=AMBAR)
        else:
            self.btn_test.configure(text="MODO TEST",
                                    fg=VERDE_OSC if e.bloqueado else ROJO)

        if e.camaras_pedidas is not None:
            self.btn_camaras.configure(
                text="ENCENDIENDO CAM." if e.camaras_pedidas
                else "APAGANDO CAM.", fg=AMBAR)
        elif e.camaras_cohete and e.camaras_apagando:
            self.btn_camaras.configure(text="CERRANDO VIDEOS", fg=AMBAR)
        elif e.camaras_cohete is None:
            self.btn_camaras.configure(text="CAMARAS ?", fg=CIAN)
        elif e.camaras_cohete:
            self.btn_camaras.configure(text="CAMARAS: ON", fg=VERDE)
        else:
            self.btn_camaras.configure(text="CAMARAS: OFF", fg=VERDE_MED)

        sim = e.simulacion_pedida or e.simulando
        self.btn_sim.configure(state="normal" if sim else "disabled",
                               fg=CIAN if sim else VERDE_OSC,
                               cursor="hand2" if sim else "arrow")

    def pedir_estado(self):
        """Se reintenta cada ~0,5 s hasta que llega un paquete de estado."""
        e = self.estado
        e.estado_pedido = True
        e.t_estado_pedido = time.monotonic()
        e.intentos_estado = 0
        self._proximo_pedir = 0.0
        e.anotar("-> pedir estado", CIAN)

    def _mantener_peticiones(self):
        e = self.estado
        ahora = time.monotonic()

        if e.ping_activo:
            if ahora - e.ping_t0 > PLAZO_PETICION_S:
                e.ping_activo = False
                e.anotar("el cohete NO contesta al ping en %.0f s (%d "
                         "intentos)" % (PLAZO_PETICION_S, e.intentos_ping),
                         ROJO)
            elif ahora >= self._proximo_ping:
                self._numero_ping = (self._numero_ping + 1) & 0xFFFF
                if self._enviar_comando(pq.UPLINK_CMD_PING, self._numero_ping,
                                        silencioso=True):
                    e.ping_envios[self._numero_ping] = ahora
                e.intentos_ping += 1
                self._proximo_ping = ahora + _con_jitter(REINTENTO_PETICION_S)

        if e.estado_pedido:
            if ahora - e.t_estado_pedido > PLAZO_PETICION_S:
                e.estado_pedido = False
                e.anotar("el cohete NO manda su estado en %.0f s (%d "
                         "intentos)" % (PLAZO_PETICION_S, e.intentos_estado),
                         ROJO)
            elif ahora >= self._proximo_pedir:
                self._enviar_comando(pq.UPLINK_CMD_PEDIR_ESTADO,
                                     silencioso=True)
                e.intentos_estado += 1
                self._proximo_pedir = ahora + _con_jitter(REINTENTO_PETICION_S)

    def _pedir_estado_si_falta(self):
        """
        Insiste cada PERIODO_PETICION_S mientras no haya llegado ningún
        paquete de estado, y deja de insistir en cuanto llega uno.

        Sólo se pide SI EL ENLACE ESTÁ VIVO. Si no llega telemetría, el cohete
        no está ahí o no nos oye, y ponerse a radiar peticiones al vacío
        gastaría aire en la dirección que menos sobra sin ninguna posibilidad
        de que sirvan.
        """
        if not self.estado.falta_estado:
            return
        if not self.estado.enlace_vivo:
            return
        ahora = time.monotonic()
        if ahora < self._proxima_peticion:
            return
        self._proxima_peticion = ahora + _con_jitter(PERIODO_PETICION_S)
        self._enviar_comando(pq.UPLINK_CMD_PEDIR_ESTADO, automatico=True)

    # --- Bucle ---

    def _tic(self):
        try:
            self._vigilar_puertos()
            self._ingerir()
            self._ingerir_cats()
            self._pedir_estado_si_falta()
            self._mantener_modo_test()
            self._mantener_simulacion()
            self._mantener_angulo()
            self._mantener_rumbo()
            self._mantener_camaras()
            self._mantener_aerofreno()
            self._mantener_peticiones()
            self._guardar_tic()
            self._pintar()
        except Exception as e:
            self.estado.anotar("fallo en el refresco: %s" % e, ROJO)
        self._id_tic = self.raiz.after(REFRESCO_MS, self._tic)

    def _ingerir(self):
        if self.demo:
            ahora = time.monotonic()
            # La cadencia del cohete: 3 Hz en tierra, 15 en vuelo
            periodo = 1.0 / self.sim.cadencia_hz()
            while ahora - self._ultimo_demo > periodo:
                self._ultimo_demo = max(self._ultimo_demo + periodo,
                                        ahora - 0.5)
                tr = self.sim.trama()
                tr.t = self._ultimo_demo  # la hora que le tocaba, no la de hoy
                self.estado.encajar(tr)
                self._guardar(tr)
            return

        if self.puente is None:
            return
        self.estado.puerto_abierto = self.puente.conectado
        # Son DOS cosas distintas y conviene no mezclarlas: crc_malos son
        # paquetes que llegaron por el aire corruptos (dicen dónde acaba la
        # cobertura), y serie_crc_malos son tramas que se estropearon en el
        # cable USB (dicen que hay un problema en la mesa, no en el enlace).
        self.estado.serie_crc_malos = self.puente.tramas_crc_malo
        self.estado.puente_descartadas = self.puente.tramas_descartadas
        for trama in self.puente.vaciar():
            self.estado.encajar(trama)
            self._guardar(trama)

    def _guardar(self, trama):
        # Despues de encajar: si la trama cierra una simulacion, encajar ya
        # ha bajado simulacion_pedida
        if self.grabador is not None:
            self.grabador.trama(trama, self.estado.simulacion_pedida)

    def _guardar_tic(self):
        g = self.grabador
        if g is None:
            return
        g.tic()
        if g.fallo is not None and not self._fallo_grabador_dicho:
            self._fallo_grabador_dicho = True
            self.estado.anotar("FALLO AL GUARDAR LOS DATOS (%s). La estacion "
                               "sigue, pero puede faltar algo en %s"
                               % (g.fallo, g.dir), ROJO)

    def _pintar(self):
        for p in self.paneles:
            p.refrescar(self.estado)
        self._pintar_modo_test()

        nuevos = min(self.estado.n_eventos - self._eventos_pintados,
                     len(self.estado.eventos))
        pendientes = list(self.estado.eventos)[-nuevos:] if nuevos > 0 \
            else []
        if pendientes:
            self.texto.configure(state="normal")
            for hora, texto, color in pendientes:
                etiqueta = {VERDE: "verde", AMBAR: "ambar", ROJO: "rojo",
                            CIAN: "cian"}.get(color, "medio")
                self.texto.insert("end", "%s  %s\n" % (hora, texto), etiqueta)
            self.texto.see("end")
            self.texto.configure(state="disabled")
            self._eventos_pintados = self.estado.n_eventos

        if self.puente is not None:
            self.estado_tx.configure(
                text="puente: %s   %d tramas   %d B"
                     % ("conectado" if self.puente.conectado else "cerrado",
                        self.puente.tramas_ok, self.puente.bytes_leidos),
                fg=VERDE_MED if self.puente.conectado else ROJO)

    def cerrar(self):
        try:
            self.raiz.after_cancel(self._id_tic)
        except Exception:
            pass
        if self.puente is not None:
            self.puente.parar()
        if self.lector_cats is not None:
            self.lector_cats.parar()
        if self.grabador is not None:
            self.grabador.cerrar()
        self.raiz.destroy()


def main():
    ap = argparse.ArgumentParser(description="Estacion de tierra del cohete")
    ap.add_argument("--puerto", help="puerto serie del puente, p.ej. COM11")
    ap.add_argument("--demo", action="store_true",
                    help="vuelo simulado, sin hardware")
    ap.add_argument("--demo-tierra", action="store_true",
                    help="como --demo pero el cohete no despega: para probar "
                         "el modo test sin hardware")
    ap.add_argument("--demo-emergencia", action="store_true",
                    help="como --demo pero la core se reinicia en vuelo a los "
                         "20 s y sigue en modo de emergencia")
    ap.add_argument("--cats", metavar="PUERTO",
                    help="puerto de la estacion de CATS (si no, se busca "
                         "sola: un ESP32 por USB)")
    ap.add_argument("--sin-cats", action="store_true",
                    help="no lee ninguna estacion de CATS")
    ap.add_argument("--listar", action="store_true",
                    help="lista los puertos y sale")
    ap.add_argument("--sin-guardar", action="store_true",
                    help="no guarda los datos recibidos en el escritorio")
    ap.add_argument("--carpeta-datos", metavar="RUTA",
                    help="guarda los datos aqui en vez de en "
                         "Escritorio/LEEM_datos")
    args = ap.parse_args()

    if args.listar:
        for dispositivo, descripcion in pte.puertos_disponibles():
            print("%-8s %s%s" % (dispositivo, descripcion,
                                 "   <- CATS" if ct.es_puerto_cats(dispositivo)
                                 else ""))
        return 0

    raiz = tk.Tk()
    Estacion(raiz, puerto=args.puerto,
             demo=args.demo or args.demo_tierra or args.demo_emergencia,
             demo_tierra=args.demo_tierra, guardar=not args.sin_guardar,
             carpeta_datos=args.carpeta_datos,
             demo_emergencia=args.demo_emergencia,
             puerto_cats=args.cats, sin_cats=args.sin_cats)
    raiz.mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
