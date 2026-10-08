# -*- coding: utf-8 -*-
"""
Los paneles de la estación de tierra.

Cada uno lee del objeto Estado (ver estacion.py) y se pinta solo. Ninguno
guarda datos: si un panel necesita historia, la saca de Estado. Así se puede
añadir, quitar o recolocar cualquiera sin tocar el resto.

REGLA QUE CONVIENE RESPETAR AL AÑADIR PANELES: aquí no se calcula nada que no
sea presentación. Toda la interpretación del paquete está en paquete.py y toda
la acumulación en Estado. Si un panel empieza a hacer cuentas, ese cálculo
acaba duplicado en otro sitio y los dos se separan.
"""

import math
import time

from widgets import (AMBAR, APAGADO, AZUL, CIAN, FONDO, MAGENTA, REJILLA, ROJO,
                     VERDE, VERDE_MED, VERDE_OSC, Panel, barra_con_centro,
                     barra_segmentada, circulo_fase, fuente, lampara, rejilla,
                     traza)

import paquete as pq


def _num(v, fmt="%.1f", guion="--"):
    """Formatea, o pone guiones si no hay dato. Nunca inventa un cero."""
    if v is None:
        return guion
    try:
        return fmt % v
    except Exception:
        return guion


def _hhmmss(segundos):
    s = int(segundos)
    return "%02d:%02d:%02d" % (s // 3600, (s // 60) % 60, s % 60)


def _edad(segundos):
    if segundos is None:
        return "--"
    if segundos < 90:
        return "%.0f s" % segundos
    if segundos < 5400:
        return "%.0f min" % (segundos / 60.0)
    return "%.1f h" % (segundos / 3600.0)


def _mb(megas):
    """MB o GB, lo que se lea mejor."""
    if megas >= 1024:
        return "%.1f GB" % (megas / 1024.0)
    return "%d MB" % megas


# --- Cabecera ---------------------------------------------------------------


class PanelCabecera(Panel):
    """Barra superior: identidad, reloj, salud del enlace."""

    def pintar(self):
        e = self.estado
        x0, y0, x1, y1 = self.area()
        cy = (y0 + y1) / 2.0

        self.create_text(x0 + 4, cy - 7, text="L E E M", anchor="w",
                         fill=VERDE, font=fuente(13, True))
        self.create_text(x0 + 4, cy + 8, text="ESTACION DE TIERRA",
                         anchor="w", fill=VERDE_MED, font=fuente(8))

        if e is None:
            return

        # Semáforo del enlace: lo primero que mira uno al llegar
        if not e.puerto_abierto:
            col, txt = ROJO, "SIN PUERTO"
        elif e.simulando:
            # Va por delante de ENLACE OK a proposito: con el enlace perfecto,
            # lo que mas importa saber es que los datos NO son de un vuelo.
            col, txt = CIAN, "SIM %d %%" % e.estado_paquete.sim_progreso
        elif e.enlace_vivo:
            col, txt = VERDE, "ENLACE OK"
        else:
            col, txt = AMBAR, "SIN SENAL"
        self.create_rectangle(x0 + 150, cy - 11, x0 + 258, cy + 11,
                              outline=col)
        self.create_text(x0 + 204, cy, text=txt, fill=col,
                         font=fuente(11, True))

        campos = [
            ("PUERTO", e.nombre_puerto),
            ("PAQUETES", "%d" % e.recibidos),
            ("PERDIDOS", "%d" % e.perdidos),
            ("CADENCIA", "%.1f Hz" % e.hz),
            ("EDAD", "%.1f s" % e.edad_s if e.paquete else "--"),
            ("SESION", e.reloj_sesion()),
        ]
        x = x0 + 285
        for etiqueta, valor in campos:
            self.create_text(x, cy - 7, text=etiqueta, anchor="w",
                             fill=VERDE_OSC, font=fuente(7))
            self.create_text(x, cy + 7, text=valor, anchor="w", fill=VERDE,
                             font=fuente(10, True))
            x += 108

        self.create_text(x1 - 4, cy, text=time.strftime("%H:%M:%S"),
                         anchor="e", fill=VERDE_MED, font=fuente(12, True))


# --- Columna de medidores ---------------------------------------------------


class PanelMedidores(Panel):
    """
    Los seis números que hay que poder mirar sin leer: cada uno con su barra
    vertical y su círculo numerado, como el bloque izquierdo de la consola de
    referencia.
    """

    # (etiqueta, atributo, mínimo, máximo, unidad, color)
    MEDIDAS = (
        ("ALT", "altura_m", 0.0, 3000.0, "m", VERDE),
        ("VEL", "vel_vertical_mps", -100.0, 300.0, "m/s", VERDE),
        ("ACC", None, 0.0, 16.0, "g", AMBAR),
        ("AER", "aerofreno_pct", 0.0, 100.0, "%", CIAN),
        ("BAT", "bateria_v", 9.0, 20.4, "V", VERDE),
        ("%BAT", "bateria_pct", 0.0, 100.0, "%", VERDE),
        ("SEN", None, -130.0, -20.0, "dBm", MAGENTA),
    )

    # Por debajo de esto la barra de carga se pinta en rojo
    BATERIA_BAJA_PCT = 20.0

    def pintar(self):
        e = self.estado
        x0, y0, x1, y1 = self.area()
        p = e.paquete if e else None
        n = len(self.MEDIDAS)
        ancho = (x1 - x0) / float(n)

        for i, (etq, attr, lo, hi, uni, col) in enumerate(self.MEDIDAS):
            cx = x0 + ancho * (i + 0.5)

            if attr is not None and p is not None:
                v = getattr(p, attr)
            elif p is not None and etq == "ACC":
                v = p.accel_modulo_g
            elif etq == "SEN" and e and e.trama is not None:
                v = e.trama.rssi_dbm
            else:
                v = None

            if (etq == "%BAT" and v is not None and
                    v < self.BATERIA_BAJA_PCT):
                col = ROJO

            # Círculo numerado
            self.create_oval(cx - 8, y0 + 2, cx + 8, y0 + 18,
                             outline=VERDE_OSC)
            self.create_text(cx, y0 + 10, text=str(i + 1), fill=VERDE_MED,
                             font=fuente(8))

            bar_sup, bar_inf = y0 + 24, y1 - 26
            frac = 0.0 if v is None else (v - lo) / float(hi - lo)
            barra_segmentada(self, cx - 9, bar_sup, cx + 9, bar_inf,
                             frac, color=col, vertical=True)
            self.create_rectangle(cx - 9, bar_sup, cx + 9, bar_inf,
                                  outline=VERDE_OSC)

            self.create_text(cx, y1 - 17, text=etq, fill=VERDE_MED,
                             font=fuente(8, True))
            self.create_text(cx, y1 - 5,
                             text=_num(v, "%.0f" if abs(hi) > 50 else "%.1f"),
                             fill=col if v is not None else VERDE_OSC,
                             font=fuente(9, True))


# --- Trazas -----------------------------------------------------------------


class PanelPerfil(Panel):
    """Altura contra tiempo. La gráfica que de verdad cuenta el vuelo."""

    def pintar(self):
        e = self.estado
        x0, y0, x1, y1 = self.area()
        rejilla(self, x0, y0, x1, y1, nx=8, ny=4)
        if not e or not e.hist_altura:
            self.create_text((x0 + x1) / 2, (y0 + y1) / 2, text="SIN DATOS",
                             fill=VERDE_OSC, font=fuente(9))
            return

        datos = list(e.hist_altura)
        lo, hi = traza(self, x0 + 1, y0 + 1, x1 - 1, y1 - 1, datos,
                       color=VERDE, relleno=True)
        self.create_text(x0 + 4, y0 + 7, text="%.0f m" % hi, anchor="w",
                         fill=VERDE_MED, font=fuente(7))
        self.create_text(x0 + 4, y1 - 7, text="%.0f m" % lo, anchor="w",
                         fill=VERDE_MED, font=fuente(7))
        self.create_text(x1 - 4, y0 + 7,
                         text="AHORA %.1f m" % datos[-1], anchor="e",
                         fill=VERDE, font=fuente(8, True))
        # Apogeo previsto por el guiado, para comparar de un vistazo
        p = e.paquete
        if p is not None and p.apogeo_m > 0 and hi > lo:
            f = (p.apogeo_m - lo) / (hi - lo)
            if 0.0 <= f <= 1.0:
                y = y1 - f * (y1 - y0)
                self.create_line(x0, y, x1, y, fill=AMBAR, dash=(3, 3))
                self.create_text(x1 - 4, y - 6, text="APOGEO PREVISTO",
                                 anchor="e", fill=AMBAR, font=fuente(7))


class PanelVelocidad(Panel):
    """
    Velocidad vertical reciente, en barras. Con signo y centrada: el cohete se
    pasa medio vuelo bajando y eso tiene que verse igual de claro que la
    subida.
    """

    def pintar(self):
        e = self.estado
        x0, y0, x1, y1 = self.area()
        cy = (y0 + y1) / 2.0
        self.create_line(x0, cy, x1, cy, fill=VERDE_OSC)

        if not e or not e.hist_vv:
            return
        datos = list(e.hist_vv)
        fondo = max(20.0, max(abs(v) for v in datos) * 1.15)
        n = len(datos)
        ancho = (x1 - x0) / float(max(n, 1))
        for i, v in enumerate(datos):
            a = x0 + i * ancho
            alto = (v / fondo) * (cy - y0 - 2)
            col = VERDE if v >= 0 else MAGENTA
            self.create_rectangle(a, cy - alto, a + max(1.0, ancho - 1.5), cy,
                                  fill=col, outline="")
        self.create_text(x0 + 3, y0 + 6, text="+%.0f" % fondo, anchor="w",
                         fill=VERDE_OSC, font=fuente(7))
        self.create_text(x0 + 3, y1 - 6, text="-%.0f" % fondo, anchor="w",
                         fill=VERDE_OSC, font=fuente(7))
        self.create_text(x1 - 4, y0 + 7, text="%+.2f m/s" % datos[-1],
                         anchor="e", fill=VERDE, font=fuente(9, True))


# --- Fases ------------------------------------------------------------------


class PanelFases(Panel):
    """Las ocho fases de vuelo en fila. La activa se enciende."""

    def pintar(self):
        e = self.estado
        x0, y0, x1, y1 = self.area()
        p = e.paquete if e else None
        activa = p.fase if p is not None else -1
        cy = (y0 + y1) / 2.0 - 4
        n = len(pq.FASES)
        paso = (x1 - x0) / float(n)

        for i in range(n):
            cx = x0 + paso * (i + 0.5)
            nombre = pq.FASES[i]
            circulo_fase(self, cx, cy, 13, nombre[0], i == activa)
            self.create_text(cx, y1 - 5, text=nombre, fill=(
                VERDE if i == activa else VERDE_OSC), font=fuente(7, i == activa))
            if i < n - 1:
                self.create_line(cx + 14, cy, cx + paso - 14, cy,
                                 fill=VERDE_OSC if i >= activa else VERDE_MED)


# --- Actitud ----------------------------------------------------------------


class PanelActitud(Panel):
    """
    Esfera de actitud. El vector brillante es el morro; la retícula es el
    marco NED, con el horizonte marcado.

    Se enseña la ELEVACION del morro y no el cabeceo de Euler porque la
    elevación no tiene bloqueo de cardán, y el cohete se pasa toda la cuenta
    atrás justo en el punto donde Euler se rompe.
    """

    AZIMUT_CAMARA = math.radians(32.0)
    ELEV_CAMARA = math.radians(20.0)

    def _proyectar(self, n, e, d):
        X, Y, Z = e, n, -d
        ca, sa = math.cos(self.AZIMUT_CAMARA), math.sin(self.AZIMUT_CAMARA)
        x2 = X * ca + Y * sa
        y2 = -X * sa + Y * ca
        ce, se = math.cos(self.ELEV_CAMARA), math.sin(self.ELEV_CAMARA)
        sx = x2
        sy = -(y2 * se + Z * ce)
        prof = y2 * ce - Z * se  # < 0 = hacia el observador
        return sx, sy, prof

    def _curva(self, puntos, cx, cy, r, color_frente, color_fondo, ancho=1):
        tramo = []
        delante_previo = None
        for (n, e, d) in puntos:
            sx, sy, prof = self._proyectar(n, e, d)
            delante = prof < 0
            px, py = cx + sx * r, cy + sy * r
            if delante_previo is not None and delante != delante_previo:
                if len(tramo) >= 4:
                    self.create_line(
                        tramo, fill=color_frente if delante_previo
                        else color_fondo, width=ancho)
                tramo = tramo[-2:] if len(tramo) >= 2 else []
            tramo.extend((px, py))
            delante_previo = delante
        if len(tramo) >= 4:
            self.create_line(tramo, fill=color_frente if delante_previo
                             else color_fondo, width=ancho)

    def pintar(self):
        e = self.estado
        x0, y0, x1, y1 = self.area()
        cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0 - 4
        r = min(x1 - x0, y1 - y0) / 2.0 - 16
        if r < 10:
            return

        self.create_oval(cx - r, cy - r, cx + r, cy + r, outline=REJILLA)

        paso = math.radians(6)
        # Paralelos
        for lat_g in (-60, -30, 0, 30, 60):
            lat = math.radians(lat_g)
            pts = []
            t = 0.0
            while t <= 2 * math.pi + paso:
                pts.append((math.cos(lat) * math.cos(t),
                            math.cos(lat) * math.sin(t),
                            -math.sin(lat)))
                t += paso
            horizonte = (lat_g == 0)
            self._curva(pts, cx, cy, r,
                        VERDE_MED if horizonte else VERDE_OSC,
                        VERDE_OSC if horizonte else REJILLA,
                        2 if horizonte else 1)
        # Meridianos
        for lon_g in range(0, 180, 30):
            lon = math.radians(lon_g)
            pts = []
            t = 0.0
            while t <= 2 * math.pi + paso:
                pts.append((math.cos(t) * math.cos(lon),
                            math.cos(t) * math.sin(lon),
                            -math.sin(t)))
                t += paso
            self._curva(pts, cx, cy, r, VERDE_OSC, REJILLA)

        # Rosa de los vientos sobre el horizonte
        for etq, (n, ee) in (("N", (1, 0)), ("E", (0, 1)),
                             ("S", (-1, 0)), ("O", (0, -1))):
            sx, sy, prof = self._proyectar(n, ee, 0)
            self.create_text(cx + sx * (r + 9), cy + sy * (r + 9), text=etq,
                             fill=VERDE_MED if prof < 0 else VERDE_OSC,
                             font=fuente(8, True))

        p = e.paquete if e else None
        if p is None:
            self.create_text(cx, y1 - 6, text="SIN ACTITUD", fill=VERDE_OSC,
                             font=fuente(8))
            return

        # Vector morro
        n, ee, d = p.morro_ned
        sx, sy, prof = self._proyectar(n, ee, d)
        px, py = cx + sx * r, cy + sy * r
        col = VERDE if p.nav_valida else AMBAR
        self.create_line(cx, cy, px, py, fill=col, width=2,
                         arrow="last", arrowshape=(9, 11, 4))
        self.create_oval(px - 3, py - 3, px + 3, py + 3, fill=col,
                         outline=FONDO)

        self.create_text(x0, y1 - 14, anchor="w",
                         text="ELEVACION %+6.1f" % p.elevacion_deg,
                         fill=col, font=fuente(9, True))
        self.create_text(x0, y1 - 3, anchor="w",
                         text="RUMBO     %6.1f" % p.rumbo_deg,
                         fill=VERDE_MED, font=fuente(9))
        al, ca_, gu = p.euler_deg
        self.create_text(x1, y1 - 14, anchor="e",
                         text="ALABEO %+7.1f" % al, fill=VERDE_MED,
                         font=fuente(8))
        self.create_text(x1, y1 - 3, anchor="e",
                         text="|q| %.3f" % p.norma_quat,
                         fill=VERDE_MED if abs(p.norma_quat - 1) < 0.02
                         else ROJO, font=fuente(8))


# --- Números grandes --------------------------------------------------------


class PanelNumeros(Panel):
    """Las cifras que se cantan en voz alta durante el vuelo."""

    def pintar(self):
        e = self.estado
        x0, y0, x1, y1 = self.area()
        p = e.paquete if e else None

        filas = (
            ("ALTURA", _num(p and p.altura_m, "%.1f"), "m", VERDE, 22),
            ("VEL VERTICAL", _num(p and p.vel_vertical_mps, "%+.2f"), "m/s",
             VERDE, 22),
            ("APOGEO PREVISTO", _num(p and p.apogeo_m, "%.0f"), "m", AMBAR, 16),
            ("AEROFRENO", _num(p and p.aerofreno_pct, "%.0f"), "%", CIAN, 16),
        )
        alto = (y1 - y0) / float(len(filas))
        for i, (etq, val, uni, col, tam) in enumerate(filas):
            cy = y0 + alto * i
            self.create_text(x0 + 2, cy + 9, text=etq, anchor="w",
                             fill=VERDE_OSC, font=fuente(7))
            self.create_text(x1 - 26, cy + alto * 0.62, text=val, anchor="e",
                             fill=col if p is not None else VERDE_OSC,
                             font=fuente(tam, True))
            self.create_text(x1 - 2, cy + alto * 0.62, text=uni, anchor="e",
                             fill=VERDE_MED, font=fuente(9))
            if i:
                self.create_line(x0, cy, x1, cy, fill=REJILLA)

        if p is not None and p.aerofreno_pct > 0:
            barra_segmentada(self, x0 + 2, y1 - 5, x1 - 2, y1 - 1,
                             p.aerofreno_pct / 100.0, color=CIAN)


# --- Aceleración ------------------------------------------------------------


class PanelAceleracion(Panel):
    """Aceleración en ejes de cuerpo. X es el morro."""

    FONDO_G = 16.0

    def pintar(self):
        e = self.estado
        x0, y0, x1, y1 = self.area()
        p = e.paquete if e else None
        ejes = ("X MORRO", "Y DERECHA", "Z ABAJO")
        # Alto fijo y el grupo centrado: repartir toda la altura del panel
        # entre tres barras las deja nadando y cuesta compararlas.
        alto = 30.0
        y_ini = y0 + max(0.0, ((y1 - y0 - 14) - alto * 3) / 2.0)

        for i, etq in enumerate(ejes):
            cy = y_ini + alto * i + alto / 2.0
            v = p.accel_g[i] if p is not None else 0.0
            self.create_text(x0 + 2, cy - 9, text=etq, anchor="w",
                             fill=VERDE_OSC, font=fuente(7))
            self.create_text(x1 - 2, cy - 9, text=_num(p and p.accel_g[i],
                                                       "%+.2f g"), anchor="e",
                             fill=VERDE, font=fuente(9, True))
            barra_con_centro(self, x0 + 2, cy + 1, x1 - 2, cy + 9, v,
                             self.FONDO_G, color=AMBAR)

        mod = p.accel_modulo_g if p is not None else None
        self.create_text((x0 + x1) / 2, y1 - 6,
                         text="MODULO %s g" % _num(mod, "%.2f"),
                         fill=VERDE_MED, font=fuente(8, True))


# --- Mapa -------------------------------------------------------------------


class PanelMapa(Panel):
    """Recorrido en planta (norte/este). El origen es donde arrancó el EKF."""

    def pintar(self):
        e = self.estado
        x0, y0, x1, y1 = self.area()
        cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
        r = min(x1 - x0, y1 - y0) / 2.0 - 10

        pista = list(e.pista) if e else []
        alcance = 50.0
        if pista:
            alcance = max(50.0, max(max(abs(a), abs(b)) for a, b in pista) * 1.2)

        for k in (0.33, 0.66, 1.0):
            self.create_oval(cx - r * k, cy - r * k, cx + r * k, cy + r * k,
                             outline=REJILLA)
        self.create_line(cx - r, cy, cx + r, cy, fill=REJILLA)
        self.create_line(cx, cy - r, cx, cy + r, fill=REJILLA)
        self.create_text(cx, cy - r - 6, text="N", fill=VERDE_MED,
                         font=fuente(8, True))
        self.create_text(x1 - 2, y1 - 5, text="%.0f m" % alcance, anchor="e",
                         fill=VERDE_OSC, font=fuente(7))

        if len(pista) >= 2:
            pts = []
            for n, ee in pista:
                pts.append(cx + (ee / alcance) * r)
                pts.append(cy - (n / alcance) * r)
            self.create_line(pts, fill=VERDE_MED)
            self.create_oval(pts[-2] - 3, pts[-1] - 3, pts[-2] + 3,
                             pts[-1] + 3, fill=VERDE, outline=FONDO)

        p = e.paquete if e else None
        if p is not None:
            self.create_text(x0 + 2, y0 + 6, anchor="w",
                             text="N %+.0f  E %+.0f m" % (p.norte_m, p.este_m),
                             fill=VERDE_MED, font=fuente(8))


# --- Estado del sistema -----------------------------------------------------


class PanelSistema(Panel):
    """
    Lámparas de salud. Arriba lo que trae el paquete de vuelo (15 Hz); abajo
    lo que trae el paquete de estado, que llega al arrancar y cuando se pide.

    Las lámparas de sensor son de TRES estados y no de dos, porque un sensor
    apagado puede ser que falló o que esa placa no lo monta, y confundir las
    dos cosas manda a buscar una avería que no existe.
    """

    def _lampara_sensor(self, x, y, nombre, estado_paq):
        if estado_paq is None:
            valor = None
        else:
            valor = estado_paq.sensor(nombre)

        if valor == "ok":
            col, encendida = VERDE, True
        elif valor == "fallo":
            col, encendida = ROJO, True
        elif valor == "no lleva":
            col, encendida = VERDE_OSC, False
        else:
            col, encendida = VERDE_OSC, False
        lampara(self, x, y, encendida, nombre, color=col,
                color_texto=col if valor in ("ok", "fallo") else VERDE_OSC)

    def pintar(self):
        e = self.estado
        x0, y0, x1, y1 = self.area()
        p = e.paquete if e else None
        s = e.estado_paquete if e else None

        y = y0 + 8
        self.create_text(x0, y, text="ESTADO", anchor="w", fill=VERDE_OSC,
                         font=fuente(7))
        y += 14
        emer = p is not None and p.estado == pq.ESTADO_EMERGENCIA
        self.create_text(x0, y, anchor="w",
                         text=p.estado_nombre if p else "--",
                         fill=ROJO if emer else VERDE if p else VERDE_OSC,
                         font=fuente(11, True))

        col_nav = VERDE if (p and p.nav_valida) else ROJO
        lampara(self, x0 + 150, y, bool(p and p.nav_valida), "NAV VALIDA",
                color=col_nav)
        y += 17

        if emer or (s is not None and s.version >= 6 and s.emergencia):
            # Reiniciado en vuelo: decide el barometro (App/emergencia.h)
            sv6 = s if (s is not None and s.version >= 6) else None
            texto = "SOLO BARO  apogeo %s  main %s  baro %s" % (
                "SI" if sv6 and sv6.emer_apogeo else "no",
                "SI" if sv6 and sv6.emer_main else "no",
                "NO FIABLE" if sv6 and sv6.baro_no_fiable else "fiable")
            self.create_rectangle(x0, y - 8, x1, y + 9, outline=ROJO)
            self.create_text((x0 + x1) / 2, y, text=texto, fill=ROJO,
                             font=fuente(8, True))
        elif s is not None and s.version >= 6 and s.postvuelo:
            self.create_text(x0, y, anchor="w", fill=CIAN, font=fuente(8),
                             text="POSTVUELO: no toca SD ni camaras")
        elif p is not None and p.error:
            self.create_rectangle(x0, y - 8, x1, y + 9, outline=ROJO)
            self.create_text((x0 + x1) / 2, y, text=p.error_nombre, fill=ROJO,
                             font=fuente(8, True))
        else:
            self.create_text(x0, y, anchor="w", text="sin errores",
                             fill=VERDE_OSC, font=fuente(8))
        y += 18

        self.create_text(x0, y, text="PIROTECNIA (continuidad)", anchor="w",
                         fill=VERDE_OSC, font=fuente(7))
        y += 14
        for i, etq in enumerate(pq.PIROS):
            lampara(self, x0 + 6 + i * 52, y, bool(p and p.pirotecnia[i]), etq,
                    color=AMBAR)
        y += 20

        self.create_line(x0, y, x1, y, fill=REJILLA)
        y += 12

        # --- A partir de aquí, paquete de estado ---
        self.create_text(x0, y, text="PLACAS", anchor="w", fill=VERDE_OSC,
                         font=fuente(7))
        if s is None:
            self.create_text(x1, y, text="sin paquete de estado", anchor="e",
                             fill=AMBAR, font=fuente(7))
        y += 14
        for i, etq in enumerate(pq.PLACAS):
            viva = bool(s and (s.placas & (1 << i)))
            lampara(self, x0 + 6 + i * 78, y, viva, etq.upper(),
                    color=VERDE if viva else ROJO if s else VERDE_OSC)
        y += 20

        # Camaras: lo que dice el cohete (paquete de estado v5 o acuse) y,
        # con el paquete de camaras, si graban de verdad
        cam = e.camaras_cohete if e else None
        pc = e.camaras_paquete if e else None
        if cam is None:
            texto, col = "CAMARAS  --", VERDE_OSC
        elif cam and e.camaras_apagando:
            texto, col = "CAMARAS CERRANDO VIDEOS", AMBAR
        elif cam and pc is not None and pc.alimentacion:
            n = pc.grabando
            mal = any(c.mal for c in pc.camaras)
            texto = "CAMARAS ON  graban %d de 2" % n
            col = ROJO if any(c.no_graba for c in pc.camaras) else \
                VERDE if (n > 0 and not mal) else AMBAR
        elif cam:
            texto, col = "CAMARAS ENCENDIDAS", VERDE
        else:
            texto, col = "CAMARAS APAGADAS", AMBAR
        if e is not None and e.camaras_pedidas is not None:
            texto += "  (pidiendo %s...)" % ("ON" if e.camaras_pedidas
                                            else "OFF")
        lampara(self, x0 + 6, y, bool(cam), texto, color=col, color_texto=col)
        y += 20

        self.create_text(x0, y, text="SENSORES DE AVIONICA", anchor="w",
                         fill=VERDE_OSC, font=fuente(7))
        y += 14
        por_fila = 4
        ancho = (x1 - x0) / por_fila
        for i, nombre in enumerate(pq.SENSORES):
            fila, col = divmod(i, por_fila)
            self._lampara_sensor(x0 + 6 + col * ancho, y + fila * 16, nombre, s)
        y += 2 * 16 + 6

        if s is not None:
            self.create_text(x0, y, anchor="w", fill=VERDE_OSC,
                             font=fuente(7),
                             text="calibracion %s    v%d  #%d"
                                  % ("hecha" if s.calibracion_hecha
                                     else "sin hacer", s.version, s.contador))


# --- Salud: tarjetas y buses ------------------------------------------------


class PanelSalud(Panel):
    """
    Tarjetas, buses y tiempo encendido. Todo del paquete de estado.

    Arriba del todo va lo más importante de este panel: si el paquete de
    estado ha llegado o no. Un panel lleno de ceros porque nunca llegó nada
    se parece demasiado a un panel lleno de ceros porque todo está a cero.
    """

    def pintar(self):
        e = self.estado
        x0, y0, x1, y1 = self.area()
        s = e.estado_paquete if e else None

        y = y0 + 8
        if s is None:
            self.create_rectangle(x0, y - 8, x1, y + 22, outline=AMBAR)
            self.create_text((x0 + x1) / 2, y + 1, text="SIN PAQUETE DE ESTADO",
                             fill=AMBAR, font=fuente(9, True))
            if e is not None:
                self.create_text((x0 + x1) / 2, y + 13, fill=VERDE_MED,
                                 font=fuente(7),
                                 text="pidiendolo cada %.0f s   %d enviadas"
                                      % (e.periodo_peticion_s,
                                         e.peticiones_estado))
            y += 34
        else:
            edad = e.edad_estado_s
            self.create_text(x0, y, anchor="w", fill=VERDE_MED,
                             font=fuente(7),
                             text="recibido hace %s" % _edad(edad))
            self.create_text(x1, y, anchor="e",
                             text="ENCENDIDO %s" % _hhmmss(s.segundos_encendido),
                             fill=VERDE_MED, font=fuente(7))
            y += 16

        self.create_text(x0, y, text="TARJETAS", anchor="w", fill=VERDE_OSC,
                         font=fuente(7))
        y += 13
        for i in range(2):
            self.create_text(x0 + 2, y, anchor="w", text="SD%d" % (i + 1),
                             fill=VERDE_MED, font=fuente(8, True))
            if s is None:
                self.create_text(x0 + 34, y, anchor="w", text="--",
                                 fill=VERDE_OSC, font=fuente(8))
            else:
                montada = s.sd_estado[i] == 3
                self.create_text(x0 + 34, y, anchor="w", text=s.sd_nombre(i),
                                 fill=VERDE if montada else ROJO,
                                 font=fuente(8))
                self.create_text(x1 - 2, y, anchor="e", fill=VERDE_MED,
                                 font=fuente(8),
                                 text="%s libres   %d arranques   test %s"
                                      % (_mb(s.sd_libres_mb[i]),
                                         s.sd_arranques[i],
                                         "OK" if s.sd_test_ok[i] else "NO"))
            y += 15

        y += 4
        y = self._camaras(e, x0, x1, y)

        y += 4
        self.create_text(x0, y, text="BUSES", anchor="w", fill=VERDE_OSC,
                         font=fuente(7))
        if s is not None:
            self.create_text(x1 - 2, y, anchor="e", fill=VERDE_OSC,
                             font=fuente(7),
                             text="wake ups reenviados %d    radio sin "
                                  "encolar %d" % (s.wake_up_reintentos,
                                                  s.radio_fallidos))
        y += 13
        for i in range(2):
            self.create_text(x0 + 2, y, anchor="w", text="CAN%d" % (i + 1),
                             fill=VERDE_MED, font=fuente(8, True))
            if s is None:
                self.create_text(x0 + 44, y, anchor="w", text="--",
                                 fill=VERDE_OSC, font=fuente(8))
            else:
                hz = s.can_hz[i]
                err = s.can_errores[i]
                self.create_text(x0 + 44, y, anchor="w",
                                 text="%5d tramas/s" % hz,
                                 fill=VERDE if hz > 0 else AMBAR,
                                 font=fuente(8))
                self.create_text(x1 - 2, y, anchor="e",
                                 text="%d errores" % err,
                                 fill=VERDE_MED if err == 0 else ROJO,
                                 font=fuente(8))
            y += 15


    def _camaras(self, e, x0, x1, y):
        """Una linea por camara: estado (GRABANDO, NO GRABA...) y a la
        derecha con que graba y cuanta tarjeta le queda."""
        pc = e.camaras_paquete if e else None
        titulo = "CAMARAS"
        if pc is not None:
            titulo += "   recibido hace %s" % _edad(
                time.monotonic() - e.t_camaras_paquete)
            if not pc.hora_gps:
                titulo += "   sin hora GPS"
        self.create_text(x0, y, text=titulo, anchor="w", fill=VERDE_OSC,
                         font=fuente(7))
        y += 13
        for i in range(2):
            self.create_text(x0 + 2, y, anchor="w", text="CAM%d" % (i + 1),
                             fill=VERDE_MED, font=fuente(8, True))
            if pc is None:
                self.create_text(x0 + 40, y, anchor="w", text="--",
                                 fill=VERDE_OSC, font=fuente(8))
                y += 15
                continue
            c = pc.camaras[i]
            texto = c.estado_nombre
            if c.grabando and c.grabando_s:
                texto += " %d:%02d" % (c.grabando_s // 60, c.grabando_s % 60)
            if c.no_graba or c.sin_tarjeta:
                col = ROJO
            elif c.estado == pq.CAMARA_GRABANDO:
                col = VERDE
            elif c.estado == pq.CAMARA_SIN_CORRIENTE:
                col = VERDE_OSC
            else:
                col = AMBAR
            self.create_text(x0 + 40, y, anchor="w", text=texto, fill=col,
                             font=fuente(8, True))
            if c.sd_leida or c.ajustes_leidos:
                tarjeta = "sin tarjeta" if c.sin_tarjeta else \
                    "%s/%s  %d min" % (_mb(c.libre_mb), _mb(c.total_mb),
                                       c.queda_s // 60)
                self.create_text(x1 - 2, y, anchor="e", fill=VERDE_MED,
                                 font=fuente(8),
                                 text="%s  %s%s" % (
                                     c.resolucion or "", tarjeta,
                                     "  reloj" if c.reloj_puesto else ""))
            y += 15
        return y


# --- Enlace -----------------------------------------------------------------


class PanelEnlace(Panel):
    """
    Calidad de radio. Los tres números que deciden si el enlace aguanta, más
    el reparto del ciclo de trabajo, que es un límite legal y no una opinión.
    """

    ALTO_TRAZA = 44  # alto de cada chispograma en píxeles

    def pintar(self):
        e = self.estado
        x0, y0, x1, y1 = self.area()
        t = e.trama if e else None

        campos = (
            ("RSSI", _num(t and t.rssi_dbm, "%+d" if t is None else "%.0f"),
             "dBm"),
            ("SNR", _num(t and t.snr_db, "%+.2f"), "dB"),
            ("DESV F", _num(t and t.df_hz, "%+.0f"), "Hz"),
        )
        ancho = (x1 - x0) / 3.0
        for i, (etq, val, uni) in enumerate(campos):
            cx = x0 + ancho * (i + 0.5)
            self.create_text(cx, y0 + 7, text=etq, fill=VERDE_OSC,
                             font=fuente(7))
            self.create_text(cx, y0 + 22, text=val,
                             fill=VERDE if t else VERDE_OSC,
                             font=fuente(14, True))
            self.create_text(cx, y0 + 34, text=uni, fill=VERDE_MED,
                             font=fuente(7))

        # Chispogramas: la tendencia importa más que el valor puntual
        yb = y0 + 44
        for datos, col, etq in ((e.hist_rssi if e else [], VERDE, "RSSI"),
                                (e.hist_snr if e else [], CIAN, "SNR")):
            if datos:
                traza(self, x0 + 30, yb, x1 - 2, yb + self.ALTO_TRAZA,
                      list(datos), color=col)
            self.create_rectangle(x0 + 30, yb, x1 - 2, yb + self.ALTO_TRAZA,
                                  outline=REJILLA)
            self.create_text(x0 + 2, yb + self.ALTO_TRAZA / 2, text=etq,
                             anchor="w", fill=VERDE_OSC, font=fuente(7))
            yb += self.ALTO_TRAZA + 5

        # Ciclo de trabajo sobre la ventana de una hora
        if e is not None:
            yb += 4
            self.create_text(x0 + 2, yb, anchor="w",
                             text="CICLO DE TRABAJO (1 h)", fill=VERDE_OSC,
                             font=fuente(7))
            yb += 11
            frac = e.duty_fraccion
            col = VERDE if frac < 0.8 else (AMBAR if frac < 1.0 else ROJO)
            barra_segmentada(self, x0 + 2, yb, x1 - 56, yb + 8,
                             min(1.0, frac), color=col)
            self.create_text(x1 - 2, yb + 4, anchor="e",
                             text="%.0f%% del 10%%" % (frac * 100.0),
                             fill=col, font=fuente(8, True))

            yb += 18
            self.create_text(x0 + 2, yb, anchor="w", fill=VERDE_OSC,
                             font=fuente(7),
                             text="aire %d  cable %d  acuses %d  cortes %d"
                                  % (e.crc_malos, e.serie_crc_malos,
                                     e.acuses, e.cortes))


# --- Crudo ------------------------------------------------------------------


class PanelCrudo(Panel):
    """
    Los 24 bytes tal cual llegaron. Cuando algo no cuadra, esto es lo único
    que no miente: todo lo demás es interpretación de esta estación.
    """

    def pintar(self):
        e = self.estado
        x0, y0, x1, y1 = self.area()
        p = e.paquete if e else None

        if p is None:
            self.create_text((x0 + x1) / 2, (y0 + y1) / 2, text="SIN PAQUETE",
                             fill=VERDE_OSC, font=fuente(8))
            return

        b = p.crudo
        y = y0 + 8
        for fila in range(3):
            trozo = b[fila * 8:(fila + 1) * 8]
            self.create_text(x0, y, anchor="w", text="%02X" % (fila * 8),
                             fill=VERDE_OSC, font=fuente(8))
            self.create_text(x0 + 22, y, anchor="w",
                             text=" ".join("%02X" % c for c in trozo),
                             fill=VERDE, font=fuente(9))
            y += 15

        self.create_text(x0, y + 4, anchor="w",
                         text="CONTADOR %3d   TEMP BARO %+d C"
                              % (p.contador, p.temperatura_c),
                         fill=VERDE_MED, font=fuente(8))
        self.create_text(x0, y + 17, anchor="w",
                         text="BATERIA  %.2f V" % p.bateria_v,
                         fill=VERDE_MED, font=fuente(8))

        # Como oye el COHETE a tierra (lo mide su radio al recibir; llega en
        # los acuses, por ejemplo el del PING)
        oye = e.cohete_oye
        if oye is None:
            self.create_text(x0, y + 32, anchor="w",
                             text="COHETE OYE  -- (manda un PING)",
                             fill=VERDE_OSC, font=fuente(8))
        else:
            rssi, snr, df, t, cmd, valor = oye
            de = ("PING #%d" % valor if cmd == pq.UPLINK_CMD_PING
                  else pq.UPLINK_NOMBRES.get(cmd, "0x%02X" % cmd))
            self.create_text(x0, y + 32, anchor="w",
                             text="COHETE OYE  %s, hace %.0f s"
                                  % (de, time.monotonic() - t),
                             fill=VERDE_MED, font=fuente(8))
            self.create_text(x0, y + 45, anchor="w",
                             text="RSSI %d dBm   SNR %d dB" % (rssi, snr),
                             fill=CIAN, font=fuente(9, True))
            self.create_text(x0, y + 59, anchor="w",
                             text="ERROR FREC. %+d Hz" % df,
                             fill=CIAN, font=fuente(9, True))


# --- Huecos -----------------------------------------------------------------


class PanelRanuras(Panel):
    """
    Fila de ranuras reservadas, al estilo de la consola de referencia. Cada
    una es sitio ya hecho para un indicador futuro: se rellenan dando de alta
    su nombre en RANURAS, sin tocar la distribución.
    """

    RANURAS = (
        "GPS", "SATS", "HDOP", "VEL GPS", "BARO 1", "BARO 2", "IMU 1",
        "IMU 2", "MAG", "SD 1", "SD 2", "CAN 1", "CAN 2", "3V3", "5V",
        "11V", "CORR", "SERVO", "PIRO", "T EXT", "MACH", "CD", "VIENTO",
        "RESERVA",
    )

    def pintar(self):
        x0, y0, x1, y1 = self.area()
        n = len(self.RANURAS)
        ancho = (x1 - x0) / float(n)
        for i, etq in enumerate(self.RANURAS):
            a = x0 + ancho * i
            self.create_rectangle(a + 1, y0 + 10, a + ancho - 2, y1 - 11,
                                  outline=REJILLA)
            self.create_text(a + ancho / 2, y0 + 5, text="V%02d" % (i + 1),
                             fill=VERDE_OSC, font=fuente(6))
            self.create_text(a + ancho / 2, (y0 + y1) / 2, text="--",
                             fill=APAGADO, font=fuente(9, True))
            self.create_text(a + ancho / 2, y1 - 5, text=etq, fill=VERDE_OSC,
                             font=fuente(6))
