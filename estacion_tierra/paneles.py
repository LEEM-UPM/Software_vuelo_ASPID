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
import os
import struct
import time

from widgets import (AMBAR, APAGADO, AZUL, CIAN, FONDO, MAGENTA, REJILLA, ROJO,
                     VIOLETA,
                     VERDE, VERDE_MED, VERDE_OSC, Panel, atenuar,
                     barra_con_centro,
                     barra_segmentada, circulo_fase, fuente, lampara, rejilla,
                     traza)

import cats as ct
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

        # Logo: una franja con los colores de las cuatro familias de paneles
        for k, col in enumerate((VERDE, AZUL, CIAN, VIOLETA)):
            self.create_rectangle(x0 + 2 + k * 4, cy - 12, x0 + 4 + k * 4,
                                  cy + 12, fill=col, outline="")
        self.create_text(x0 + 22, cy - 7, text="L E E M", anchor="w",
                         fill=VERDE, font=fuente(13, True))
        self.create_text(x0 + 22, cy + 8, text="ESTACION DE TIERRA",
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

        # La mascota, en el hueco entre los campos y el reloj
        _serpiente(self, x + 10, x1 - 110, cy, y1 - y0)


# --- La serpiente del parche de la mision (solo decoracion) -----------------

# Los colores del parche ASPID / LEEM '26
SERP_CIAN = "#3fc8d9"
SERP_AMARILLO = "#f5cf3a"
SERP_ROSA = "#e3549e"
SERP_CUERPO = "#2a1d3d"
SERP_CREMA = "#f1e3b4"


def _estrella(c, x, y, r, color):
    """Estrella de cuatro puntas, como las del parche."""
    q = r * 0.3
    c.create_polygon(x, y - r, x + q, y - q, x + r, y, x + q, y + q,
                     x, y + r, x - q, y + q, x - r, y, x - q, y - q,
                     fill=color, outline="")


def _serpiente(c, xa, xb, cy, alto):
    """
    Una serpiente de lineas que repta despacio, con los colores del parche.
    Se adapta al hueco que haya; si es muy estrecho, no se pinta. No lleva
    ningun dato: es decoracion.
    """
    if xb - xa < 170:
        return
    # Rotulo a la izquierda
    c.create_text(xa, cy - 7, text="ASPID", anchor="w", fill=SERP_CIAN,
                  font=fuente(11, True))
    c.create_text(xa, cy + 9, text="LEEM '26", anchor="w",
                  fill=SERP_AMARILLO, font=fuente(8, True))
    xa += 78

    t = time.monotonic()
    largo = xb - xa - 16                      # sitio para la cabeza
    amp = alto * 0.22
    n = 44
    fase = t * 1.6
    pts = []
    for i in range(n + 1):
        f = i / float(n)
        # Mas onda en la cola que junto a la cabeza
        env = 0.35 + 0.65 * (1.0 - f) ** 0.7
        pts.append((xa + f * largo,
                    cy + amp * env * math.sin(2 * math.pi * 2.2 * f - fase)))

    def grosor(i):
        return 2.0 + 8.0 * (i / float(n)) ** 0.8   # de la cola a la cabeza

    # Tres pasadas para que el contorno quede siempre por debajo
    for capa in range(3):
        for i in range(n):
            (ax, ay), (bx, by) = pts[i], pts[i + 1]
            w = grosor(i)
            if capa == 0:
                c.create_line(ax, ay, bx, by, fill=SERP_CIAN, width=w + 3,
                              capstyle="round")
            elif capa == 1:
                c.create_line(ax, ay, bx, by, fill=SERP_CUERPO, width=w,
                              capstyle="round")
            elif w > 4:
                c.create_line(ax, ay, bx, by, fill=SERP_ROSA,
                              width=max(1.0, w * 0.3), capstyle="round")
    # Anillos crema
    for i in range(6, n, 5):
        (ax, ay), (bx, by) = pts[i], pts[i + 1]
        dx, dy = bx - ax, by - ay
        d = math.hypot(dx, dy) or 1.0
        nx, ny = -dy / d, dx / d
        h = grosor(i) / 2.0
        c.create_line(ax - nx * h, ay - ny * h, ax + nx * h, ay + ny * h,
                      fill=SERP_CREMA, width=1)

    # Cabeza, mirando hacia donde va el cuerpo
    (ax, ay), (hx, hy) = pts[-2], pts[-1]
    dx, dy = hx - ax, hy - ay
    d = math.hypot(dx, dy) or 1.0
    ux, uy = dx / d, dy / d
    vx, vy = -uy, ux
    L, W = 16.0, 8.5
    cabeza = [hx - ux * 2 + vx * W, hy - uy * 2 + vy * W,
              hx + ux * L * 0.7 + vx * W * 0.8, hy + uy * L * 0.7 + vy * W * 0.8,
              hx + ux * L, hy + uy * L,
              hx + ux * L * 0.7 - vx * W * 0.8, hy + uy * L * 0.7 - vy * W * 0.8,
              hx - ux * 2 - vx * W, hy - uy * 2 - vy * W]
    c.create_polygon(cabeza, fill=SERP_CUERPO, outline=SERP_CIAN, width=2,
                     smooth=True)
    ex, ey = hx + ux * L * 0.45 - vx * W * 0.35, hy + uy * L * 0.45 - vy * W * 0.35
    c.create_oval(ex - 2.2, ey - 2.2, ex + 2.2, ey + 2.2, fill=SERP_AMARILLO,
                  outline="")
    c.create_oval(ex - 0.8, ey - 1.6, ex + 0.8, ey + 1.6, fill=FONDO,
                  outline="")
    # Lengua, a ratos
    if (t % 1.6) < 0.5:
        tx, ty = hx + ux * (L + 1), hy + uy * (L + 1)
        fx, fy = tx + ux * 7, ty + uy * 7
        c.create_line(tx, ty, fx, fy, fill=SERP_ROSA, width=1.5)
        for sg in (1, -1):
            c.create_line(fx, fy, fx + ux * 3 + vx * 2.5 * sg,
                          fy + uy * 3 + vy * 2.5 * sg, fill=SERP_ROSA,
                          width=1.5)

    # Estrellas del parche
    for fx, fy, r in ((0.18, -0.36, 4), (0.55, 0.38, 3), (0.86, -0.34, 3.5)):
        _estrella(c, xa + fx * largo, cy + fy * alto, r, SERP_AMARILLO)


# --- Columna de medidores ---------------------------------------------------


def _marca_cats(panel, e, x, y, anchor="center"):
    """Con el boton DATOS en CATS, que se vea en cada panel que lo pintado
    es de CATS y no nuestro."""
    if e is not None and e.ver_cats:
        panel.create_text(x, y, text="CATS", anchor=anchor, fill=MAGENTA,
                          font=fuente(8, True))


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
        # La batería del CATS, que es SUYA: siempre de CATS, elija lo que
        # elija el botón DATOS
        ("CATS", None, 0.0, 100.0, "%", MAGENTA),
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

            if e is not None and attr in e.SOLAPADOS:
                v = e.dato(attr)   # de CATS si se ha elegido CATS
                if e.ver_cats:
                    col = MAGENTA
            elif attr is not None and p is not None:
                v = getattr(p, attr)
            elif p is not None and etq == "ACC":
                v = p.accel_modulo_g
            elif etq == "SEN" and e and e.trama is not None:
                v = e.trama.rssi_dbm
            elif etq == "CATS" and e is not None:
                v = e.cats_bateria_pct   # promediada: ver ahí por qué
            else:
                v = None

            if (etq in ("%BAT", "CATS") and v is not None and
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
                             # Sin decimales en las escalas grandes: el RSSI
                             # (-130 a -20) con ".0" invadía la columna de al lado
                             text=_num(v, "%.0f" if max(abs(lo), abs(hi)) > 50
                                       else "%.1f"),
                             fill=col if v is not None else VERDE_OSC,
                             font=fuente(9, True))


# --- Trazas -----------------------------------------------------------------


class PanelPerfil(Panel):
    """Altura contra tiempo. La gráfica que de verdad cuenta el vuelo."""

    def pintar(self):
        e = self.estado
        x0, y0, x1, y1 = self.area()
        rejilla(self, x0, y0, x1, y1, nx=8, ny=4)
        _marca_cats(self, e, (x0 + x1) / 2, y0 + 7)
        hist = e.hist("altura") if e else None
        if not hist:
            self.create_text((x0 + x1) / 2, (y0 + y1) / 2, text="SIN DATOS",
                             fill=VERDE_OSC, font=fuente(9))
            return

        datos = list(hist)
        lo, hi = traza(self, x0 + 1, y0 + 1, x1 - 1, y1 - 1, datos,
                       color=MAGENTA if e.ver_cats else VERDE, relleno=True)
        self.create_text(x0 + 4, y0 + 7, text="%.0f m" % hi, anchor="w",
                         fill=VERDE_MED, font=fuente(7))
        self.create_text(x0 + 4, y1 - 7, text="%.0f m" % lo, anchor="w",
                         fill=VERDE_MED, font=fuente(7))
        self.create_text(x1 - 4, y0 + 7,
                         text="AHORA %.1f m" % datos[-1], anchor="e",
                         fill=VERDE, font=fuente(8, True))
        # Apogeo previsto por el guiado, para comparar de un vistazo (es
        # nuestro: viendo CATS no pinta)
        p = e.paquete
        if not e.ver_cats and p is not None and p.apogeo_m > 0 and hi > lo:
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

        _marca_cats(self, e, x0 + 46, y0 + 6, anchor="w")
        hist = e.hist("vv") if e else None
        if not hist:
            return
        datos = list(hist)
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
    """
    Las ocho fases de vuelo en fila. La activa se enciende. Con el botón
    DATOS en CATS, los ocho estados de CATS (cats.ESTADOS) en magenta: no
    son nuestras fases (no tiene control ni apogeo, sí calibración y READY),
    así que se enseñan los suyos y no la traducción de cats.FASE_PROPIA.
    """

    def pintar(self):
        e = self.estado
        x0, y0, x1, y1 = self.area()
        if e is not None and e.ver_cats:
            nombres = ct.ESTADOS
            activa = e.cats.estado if e.cats is not None else -1
            color = MAGENTA
        else:
            nombres = [pq.FASES[i] for i in range(len(pq.FASES))]
            activa = e.dato("fase") if e else None
            color = VERDE
        if activa is None:
            activa = -1
        _marca_cats(self, e, x1 - 4, y0 + 6, anchor="e")
        cy = (y0 + y1) / 2.0 - 4
        n = len(nombres)
        paso = (x1 - x0) / float(n)

        for i in range(n):
            cx = x0 + paso * (i + 0.5)
            nombre = nombres[i]
            circulo_fase(self, cx, cy, 13, nombre[0], i == activa, color)
            self.create_text(cx, y1 - 5, text=nombre, fill=(
                color if i == activa else VERDE_OSC), font=fuente(7, i == activa))
            if i < n - 1:
                self.create_line(cx + 14, cy, cx + paso - 14, cy,
                                 fill=VERDE_OSC if i >= activa else VERDE_MED)


# --- Actitud ----------------------------------------------------------------


# Modelo 3D del cohete para el panel de actitud: STL binario, eje Z del
# modelo a lo largo del cohete (aletas en Z = 0, morro en Z máximo).
MODELO_COHETE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             "modelos", "ASPID_lowpoly.stl")


def _cargar_stl(ruta):
    """
    Triángulos del STL en ejes de CUERPO (X morro, Y derecha, Z abajo),
    centrados en la mitad del cohete y escalados a media longitud 1.
    STL Z -> cuerpo X, STL X -> cuerpo Y, STL Y -> cuerpo Z (giro puro).
    None si no se puede leer: el panel vuelve a la flecha.
    """
    try:
        with open(ruta, "rb") as fh:
            datos = fh.read()
        n = struct.unpack_from("<I", datos, 80)[0]
        if len(datos) < 84 + 50 * n or n == 0:
            return None
        tris = []
        for i in range(n):
            v = struct.unpack_from("<12f", datos, 84 + 50 * i)
            tris.append((v[3:6], v[6:9], v[9:12]))
    except (OSError, struct.error):
        return None
    zs = [p[2] for t in tris for p in t]
    z_min, z_max = min(zs), max(zs)
    medio = (z_min + z_max) / 2.0
    escala = 2.0 / (z_max - z_min)
    return [tuple(((p[2] - medio) * escala, p[0] * escala, p[1] * escala)
                  for p in t) for t in tris]


class PanelActitud(Panel):
    """
    Actitud: el modelo 3D del cohete (MODELO_COHETE) girado con el
    cuaternión del paquete, visto desde una cámara fija (norte al fondo a la
    derecha, algo por encima del horizonte). Gira entero, así que también se
    ve el alabeo.

    Se enseña la ELEVACION del morro y no el cabeceo de Euler porque la
    elevación no tiene bloqueo de cardán, y el cohete se pasa toda la cuenta
    atrás justo en el punto donde Euler se rompe.
    """

    AZIMUT_CAMARA = math.radians(32.0)
    ELEV_CAMARA = math.radians(20.0)
    # Luz en ejes de cámara (x derecha, y abajo, prof < 0 hacia el observador)
    LUZ = (-0.45, -0.55, -0.70)

    _modelo = None
    _modelo_leido = False

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

    @classmethod
    def _cargar_modelo(cls):
        if not cls._modelo_leido:
            cls._modelo_leido = True
            cls._modelo = _cargar_stl(MODELO_COHETE)
        return cls._modelo

    def _pintar_modelo(self, modelo, quat, cx, cy, r, color):
        w, x, y, z = quat
        nq = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
        w, x, y, z = w / nq, x / nq, y / nq, z / nq
        # Cuerpo -> NED (la primera columna es morro_ned de paquete.py)
        m = ((1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
             (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
             (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)))
        lx, ly, lz = self.LUZ
        nl = math.sqrt(lx * lx + ly * ly + lz * lz)
        caras = []
        for tri in modelo:
            q = []
            for (bx, by, bz) in tri:
                q.append(self._proyectar(
                    m[0][0] * bx + m[0][1] * by + m[0][2] * bz,
                    m[1][0] * bx + m[1][1] * by + m[1][2] * bz,
                    m[2][0] * bx + m[2][1] * by + m[2][2] * bz))
            (ax, ay, az), (bx, by, bz), (qx, qy, qz) = q
            ux, uy, uz = bx - ax, by - ay, bz - az
            vx, vy, vz = qx - ax, qy - ay, qz - az
            nx, ny, nz = uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx
            nn = math.sqrt(nx * nx + ny * ny + nz * nz)
            if nn == 0.0:
                continue
            luz = abs(nx * lx + ny * ly + nz * lz) / (nn * nl)
            caras.append((az + bz + qz, luz, q))
        # Pintor: lo más lejano primero
        caras.sort(key=lambda c: -c[0])
        for _, luz, q in caras:
            col = atenuar(color, 0.25 + 0.75 * luz)
            self.create_polygon([v for (sx, sy, _) in q
                                 for v in (cx + sx * r, cy + sy * r)],
                                fill=col, outline=col)

    def pintar(self):
        e = self.estado
        x0, y0, x1, y1 = self.area()
        cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0 - 4
        r = min(x1 - x0, y1 - y0) / 2.0 - 16
        if r < 10:
            return

        p = e.paquete if e else None
        if p is None:
            self.create_text(cx, y1 - 6, text="SIN ACTITUD", fill=VERDE_OSC,
                             font=fuente(8))
            return

        col = VERDE if p.nav_valida else AMBAR
        modelo = self._cargar_modelo()
        if modelo is not None:
            # Un poco menor que r: con el cohete vertical las aletas pisaban
            # los textos de abajo
            self._pintar_modelo(modelo, p.quat, cx, cy, r * 0.9, col)
        else:
            # Sin el STL, el vector morro como antes
            n, ee, d = p.morro_ned
            sx, sy, prof = self._proyectar(n, ee, d)
            px, py = cx + sx * r, cy + sy * r
            self.create_line(cx, cy, px, py, fill=col, width=2,
                             arrow="last", arrowshape=(9, 11, 4))

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


# Apogeo al que apunta el guiado (gc_tun.apogee_target_agl_m del GNC v1.5).
# El error del apogeo previsto se mide contra esto.
APOGEO_OBJETIVO_M = 3000.0


class PanelNumeros(Panel):
    """Las cifras que se cantan en voz alta durante el vuelo."""

    def pintar(self):
        e = self.estado
        x0, y0, x1, y1 = self.area()
        p = e.paquete if e else None

        cats = e is not None and e.ver_cats
        _marca_cats(self, e, x1 - 4, y0 + 6, anchor="e")

        # Error del apogeo: previsto - objetivo. Con el previsto a 0 (el
        # guiado aun no lo calcula) no hay error que dar.
        apogeo = p.apogeo_m if p is not None else None
        error = (apogeo - APOGEO_OBJETIVO_M) if apogeo else None

        # (etiqueta, valor, unidad, color, tamaño, mitad derecha o None)
        filas = (
            ("ALTURA", _num(e and e.dato("altura_m"), "%.1f"), "m",
             MAGENTA if cats else VERDE, 22, None),
            ("VEL VERTICAL", _num(e and e.dato("vel_vertical_mps"), "%+.2f"),
             "m/s", MAGENTA if cats else VERDE, 22, None),
            ("APOGEO PREVISTO", _num(apogeo, "%.0f"), "m", AMBAR, 16,
             ("ERROR (OBJETIVO %.0f m)" % APOGEO_OBJETIVO_M,
              _num(error, "%+.0f"), "m", AMBAR)),
            ("AEROFRENO", _num(p and p.aerofreno_pct, "%.0f"), "%", CIAN, 16,
             None),
        )
        alto = (y1 - y0) / float(len(filas))
        xm = (x0 + x1) / 2.0
        for i, (etq, val, uni, col, tam, der) in enumerate(filas):
            cy = y0 + alto * i
            # Con mitad derecha, lo de la izquierda acaba en el centro
            xv = (xm - 10) if der else x1
            self.create_text(x0 + 2, cy + 9, text=etq, anchor="w",
                             fill=VERDE_OSC, font=fuente(7))
            self.create_text(xv - 26, cy + alto * 0.62, text=val, anchor="e",
                             fill=col if p is not None else VERDE_OSC,
                             font=fuente(tam, True))
            self.create_text(xv - 2, cy + alto * 0.62, text=uni, anchor="e",
                             fill=VERDE_MED, font=fuente(9))
            if der:
                d_etq, d_val, d_uni, d_col = der
                self.create_line(xm, cy + 4, xm, cy + alto - 4, fill=REJILLA)
                self.create_text(xm + 8, cy + 9, text=d_etq, anchor="w",
                                 fill=VERDE_OSC, font=fuente(7))
                self.create_text(x1 - 26, cy + alto * 0.62, text=d_val,
                                 anchor="e",
                                 fill=d_col if p is not None else VERDE_OSC,
                                 font=fuente(tam, True))
                self.create_text(x1 - 2, cy + alto * 0.62, text=d_uni,
                                 anchor="e", fill=VERDE_MED, font=fuente(9))
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

        pista = list(e.hist("pista")) if e else []
        _marca_cats(self, e, x1 - 4, y0 + 6, anchor="e")
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
        if e is not None and e.ver_cats:
            c = e.cats
            if c is not None and c.gps_valido:
                self.create_text(x0 + 2, y0 + 6, anchor="w",
                                 text="LAT %.5f   LON %.5f" % (c.lat, c.lon),
                                 fill=MAGENTA, font=fuente(8))
            elif c is not None:
                self.create_text(x0 + 2, y0 + 6, anchor="w",
                                 text="CATS sin GPS", fill=AMBAR,
                                 font=fuente(8))
        elif p is not None:
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
        # GNC v1.6: listo para volar (paquete de estado v7). Es lo que hay
        # que mirar antes de armar el iniciador, no NAV VALIDA.
        listo = s.nav_listo if s is not None else None
        if listo is None:
            lampara(self, x0 + 240, y, False, "LISTO ?", color=VERDE_OSC)
        elif listo:
            lampara(self, x0 + 240, y, True, "LISTO VUELO", color=VERDE)
        else:
            falta = pq.nav_init_falta(s.nav_init)
            lampara(self, x0 + 240, y, True,
                    "FALTA " + (falta[0] if falta else "?"), color=AMBAR)
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
        # Los dos receptores de la estación de CATS, se elija lo que se elija
        # en DATOS: magenta oyendo, rojo perdido, apagado si nunca oyó nada
        xc = x1 - 124
        self.create_text(xc - 6, y, text="CATS (antenas)", anchor="w",
                         fill=VERDE_OSC, font=fuente(7))
        y += 14
        for i, etq in enumerate(pq.PIROS):
            lampara(self, x0 + 6 + i * 46, y, bool(p and p.pirotecnia[i]), etq,
                    color=AMBAR)
        for i, link in enumerate((1, 2)):
            oye = e.cats_link(link) if e else None
            lampara(self, xc + i * 64, y, oye is not None, "LINK %d" % link,
                    color=MAGENTA if oye else ROJO)
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
