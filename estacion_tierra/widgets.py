# -*- coding: utf-8 -*-
"""
Paleta, fuentes y primitivas de dibujo de la estación de tierra.

La estética es la de una consola de fósforo verde: fondo negro, todo en
monoespaciada, marcos finos y barras segmentadas en vez de continuas. No es
sólo decoración: en una pantalla al sol, en el campo, el contraste alto y la
tipografía gruesa se leen mucho mejor que una interfaz clara.

Todo se pinta sobre Canvas de Tkinter. Cada panel se borra y se redibuja
entero en cada refresco (ver Panel.refrescar). Es más código tonto pero mucho
menos frágil que ir actualizando elementos uno a uno, y a 12 Hz con estos
tamaños va sobrado.
"""

import tkinter as tk
import tkinter.font as tkfont

# --- Paleta -----------------------------------------------------------------

FONDO = "#000000"
VERDE = "#00ff5a"        # dato vivo, lo que hay que leer
VERDE_MED = "#00d24e"    # etiquetas
# Marcos, y tambien mucho texto pequeño (pies, unidades, lo apagado). Era
# #00702a y no se leia bien (2026-10-10): mas claro, sigue por debajo de
# VERDE_MED para que se note que es secundario.
VERDE_OSC = "#00a33d"
REJILLA = "#00381a"      # rejillas y guías
APAGADO = "#0c2113"      # lámpara apagada, segmento vacío
AMBAR = "#ccff00"
CIAN = "#00ffd5"
MAGENTA = "#ff2fa0"
ROJO = "#ff3b30"
AZUL = "#3ba0ff"
# Solo decorativo (acento de los paneles de sistema): no significa nada
VIOLETA = "#a37bff"

# Acento de cada panel, por familias: se pinta en las esquinas y en la
# marca del titulo. Es decoracion que ademas agrupa, pero NO es un dato:
# por eso no se usan ROJO ni AMBAR (alarma y aviso) ni MAGENTA (datos de
# CATS).
ACENTOS = {
    # vuelo
    "MEDIDORES": VERDE, "PERFIL DE ALTURA": VERDE,
    "VELOCIDAD VERTICAL": VERDE, "FASE DE VUELO": VERDE, "VUELO": VERDE,
    # navegacion
    "ACTITUD": AZUL, "RECORRIDO": AZUL, "ACELERACION": AZUL,
    # radio
    "ENLACE": CIAN, "PAQUETE CRUDO": CIAN, "RANURAS RESERVADAS": CIAN,
    # sistema
    "SISTEMA": VIOLETA, "SALUD": VIOLETA,
}

def atenuar(color, f):
    """El mismo color, mas oscuro: f = 1 igual, f = 0 negro."""
    r, g, b = (int(color[i:i + 2], 16) for i in (1, 3, 5))
    return "#%02x%02x%02x" % (int(r * f), int(g * f), int(b * f))


# --- Fuentes ----------------------------------------------------------------

_FAMILIA = None


def familia():
    """La primera monoespaciada decente que haya instalada."""
    global _FAMILIA
    if _FAMILIA is None:
        disponibles = set(tkfont.families())
        for f in ("Consolas", "DejaVu Sans Mono", "Courier New", "Courier"):
            if f in disponibles:
                _FAMILIA = f
                break
        else:
            _FAMILIA = "TkFixedFont"
    return _FAMILIA


# Nada por debajo de esto: en Consolas, 7 puntos no se lee bien en campo
TAM_MINIMO = 8


def fuente(tam, negrita=False):
    tam = max(tam, TAM_MINIMO)
    return (familia(), tam, "bold") if negrita else (familia(), tam)


# --- Primitivas -------------------------------------------------------------


def barra_segmentada(c, x0, y0, x1, y1, fraccion, color=VERDE,
                     segmentos=None, vertical=False, fondo=APAGADO):
    """
    Barra hecha de bloques sueltos en vez de un rectángulo continuo.

    Se lee mejor de un vistazo que una barra lisa: el ojo cuenta bloques sin
    tener que estimar longitudes, que es justo lo que hace falta cuando miras
    doce indicadores a la vez.
    """
    fraccion = max(0.0, min(1.0, fraccion))
    largo = (y1 - y0) if vertical else (x1 - x0)
    if segmentos is None:
        segmentos = max(4, int(largo // 5))
    paso = largo / float(segmentos)
    encendidos = int(round(fraccion * segmentos))

    for i in range(segmentos):
        col = color if i < encendidos else fondo
        if vertical:
            # De abajo arriba, que es como se espera de un medidor vertical
            b = y1 - i * paso
            c.create_rectangle(x0, b - paso + 1, x1, b, fill=col, outline="")
        else:
            a = x0 + i * paso
            c.create_rectangle(a, y0, a + paso - 1, y1, fill=col, outline="")


def barra_con_centro(c, x0, y0, x1, y1, valor, fondo_escala, color=VERDE):
    """
    Barra con signo: crece a la derecha o a la izquierda desde el centro.
    Para aceleraciones y velocidades, donde el signo importa tanto como el
    módulo.
    """
    cx = (x0 + x1) / 2.0
    c.create_rectangle(x0, y0, x1, y1, outline=REJILLA)
    c.create_line(cx, y0 - 2, cx, y1 + 2, fill=VERDE_OSC)
    if fondo_escala <= 0:
        return
    f = max(-1.0, min(1.0, valor / float(fondo_escala)))
    ancho = f * (x1 - x0) / 2.0
    if abs(ancho) < 1:
        return
    if ancho > 0:
        c.create_rectangle(cx, y0 + 1, cx + ancho, y1 - 1, fill=color,
                           outline="")
    else:
        c.create_rectangle(cx + ancho, y0 + 1, cx, y1 - 1, fill=color,
                           outline="")


def rejilla(c, x0, y0, x1, y1, nx=6, ny=4, color=REJILLA):
    for i in range(1, nx):
        x = x0 + (x1 - x0) * i / float(nx)
        c.create_line(x, y0, x, y1, fill=color)
    for j in range(1, ny):
        y = y0 + (y1 - y0) * j / float(ny)
        c.create_line(x0, y, x1, y, fill=color)
    c.create_rectangle(x0, y0, x1, y1, outline=VERDE_OSC)


def traza(c, x0, y0, x1, y1, datos, minimo=None, maximo=None, color=VERDE,
          relleno=False):
    """
    Dibuja una serie histórica como una polilínea. Devuelve (min, max) usados,
    para que quien llame pueda rotular la escala.
    """
    if not datos:
        return (0.0, 1.0)
    lo = min(datos) if minimo is None else minimo
    hi = max(datos) if maximo is None else maximo
    if hi - lo < 1e-6:
        hi = lo + 1.0
    n = len(datos)
    if n < 2:
        return (lo, hi)

    paso = (x1 - x0) / float(n - 1)
    puntos = []
    for i, v in enumerate(datos):
        f = (v - lo) / (hi - lo)
        puntos.append(x0 + i * paso)
        puntos.append(y1 - f * (y1 - y0))

    if relleno:
        c.create_polygon([x0, y1] + puntos + [x1, y1], fill=APAGADO,
                         outline="")
    c.create_line(puntos, fill=color, width=1)
    return (lo, hi)


def lampara(c, x, y, encendida, etiqueta="", color=VERDE, r=5,
            color_texto=None):
    """Indicador cuadrado con etiqueta a la derecha."""
    col = color if encendida else APAGADO
    c.create_rectangle(x - r, y - r, x + r, y + r, fill=col,
                       outline=VERDE_OSC)
    if etiqueta:
        c.create_text(x + r + 5, y, text=etiqueta, anchor="w",
                      fill=color_texto or (color if encendida else VERDE_MED),
                      font=fuente(8))


def circulo_fase(c, x, y, r, texto, activo, color=VERDE):
    col = color if activo else APAGADO
    borde = color if activo else VERDE_OSC
    c.create_oval(x - r, y - r, x + r, y + r, fill=col, outline=borde,
                  width=2 if activo else 1)
    c.create_text(x, y, text=texto, fill=FONDO if activo else VERDE_MED,
                  font=fuente(9, activo))


# --- Panel base -------------------------------------------------------------


class Panel(tk.Canvas):
    """
    Un recuadro con título. Las subclases sólo implementan pintar().

    El marco lleva marcas en las esquinas en vez de un borde liso: es lo que le
    da el aire de instrumento y, de paso, hace obvio dónde acaba cada panel
    cuando hay quince pegados.
    """

    def __init__(self, padre, titulo="", color=VERDE_OSC, **kw):
        kw.setdefault("bg", FONDO)
        kw.setdefault("highlightthickness", 0)
        kw.setdefault("bd", 0)
        # OJO: un Canvas de Tkinter pide 378x265 si no se le dice otra cosa, y
        # el grid respeta esa petición. Con quince paneles, la suma se sale de
        # la pantalla y las filas de abajo desaparecen. Pidiendo casi nada, el
        # tamaño real lo deciden los pesos y los minsize del grid, que es lo
        # que queremos.
        kw.setdefault("width", 10)
        kw.setdefault("height", 10)
        tk.Canvas.__init__(self, padre, takefocus=0, **kw)
        self.titulo = titulo
        self.color_marco = color
        self.acento = ACENTOS.get(titulo, VERDE_MED)
        # El marco, del color de su familia pero apagado: agrupa sin quitarle
        # protagonismo a los datos. Los paneles sin familia, como estaban.
        if titulo in ACENTOS and color == VERDE_OSC:
            self.color_marco = atenuar(self.acento, 0.45)
        self.estado = None
        self.w = int(kw.get("width", 120))
        self.h = int(kw.get("height", 80))
        self.bind("<Configure>", self._al_configurar)

    def _al_configurar(self, ev):
        self.w, self.h = ev.width, ev.height
        self.refrescar()

    def area(self):
        """Caja útil, ya descontados marco y título."""
        arriba = 17 if self.titulo else 5
        return (6, arriba, self.w - 6, self.h - 5)

    def _marco(self):
        x0, y0, x1, y1 = 1, 1, self.w - 2, self.h - 2
        self.create_rectangle(x0, y0, x1, y1, outline=self.color_marco)
        # Marcas de esquina, más brillantes que el resto del borde
        d = 7
        for (ex, ey, dx, dy) in ((x0, y0, 1, 1), (x1, y0, -1, 1),
                                 (x0, y1, 1, -1), (x1, y1, -1, -1)):
            self.create_line(ex, ey, ex + d * dx, ey, fill=self.acento)
            self.create_line(ex, ey, ex, ey + d * dy, fill=self.acento)
        if self.titulo:
            fin = 20 + 6.2 * len(self.titulo)
            self.create_rectangle(9, 2, fin + 4, 13, fill=FONDO, outline="")
            # Marca de color antes del titulo
            self.create_rectangle(11, 4, 15, 12, fill=self.acento,
                                  outline="")
            self.create_text(19, 8, text=self.titulo, anchor="w",
                             fill=VERDE_MED, font=fuente(8, True))
            # Tres puntos de "instrumento" arriba a la derecha
            for k in range(3):
                x = self.w - 14 - k * 6
                self.create_rectangle(x, 0, x + 2, 2, fill=self.acento,
                                      outline="")

    def refrescar(self, estado=None):
        if estado is not None:
            self.estado = estado
        self.delete("all")
        self._marco()
        try:
            self.pintar()
        except Exception as e:
            # Un panel que peta no debe tumbar la estación entera: en vuelo
            # vale más una casilla rota que una ventana cerrada.
            self.create_text(self.w / 2, self.h / 2, text="FALLO: %s" % e,
                             fill=ROJO, font=fuente(7), width=self.w - 20)

    def pintar(self):
        pass


class PanelLibre(Panel):
    """
    Hueco reservado. Se deja dibujado a propósito, no en blanco: así se ve de
    un vistazo cuánto sitio queda y para qué está pensado cada trozo.
    """

    def __init__(self, padre, titulo="LIBRE", nota="", **kw):
        Panel.__init__(self, padre, titulo=titulo, color=REJILLA, **kw)
        self.nota = nota

    def pintar(self):
        x0, y0, x1, y1 = self.area()
        paso = 14
        x = x0 - (y1 - y0)
        while x < x1 + (y1 - y0):
            self.create_line(max(x, x0), y0 + max(0, x0 - x),
                             min(x + (y1 - y0), x1),
                             y0 + min(y1 - y0, x1 - x), fill="#0a1c0e")
            x += paso
        if self.nota:
            self.create_text((x0 + x1) / 2, (y0 + y1) / 2, text=self.nota,
                             fill=VERDE_OSC, font=fuente(8),
                             width=x1 - x0 - 10, justify="center")
