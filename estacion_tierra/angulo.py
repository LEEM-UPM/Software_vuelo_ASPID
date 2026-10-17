# -*- coding: utf-8 -*-
"""
Panel del ángulo de lanzamiento: aparece en el sitio del RECORRIDO, igual
que el de pruebas.

El ángulo es el de la RAMPA medido DESDE LA VERTICAL (0 = rampa vertical).
Se puede elegir de tres maneras, y las tres se mantienen sincronizadas:

  - arrastrando el cohete de la ruleta
  - con la rueda del ratón encima de la ruleta: 0,5 grados por paso, o 0,1
    con Mayúsculas pulsada
  - escribiéndolo en la casilla (Intro o salir de la casilla para aplicarlo;
    vale coma o punto)

Debajo va el RUMBO: hacia dónde se inclina el morro, en grados desde el norte
geográfico en sentido horario (0 N, 90 E, 180 S, 270 O). Con la rampa
vertical da igual. El cohete convierte los dos al convenio del GNC y mide él
mismo el alabeo (cómo está girada la placa) con el acelerómetro.

ENVIAR manda los dos al cohete y la estación reintenta cada uno cada ~0,5 s
hasta que el cohete CONTESTA CON ESE MISMO VALOR. No basta con que conteste: el acuse
trae el valor que el cohete tiene guardado, y sólo si coincide con el pedido
se da por confirmado.

En la ruleta se ven dos cosas a la vez, y conviene no confundirlas:
  el cohete brillante  el valor elegido aquí, todavía no enviado o enviándose
  la línea cian        lo que el cohete tiene guardado, según su último acuse
"""

import math
import tkinter as tk

import paquete as pq
from widgets import (AMBAR, CIAN, FONDO, REJILLA, ROJO, VERDE, VERDE_MED,
                     VERDE_OSC, fuente)

PASO_RUEDA = 0.5
PASO_RUEDA_FINO = 0.1


class PanelAngulo(tk.Frame):

    def __init__(self, padre, enviar, cerrar):
        """
        enviar(grados)  manda el ángulo al cohete
        cerrar()        oculta el panel
        """
        tk.Frame.__init__(self, padre, bg=FONDO, highlightthickness=1,
                          highlightbackground=CIAN)
        self._enviar = enviar
        self._cerrar = cerrar
        self.valor = 0.0
        self.rumbo = 0.0
        self._cohete_pintado = "sin pintar"
        self._construir()
        self._pintar_ruleta()

    # --- Construcción ---

    def _construir(self):
        cab = tk.Frame(self, bg=FONDO)
        cab.pack(fill="x", padx=8, pady=(5, 0))
        tk.Label(cab, text="ANGULO DE LANZAMIENTO", bg=FONDO, fg=CIAN,
                 font=fuente(11, True)).pack(side="left")
        tk.Button(cab, text="CERRAR", bg=FONDO, fg=VERDE_MED,
                  activebackground=VERDE_MED, activeforeground=FONDO,
                  relief="solid", bd=1, highlightthickness=0,
                  font=fuente(8, True), cursor="hand2",
                  command=self._cerrar).pack(side="right")

        tk.Label(self, text="desde la vertical: 0 = rampa vertical",
                 bg=FONDO, fg=VERDE_OSC, font=fuente(7),
                 anchor="w").pack(fill="x", padx=8)

        cuerpo = tk.Frame(self, bg=FONDO)
        cuerpo.pack(fill="both", expand=True, padx=8, pady=4)

        # Ruleta
        self.lienzo = tk.Canvas(cuerpo, bg=FONDO, highlightthickness=0, bd=0,
                                width=10, height=10, cursor="hand2")
        self.lienzo.pack(side="left", fill="both", expand=True)
        self.lienzo.bind("<Configure>", lambda e: self._pintar_ruleta())
        self.lienzo.bind("<Button-1>", self._arrastrar)
        self.lienzo.bind("<B1-Motion>", self._arrastrar)
        self.lienzo.bind("<MouseWheel>", self._rueda)

        # Columna derecha: casilla, enviar y estado
        der = tk.Frame(cuerpo, bg=FONDO, width=170)
        der.pack(side="right", fill="y", padx=(8, 0))
        der.pack_propagate(False)

        fila = tk.Frame(der, bg=FONDO)
        fila.pack(fill="x")
        self.var = tk.StringVar(value="0.00")
        self.casilla = tk.Entry(fila, textvariable=self.var, width=7,
                                justify="right", bg=FONDO, fg=VERDE,
                                insertbackground=VERDE, relief="solid", bd=1,
                                highlightthickness=0, font=fuente(16, True))
        self.casilla.pack(side="left")
        tk.Label(fila, text=" grados", bg=FONDO, fg=VERDE_MED,
                 font=fuente(9)).pack(side="left")
        self.casilla.bind("<Return>", self._desde_casilla)
        self.casilla.bind("<FocusOut>", self._desde_casilla)

        fila_r = tk.Frame(der, bg=FONDO)
        fila_r.pack(fill="x", pady=(2, 0))
        self.var_rumbo = tk.StringVar(value="0.00")
        self.casilla_rumbo = tk.Entry(fila_r, textvariable=self.var_rumbo,
                                      width=7, justify="right", bg=FONDO,
                                      fg=VERDE, insertbackground=VERDE,
                                      relief="solid", bd=1,
                                      highlightthickness=0,
                                      font=fuente(12, True))
        self.casilla_rumbo.pack(side="left")
        tk.Label(fila_r, text=" rumbo", bg=FONDO, fg=VERDE_MED,
                 font=fuente(9)).pack(side="left")
        tk.Label(der, text="desde el norte: 0 N 90 E 180 S 270 O", bg=FONDO,
                 fg=VERDE_OSC, font=fuente(7), anchor="w").pack(fill="x")
        self.casilla_rumbo.bind("<Return>", self._desde_casilla_rumbo)
        self.casilla_rumbo.bind("<FocusOut>", self._desde_casilla_rumbo)

        self.btn_enviar = tk.Button(
            der, text="ENVIAR AL COHETE", bg=FONDO, fg=CIAN,
            activebackground=CIAN, activeforeground=FONDO, relief="solid",
            bd=1, highlightthickness=0, font=fuente(10, True), cursor="hand2",
            command=self._pulsar_enviar)
        self.btn_enviar.pack(fill="x", pady=(4, 2))

        tk.Label(der, text="EN EL COHETE", bg=FONDO, fg=VERDE_OSC,
                 font=fuente(7), anchor="w").pack(fill="x")
        self.lbl_cohete = tk.Label(der, text="--", bg=FONDO, fg=CIAN,
                                   font=fuente(8, True), anchor="w")
        self.lbl_cohete.pack(fill="x")
        self.lbl_origen = tk.Label(der, text="", bg=FONDO, fg=VERDE_MED,
                                   font=fuente(7), anchor="w", justify="left",
                                   wraplength=165)
        self.lbl_origen.pack(fill="x")
        self.lbl_estado = tk.Label(der, text="", bg=FONDO, fg=AMBAR,
                                   font=fuente(8, True), anchor="w",
                                   justify="left", wraplength=165)
        self.lbl_estado.pack(fill="x", pady=(2, 0))

    # --- Elección del valor ---

    def poner(self, grados):
        grados = max(0.0, min(pq.ANGULO_MAX_GRADOS, float(grados)))
        self.valor = round(grados, 2)
        self.var.set("%.2f" % self.valor)
        self._pintar_ruleta()

    def poner_rumbo(self, grados):
        r = float(grados) % 360.0
        self.rumbo = round(min(r, pq.RUMBO_MAX_GRADOS), 2)
        self.var_rumbo.set("%.2f" % self.rumbo)

    def _geometria(self):
        w = max(self.lienzo.winfo_width(), 40)
        h = max(self.lienzo.winfo_height(), 40)
        r = min(w - 34, h - 30)
        return 16, h - 14, max(r, 20)   # pivote x, pivote y, radio

    def _arrastrar(self, ev):
        px, py, _ = self._geometria()
        dx, dy = ev.x - px, py - ev.y
        if dx <= 0 and dy <= 0:
            return
        self.poner(math.degrees(math.atan2(max(dx, 0), max(dy, 0))))

    def _rueda(self, ev):
        paso = PASO_RUEDA_FINO if (ev.state & 0x0001) else PASO_RUEDA
        self.poner(self.valor + (paso if ev.delta > 0 else -paso))

    def _desde_casilla(self, _ev=None):
        texto = self.var.get().strip().replace(",", ".")
        try:
            self.poner(float(texto))
        except ValueError:
            self.var.set("%.2f" % self.valor)   # se deshace lo no valido

    def _desde_casilla_rumbo(self, _ev=None):
        texto = self.var_rumbo.get().strip().replace(",", ".")
        try:
            r = float(texto) % 360.0
            self.rumbo = round(min(r, pq.RUMBO_MAX_GRADOS), 2)
        except ValueError:
            pass   # se deshace lo no valido
        self.var_rumbo.set("%.2f" % self.rumbo)

    def _pulsar_enviar(self):
        self._desde_casilla()
        self._desde_casilla_rumbo()
        self._enviar(self.valor, self.rumbo)

    # --- Dibujo ---

    def _punto(self, grados, radio):
        px, py, _ = self._geometria()
        a = math.radians(grados)
        return px + radio * math.sin(a), py - radio * math.cos(a)

    def _pintar_ruleta(self, cohete=None):
        c = self.lienzo
        c.delete("all")
        px, py, r = self._geometria()

        # Arco de 0 a 90 grados con marcas
        c.create_arc(px - r, py - r, px + r, py + r, start=0, extent=90,
                     style="arc", outline=VERDE_OSC)
        for g in range(0, 91, 5):
            largo = 10 if g % 15 == 0 else 5
            x0, y0 = self._punto(g, r)
            x1, y1 = self._punto(g, r - largo)
            c.create_line(x0, y0, x1, y1,
                          fill=VERDE_MED if g % 15 == 0 else REJILLA)
            if g % 15 == 0:
                tx, ty = self._punto(g, r + 11)
                c.create_text(tx, ty, text=str(g), fill=VERDE_OSC,
                              font=fuente(7))

        # Vertical de referencia y suelo
        c.create_line(px, py, px, py - r, fill=REJILLA, dash=(3, 3))
        c.create_line(px - 8, py, px + r, py, fill=VERDE_OSC)

        # Lo que tiene el cohete
        if cohete is None:
            cohete = getattr(self, "_cohete_valor", None)
        self._cohete_valor = cohete
        if cohete is not None:
            x, y = self._punto(cohete, r * 0.92)
            c.create_line(px, py, x, y, fill=CIAN, width=1, dash=(5, 2))

        # Lo elegido aqui: la rampa y el cohete encima
        x, y = self._punto(self.valor, r * 0.92)
        c.create_line(px, py, x, y, fill=VERDE_MED, width=2)
        a = math.radians(self.valor)
        ux, uy = math.sin(a), -math.cos(a)        # a lo largo de la rampa
        nx, ny = -uy, ux                          # perpendicular
        base = r * 0.45
        punta = r * 0.80
        anch = 4.0
        bx, by = px + ux * base, py + uy * base
        tx, ty = px + ux * punta, py + uy * punta
        c.create_polygon(bx + nx * anch, by + ny * anch,
                         tx + nx * anch * 0.8, ty + ny * anch * 0.8,
                         px + ux * (punta + 9), py + uy * (punta + 9),
                         tx - nx * anch * 0.8, ty - ny * anch * 0.8,
                         bx - nx * anch, by - ny * anch,
                         fill=VERDE, outline="")
        c.create_oval(px - 3, py - 3, px + 3, py + 3, fill=VERDE_MED,
                      outline="")
        c.create_text(px + r * 0.30, py - 10, anchor="w",
                      text="%.2f" % self.valor, fill=VERDE,
                      font=fuente(10, True))

    # --- Refresco desde el estado ---

    def actualizar(self, e):
        if e.angulo_cohete is None:
            self.lbl_cohete.configure(text="desconocido", fg=VERDE_OSC)
            self.lbl_origen.configure(text="")
        else:
            rumbo = ("rumbo %.2f" % e.rumbo_cohete
                     if e.rumbo_cohete is not None else "rumbo ?")
            self.lbl_cohete.configure(
                text="%.2f grados, %s" % (e.angulo_cohete, rumbo), fg=CIAN)
            if e.angulo_definido_cohete:
                origen = "guardado en la SD" if e.angulo_guardado_sd else \
                    "NO se pudo guardar en la SD"
            else:
                origen = "sin definir: vertical por defecto"
            self.lbl_origen.configure(
                text=origen + "\n(en tierra el GNC se realinea con el al "
                "momento)")

        if e.angulo_cohete != self._cohete_pintado:
            self._cohete_pintado = e.angulo_cohete
            self._pintar_ruleta(e.angulo_cohete)

        textos = {
            "consultando": ("preguntando al cohete... intento %d"
                            % e.intentos_angulo, AMBAR),
            "enviando": ("enviando %.2f... intento %d"
                         % (e.angulo_pedido or 0.0, e.intentos_angulo), AMBAR),
            "confirmado": ("CONFIRMADO por el cohete", VERDE),
            "denegado": ("DENEGADO: %s" % e.angulo_motivo, ROJO),
            "sin respuesta": ("SIN RESPUESTA del cohete", ROJO),
        }
        txt, col = textos.get(e.angulo_estado, ("", AMBAR))
        # El rumbo, si hay algo que contar y el angulo no tiene nada peor
        if e.rumbo_estado in ("denegado", "sin respuesta") or \
                (e.rumbo_estado == "enviando" and not txt):
            r_txt = {"enviando": ("rumbo: enviando... intento %d"
                                  % e.intentos_rumbo, AMBAR),
                     "denegado": ("rumbo DENEGADO: %s" % e.rumbo_motivo, ROJO),
                     "sin respuesta": ("rumbo: SIN RESPUESTA", ROJO)}
            txt, col = r_txt[e.rumbo_estado]

        # Si lo elegido en la ruleta no es lo que tiene el cohete, eso es lo
        # que hay que ver: un "CONFIRMADO" de un valor anterior al lado de un
        # numero distinto se lee como si el nuevo estuviera confirmado.
        ocupado = e.angulo_estado in ("enviando", "consultando")
        if (not ocupado and e.angulo_cohete is not None and
                abs(self.valor - e.angulo_cohete) > 0.005):
            txt, col = ("%.2f SIN ENVIAR: el cohete sigue en %.2f"
                        % (self.valor, e.angulo_cohete), AMBAR)
        self.lbl_estado.configure(text=txt, fg=col)
