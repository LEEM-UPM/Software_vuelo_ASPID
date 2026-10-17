# -*- coding: utf-8 -*-
"""
Panel de la altura del aerofreno: aparece en el sitio de SALUD.

Por debajo de esa altura (metros sobre la rampa, AGL) el guiado no abre el
aerofreno (App/freno_altura.h; en el GNC, gc_tun.min_ctrl_alt_agl_m). El
cohete la guarda en las dos SD y la lee al arrancar.

Funciona igual que el del ángulo de lanzamiento: ENVIAR la manda y la
estación la reintenta cada ~0,5 s hasta que el cohete CONTESTA CON ESA MISMA
ALTURA. Al abrir el panel, si no se sabe qué tiene el cohete, se le pregunta,
y la casilla arranca en lo que conteste.
"""

import tkinter as tk

import paquete as pq
from widgets import (AMBAR, CIAN, FONDO, ROJO, VERDE, VERDE_MED, VERDE_OSC,
                     fuente)


class PanelFreno(tk.Frame):

    def __init__(self, padre, enviar, cerrar):
        """
        enviar(metros)  manda la altura al cohete
        cerrar()        oculta el panel
        """
        tk.Frame.__init__(self, padre, bg=FONDO, highlightthickness=1,
                          highlightbackground=CIAN)
        self._enviar = enviar
        self._cerrar = cerrar
        self.valor = pq.FRENO_FABRICA_M
        self._construir()

    # --- Construcción ---

    def _construir(self):
        cab = tk.Frame(self, bg=FONDO)
        cab.pack(fill="x", padx=8, pady=(4, 0))
        tk.Label(cab, text="ALTURA DEL AEROFRENO", bg=FONDO, fg=CIAN,
                 font=fuente(10, True)).pack(side="left")
        tk.Button(cab, text="CERRAR", bg=FONDO, fg=VERDE_MED,
                  activebackground=VERDE_MED, activeforeground=FONDO,
                  relief="solid", bd=1, highlightthickness=0,
                  font=fuente(8, True), cursor="hand2",
                  command=self._cerrar).pack(side="right")

        fila = tk.Frame(self, bg=FONDO)
        fila.pack(fill="x", padx=8, pady=(4, 0))
        self.var = tk.StringVar(value="%d" % self.valor)
        self.casilla = tk.Entry(fila, textvariable=self.var, width=5,
                                justify="right", bg=FONDO, fg=VERDE,
                                insertbackground=VERDE, relief="solid", bd=1,
                                highlightthickness=0, font=fuente(14, True))
        self.casilla.pack(side="left")
        tk.Label(fila, text=" m AGL", bg=FONDO, fg=VERDE_MED,
                 font=fuente(9)).pack(side="left")
        self.casilla.bind("<Return>", self._desde_casilla)
        self.casilla.bind("<FocusOut>", self._desde_casilla)

        self.btn_enviar = tk.Button(
            fila, text="ENVIAR AL COHETE", bg=FONDO, fg=CIAN,
            activebackground=CIAN, activeforeground=FONDO, relief="solid",
            bd=1, highlightthickness=0, font=fuente(9, True), cursor="hand2",
            command=self._pulsar_enviar)
        self.btn_enviar.pack(side="right")

        tk.Label(self, text="por debajo no se abre el aerofreno: de %d a %d m"
                 % (pq.FRENO_MIN_M, pq.FRENO_MAX_M), bg=FONDO, fg=VERDE_OSC,
                 font=fuente(7), anchor="w").pack(fill="x", padx=8)

        fila_c = tk.Frame(self, bg=FONDO)
        fila_c.pack(fill="x", padx=8, pady=(4, 0))
        tk.Label(fila_c, text="EN EL COHETE ", bg=FONDO, fg=VERDE_OSC,
                 font=fuente(7)).pack(side="left")
        self.lbl_cohete = tk.Label(fila_c, text="--", bg=FONDO, fg=CIAN,
                                   font=fuente(10, True), anchor="w")
        self.lbl_cohete.pack(side="left")
        self.lbl_origen = tk.Label(fila_c, text="", bg=FONDO, fg=VERDE_MED,
                                   font=fuente(7), anchor="w")
        self.lbl_origen.pack(side="left", padx=(6, 0))

        self.lbl_estado = tk.Label(self, text="", bg=FONDO, fg=AMBAR,
                                   font=fuente(8, True), anchor="w",
                                   justify="left")
        self.lbl_estado.pack(fill="x", padx=8, pady=(2, 4))
        self.bind("<Configure>", self._ajustar)

    def _ajustar(self, ev):
        """El estado se parte al ancho que tenga el panel"""
        self.lbl_estado.configure(wraplength=max(ev.width - 20, 100))

    # --- Elección del valor ---

    def poner(self, metros):
        metros = max(pq.FRENO_MIN_M, min(pq.FRENO_MAX_M, int(round(metros))))
        self.valor = metros
        self.var.set("%d" % self.valor)

    def _desde_casilla(self, _ev=None):
        texto = self.var.get().strip().replace(",", ".")
        try:
            self.poner(float(texto))
        except ValueError:
            self.var.set("%d" % self.valor)   # se deshace lo no valido

    def _pulsar_enviar(self):
        self._desde_casilla()
        self._enviar(self.valor)

    # --- Refresco desde el estado ---

    def actualizar(self, e):
        if e.freno_cohete is None:
            self.lbl_cohete.configure(text="desconocido", fg=VERDE_OSC)
            self.lbl_origen.configure(text="")
        else:
            self.lbl_cohete.configure(text="%d m" % e.freno_cohete, fg=CIAN)
            if e.freno_definido_cohete:
                origen = "guardada en la SD" if e.freno_guardado_sd else \
                    "NO se pudo guardar en la SD"
            else:
                origen = "sin definir: la de fabrica"
            self.lbl_origen.configure(text=origen)

        textos = {
            "consultando": ("preguntando al cohete... intento %d"
                            % e.intentos_freno, AMBAR),
            "enviando": ("enviando %d m... intento %d"
                         % (e.freno_pedido or 0, e.intentos_freno), AMBAR),
            "confirmado": ("CONFIRMADA por el cohete", VERDE),
            "denegado": ("DENEGADA: %s" % e.freno_motivo, ROJO),
            "sin respuesta": ("SIN RESPUESTA del cohete", ROJO),
        }
        txt, col = textos.get(e.freno_estado, ("", AMBAR))

        # Si lo de la casilla no es lo que tiene el cohete, eso es lo que hay
        # que ver: un "CONFIRMADA" de un valor anterior al lado de un numero
        # distinto se lee como si el nuevo estuviera confirmado.
        ocupado = e.freno_estado in ("enviando", "consultando")
        if not ocupado and e.freno_cohete is not None and \
                self.valor != e.freno_cohete:
            txt, col = ("%d m SIN ENVIAR: el cohete sigue en %d m"
                        % (self.valor, e.freno_cohete), AMBAR)
        self.lbl_estado.configure(text=txt, fg=col)
