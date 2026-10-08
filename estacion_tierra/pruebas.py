# -*- coding: utf-8 -*-
"""
Panel de pruebas: aparece en el sitio del RECORRIDO cuando se pide el modo
test, y desde aquí se encienden los pirotécnicos.

LA PANTALLA ENSEÑA LO QUE DICE EL COHETE, NO LO QUE SE HA PULSADO. Pulsar
"MODO TEST" abre este panel, pero los botones de disparo siguen apagados
hasta que un paquete de estado RECIENTE confirma que el cohete ha entrado. Con
un enlace de subida que pierde el 70 % de los comandos, dar por hecho que una
orden ha llegado es la forma más rápida de creerse en un estado que no es.

CADA DISPARO PIDE DOS CLICS. El primero arma el botón (se pone rojo y dice
CONFIRMAR) durante TIEMPO_CONFIRMAR_MS; el segundo dispara. Un clic suelto o
un dedo que resbala no enciende nada.

AL CONFIRMAR SE MANDA UNA RÁFAGA, no un comando. Con el 30 % de acierto
medido, uno solo fallaría dos de cada tres veces; cinco seguidos llegan el
83 % de las veces. Repetirlo no hace daño: power mantiene el canal 1 s y lo
apaga solo, cada repetición sólo reinicia ese segundo (ICD 6.3).

Esto NO reintenta solo un disparo que no se confirma. Si no llega, que lo
decida una persona con otros dos clics.

Para añadir pruebas nuevas: un botón más en _construir_otras() con su
comando, y el comando en App/uplink.c.

AEROFRENO. PROBAR AEROFRENO (dos clics) hace en el cohete 3 veces todo
fuera / recogido y luego una rampa de salida y otra de entrada, 7 s en total.
La casilla con ENVIAR EXTENSION deja el aerofreno en ese % de la carrera
hasta otra orden o hasta salir del modo test (un clic: escribir el número ya
es deliberado). Las dos se reintentan solas hasta que el cohete contesta, y
la extensión real se ve en el medidor de aerofreno, que ahora muestra lo que
se manda al servo.
"""

import time
import tkinter as tk

import paquete as pq
from widgets import (AMBAR, APAGADO, CIAN, FONDO, REJILLA, ROJO, VERDE,
                     VERDE_MED, VERDE_OSC, fuente)

TIEMPO_CONFIRMAR_MS = 3000
RAFAGA_DISPARO = 5
ESPACIO_RAFAGA_MS = 150

# Tras disparar, el botón dice ENVIADO este rato
TIEMPO_ENVIADO_S = 2.5

# Casillas de OTRAS PRUEBAS (la primera es SIMULAR RUTA)
CASILLAS_LIBRES = 4

# Claves de los botones que no son pirotecnicos en _botones / _armado
SIMULAR = "SIMULAR"
AEROFRENO = "AEROFRENO"



class PanelPruebas(tk.Frame):

    def __init__(self, padre, rafaga, salir, simular, probar_aerofreno,
                 extension_aerofreno):
        """
        rafaga(cmd, param, n, espacio_ms, texto)  manda n comandos seguidos
        salir()                                   pide salir del modo test
        simular()                                 pide simular la ruta
        probar_aerofreno()                        pide la prueba de aerofreno
        extension_aerofreno(pct)                  pide un % de extension
        """
        tk.Frame.__init__(self, padre, bg=FONDO, highlightthickness=1,
                          highlightbackground=ROJO)
        self._rafaga = rafaga
        self._salir = salir
        self._simular = simular
        self._probar_aerofreno = probar_aerofreno
        self._extension_aerofreno = extension_aerofreno
        self._habilitado = False
        self._armado = {}       # nombre -> id del after que lo desarma
        self._enviado = {}      # nombre -> monotonic del disparo
        self._botones = {}
        self._ultimo_p = None   # ultimo paquete de vuelo, para la continuidad
        self._construir()

    # --- Construcción ---

    def _construir(self):
        cab = tk.Frame(self, bg=FONDO)
        cab.pack(fill="x", padx=8, pady=(5, 0))
        tk.Label(cab, text="MODO TEST", bg=FONDO, fg=ROJO,
                 font=fuente(11, True)).pack(side="left")
        self.lbl_estado = tk.Label(cab, text="", bg=FONDO, fg=AMBAR,
                                   font=fuente(9, True))
        self.lbl_estado.pack(side="right")

        self.lbl_info = tk.Label(self, text="", bg=FONDO, fg=VERDE_MED,
                                 font=fuente(8), anchor="w")
        self.lbl_info.pack(fill="x", padx=8)

        tk.Label(self, text="PIROTECNIA   (dos clics: armar y confirmar)",
                 bg=FONDO, fg=VERDE_OSC, font=fuente(7),
                 anchor="w").pack(fill="x", padx=8, pady=(8, 2))

        fila = tk.Frame(self, bg=FONDO)
        fila.pack(fill="x", padx=8)
        for i, nombre in enumerate(pq.CANAL_PIRO):
            fila.columnconfigure(i, weight=1, uniform="piro")
            b = tk.Button(fila, text=nombre, bg=FONDO, fg=VERDE_OSC,
                          activebackground=ROJO, activeforeground=FONDO,
                          disabledforeground=VERDE_OSC, relief="solid", bd=1,
                          highlightthickness=0, font=fuente(10, True),
                          height=3, cursor="hand2",
                          command=lambda n=nombre: self._pulsar(n))
            b.grid(row=0, column=i, sticky="nsew", padx=3)
            self._botones[nombre] = b

        self._construir_otras()

        tk.Button(self, text="SALIR DEL MODO TEST", bg=FONDO, fg=AMBAR,
                  activebackground=AMBAR, activeforeground=FONDO,
                  relief="solid", bd=1, highlightthickness=0,
                  font=fuente(9, True), cursor="hand2",
                  command=self._salir).pack(fill="x", padx=8, pady=(10, 8),
                                            side="bottom")

    def _construir_otras(self):
        tk.Label(self, text="OTRAS PRUEBAS", bg=FONDO, fg=VERDE_OSC,
                 font=fuente(7), anchor="w").pack(fill="x", padx=8,
                                                  pady=(10, 2))
        fila = tk.Frame(self, bg=FONDO)
        fila.pack(fill="x", padx=8)
        for i in range(CASILLAS_LIBRES):
            fila.columnconfigure(i, weight=1, uniform="otra")

        # SIMULAR RUTA: el GNC vuela VUELO.BIN y la radio manda lo que sale.
        # Mueve el aerofreno DE VERDAD, por eso tambien pide dos clics.
        b = tk.Button(fila, text="SIMULAR\nRUTA", bg=FONDO, fg=VERDE_OSC,
                      activebackground=CIAN, activeforeground=FONDO,
                      disabledforeground=VERDE_OSC, relief="solid", bd=1,
                      highlightthickness=0, font=fuente(8, True),
                      cursor="hand2",
                      command=lambda: self._pulsar(SIMULAR))
        b.grid(row=0, column=0, sticky="nsew", padx=3)
        self._botones[SIMULAR] = b

        # PROBAR AEROFRENO: la secuencia de App/airbrake.c. Mueve el
        # aerofreno, asi que dos clics como lo demas.
        b = tk.Button(fila, text="PROBAR\nAEROFRENO", bg=FONDO, fg=VERDE_OSC,
                      activebackground=AMBAR, activeforeground=FONDO,
                      disabledforeground=VERDE_OSC, relief="solid", bd=1,
                      highlightthickness=0, font=fuente(8, True),
                      cursor="hand2",
                      command=lambda: self._pulsar(AEROFRENO))
        b.grid(row=0, column=1, sticky="nsew", padx=3)
        self._botones[AEROFRENO] = b

        # Extension a mano: casilla 0..100 y su boton
        caja = tk.Frame(fila, bg=FONDO, highlightthickness=1,
                        highlightbackground=REJILLA)
        caja.grid(row=0, column=2, sticky="nsew", padx=3)
        self.var_ext = tk.StringVar(value="50")
        self.casilla_ext = tk.Entry(caja, textvariable=self.var_ext, width=4,
                                    justify="right", bg=FONDO, fg=VERDE,
                                    insertbackground=VERDE, relief="flat",
                                    bd=0, highlightthickness=0,
                                    disabledbackground=FONDO,
                                    disabledforeground=VERDE_OSC,
                                    font=fuente(14, True))
        self.casilla_ext.pack(side="left", expand=True, fill="both",
                              padx=(4, 0))
        tk.Label(caja, text="%", bg=FONDO, fg=VERDE_MED,
                 font=fuente(10, True)).pack(side="left", padx=(0, 4))
        self.casilla_ext.bind("<Return>", lambda e: self._enviar_extension())

        self.btn_ext = tk.Button(fila, text="ENVIAR\nEXTENSION", bg=FONDO,
                                 fg=VERDE_OSC, activebackground=AMBAR,
                                 activeforeground=FONDO,
                                 disabledforeground=VERDE_OSC, relief="solid",
                                 bd=1, highlightthickness=0,
                                 font=fuente(8, True), cursor="hand2",
                                 command=self._enviar_extension)
        self.btn_ext.grid(row=0, column=3, sticky="nsew", padx=3)

        # Lo que dice el cohete de las pruebas de aerofreno
        self.lbl_aero = tk.Label(self, text="", bg=FONDO, fg=VERDE_MED,
                                 font=fuente(8), anchor="w")
        self.lbl_aero.pack(fill="x", padx=8, pady=(3, 0))
        self._aviso_ext = ""

    def _enviar_extension(self):
        if not self._habilitado:
            return
        texto = self.var_ext.get().strip().replace(",", ".")
        try:
            valor = float(texto)
        except ValueError:
            self._aviso_ext = "'%s' no es un numero" % texto
            return
        if not 0.0 <= valor <= 100.0:
            self._aviso_ext = "la extension va de 0 a 100"
            return
        pct = int(round(valor))
        self.var_ext.set(str(pct))
        self._aviso_ext = ""
        self._extension_aerofreno(pct)

    # --- Disparo ---

    def _pulsar(self, nombre):
        if not self._habilitado:
            return
        if nombre in self._armado:
            self.after_cancel(self._armado.pop(nombre))
            self._disparar(nombre)
        else:
            # Armar uno desarma los demás: sólo puede haber uno a punto
            for otro in list(self._armado):
                self.after_cancel(self._armado.pop(otro))
            self._armado[nombre] = self.after(
                TIEMPO_CONFIRMAR_MS, lambda: self._desarmar(nombre))
        self._pintar_botones(self._ultimo_p)

    def _desarmar(self, nombre):
        self._armado.pop(nombre, None)

    def _disparar(self, nombre):
        if nombre == SIMULAR:
            self._enviado[nombre] = time.monotonic()
            self._simular()
            return
        if nombre == AEROFRENO:
            self._enviado[nombre] = time.monotonic()
            self._probar_aerofreno()
            return
        canal = pq.CANAL_PIRO[nombre]
        self._enviado[nombre] = time.monotonic()
        self._rafaga(pq.UPLINK_CMD_PIRO, canal, RAFAGA_DISPARO,
                     ESPACIO_RAFAGA_MS, "DISPARO %s" % nombre)

    # --- Refresco ---

    def actualizar(self, estado):
        s = estado.estado_paquete
        confirmado = estado.modo_test_confirmado
        pedido = estado.modo_test_pedido
        self._habilitado = confirmado and pedido

        if not self._habilitado:
            for n in list(self._armado):
                self.after_cancel(self._armado.pop(n))

        if confirmado and pedido:
            self.lbl_estado.configure(
                text="COHETE EN MODO TEST   quedan %d s" % s.test_restante_s,
                fg=ROJO)
        elif pedido and s is not None and not s.admite_modo_test:
            # El cohete esta donde no admite modo test: decirlo, en vez de
            # dejar al operador esperando una confirmacion que no va a llegar
            self.lbl_estado.configure(
                text="DENEGADO: cohete en %s / %s, solo en tierra"
                     % (pq.ESTADOS.get(s.estado, "?"),
                        pq.FASES.get(s.fase, "?")), fg=ROJO)
        elif pedido:
            self.lbl_estado.configure(
                text="esperando confirmacion... intento %d"
                     % estado.intentos_test, fg=AMBAR)
        elif confirmado:
            self.lbl_estado.configure(
                text="saliendo del modo test... intento %d"
                     % estado.intentos_test, fg=AMBAR)
        else:
            self.lbl_estado.configure(text="", fg=AMBAR)

        if s is not None and s.version >= 2:
            ultimo = pq.NOMBRE_CANAL.get(s.ultimo_piro, "ninguno")
            self.lbl_info.configure(
                text="aceptados por el cohete %d   ultimo %s   denegados %d   "
                     "enviados desde aqui %d"
                     % (s.piros_aceptados, ultimo, s.comandos_denegados,
                        estado.disparos_enviados), fg=VERDE_MED)
        elif s is not None:
            self.lbl_info.configure(
                text="el cohete manda el paquete de estado v%d: no sabe de "
                     "modo test, hay que regrabarlo" % s.version, fg=ROJO)
        else:
            self.lbl_info.configure(text="sin paquete de estado todavia",
                                    fg=VERDE_MED)

        self._pintar_aerofreno(estado)
        self._ultimo_p = estado.paquete
        self._pintar_botones(self._ultimo_p)

    def _pintar_aerofreno(self, e):
        estado = "normal" if self._habilitado else "disabled"
        self.casilla_ext.configure(state=estado)
        self.btn_ext.configure(state=estado,
                               fg=AMBAR if self._habilitado else VERDE_OSC)

        partes = []
        color = VERDE_MED
        pe = e.aero_prueba_estado
        if pe == "enviando":
            partes.append("prueba #%d: esperando al cohete... intento %d"
                          % (e.aero_prueba_num, e.intentos_aero_prueba))
            color = AMBAR
        elif pe == "en curso":
            quedan = max(0.0, pq.PRUEBA_AEROFRENO_S -
                         (time.monotonic() - e.aero_prueba_t))
            partes.append("prueba #%d EN CURSO, quedan %.0f s"
                          % (e.aero_prueba_num, quedan))
            color = AMBAR
        elif pe == "terminada":
            partes.append("prueba #%d terminada" % e.aero_prueba_num)
        elif pe == "denegada":
            partes.append("prueba DENEGADA: %s" % e.aero_prueba_motivo)
            color = ROJO
        elif pe == "sin respuesta":
            partes.append("prueba: el cohete no contesta")
            color = ROJO

        xe = e.aero_ext_estado
        if self._aviso_ext:
            partes.append(self._aviso_ext)
            color = ROJO
        elif xe == "enviando":
            partes.append("extension %d %%: esperando... intento %d"
                          % (e.aero_ext_pedida, e.intentos_aero_ext))
            color = AMBAR if color != ROJO else color
        elif xe == "confirmada":
            partes.append("extension %d %% confirmada" % e.aero_ext_pedida)
        elif xe == "denegada":
            partes.append("extension DENEGADA: %s" % e.aero_ext_motivo)
            color = ROJO
        elif xe == "sin respuesta":
            partes.append("extension: el cohete no contesta")
            color = ROJO

        p = e.paquete
        if p is not None:
            partes.append("aerofreno ahora %.0f %%" % p.aerofreno_pct)
        self.lbl_aero.configure(text="AEROFRENO  " + "   |   ".join(partes),
                                fg=color)

    def _pintar_botones(self, p):
        ahora = time.monotonic()
        for i, (nombre, b) in enumerate(self._botones.items()):
            if nombre == SIMULAR:
                self._pintar_simular(b, ahora)
                continue
            if nombre == AEROFRENO:
                self._pintar_probar_aero(b, ahora)
                continue
            continuidad = None if p is None else p.pirotecnia[i]
            linea = ("--" if continuidad is None else
                     "CONTINUIDAD" if continuidad else "sin continuidad")

            if nombre in self._armado:
                b.configure(text="CONFIRMAR\n%s" % nombre, bg=ROJO, fg=FONDO,
                            state="normal")
            elif ahora - self._enviado.get(nombre, -99.0) < TIEMPO_ENVIADO_S:
                b.configure(text="ENVIADO\n%s" % nombre, bg=FONDO, fg=AMBAR,
                            state="normal" if self._habilitado else "disabled")
            elif self._habilitado:
                b.configure(text="%s\n%s" % (nombre, linea), bg=FONDO,
                            fg=VERDE if continuidad else VERDE_MED,
                            state="normal")
            else:
                b.configure(text="%s\n%s" % (nombre, linea), bg=FONDO,
                            fg=VERDE_OSC, state="disabled")

    def _pintar_probar_aero(self, b, ahora):
        if AEROFRENO in self._armado:
            b.configure(text="CONFIRMAR:\nMUEVE EL AEROFRENO",
                        bg=AMBAR, fg=FONDO, state="normal")
        elif ahora - self._enviado.get(AEROFRENO, -99.0) < TIEMPO_ENVIADO_S:
            b.configure(text="ENVIADO", bg=FONDO, fg=AMBAR, state="normal")
        elif self._habilitado:
            b.configure(text="PROBAR\nAEROFRENO", bg=FONDO, fg=AMBAR,
                        state="normal")
        else:
            b.configure(text="PROBAR\nAEROFRENO", bg=FONDO, fg=VERDE_OSC,
                        state="disabled")

    def _pintar_simular(self, b, ahora):
        if SIMULAR in self._armado:
            # La simulacion pasa por el flujo de vuelo: ENCIENDE 1A y 1B al
            # llegar a DROGUE y 2A en MAIN, de verdad. Que se lea antes de
            # confirmar.
            b.configure(text="CONFIRMAR: ENCIENDE\n1A 1B 2A DE VERDAD",
                        bg=ROJO, fg=FONDO, state="normal")
        elif ahora - self._enviado.get(SIMULAR, -99.0) < TIEMPO_ENVIADO_S:
            b.configure(text="ENVIADO", bg=FONDO, fg=AMBAR, state="normal")
        elif self._habilitado:
            b.configure(text="SIMULAR\nRUTA", bg=FONDO, fg=CIAN,
                        state="normal")
        else:
            b.configure(text="SIMULAR\nRUTA", bg=FONDO, fg=VERDE_OSC,
                        state="disabled")
