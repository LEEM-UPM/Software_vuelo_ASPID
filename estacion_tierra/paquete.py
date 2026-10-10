# -*- coding: utf-8 -*-
"""
Decodificador del paquete de telemetría de vuelo (24 bytes).

Es el espejo EXACTO de core_h7_v0/App/flight_packet.c. El paquete NO lleva
número de versión: si allí cambia una escala y aquí no, la estación de tierra
enseña números creíbles y equivocados sin avisar. Cualquier cambio hay que
hacerlo en los dos sitios a la vez.

CONVENIO DEL CUATERNIÓN, verificado en banco (2026-09-28) contra la salida
Euler del propio modelo: con q = (w, x, y, z) leído como la actitud del cuerpo
respecto a NED, las fórmulas ZYX de abajo reproducen el roll/pitch/yaw que
imprime la consola del core con menos de 0,1 grados de diferencia. No es una
suposición, está medido.

Ejes: X = morro, Y = derecha, Z = abajo. Marco de navegación NED.
"""

import math

BYTES = 24

FASES = {
    0: "IDLE",
    1: "BOOST",
    2: "COAST LIBRE",
    3: "COAST CONTROL",
    4: "APOGEO",
    5: "DROGUE",
    6: "MAIN",
    7: "EN TIERRA",
}

# Estados del cohete del GNC v1.2 (App/gnc.h, GNC_ESTADO_*). OJO: la v1.1
# tenia READY en el 2 y EN VUELO en el 3; ya no hay READY.
ESTADO_POWER_ON = 0
ESTADO_MANTENIMIENTO = 1
ESTADO_EN_VUELO = 2
ESTADO_ERROR = 3
# No es del GNC: la core se ha reiniciado en vuelo y sigue solo con el
# barometro (App/emergencia.h)
ESTADO_EMERGENCIA = 4
ESTADOS = {
    ESTADO_POWER_ON: "POWER ON",
    ESTADO_MANTENIMIENTO: "MANTENIMIENTO",
    ESTADO_EN_VUELO: "EN VUELO",
    ESTADO_ERROR: "ERROR",
    ESTADO_EMERGENCIA: "EMERGENCIA",
}

# Sacados de third_party/aspid_gnc/state_machine/state_machine.c.
ERRORES = {
    0: "",
    1: "NAV INVALIDA DEMASIADO TIEMPO",
    2: "ESTANCAMIENTO",
    3: "ENTRADAS NO FINITAS",
    4: "PARADA ORDENADA DESDE TIERRA",
}

PIROS = ("1A", "1B", "2A", "2B")

# Escalas: las mismas constantes de flight_packet.c
ALT_POR_LSB_M = 0.25
POS_POR_LSB_M = 1.0
VVEL_POR_LSB_MPS = 0.05
HVEL_POR_LSB_MPS = 1.0
ACCEL_POR_LSB_G = 0.25
APOGEO_POR_LSB_M = 25.0
BATERIA_POR_LSB_MV = 80.0
AEROFRENO_FONDO = 250.0


# Bateria LiPo 3S: tension EN REPOSO por celda -> carga, la tabla habitual.
# No es una recta: la curva es muy plana en el medio (3,75 a 3,85 V cubren del
# 25 al 55 %) y cae deprisa al final, asi que una recta entre 3,3 y 4,2 V
# diria "50 %" cuando queda bastante menos.
CELDAS_BATERIA = 3
_LIPO_V_PCT = (
    (3.27, 0), (3.61, 5), (3.69, 10), (3.71, 15), (3.73, 20), (3.75, 25),
    (3.77, 30), (3.79, 35), (3.80, 40), (3.82, 45), (3.84, 50), (3.85, 55),
    (3.87, 60), (3.91, 65), (3.95, 70), (3.98, 75), (4.02, 80), (4.08, 85),
    (4.11, 90), (4.15, 95), (4.20, 100),
)


def porcentaje_lipo(voltios, celdas=CELDAS_BATERIA):
    """
    Carga aproximada de una LiPo a partir de su tension total, 0 a 100.

    Por encima de 4,20 V por celda se queda en 100 y por debajo de 3,27 en 0.
    OJO: la tabla es en REPOSO. Con carga la tension cae (y mas en vuelo, con
    el servo y la radio), asi que en marcha el porcentaje sale por debajo del
    real y se recupera al soltar la carga.
    """
    v = voltios / float(celdas)
    if v <= _LIPO_V_PCT[0][0]:
        return 0.0
    if v >= _LIPO_V_PCT[-1][0]:
        return 100.0
    for (v0, p0), (v1, p1) in zip(_LIPO_V_PCT, _LIPO_V_PCT[1:]):
        if v <= v1:
            return p0 + (p1 - p0) * (v - v0) / (v1 - v0)
    return 100.0


def _i16(b, i):
    v = b[i] | (b[i + 1] << 8)
    return v - 65536 if v > 32767 else v


def _i8(b, i):
    v = b[i]
    return v - 256 if v > 127 else v


class Paquete(object):
    """Un paquete de vuelo ya interpretado, en unidades de ingeniería."""

    __slots__ = (
        "crudo", "contador", "pirotecnia", "fase", "nav_valida", "estado",
        "error", "altura_m", "norte_m", "este_m", "vel_vertical_mps",
        "vel_norte_mps", "vel_este_mps", "accel_g", "quat", "apogeo_m",
        "aerofreno_pct", "bateria_v", "temperatura_c",
    )

    # --- Derivados: se calculan a partir del cuaternión ---

    @property
    def morro_ned(self):
        """Dirección del morro en NED. Norma 1 salvo por la cuantización."""
        w, x, y, z = self.quat
        return (
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y + w * z),
            2.0 * (x * z - w * y),
        )

    @property
    def elevacion_deg(self):
        """
        Elevación del morro sobre la horizontal. Se usa esto y no el cabeceo
        de Euler porque no tiene bloqueo de cardán: a 90 grados sigue siendo
        un número bien definido, y 90 grados es justo la actitud del cohete en
        la rampa.
        """
        d = max(-1.0, min(1.0, self.morro_ned[2]))
        return math.degrees(math.asin(-d))

    @property
    def rumbo_deg(self):
        """Azimut del morro, 0 = norte, creciendo hacia el este."""
        n, e, _ = self.morro_ned
        return math.degrees(math.atan2(e, n)) % 360.0

    @property
    def euler_deg(self):
        """(alabeo, cabeceo, guiñada) en grados, convenio ZYX."""
        w, x, y, z = self.quat
        alabeo = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        s = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
        cabeceo = math.asin(s)
        guinada = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return (math.degrees(alabeo), math.degrees(cabeceo),
                math.degrees(guinada))

    @property
    def bateria_pct(self):
        """
        Carga suponiendo una LiPo 3S (porcentaje_lipo). None si el cohete no
        tiene dato de bateria: el byte 22 va a 0 cuando la power no lo da, y
        eso no es "bateria vacia".
        """
        if self.bateria_v <= 0.0:
            return None
        return porcentaje_lipo(self.bateria_v)

    @property
    def accel_modulo_g(self):
        x, y, z = self.accel_g
        return math.sqrt(x * x + y * y + z * z)

    @property
    def velocidad_horizontal_mps(self):
        return math.hypot(self.vel_norte_mps, self.vel_este_mps)

    @property
    def norma_quat(self):
        """
        Debería valer 1. Se enseña porque es el control de sanidad más barato
        que hay: si se aleja de 1, o el enlace ha corrompido bytes que el CRC
        no pilló o el convenio de escala ya no es el que cree esta estación.
        """
        return math.sqrt(sum(c * c for c in self.quat))

    @property
    def fase_nombre(self):
        return FASES.get(self.fase, "?")

    @property
    def estado_nombre(self):
        return ESTADOS.get(self.estado, "?")

    @property
    def error_nombre(self):
        if self.error == 0:
            return ""
        return ERRORES.get(self.error, "DESCONOCIDO 0x%02X" % self.error)


def decodificar(b):
    """bytes -> Paquete. Devuelve None si la longitud no es la esperada."""
    if b is None or len(b) != BYTES:
        return None

    p = Paquete()
    p.crudo = bytes(b)
    p.contador = b[0]

    p.pirotecnia = tuple(bool(b[1] & (1 << i)) for i in range(4))
    p.fase = (b[1] >> 4) & 0x07
    p.nav_valida = bool(b[1] & 0x80)

    p.estado = b[2] & 0x07
    p.error = (b[2] >> 3) & 0x1F

    p.altura_m = _i16(b, 3) * ALT_POR_LSB_M
    p.norte_m = _i16(b, 5) * POS_POR_LSB_M
    p.este_m = _i16(b, 7) * POS_POR_LSB_M

    p.vel_vertical_mps = _i16(b, 9) * VVEL_POR_LSB_MPS
    p.vel_norte_mps = _i8(b, 11) * HVEL_POR_LSB_MPS
    p.vel_este_mps = _i8(b, 12) * HVEL_POR_LSB_MPS

    p.accel_g = tuple(_i8(b, 13 + i) * ACCEL_POR_LSB_G for i in range(3))
    p.quat = tuple(_i8(b, 16 + i) / 127.0 for i in range(4))

    p.apogeo_m = b[20] * APOGEO_POR_LSB_M
    p.aerofreno_pct = b[21] * 100.0 / AEROFRENO_FONDO
    p.bateria_v = b[22] * BATERIA_POR_LSB_MV / 1000.0
    p.temperatura_c = _i8(b, 23)
    return p


# === Paquete de estado (App/status_packet.h) ==============================

ESTADO_BYTES = 32
ESTADO_MARCA = 0xA5
ESTADO_MB_POR_LSB = 16

SENSORES = ("baro1", "baro2", "imu1", "imu2", "mag", "gps", "vn100")

SD_ESTADOS = {
    0: "sin iniciar",
    1: "no responde",
    2: "no monta",
    3: "montada",
}

PLACAS = ("power", "avionica", "rf")


class PaqueteEstado(object):
    """La salud del cohete, no su vuelo. Llega al arrancar y bajo petición."""

    __slots__ = (
        "crudo", "version", "contador", "placas", "init_valido",
        "calibracion_hecha", "nav_valida", "sensores_ok", "sensores_tiene",
        "sd_estado", "sd_test_ok", "sd_cd", "sd_arranques", "sd_libres_mb",
        "can_hz", "can_errores", "segundos_encendido", "wake_up_reintentos",
        "radio_fallidos", "fase", "estado",
        # Version 2: modo test (App/uplink.h)
        "test_activo", "test_restante_s", "ultimo_piro", "piros_aceptados",
        "comandos_denegados",
        # Version 3: simulacion de ruta y motivo de denegacion
        "simulando", "sim_progreso", "motivo_denegacion",
        # Version 4: despliegue de paracaidas (App/despliegue.h)
        "drogue_mandado", "main_mandado", "despliegue_rechazado",
        # Version 5: camaras (App/camaras.h). None con un cohete anterior
        "camaras",
        # Version 6: reinicio en vuelo (App/emergencia.h)
        "postvuelo", "emergencia", "emer_apogeo", "emer_main",
        "baro_no_fiable", "nav_init", "nav_listo",
    )

    @property
    def motivo_denegacion_texto(self):
        return MOTIVOS_DENEGACION.get(self.motivo_denegacion,
                                      "motivo %d" % self.motivo_denegacion)

    def sensor(self, nombre):
        """
        Devuelve 'ok', 'fallo' o 'no lleva' para ese sensor.

        Los tres casos son distintos y hacen falta los tres: un bit a 0 en
        'ok' puede ser que el sensor falló o que la placa no lo monta, y sin
        el mapa de lo que lleva se acaba persiguiendo un fallo que no existe.
        """
        if nombre not in SENSORES:
            return "no lleva"
        i = SENSORES.index(nombre)
        if not (self.sensores_tiene & (1 << i)):
            return "no lleva"
        return "ok" if (self.sensores_ok & (1 << i)) else "fallo"

    @property
    def sensores_en_fallo(self):
        return [n for n in SENSORES if self.sensor(n) == "fallo"]

    @property
    def placas_ausentes(self):
        return [n for i, n in enumerate(PLACAS)
                if not (self.placas & (1 << i))]

    def sd_nombre(self, i):
        return SD_ESTADOS.get(self.sd_estado[i], "?")

    @property
    def admite_modo_test(self):
        """
        Espejo de _en_tierra() en App/uplink.c: estado POWER ON, MANTENIMIENTO
        o ERROR, y fase IDLE o EN TIERRA. Solo sirve para EXPLICAR en pantalla
        por que se deniega; quien decide es el cohete, no esto.
        """
        return (self.estado in (ESTADO_POWER_ON, ESTADO_MANTENIMIENTO,
                                ESTADO_ERROR) and self.fase in (0, 7))

    @property
    def todo_bien(self):
        """
        Resumen honesto para el semáforo. No mira las tarjetas ni los buses
        con criterios finos: si falta una placa, falla un sensor que la placa
        sí lleva, o una SD no está montada, hay algo que mirar.
        """
        if self.placas != 0x07:
            return False
        if self.sensores_en_fallo:
            return False
        if self.sd_estado[0] != 3 or self.sd_estado[1] != 3:
            return False
        return True


def decodificar_estado(b):
    """bytes -> PaqueteEstado, o None si no lo es."""
    if b is None or len(b) != ESTADO_BYTES or b[0] != ESTADO_MARCA:
        return None

    p = PaqueteEstado()
    p.crudo = bytes(b)
    p.version = b[1]
    p.contador = b[2]

    p.placas = b[3] & 0x07
    p.init_valido = (b[3] >> 3) & 0x07   # b0 avionica, b1 power, b2 rf
    p.calibracion_hecha = bool(b[3] & 0x40)
    p.nav_valida = bool(b[3] & 0x80)

    p.sensores_ok = b[4] & 0x7F
    p.sensores_tiene = b[5] & 0x7F

    p.sd_estado = (b[6] & 0x03, (b[6] >> 2) & 0x03)
    p.sd_test_ok = (bool(b[6] & 0x10), bool(b[6] & 0x20))
    p.sd_cd = (bool(b[6] & 0x40), bool(b[6] & 0x80))
    p.sd_arranques = (b[7], b[8])
    p.sd_libres_mb = ((b[9] | (b[10] << 8)) * ESTADO_MB_POR_LSB,
                      (b[11] | (b[12] << 8)) * ESTADO_MB_POR_LSB)

    p.can_hz = (b[13] | (b[14] << 8), b[15] | (b[16] << 8))
    p.can_errores = (b[17], b[18])

    p.segundos_encendido = b[19] | (b[20] << 8)
    p.wake_up_reintentos = b[21]
    p.radio_fallidos = b[22] | (b[23] << 8)

    p.fase = b[24] & 0x07
    p.estado = (b[24] >> 3) & 0x07

    # Version 2. En la 1 estos bytes eran reservados a cero, asi que un cohete
    # antiguo sale como "modo test inactivo", que es lo correcto.
    p.test_activo = bool(b[25] & 0x01)
    p.ultimo_piro = (b[25] >> 1) & 0x07
    p.test_restante_s = b[26]
    p.piros_aceptados = b[27]
    p.comandos_denegados = b[28]

    # Version 3. Con un cohete anterior salen a cero: "no simula".
    p.simulando = bool(b[25] & 0x10)
    p.sim_progreso = b[29]
    p.motivo_denegacion = b[30]

    # Version 4. Se olvida en cada vuelo o simulacion nueva.
    p.drogue_mandado = bool(b[31] & 0x01)
    p.main_mandado = bool(b[31] & 0x02)
    p.despliegue_rechazado = bool(b[31] & 0x04)

    # Version 5. Antes no lo decia: None = no se sabe, no "apagadas".
    p.camaras = bool(b[31] & 0x08) if p.version >= 5 else None

    # Version 6: reiniciado en vuelo (modo de emergencia, solo barometro) o
    # tras el vuelo (POSTVUELO: no toca la SD ni enciende las camaras)
    v6 = p.version >= 6
    p.postvuelo = v6 and bool(b[25] & 0x20)
    p.emergencia = v6 and bool(b[31] & 0x10)
    p.emer_apogeo = v6 and bool(b[31] & 0x20)
    p.emer_main = v6 and bool(b[31] & 0x40)
    p.baro_no_fiable = v6 and bool(b[31] & 0x80)

    # Version 7 (GNC v1.6): nav_init_flags, lo que tiene el filtro de la
    # rampa. None con un cohete anterior: no se sabe.
    if p.version >= 7:
        p.nav_init = ((0x01 if b[24] & 0x40 else 0) |
                      (0x02 if b[24] & 0x80 else 0) |
                      (0x04 if b[25] & 0x40 else 0) |
                      (0x08 if b[25] & 0x80 else 0))
        # Listo para volar: las tres referencias (alineado, 0 AGL, origen
        # GNSS). Es lo que hay que ver antes de armar el iniciador.
        p.nav_listo = (p.nav_init & 0x07) == 0x07
    else:
        p.nav_init = None
        p.nav_listo = None
    return p


NAV_INIT_FALTA = ((0x01, "ALINEADO"), (0x02, "0 AGL"), (0x04, "GNSS"))


def nav_init_falta(flags):
    """Lo que le falta al GNC para estar listo, para pintarlo."""
    return [n for b, n in NAV_INIT_FALTA if not (flags & b)]


# === Acuse (App/uplink.h) =================================================

# 12 bytes desde el 2026-10-04 (los 4 ultimos: como oyo el cohete el
# comando). Se aceptan tambien los de 8 de un cohete con firmware anterior.
ACUSE_BYTES = 12
ACUSE_BYTES_ANTES = 8
ACUSE_MARCA = 0xC3


class PaqueteAcuse(object):
    """
    Respuesta del cohete a un comando concreto. Lleva el valor que el cohete
    tiene AHORA, no el que se pidio: confirmar es comprobar que coinciden.
    """

    __slots__ = ("crudo", "cmd", "resultado", "valor", "contador",
                 "definido", "guardado", "rx_valida", "rx_rssi_dbm",
                 "rx_snr_db", "rx_df_hz")

    @property
    def aceptado(self):
        return self.resultado == 0

    @property
    def motivo_texto(self):
        return MOTIVOS_DENEGACION.get(self.resultado,
                                      "motivo %d" % self.resultado)


def decodificar_acuse(b):
    if b is None or len(b) not in (ACUSE_BYTES, ACUSE_BYTES_ANTES) or \
            b[0] != ACUSE_MARCA:
        return None
    a = PaqueteAcuse()
    a.crudo = bytes(b)
    a.cmd = b[1]
    a.resultado = b[2]
    a.valor = b[3] | (b[4] << 8)
    a.contador = b[5]
    a.definido = bool(b[6] & 0x01)
    a.guardado = bool(b[6] & 0x02)
    # Como oyo la radio del COHETE este comando (App/uplink.h). Solo si el
    # acuse es de 12 bytes y el bit 0 del byte 7 dice que la RF lo mando.
    a.rx_valida = len(b) == ACUSE_BYTES and bool(b[7] & 0x01)
    a.rx_rssi_dbm = _i8(b, 8) if a.rx_valida else None
    a.rx_snr_db = _i8(b, 9) if a.rx_valida else None
    a.rx_df_hz = _i16(b, 10) if a.rx_valida else None
    return a


# === Camaras (App/camaras.h y App/camara.h) ==============================

CAMARAS_BYTES = 32
CAMARAS_MARCA = 0xCA
CAMARAS_MB_POR_LSB = 16
CAMARA_BYTES = 14

CAMARA_SIN_CORRIENTE = 0
CAMARA_ARRANCANDO = 1
CAMARA_NO_CONTESTA = 2
CAMARA_COMPROBANDO = 3
CAMARA_GRABANDO = 4
CAMARA_PARADA = 5
CAMARA_PARANDO = 6
ESTADOS_CAMARA = {
    CAMARA_SIN_CORRIENTE: "SIN CORRIENTE",
    CAMARA_ARRANCANDO: "ARRANCANDO",
    CAMARA_NO_CONTESTA: "NO CONTESTA",
    CAMARA_COMPROBANDO: "COMPROBANDO",
    CAMARA_GRABANDO: "GRABANDO",
    CAMARA_PARADA: "PARADA",
    CAMARA_PARANDO: "PARANDO",
}

# camaras_origen_t
ORIGENES_CAMARAS = {
    0: "desde el arranque",
    1: "por el despegue",
    2: "desde tierra",
    3: "al terminar el vuelo",
}

# Valor del acuse de UPLINK_CMD_CAMARAS
ACUSE_CAMARAS_ENCENDIDAS = 0x01
ACUSE_CAMARAS_APAGANDO = 0x02


class Camara(object):
    """Una camara: si graba (lo deduce el cohete del espacio libre de su
    tarjeta), con que graba y su tarjeta."""

    __slots__ = ("estado", "quiere_grabar", "no_graba", "reloj_puesto",
                 "sd_leida", "pal", "ajustes_leidos", "sin_tarjeta",
                 "lineas", "fps", "libre_mb", "total_mb", "queda_s",
                 "grabando_s", "pulsaciones")

    @property
    def estado_nombre(self):
        if self.no_graba:
            return "NO GRABA"
        return ESTADOS_CAMARA.get(self.estado, "estado %d" % self.estado)

    @property
    def grabando(self):
        return self.estado in (CAMARA_GRABANDO, CAMARA_PARANDO)

    @property
    def resolucion(self):
        """'1080p60 NTSC', o None si no se ha leido en este encendido."""
        if not self.ajustes_leidos or not self.lineas:
            return None
        return "%dp%d %s" % (self.lineas, self.fps,
                             "PAL" if self.pal else "NTSC")

    @property
    def mal(self):
        """Algo que mirar: no graba queriendo, o con corriente y mudo."""
        return self.no_graba or self.sin_tarjeta or \
            self.estado == CAMARA_NO_CONTESTA


class PaqueteCamaras(object):
    """Las dos camaras. Llega cada 5 s con ellas encendidas, en cada cambio,
    al arrancar y cuando se pide el estado."""

    __slots__ = ("crudo", "version", "contador", "alimentacion", "apagando",
                 "origen", "en_vuelo", "hora_gps", "ultimo_por_plazo",
                 "camaras")

    @property
    def origen_texto(self):
        return ORIGENES_CAMARAS.get(self.origen, "?")

    @property
    def grabando(self):
        return sum(1 for c in self.camaras if c.grabando)


def _u16(b, i):
    return b[i] | (b[i + 1] << 8)


def decodificar_camaras(b):
    if b is None or len(b) != CAMARAS_BYTES or b[0] != CAMARAS_MARCA:
        return None
    p = PaqueteCamaras()
    p.crudo = bytes(b)
    p.version = b[1]
    p.contador = b[2]
    p.alimentacion = bool(b[3] & 0x01)
    p.apagando = bool(b[3] & 0x02)
    p.origen = (b[3] >> 2) & 0x03
    p.en_vuelo = bool(b[3] & 0x10)
    p.hora_gps = bool(b[3] & 0x20)
    p.ultimo_por_plazo = bool(b[3] & 0x40)
    p.camaras = []
    for i in range(2):
        o = 4 + CAMARA_BYTES * i
        c = Camara()
        c.estado = b[o]
        f = b[o + 1]
        c.quiere_grabar = bool(f & 0x01)
        c.no_graba = bool(f & 0x02)
        c.reloj_puesto = bool(f & 0x04)
        c.sd_leida = bool(f & 0x08)
        c.pal = bool(f & 0x10)
        c.ajustes_leidos = bool(f & 0x20)
        c.sin_tarjeta = bool(f & 0x40)
        c.lineas = _u16(b, o + 2)
        c.fps = b[o + 4]
        c.libre_mb = _u16(b, o + 5) * CAMARAS_MB_POR_LSB
        c.total_mb = _u16(b, o + 7) * CAMARAS_MB_POR_LSB
        c.queda_s = _u16(b, o + 9)
        c.grabando_s = _u16(b, o + 11)
        c.pulsaciones = b[o + 13]
        p.camaras.append(c)
    p.camaras = tuple(p.camaras)
    return p


def decodificar_cualquiera(b):
    """
    Reparte un payload al decodificador que toque.
    Devuelve ("vuelo"|"estado"|"camaras"|"acuse"|None, objeto o None).

    El de cámaras mide 32 como el de estado: se distinguen por la marca
    (0xA5 / 0xCA), que en los dos es fija.

    OJO CON EL ORDEN Y LAS CONDICIONES. El paquete de vuelo se reconoce por
    MEDIR 24 BYTES, no por su primer byte: ese byte es un contador y vale
    0xA5 una vez cada 256 paquetes. Si se mirara primero la marca, uno de
    cada 256 paquetes de vuelo se leería como si fuera de estado.
    """
    if b is None:
        return (None, None)
    if len(b) == BYTES:
        return ("vuelo", decodificar(b))
    if len(b) == ESTADO_BYTES and b[0] == ESTADO_MARCA:
        return ("estado", decodificar_estado(b))
    if len(b) == CAMARAS_BYTES and b[0] == CAMARAS_MARCA:
        return ("camaras", decodificar_camaras(b))
    if len(b) in (ACUSE_BYTES, ACUSE_BYTES_ANTES) and b[0] == ACUSE_MARCA:
        return ("acuse", decodificar_acuse(b))
    return (None, None)


# === Comandos de subida (App/uplink.h) ====================================

UPLINK_MARCA = 0x5A
UPLINK_CMD_PEDIR_ESTADO = 0x01
UPLINK_CMD_PING = 0x02        # 16 bits: numero de ping, el acuse lo devuelve
UPLINK_CMD_MODO_TEST = 0x10   # parametro: 1 entrar/renovar, 0 salir
UPLINK_CMD_PIRO = 0x11        # parametro: canal, ver CANAL_PIRO
UPLINK_CMD_SIMULACION = 0x12  # parametro: 1 empezar, 0 parar
UPLINK_CMD_ANGULO = 0x13      # 16 bits: centesimas de grado desde la vertical
UPLINK_CMD_PRUEBA_AEROFRENO = 0x14     # numero de prueba 1..255; 0 la para
UPLINK_CMD_EXTENSION_AEROFRENO = 0x15  # 0..100 % de la carrera
UPLINK_CMD_RUMBO = 0x16       # 16 bits: rumbo de la rampa, centesimas de grado
UPLINK_CMD_CAMARAS = 0x17     # 1 encender (siempre), 0 apagar (solo en tierra;
                              # primero paran de grabar, luego se cortan)
                              # desde el norte geografico, horario, 0..35999

# Duracion de la prueba de aerofreno en el cohete (App/airbrake.h): 3 veces
# fuera/dentro de 0,5 s y rampas de 2 s
PRUEBA_AEROFRENO_S = 7.0

# Comandos con parametro de 16 bits: van en el formato de 5 bytes
COMANDOS_16 = (UPLINK_CMD_ANGULO, UPLINK_CMD_PING, UPLINK_CMD_RUMBO)

# Angulo: valor especial que solo pide el acuse con el valor del cohete
ANGULO_CONSULTA = 0xFFFF
ANGULO_MAX_GRADOS = 90.0
RUMBO_MAX_GRADOS = 359.99

UPLINK_NOMBRES = {
    UPLINK_CMD_PEDIR_ESTADO: "pedir estado",
    UPLINK_CMD_PING: "ping",
    UPLINK_CMD_MODO_TEST: "modo test",
    UPLINK_CMD_PIRO: "pirotecnico",
    UPLINK_CMD_SIMULACION: "simular ruta",
    UPLINK_CMD_ANGULO: "angulo de lanzamiento",
    UPLINK_CMD_PRUEBA_AEROFRENO: "prueba de aerofreno",
    UPLINK_CMD_EXTENSION_AEROFRENO: "extension de aerofreno",
    UPLINK_CMD_RUMBO: "rumbo de la rampa",
    UPLINK_CMD_CAMARAS: "camaras",
}

# uplink_denegado_t de App/uplink.h, en el mismo orden
MOTIVOS_DENEGACION = {
    0: "",
    1: "el cohete no esta en tierra",
    2: "no esta en modo test",
    3: "canal de pirotecnico invalido",
    4: "power no responde",
    5: "esta simulando una ruta",
    6: "no hay VUELO.BIN en la SD1",
    7: "valor fuera de rango (angulo 0-90, rumbo 0-359,99, extension 0-100)",
}

# Canal de CMD_PYRO_FIRE (ICD 6.3): ojo, empieza en 1, no en 0
CANAL_PIRO = {"1A": 1, "1B": 2, "2A": 3, "2B": 4}
NOMBRE_CANAL = {v: k for k, v in CANAL_PIRO.items()}


def comando(cmd_id, parametro=0):
    """
    Monta un comando de 4 bytes para el cohete: marca, id, parámetro y XOR.

    El XOR va ADEMÁS del CRC de LoRa porque son cosas distintas: el CRC dice
    que los bits llegaron como salieron, no que lo que salió fuera un comando
    nuestro. En el aire hay más cosas, y la marca más el XOR hacen falsísimo
    que un paquete ajeno se ejecute como si fuera del cohete.
    """
    cmd_id &= 0xFF
    parametro &= 0xFF
    return bytes([UPLINK_MARCA, cmd_id, parametro,
                  UPLINK_MARCA ^ cmd_id ^ parametro])


def comando16(cmd_id, valor):
    """Comando de 5 bytes con parametro de 16 bits (App/uplink.h)."""
    cmd_id &= 0xFF
    lo, hi = valor & 0xFF, (valor >> 8) & 0xFF
    return bytes([UPLINK_MARCA, cmd_id, lo, hi,
                  UPLINK_MARCA ^ cmd_id ^ lo ^ hi])


def comando_para(cmd_id, parametro=0):
    """El formato que toque segun el comando."""
    if cmd_id in COMANDOS_16:
        return comando16(cmd_id, parametro)
    return comando(cmd_id, parametro)
