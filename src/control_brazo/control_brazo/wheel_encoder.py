"""
WheelEncoder
============
Trackea la posicion real del eje de salida de un servo Dynamixel
con reductora, resolviendo el wrap-around de los 300° del sensor.

Responsabilidades:
  - Acumular vueltas completas detectando wraps en el rango 0-1023
  - Aplicar filtro de mediana (N=3) para suavizar ruido
  - Rechazar lecturas espurias (delta > MAX_DELTA_TICK)
  - Calcular angulo_salida = (angulo_motor / reduccion) - offset
  - Proveer error circular para el controlador de posicion
"""

import collections
from typing import Optional

# Constantes del protocolo Dynamixel AX/EX modo rueda
POS_MAX        = 1023
POS_RANGE      = 300.0   # grados mecanicos del servo
WRAP_THRESH    = 512
MEDIAN_N       = 3
# Debe admitir el delta real a velocidad maxima entre lecturas (~120 ticks a
# 10 Hz) pero seguir por debajo de WRAP_THRESH; 80 era tan bajo que rechazaba
# lecturas legitimas y luego confundia el acumulado con un wrap.
MAX_DELTA_TICK = 300


class WheelEncoder:
    def __init__(self, reduccion: float = 1.0):
        self.reduccion     = max(0.01, reduccion)
        self._prev         = -1
        self._vueltas      = 0
        self._offset_deg   = 0.0
        self._median_buf   = collections.deque(maxlen=MEDIAN_N)
        self._rejected     = 0
        self.angulo_motor  = 0.0
        self.angulo_salida = 0.0
        # Igual que angulo_salida pero SIN envolver a 360° (multivuelta).
        # Lo usan las patas para mostrar grados acumulados (pueden pasar de 360).
        self.angulo_salida_acum = 0.0
        self.lectura_ok    = True
        self.pos_filt      = 0
        self.target_deg    : Optional[float] = None
        self.vel_pct       = 30
        self.deadband_deg  = 0.5
        self.invertir_giro = False
        self._watch_target : Optional[float] = None
        self._watch_best   = 0.0
        self._watch_t0     = 0.0

    # ------------------------------------------------------------------
    #  Control de ciclo de vida
    # ------------------------------------------------------------------

    def reset_full(self):
        """Reinicia todo el estado. Llamar al (re)conectar."""
        self._prev         = -1
        self._vueltas      = 0
        self._offset_deg   = 0.0
        self._median_buf.clear()
        self._rejected     = 0
        self.angulo_motor  = 0.0
        self.angulo_salida = 0.0
        self.lectura_ok    = True
        self.target_deg    = None
        self._watch_target = None

    def set_zero(self):
        """Define la posicion actual como 0° (calibracion de home)."""
        self._offset_deg   = self.angulo_motor / self.reduccion
        self.angulo_salida = 0.0
        self.angulo_salida_acum = 0.0

    # ------------------------------------------------------------------
    #  Actualizacion de posicion (llamar cada ciclo del driver)
    # ------------------------------------------------------------------

    def update(self, pos_raw: int):
        """
        Recibe un tick crudo del registro PRESENT_POSITION del servo.
        Actualiza angulo_motor y angulo_salida.
        """
        self._median_buf.append(pos_raw)
        pos = self._median3(self._median_buf)
        if pos is None:
            return

        self.pos_filt = pos

        if self._prev == -1:
            self._prev = pos
            return

        delta     = pos - self._prev
        abs_delta = abs(delta)
        circular_delta = min(abs_delta, POS_MAX + 1 - abs_delta)

        if circular_delta > MAX_DELTA_TICK:
            self._rejected += 1
            if self._rejected < 5:
                self.lectura_ok = False
                return

        self._rejected  = 0
        self.lectura_ok = True

        if delta < -WRAP_THRESH:
            self._vueltas += 1
        elif delta > WRAP_THRESH:
            self._vueltas -= 1

        self._prev        = pos
        self.angulo_motor = (self._vueltas * POS_RANGE
                             + (pos / POS_MAX) * POS_RANGE)
        raw_salida        = self.angulo_motor / self.reduccion
        self.angulo_salida_acum = raw_salida - self._offset_deg
        self.angulo_salida = self.angulo_salida_acum % 360.0

    # ------------------------------------------------------------------
    #  Utilidades
    # ------------------------------------------------------------------

    @staticmethod
    def _median3(buf):
        n = len(buf)
        if n == 0: return None
        if n < 3:  return buf[-1]
        a, b, c = buf[0], buf[1], buf[2]
        if a <= b <= c or c <= b <= a: return b
        if b <= a <= c or c <= a <= b: return a
        return c

    @staticmethod
    def error_circular(target_deg: float, actual_deg: float) -> float:
        """Calcula el error de posicion en [-180, 180]."""
        return ((target_deg - actual_deg + 180.0) % 360.0) - 180.0

    def control_estancado(self, error_abs: float, now: float,
                          umbral_deg: float = 0.5,
                          timeout_s: float = 3.0) -> bool:
        """
        Watchdog de progreso: True si el error no mejora hace timeout_s.
        Protege contra perdida de tracking (el servo giraria sin parar).
        """
        if self.target_deg != self._watch_target:
            self._watch_target = self.target_deg
            self._watch_best   = error_abs
            self._watch_t0     = now
            return False
        if error_abs < self._watch_best - umbral_deg:
            self._watch_best = error_abs
            self._watch_t0   = now
            return False
        return (now - self._watch_t0) > timeout_s
