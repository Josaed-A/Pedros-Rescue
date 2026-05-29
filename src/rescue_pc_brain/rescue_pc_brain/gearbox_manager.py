import time

from rescue_pc_brain import control_config as cfg


class GearboxManager:
    def __init__(self):
        self.current_gear = cfg.DEFAULT_GEAR
        self.direction = cfg.DEFAULT_DIRECTION

        self.last_triangle_state = 0
        self.last_circle_state = 0
        self.last_x_state = 0

        self.low_speed_since = None

    def get_gear_limit(self):
        return cfg.GEAR_LIMITS[self.current_gear]

    def shift_up(self):
        if self.current_gear < cfg.MAX_GEAR:
            self.current_gear += 1

    def shift_down(self):
        if self.current_gear > cfg.MIN_GEAR:
            self.current_gear -= 1

    def can_toggle_direction(self, real_speed_abs):
        return real_speed_abs <= cfg.REAL_SPEED_ZERO_THRESHOLD

    def toggle_direction(self):
        if self.direction == cfg.DIRECTION_FORWARD:
            self.direction = cfg.DIRECTION_REVERSE
        else:
            self.direction = cfg.DIRECTION_FORWARD

    def direction_sign(self):
        if self.direction == cfg.DIRECTION_FORWARD:
            return 1.0

        return -1.0

    def handle_manual_buttons(self, controller_state, real_speed_abs):
        """
        L2 + Triángulo = subir caja
        L2 + Círculo   = bajar caja
        L2 + X         = cambiar FORWARD/REVERSE, solo si velocidad real es cero
        """

        l2_active = controller_state.l2_pressed == 1

        triangle_pressed = controller_state.triangle_pressed
        circle_pressed = controller_state.circle_pressed
        x_pressed = controller_state.x_pressed

        if l2_active:
            if triangle_pressed == 1 and self.last_triangle_state == 0:
                self.shift_up()

            if circle_pressed == 1 and self.last_circle_state == 0:
                self.shift_down()

            if x_pressed == 1 and self.last_x_state == 0:
                if self.can_toggle_direction(real_speed_abs):
                    self.toggle_direction()

        self.last_triangle_state = triangle_pressed
        self.last_circle_state = circle_pressed
        self.last_x_state = x_pressed

    def handle_auto_downshift(self, real_speed_abs):
        """
        Descenso automático por velocidad real.

        Si la velocidad real cae por debajo del umbral durante T segundos,
        baja una caja automáticamente.

        Si la velocidad real vuelve a subir, se cancela el conteo.
        """

        now = time.monotonic()

        if real_speed_abs > cfg.AUTO_DOWNSHIFT_REAL_SPEED_THRESHOLD:
            self.low_speed_since = None
            return

        if self.current_gear <= cfg.MIN_GEAR:
            self.low_speed_since = None
            return

        if self.low_speed_since is None:
            self.low_speed_since = now
            return

        elapsed = now - self.low_speed_since

        if elapsed >= cfg.AUTO_DOWNSHIFT_DELAY_SECONDS:
            self.shift_down()
            self.low_speed_since = now

    def update(self, controller_state, real_speed_abs):
        """
        Actualiza cajas y dirección.

        real_speed_abs viene desde la Raspberry por /real_speed_abs.
        """

        self.handle_manual_buttons(
            controller_state,
            real_speed_abs
        )

        self.handle_auto_downshift(real_speed_abs)