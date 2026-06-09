from rescue_pc_brain import control_config as cfg


class GearboxManager:
    def __init__(self):
        self.current_gear = cfg.DEFAULT_GEAR

        self.last_l1_state = 0
        self.last_r1_state = 0

    def get_gear_limit(self):
        return cfg.GEAR_LIMITS[self.current_gear]

    def shift_up(self):
        if self.current_gear < cfg.MAX_GEAR:
            self.current_gear += 1

    def shift_down(self):
        if self.current_gear > cfg.MIN_GEAR:
            self.current_gear -= 1

    def update(self, controller_state):
        """
        R1 sube caja.
        L1 baja caja.
        """

        l1_pressed = controller_state.l1_pressed
        r1_pressed = controller_state.r1_pressed

        if r1_pressed == 1 and self.last_r1_state == 0:
            self.shift_up()

        if l1_pressed == 1 and self.last_l1_state == 0:
            self.shift_down()

        self.last_l1_state = l1_pressed
        self.last_r1_state = r1_pressed
