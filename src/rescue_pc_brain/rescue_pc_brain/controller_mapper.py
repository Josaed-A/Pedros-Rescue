from dataclasses import dataclass

from rescue_pc_brain import control_config as cfg


@dataclass
class ControllerState:
    joystick_x: float
    joystick_y: float
    r2_value: float

    l1_pressed: int
    l2_pressed: int

    x_pressed: int
    circle_pressed: int
    triangle_pressed: int
    share_pressed: int


class ControllerMapper:
    def clamp(self, value, min_value=0.0, max_value=1.0):
        if value > max_value:
            return max_value

        if value < min_value:
            return min_value

        return value

    def normalize_trigger(self, raw_value):
        released = cfg.R2_RELEASED_VALUE
        pressed = cfg.R2_PRESSED_VALUE

        value = (raw_value - released) / (pressed - released)

        return self.clamp(value, 0.0, 1.0)

    def from_joy_msg(self, msg):
        joystick_x = msg.axes[cfg.AXIS_LEFT_X] * cfg.STEER_MULTIPLIER
        joystick_y = msg.axes[cfg.AXIS_LEFT_Y] * cfg.JOYSTICK_Y_MULTIPLIER

        r2_raw = msg.axes[cfg.AXIS_R2]
        r2_value = self.normalize_trigger(r2_raw)

        return ControllerState(
            joystick_x=joystick_x,
            joystick_y=joystick_y,
            r2_value=r2_value,

            l1_pressed=msg.buttons[cfg.BUTTON_L1],
            l2_pressed=msg.buttons[cfg.BUTTON_L2_FULL],

            x_pressed=msg.buttons[cfg.BUTTON_X],
            circle_pressed=msg.buttons[cfg.BUTTON_CIRCLE],
            triangle_pressed=msg.buttons[cfg.BUTTON_TRIANGLE],
            share_pressed=msg.buttons[cfg.BUTTON_SHARE],
        )