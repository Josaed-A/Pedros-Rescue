from dataclasses import dataclass

from rescue_pc_brain import control_config as cfg


@dataclass
class ControllerState:
    joystick_x: float
    joystick_y: float

    l1_pressed: int
    r1_pressed: int

    x_pressed: int
    circle_pressed: int
    triangle_pressed: int


class ControllerMapper:
    def clamp(self, value, min_value=0.0, max_value=1.0):
        if value > max_value:
            return max_value

        if value < min_value:
            return min_value

        return value

    def from_joy_msg(self, msg):
        joystick_x = msg.axes[cfg.AXIS_LEFT_X] * cfg.STEER_MULTIPLIER
        joystick_y = msg.axes[cfg.AXIS_LEFT_Y] * cfg.JOYSTICK_Y_MULTIPLIER

        return ControllerState(
            joystick_x=joystick_x,
            joystick_y=joystick_y,

            l1_pressed=msg.buttons[cfg.BUTTON_L1],
            r1_pressed=msg.buttons[cfg.BUTTON_R1],

            x_pressed=msg.buttons[cfg.BUTTON_X],
            circle_pressed=msg.buttons[cfg.BUTTON_CIRCLE],
            triangle_pressed=msg.buttons[cfg.BUTTON_TRIANGLE],
        )
