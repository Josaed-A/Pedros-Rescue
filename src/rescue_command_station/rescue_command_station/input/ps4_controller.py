from dataclasses import dataclass

from rescue_command_station.control import config as cfg


@dataclass
class ControllerState:
    joystick_x: float
    joystick_y: float
    l1_pressed: int
    r1_pressed: int


class PS4ControllerMapper:
    def is_valid_joy_msg(self, msg):
        return (
            len(msg.axes) > max(cfg.AXIS_LEFT_X, cfg.AXIS_LEFT_Y) and
            len(msg.buttons) > max(cfg.BUTTON_L1, cfg.BUTTON_R1)
        )

    def from_joy_msg(self, msg):
        return ControllerState(
            joystick_x=msg.axes[cfg.AXIS_LEFT_X] * cfg.STEER_MULTIPLIER,
            joystick_y=msg.axes[cfg.AXIS_LEFT_Y] * cfg.THROTTLE_MULTIPLIER,
            l1_pressed=msg.buttons[cfg.BUTTON_L1],
            r1_pressed=msg.buttons[cfg.BUTTON_R1],
        )
