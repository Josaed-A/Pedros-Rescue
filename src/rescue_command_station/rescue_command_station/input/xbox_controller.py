from dataclasses import dataclass

from rescue_command_station.control import config as cfg


@dataclass
class ControllerState:
    joystick_x: float
    joystick_y: float
    l1_pressed: int
    r1_pressed: int


class XboxControllerMapper:
    """Map Linux xpad ``sensor_msgs/Joy`` data to the drive controls."""

    def is_valid_joy_msg(self, msg):
        return (
            len(msg.axes) > max(cfg.AXIS_LEFT_X, cfg.AXIS_LEFT_Y) and
            len(msg.buttons) > max(cfg.BUTTON_LB, cfg.BUTTON_RB)
        )

    def from_joy_msg(self, msg):
        left_x = cfg.apply_deadzone(msg.axes[cfg.AXIS_LEFT_X], cfg.DRIVE_DEADZONE)
        left_y = cfg.apply_deadzone(msg.axes[cfg.AXIS_LEFT_Y], cfg.DRIVE_DEADZONE)
        return ControllerState(
            joystick_x=left_x * cfg.STEER_MULTIPLIER,
            joystick_y=left_y * cfg.THROTTLE_MULTIPLIER,
            l1_pressed=msg.buttons[cfg.BUTTON_LB],
            r1_pressed=msg.buttons[cfg.BUTTON_RB],
        )
