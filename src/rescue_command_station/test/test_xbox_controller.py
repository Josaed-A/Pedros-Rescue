from types import SimpleNamespace
import unittest

from rescue_command_station.control import config as cfg
from rescue_command_station.control.gearbox import Gearbox
from rescue_command_station.control.tank_drive import TankDriveMixer
from rescue_command_station.input.xbox_controller import XboxControllerMapper


def joy(axes=None, buttons=None):
    return SimpleNamespace(
        axes=list([0.0] * 8 if axes is None else axes),
        buttons=list([0] * 11 if buttons is None else buttons),
    )


class XboxControllerTest(unittest.TestCase):
    def setUp(self):
        self.mapper = XboxControllerMapper()

    def test_xpad_trigger_normalization(self):
        self.assertEqual(cfg.trigger_value(-1.0), 0.0)
        self.assertEqual(cfg.trigger_value(0.0), 0.5)
        self.assertEqual(cfg.trigger_value(1.0), 1.0)

    def test_measured_resting_drift_is_ignored(self):
        msg = joy()
        msg.axes[cfg.AXIS_LEFT_X] = -0.0673

        state = self.mapper.from_joy_msg(msg)

        self.assertEqual(state.joystick_x, 0.0)

    def test_left_stick_forward_is_positive_linear_speed(self):
        msg = joy()
        msg.axes[cfg.AXIS_LEFT_Y] = -1.0

        state = self.mapper.from_joy_msg(msg)
        command = TankDriveMixer().build_command(state, Gearbox())

        self.assertGreater(command.linear_x, 0.0)
        self.assertEqual(command.angular_z, 0.0)

    def test_left_stick_right_is_negative_angular_speed(self):
        msg = joy()
        msg.axes[cfg.AXIS_LEFT_X] = 1.0

        state = self.mapper.from_joy_msg(msg)
        command = TankDriveMixer().build_command(state, Gearbox())

        self.assertEqual(command.linear_x, 0.0)
        self.assertLess(command.angular_z, 0.0)

    def test_lb_and_rb_map_to_gearbox(self):
        buttons = [0] * 11
        buttons[cfg.BUTTON_RB] = 1
        gearbox = Gearbox()

        gearbox.update_from_controller(self.mapper.from_joy_msg(joy(buttons=buttons)))
        self.assertEqual(gearbox.current_gear, 2)

        buttons[cfg.BUTTON_RB] = 0
        buttons[cfg.BUTTON_LB] = 1
        gearbox.update_from_controller(self.mapper.from_joy_msg(joy(buttons=buttons)))
        self.assertEqual(gearbox.current_gear, 1)

    def test_short_message_is_rejected(self):
        self.assertFalse(self.mapper.is_valid_joy_msg(joy(axes=[], buttons=[])))


if __name__ == '__main__':
    unittest.main()
