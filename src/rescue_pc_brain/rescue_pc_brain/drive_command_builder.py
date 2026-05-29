import math
from dataclasses import dataclass

from rescue_pc_brain import control_config as cfg


@dataclass
class DriveCommand:
    linear_x: float
    angular_z: float
    target_speed: float


class DriveCommandBuilder:
    def clamp(self, value, min_value=-1.0, max_value=1.0):
        if value > max_value:
            return max_value

        if value < min_value:
            return min_value

        return value

    def build(self, controller_state, gearbox_manager):
        """
        Velocidad deseada:

        target_speed = magnitud_joystick * R2 * limite_caja

        La reversa NO depende de bajar el joystick.
        La reversa depende del estado FORWARD / REVERSE del gearbox.

        Joystick:
        - X controla giro
        - Y aporta magnitud de movimiento
        - Si X está totalmente hacia un lado, linear.x tiende a 0 y angular.z domina
        """

        joystick_x = self.clamp(controller_state.joystick_x, -1.0, 1.0)
        joystick_y = self.clamp(controller_state.joystick_y, -1.0, 1.0)

        joystick_magnitude = math.sqrt((joystick_x ** 2) + (joystick_y ** 2))
        joystick_magnitude = self.clamp(joystick_magnitude, 0.0, 1.0)

        steer = joystick_x

        r2 = controller_state.r2_value
        gear_limit = gearbox_manager.get_gear_limit()
        direction = gearbox_manager.direction_sign()

        target_speed = joystick_magnitude * r2 * gear_limit

        linear_x = (
            target_speed *
            direction *
            (1.0 - abs(steer)) *
            cfg.MAX_LINEAR_SPEED
        )

        angular_z = (
            target_speed *
            steer *
            cfg.MAX_ANGULAR_SPEED
        )

        linear_x = self.clamp(linear_x)
        angular_z = self.clamp(angular_z)

        return DriveCommand(
            linear_x=linear_x,
            angular_z=angular_z,
            target_speed=target_speed
        )