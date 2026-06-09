from dataclasses import dataclass

from rescue_pc_brain import control_config as cfg


@dataclass
class DriveCommand:
    linear_x: float
    angular_z: float
    target_speed: float
    left_track: float
    right_track: float


class DriveCommandBuilder:
    def clamp(self, value, min_value=-1.0, max_value=1.0):
        if value > max_value:
            return max_value

        if value < min_value:
            return min_value

        return value

    def build(self, controller_state, gearbox_manager):
        """
        Control tipo tanque con joystick izquierdo.

        Y adelante/atras mueve ambas orugas en el mismo sentido.
        X lateral mezcla velocidades; con Y=0 las orugas giran opuestas.
        """

        throttle = self.clamp(controller_state.joystick_y, -1.0, 1.0)
        turn = self.clamp(controller_state.joystick_x, -1.0, 1.0)

        gear_limit = gearbox_manager.get_gear_limit()

        left_raw = throttle + turn
        right_raw = throttle - turn

        normalizer = max(1.0, abs(left_raw), abs(right_raw))

        left_track = (left_raw / normalizer) * gear_limit
        right_track = (right_raw / normalizer) * gear_limit

        linear_x = ((left_track + right_track) / 2.0) * cfg.MAX_LINEAR_SPEED
        angular_z = ((left_track - right_track) / 2.0) * cfg.MAX_ANGULAR_SPEED

        linear_x = self.clamp(linear_x)
        angular_z = self.clamp(angular_z)

        target_speed = max(abs(left_track), abs(right_track))

        return DriveCommand(
            linear_x=linear_x,
            angular_z=angular_z,
            target_speed=target_speed,
            left_track=left_track,
            right_track=right_track
        )
