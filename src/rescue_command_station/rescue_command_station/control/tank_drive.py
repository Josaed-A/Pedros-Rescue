from dataclasses import dataclass

from rescue_command_station.control import config as cfg


@dataclass
class DriveCommand:
    linear_x: float
    angular_z: float
    target_speed: float
    left_track: float
    right_track: float


class TankDriveMixer:
    def clamp(self, value, min_value=-1.0, max_value=1.0):
        if value > max_value:
            return max_value

        if value < min_value:
            return min_value

        return value

    def build_command(self, controller_state, gearbox):
        throttle = self.clamp(controller_state.joystick_y)
        turn = self.clamp(controller_state.joystick_x)
        gear_limit = gearbox.get_gear_limit()

        left_raw = throttle + turn
        right_raw = throttle - turn
        normalizer = max(1.0, abs(left_raw), abs(right_raw))

        left_track = (left_raw / normalizer) * gear_limit
        right_track = (right_raw / normalizer) * gear_limit

        linear_x = ((left_track + right_track) / 2.0) * cfg.MAX_LINEAR_SPEED
        angular_z = ((left_track - right_track) / 2.0) * cfg.MAX_ANGULAR_SPEED
        target_speed = max(abs(left_track), abs(right_track))

        return DriveCommand(
            linear_x=self.clamp(linear_x),
            angular_z=self.clamp(angular_z),
            target_speed=target_speed,
            left_track=left_track,
            right_track=right_track,
        )
