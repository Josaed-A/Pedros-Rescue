from rescue_robot_core.motion.s_curve import clamp

STOP = 'STOP'
STRAIGHT = 'STRAIGHT'
PIVOT_TURN = 'PIVOT_TURN'
CURVE_TURN = 'CURVE_TURN'


def is_zero(value, epsilon):
    return abs(value) <= epsilon


def detect_motion_state(linear_x, angular_z, epsilon):
    linear_active = not is_zero(linear_x, epsilon)
    angular_active = not is_zero(angular_z, epsilon)

    if not linear_active and not angular_active:
        return STOP

    if linear_active and not angular_active:
        return STRAIGHT

    if not linear_active and angular_active:
        return PIVOT_TURN

    return CURVE_TURN


def calculate_motor_targets(
    linear_x,
    angular_z,
    left_motor_direction,
    right_motor_direction,
    epsilon
):
    state = detect_motion_state(linear_x, angular_z, epsilon)
    linear_x = clamp(linear_x)
    angular_z = clamp(angular_z)

    if state == STOP:
        left_motor = 0.0
        right_motor = 0.0
    else:
        left_motor = linear_x + angular_z
        right_motor = linear_x - angular_z

        normalizer = max(1.0, abs(left_motor), abs(right_motor))
        left_motor = left_motor / normalizer
        right_motor = right_motor / normalizer

    left_motor = clamp(left_motor) * left_motor_direction
    right_motor = clamp(right_motor) * right_motor_direction

    return left_motor, right_motor, state
