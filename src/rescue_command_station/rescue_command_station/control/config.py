# PS4 mapping used by the current control mode.
AXIS_LEFT_X = 0
AXIS_LEFT_Y = 1

BUTTON_L1 = 4
BUTTON_R1 = 5

# Axis orientation.
STEER_MULTIPLIER = -1.0
THROTTLE_MULTIPLIER = 1.0

# Each gear is a positive maximum output limit.
GEAR_LIMITS = {
    1: 0.20,
    2: 0.40,
    3: 0.60,
    4: 0.80,
    5: 1.00,
}

MIN_GEAR = 1
MAX_GEAR = 5
DEFAULT_GEAR = 1

# Normalized ROS Twist limits.
MAX_LINEAR_SPEED = 1.0
MAX_ANGULAR_SPEED = 1.0
