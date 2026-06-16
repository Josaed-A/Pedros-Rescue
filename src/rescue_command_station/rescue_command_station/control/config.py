# PS4 mapping used by the current control mode.
AXIS_LEFT_X = 0
AXIS_LEFT_Y = 1

# Stick derecho (control de patas). En el driver joy de Linux para el DS4:
# eje 3 = horizontal derecho (RX), eje 4 = vertical derecho (RY).
AXIS_RIGHT_X = 3
AXIS_RIGHT_Y = 4
# Zona muerta para ignorar el reposo del stick.
LEGS_DEADZONE = 0.35

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
