"""Xbox Elite Series 2 mapping exposed by Linux ``xpad``/``joydev``."""

# Sticks and analog triggers.
AXIS_LEFT_X = 0
AXIS_LEFT_Y = 1
AXIS_LT = 2
AXIS_RIGHT_X = 3
AXIS_RIGHT_Y = 4
AXIS_RT = 5
DPAD_AXIS_X = 6
DPAD_AXIS_Y = 7

# Backward-compatible names used by the arm code.
AXIS_L2 = AXIS_LT
AXIS_R2 = AXIS_RT

# Face and control buttons: A, B, X, Y, LB, RB, View, Menu, Xbox, LS, RS.
BUTTON_A = 0
BUTTON_B = 1
BUTTON_X = 2
BUTTON_Y = 3
BUTTON_LB = 4
BUTTON_RB = 5
BUTTON_VIEW = 6
BUTTON_MENU = 7
BUTTON_XBOX = 8
BUTTON_LEFT_STICK = 9
BUTTON_RIGHT_STICK = 10

# Backward-compatible shoulder names used by the gearbox and arm GUI.
BUTTON_L1 = BUTTON_LB
BUTTON_R1 = BUTTON_RB

# Ignore resting-axis noise. The connected Elite 2 reported -0.067 on its left
# stick at rest, so driving uses a separate conservative threshold.
DRIVE_DEADZONE = 0.12
LEGS_DEADZONE = 0.35
ARM_DEADZONE = 0.30

# B and X toggle the two legs which are not assigned to shoulder jogging.
LEG_BUTTONS = {
    'PataDelDer': BUTTON_B,
    'PataTrasIzq': BUTTON_X,
}

# Y/A jog the EX-106 shoulder while held.
BUTTON_HOMBRO_POS = BUTTON_Y
BUTTON_HOMBRO_NEG = BUTTON_A

# On xpad, D-pad right/down are positive.
DPAD_Y_DOWN = 1.0
DPAD_X_RIGHT = 1.0

# Menu reconnects the AX-12A and EX-106 buses. The Xbox button is deliberately
# avoided because the desktop may intercept it.
BUTTON_RECONNECT = BUTTON_MENU

# xpad reports both triggers at -1.0 when released and +1.0 when fully pressed.
TRIGGER_RELEASED_VALUE = -1.0
TRIGGER_PRESSED_VALUE = 1.0


def trigger_value(axis_value):
    """Normalize an xpad trigger axis to 0.0 (released) .. 1.0 (pressed)."""
    span = TRIGGER_PRESSED_VALUE - TRIGGER_RELEASED_VALUE
    normalized = (float(axis_value) - TRIGGER_RELEASED_VALUE) / span
    return max(0.0, min(1.0, normalized))


def apply_deadzone(axis_value, deadzone):
    value = float(axis_value)
    return 0.0 if abs(value) < float(deadzone) else value


# joydev reports right/down as positive, so both driving axes are inverted:
# stick forward becomes positive linear.x and stick right becomes negative
# angular.z (a right turn under the current tank mixer).
STEER_MULTIPLIER = -1.0
THROTTLE_MULTIPLIER = -1.0

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
