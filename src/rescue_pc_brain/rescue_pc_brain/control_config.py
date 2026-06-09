# =========================
# Mapeo del control PS4
# =========================

AXIS_LEFT_X = 0
AXIS_LEFT_Y = 1
AXIS_L2 = 2
AXIS_RIGHT_X = 3
AXIS_RIGHT_Y = 4
AXIS_R2 = 5
AXIS_DPAD_X = 6
AXIS_DPAD_Y = 7

BUTTON_X = 0
BUTTON_CIRCLE = 1
BUTTON_TRIANGLE = 2
BUTTON_SQUARE = 3
BUTTON_L1 = 4
BUTTON_R1 = 5
BUTTON_L2_FULL = 6
BUTTON_R2_FULL = 7
BUTTON_SHARE = 8
BUTTON_OPTIONS = 9
BUTTON_PS = 10
BUTTON_L3 = 11
BUTTON_R3 = 12


# =========================
# Configuración de ejes
# =========================

STEER_MULTIPLIER = -1.0

JOYSTICK_Y_MULTIPLIER = 1.0


# =========================
# Cajas de velocidad
# =========================

# La caja define el máximo de velocidad disponible.
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

# =========================
# Construcción de /cmd_vel
# =========================

MAX_LINEAR_SPEED = 1.0
MAX_ANGULAR_SPEED = 1.0
