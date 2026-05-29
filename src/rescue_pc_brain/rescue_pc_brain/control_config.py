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
# Configuración de gatillo R2
# =========================

R2_RELEASED_VALUE = 1.0
R2_PRESSED_VALUE = -1.0


# =========================
# Cajas de velocidad
# =========================

# La caja define el máximo de velocidad disponible.
GEAR_LIMITS = {
    1: 0.25,
    2: 0.50,
    3: 0.75,
    4: 1.00,
}

MIN_GEAR = 1
MAX_GEAR = 4
DEFAULT_GEAR = 1

# =========================
# Descenso automático de caja por velocidad real
# =========================

# Si la velocidad real de la Raspberry baja de este valor,
# empieza a contar el tiempo para bajar caja automáticamente.
AUTO_DOWNSHIFT_REAL_SPEED_THRESHOLD = 0.08

# Tiempo que debe permanecer la velocidad real baja
# para bajar una caja.
AUTO_DOWNSHIFT_DELAY_SECONDS = 1.0




# =========================
# Reversa segura
# =========================

DIRECTION_FORWARD = "FORWARD"
DIRECTION_REVERSE = "REVERSE"

DEFAULT_DIRECTION = DIRECTION_FORWARD

# Para permitir cambio FORWARD <-> REVERSE,
# la velocidad real debe estar cerca de cero.
REAL_SPEED_ZERO_THRESHOLD = 0.03


# =========================
# Construcción de /cmd_vel
# =========================

MAX_LINEAR_SPEED = 1.0
MAX_ANGULAR_SPEED = 1.0