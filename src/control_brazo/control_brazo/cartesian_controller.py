"""
cartesian_controller.py
=======================
Matematica pura para control cartesiano del efector. Sin dependencias de ROS.

Inspirado en Universal_Robots_ROS_controllers_cartesian:
  - Interpolacion lineal de posicion + SLERP de orientacion (log-map SO3)
  - Soporte multi-waypoint: concatenacion de segmentos
  - Utilidades de error cartesiano para monitoreo de tolerancias de ruta
"""

import numpy as np


# ─────────────────────────────────────────────────────────────
#  Interpolacion de orientacion: SLERP via log-map SO(3)
# ─────────────────────────────────────────────────────────────

def slerp(R0: np.ndarray, R1: np.ndarray, t: float) -> np.ndarray:
    """
    Interpolacion esferica entre matrices de rotacion SO(3).
    t=0 -> R0, t=1 -> R1.
    Usa formula de Rodrigues: dR = R0^T R1, luego exp(t * log(dR)).
    """
    dR = R0.T @ R1
    cos_a = np.clip((np.trace(dR) - 1.0) / 2.0, -1.0, 1.0)
    angle = np.arccos(cos_a)
    if angle < 1e-8:
        return R0.copy()
    K = (dR - dR.T) / (2.0 * np.sin(angle))
    ax, ay, az = K[2, 1], K[0, 2], K[1, 0]
    a  = t * angle
    c, s = np.cos(a), np.sin(a)
    dR_t = np.array([
        [c + ax*ax*(1-c),      ax*ay*(1-c) - az*s,  ax*az*(1-c) + ay*s],
        [ay*ax*(1-c) + az*s,   c + ay*ay*(1-c),     ay*az*(1-c) - ax*s],
        [az*ax*(1-c) - ay*s,   az*ay*(1-c) + ax*s,  c + az*az*(1-c)   ],
    ])
    return R0 @ dR_t


# ─────────────────────────────────────────────────────────────
#  Segmento lineal + SLERP entre dos poses
# ─────────────────────────────────────────────────────────────

def linear_trajectory(
    p0: np.ndarray,
    R0: np.ndarray,
    p1: np.ndarray,
    R1: np.ndarray,
    n_steps: int,
) -> list[tuple[np.ndarray, np.ndarray]]:
    """
    Genera n_steps+1 poses (inicio incluido) entre (p0,R0) y (p1,R1).
    Posicion: interpolacion lineal.
    Orientacion: SLERP via log-map SO(3).
    """
    n = max(n_steps, 1)
    poses: list[tuple[np.ndarray, np.ndarray]] = []
    for i in range(n + 1):
        t = i / n
        p = (1.0 - t) * np.asarray(p0, float) + t * np.asarray(p1, float)
        R = slerp(np.asarray(R0, float), np.asarray(R1, float), t)
        poses.append((p, R))
    return poses


# ─────────────────────────────────────────────────────────────
#  Trayectoria multi-waypoint
#  Referencia: CartesianTrajectory de UR ROS controllers
# ─────────────────────────────────────────────────────────────

def multi_waypoint_trajectory(
    waypoints: list[tuple[np.ndarray, np.ndarray, int]],
) -> list[tuple[np.ndarray, np.ndarray]]:
    """
    Interpola una trayectoria que pasa por una lista de waypoints.

    Parametros
    ----------
    waypoints : list of (p, R, n_steps)
        El primer elemento es la pose inicial (pose actual del TCP).
        n_steps en el elemento i indica cuantos pasos de interpolacion
        se generan entre el waypoint i-1 y el i.

    Retorna
    -------
    list of (p, R)
        Todas las poses interpoladas, sin duplicar los puntos de union.
    """
    if len(waypoints) < 2:
        raise ValueError("Se necesitan al menos 2 waypoints (inicio + meta)")

    poses: list[tuple[np.ndarray, np.ndarray]] = []
    for i in range(len(waypoints) - 1):
        p0, R0, _  = waypoints[i]
        p1, R1, n  = waypoints[i + 1]
        segment    = linear_trajectory(p0, R0, p1, R1, n)
        if i == 0:
            poses.extend(segment)
        else:
            poses.extend(segment[1:])   # evita duplicar el punto de union
    return poses


# ─────────────────────────────────────────────────────────────
#  Utilidades de error cartesiano (para monitoreo de tolerancias)
#  Referencia: monitorExecution() + withinTolerances() del UR controller
# ─────────────────────────────────────────────────────────────

def position_error(p_actual: np.ndarray, p_ref: np.ndarray) -> float:
    """Error euclidiano de posicion en metros."""
    return float(np.linalg.norm(np.asarray(p_actual) - np.asarray(p_ref)))


def rotation_error(R_actual: np.ndarray, R_ref: np.ndarray) -> float:
    """
    Error de orientacion como angulo de rotacion relativa (rad).
    Se usa el angulo del log-map de dR = R_actual^T @ R_ref.
    """
    dR = np.asarray(R_actual).T @ np.asarray(R_ref)
    cos_a = np.clip((np.trace(dR) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.arccos(cos_a))


def within_tolerance(
    pos_err: float,
    rot_err: float,
    tol_pos: float,
    tol_rot: float,
) -> bool:
    """
    True si ambos errores estan dentro de sus tolerancias.
    tol=0 significa sin limite para esa dimension.
    """
    pos_ok = (tol_pos <= 0.0) or (pos_err <= tol_pos)
    rot_ok = (tol_rot <= 0.0) or (rot_err <= tol_rot)
    return pos_ok and rot_ok


# ─────────────────────────────────────────────────────────────
#  Limites articulares seguros (proteccion desbordamiento de bits)
#
#  AX-12A:   10 bits, 0-1023 ticks => 0-300 grados
#             Centro en 512 (150 deg). Rango util: +-150 deg = +-2.618 rad
#             Un angulo > 150 deg genera tick > 1023 => overflow uint10
#
#  EX-106+:  12 bits, 0-4095 ticks => 0-360 grados
#             Centro en 2048 (180 deg). Rango: +-180 deg = +-pi rad
# ─────────────────────────────────────────────────────────────

JOINT_LIMITS: dict[str, tuple[float, float]] = {
    'Base':      (-2.618, 2.618),   # AX-12A +-150 deg
    'Hombro':    (-3.142, 3.142),   # EX-106+ +-180 deg
    'Codo':      (-2.618, 2.618),   # AX-12A +-150 deg
    'Munieca_P': (-2.618, 2.618),   # AX-12A +-150 deg
    'Munieca_Y': (-2.618, 2.618),   # AX-12A +-150 deg
    'Munieca_R': (-2.618, 2.618),   # AX-12A +-150 deg
}


def clamp_joints(
    q_rad: np.ndarray,
    joint_names: list[str],
    limits: dict[str, tuple[float, float]] | None = None,
) -> tuple[np.ndarray, list[str]]:
    """
    Clampea los angulos articulares a sus limites seguros para evitar
    desbordamiento de bits en la conversion a ticks del servo.

    Retorna (q_clamped, joints_saturados).
    joints_saturados lista los nombres que fueron modificados.
    """
    lim = limits if limits is not None else JOINT_LIMITS
    q_out    = q_rad.copy()
    saturated: list[str] = []
    for i, name in enumerate(joint_names):
        if name in lim:
            lo, hi = lim[name]
            clamped = float(np.clip(q_out[i], lo, hi))
            if abs(clamped - float(q_out[i])) > 1e-6:
                saturated.append(name)
            q_out[i] = clamped
    return q_out, saturated
