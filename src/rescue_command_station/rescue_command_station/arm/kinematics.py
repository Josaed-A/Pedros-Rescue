"""
kinematics.py
=============
Pure NumPy kinematics for the 6-DOF arm.

This version implements the analytic spherical-wrist model from the MATLAB
snippet pasted in this file:

    q1, q2, q3 -> position with DH parameters d1, a2, a3
    q4, q5, q6 -> orientation from R36 = R03.T @ R_des

The ROS node keeps using meters. If a link value is configured as 36, it is
treated as 36 cm and converted to 0.36 m.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


# ---------------------------------------------------------------------------
# Homogeneous transforms
# ---------------------------------------------------------------------------

def rotz(t: float) -> np.ndarray:
    c, s = np.cos(t), np.sin(t)
    return np.array([
        [c, -s, 0.0, 0.0],
        [s,  c, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ], dtype=float)


def roty(t: float) -> np.ndarray:
    c, s = np.cos(t), np.sin(t)
    return np.array([
        [c, 0.0, s, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [-s, 0.0, c, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ], dtype=float)


def rotx(t: float) -> np.ndarray:
    c, s = np.cos(t), np.sin(t)
    return np.array([
        [1.0, 0.0, 0.0, 0.0],
        [0.0, c, -s, 0.0],
        [0.0, s,  c, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ], dtype=float)


def transl(x=0.0, y=0.0, z=0.0) -> np.ndarray:
    T = np.eye(4, dtype=float)
    T[0, 3], T[1, 3], T[2, 3] = x, y, z
    return T


def make_T(R: np.ndarray, p: np.ndarray) -> np.ndarray:
    T = np.eye(4, dtype=float)
    T[:3, :3] = np.asarray(R, dtype=float).reshape(3, 3)
    T[:3, 3] = np.asarray(p, dtype=float).reshape(3)
    return T


def _dh(theta: float, d: float, a: float, alpha: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [c, -s * ca,  s * sa, a * c],
        [s,  c * ca, -c * sa, a * s],
        [0.0, sa, ca, d],
        [0.0, 0.0, 0.0, 1.0],
    ], dtype=float)


def _as_meters(value: float) -> float:
    """Accept meters directly, or centimeters for values like 36."""
    value = float(value)
    return value / 100.0 if abs(value) > 5.0 else value


# ---------------------------------------------------------------------------
# Orientation utilities
# ---------------------------------------------------------------------------

def rotx3(t: float) -> np.ndarray:
    """3x3 rotation about X."""
    return rotx(t)[:3, :3]


def roty3(t: float) -> np.ndarray:
    """3x3 rotation about Y."""
    return roty(t)[:3, :3]


def rotz3(t: float) -> np.ndarray:
    """3x3 rotation about Z."""
    return rotz(t)[:3, :3]


def rot_to_rpy(R: np.ndarray) -> tuple[float, float, float]:
    """Extract roll, pitch, yaw (ZYX intrinsic) from a 3x3 matrix, in degrees."""
    R = np.asarray(R, dtype=float).reshape(3, 3)
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6
    if not singular:
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = 0.0
    return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)


# ---------------------------------------------------------------------------
# Arm geometry
# ---------------------------------------------------------------------------

@dataclass
class ArmParams:
    base_height: float = 0.12
    L1: float = 0.36
    L2: float = 0.36
    # Kept for parameter compatibility with the old node. The spherical wrist
    # model has no position offset after joint 3.
    L3: float = 0.0
    tool_length: float = 0.0
    # Offset mecanico del codo: en la calibracion fisica (q3_servo=0) el
    # eslabon L2-L3 no queda perfectamente recto, sino con esta flexion (deg).
    # Sin esto, q3=0 cae EXACTAMENTE en max_reach (brazo 100% estirado), que
    # es un limite duro del espacio de trabajo (D=1 en la ley de cosenos):
    # cualquier movimiento en casi cualquier direccion desde ahi se rechaza
    # como "fuera de alcance". El offset separa el home logico de ese borde.
    elbow_offset_deg: float = 5.0


class Arm6DOF:
    def __init__(self, p: ArmParams):
        self.p = p

    @property
    def d1(self) -> float:
        return _as_meters(self.p.base_height)

    @property
    def a2(self) -> float:
        return -abs(_as_meters(self.p.L1))

    @property
    def a3(self) -> float:
        return -abs(_as_meters(self.p.L2))

    @property
    def q3_offset(self) -> float:
        """Angulo cinematico real del codo cuando el servo reporta q3=0."""
        return float(np.radians(self.p.elbow_offset_deg))

    def max_reach(self) -> float:
        return abs(self.a2) + abs(self.a3)

    # ------------------------------------------------------------------
    # Forward kinematics
    # ------------------------------------------------------------------

    def _R03_from_angles(self, q1: float, q2: float, q3: float) -> np.ndarray:
        c1, s1 = np.cos(q1), np.sin(q1)
        c2, s2 = np.cos(q2), np.sin(q2)
        c3, s3 = np.cos(q3 + self.q3_offset), np.sin(q3 + self.q3_offset)

        R01 = np.array([
            [c1, 0.0,  s1],
            [s1, 0.0, -c1],
            [0.0, 1.0, 0.0],
        ], dtype=float)
        R12 = np.array([
            [c2, -s2, 0.0],
            [s2,  c2, 0.0],
            [0.0, 0.0, 1.0],
        ], dtype=float)
        R23 = np.array([
            [c3, -s3, 0.0],
            [s3,  c3, 0.0],
            [0.0, 0.0, 1.0],
        ], dtype=float)
        return R01 @ R12 @ R23

    def fk_chain(self, q: np.ndarray) -> list[np.ndarray]:
        q1, q2, q3, q4, q5, q6 = np.asarray(q, dtype=float).reshape(6)

        A01 = _dh(q1, self.d1, 0.0, np.pi / 2.0)
        A12 = _dh(q2, 0.0, self.a2, 0.0)
        A23 = _dh(q3 + self.q3_offset, 0.0, self.a3, 0.0)

        T03 = A01 @ A12 @ A23
        T03[:3, :3] = self._R03_from_angles(q1, q2, q3)

        A04 = T03 @ rotz(q4) @ rotx(np.pi / 2.0)
        A05 = A04 @ rotz(q5) @ rotx(-np.pi / 2.0)
        A06 = A05 @ rotz(q6)
        return [A01, A01 @ A12, T03, A04, A05, A06]

    def fk(self, q: np.ndarray) -> dict:
        chain = self.fk_chain(q)
        pts = np.array([
            np.zeros(3),
            chain[0][:3, 3],
            chain[1][:3, 3],
            chain[2][:3, 3],
            chain[4][:3, 3],
            chain[5][:3, 3],
        ], dtype=float)
        return {"T06": chain[-1], "points": pts}

    # ------------------------------------------------------------------
    # Inverse kinematics — puerto directo de cinematica_inversa_esferica()
    # ------------------------------------------------------------------

    def ik(self, Td: np.ndarray) -> np.ndarray:
        """Cinematica inversa analitica (muneca esferica), solucion "codo
        arriba" unica. Td = [R|p] deseada (4x4). Devuelve q (rad, 1x6) en
        convencion de servo (q3 ya compensa `q3_offset`, ver `_R03_from_angles`).
        """
        Td = np.asarray(Td, dtype=float).reshape(4, 4)
        P_des = Td[:3, 3]
        R_des = Td[:3, :3]
        xc, yc, zc = P_des

        # --- Cinematica de posicion (q1, q2, q3) ---
        q1 = float(np.arctan2(yc, xc))

        r_plano = float(np.hypot(xc, yc))
        z_plano = float(zc - self.d1)

        num_q3 = r_plano**2 + z_plano**2 - self.a2**2 - self.a3**2
        den_q3 = 2.0 * self.a2 * self.a3
        D = num_q3 / den_q3
        if abs(D) > 1.0 + 1e-9:
            raise ValueError(
                "El punto objetivo se encuentra fuera del espacio de trabajo del robot."
            )
        D = float(np.clip(D, -1.0, 1.0))

        q3_kin = float(np.arctan2(np.sqrt(1.0 - D**2), D))   # codo arriba
        gamma = float(np.arctan2(z_plano, r_plano))
        beta = float(np.arctan2(
            self.a3 * np.sin(q3_kin),
            self.a2 + self.a3 * np.cos(q3_kin),
        ))
        q2 = gamma - beta
        q3 = q3_kin - self.q3_offset   # convertir a convencion de servo

        # --- Cinematica de orientacion (q4, q5, q6) ---
        R03 = self._R03_from_angles(q1, q2, q3)
        R36 = R03.T @ R_des

        q5 = float(np.arctan2(
            np.sqrt(R36[0, 2]**2 + R36[1, 2]**2), R36[2, 2]))

        if abs(np.sin(q5)) < 1e-6:
            q4 = 0.0
            q6 = float(np.arctan2(-R36[1, 0], R36[0, 0]))
        else:
            q4 = float(np.arctan2(-R36[1, 2], -R36[0, 2]))
            q6 = float(np.arctan2(-R36[2, 1], R36[2, 0]))

        return np.array([q1, q2, q3, q4, q5, q6], dtype=float)
