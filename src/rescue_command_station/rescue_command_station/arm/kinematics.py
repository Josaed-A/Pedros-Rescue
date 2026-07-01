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


def _wrap(a: float) -> float:
    return (float(a) + np.pi) % (2.0 * np.pi) - np.pi


def _wrap_vec(a: np.ndarray) -> np.ndarray:
    return (np.asarray(a, dtype=float) + np.pi) % (2.0 * np.pi) - np.pi


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


# Private aliases kept for the wrist model below.
_rotz3 = rotz3
_rotx3 = rotx3


def rot_log(R: np.ndarray) -> np.ndarray:
    """SO(3) log map as axis * angle."""
    R = np.asarray(R, dtype=float).reshape(3, 3)
    cos_a = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    angle = float(np.arccos(cos_a))
    if angle < 1e-9:
        return np.zeros(3)

    if angle > np.pi - 1e-6:
        A = (R + np.eye(3)) / 2.0
        axis = np.sqrt(np.clip(np.diag(A), 0.0, 1.0))
        if axis[0] >= axis[1] and axis[0] >= axis[2]:
            axis[1] = np.copysign(axis[1], R[0, 1] + R[1, 0])
            axis[2] = np.copysign(axis[2], R[0, 2] + R[2, 0])
        elif axis[1] >= axis[2]:
            axis[0] = np.copysign(axis[0], R[0, 1] + R[1, 0])
            axis[2] = np.copysign(axis[2], R[1, 2] + R[2, 1])
        else:
            axis[0] = np.copysign(axis[0], R[0, 2] + R[2, 0])
            axis[1] = np.copysign(axis[1], R[1, 2] + R[2, 1])
        n = np.linalg.norm(axis)
        return (axis / n) * angle if n > 1e-9 else np.zeros(3)

    w = np.array([
        R[2, 1] - R[1, 2],
        R[0, 2] - R[2, 0],
        R[1, 0] - R[0, 1],
    ], dtype=float) / (2.0 * np.sin(angle))
    return w * angle


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

    @staticmethod
    def _R36_from_angles(q4: float, q5: float, q6: float) -> np.ndarray:
        # Standard spherical wrist matching the MATLAB extraction:
        # q5 = atan2(hypot(R13, R23), R33)
        # q4 = atan2(-R23, -R13)
        # q6 = atan2(-R32, R31)
        return (
            _rotz3(q4)
            @ _rotx3(np.pi / 2.0)
            @ _rotz3(q5)
            @ _rotx3(-np.pi / 2.0)
            @ _rotz3(q6)
        )

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
    # Analytic inverse kinematics
    # ------------------------------------------------------------------

    def _ik_position_candidates(
        self,
        p_des: np.ndarray,
        elbow: str = "down",
    ) -> list[np.ndarray]:
        x, y, z = np.asarray(p_des, dtype=float).reshape(3)
        q1 = float(np.arctan2(y, x))
        r_plane = float(np.hypot(x, y))
        z_plane = float(z - self.d1)

        num = r_plane**2 + z_plane**2 - self.a2**2 - self.a3**2
        den = 2.0 * self.a2 * self.a3
        D = num / den
        if D > 1.0 + 1e-9 or D < -1.0 - 1e-9:
            raise ValueError(
                "El punto objetivo esta fuera del espacio de trabajo del robot."
            )
        D = float(np.clip(D, -1.0, 1.0))

        primary_sign = 1.0 if str(elbow).lower() != "up" else -1.0
        signs = [primary_sign, -primary_sign]
        out: list[np.ndarray] = []
        for sign in signs:
            q3_kin = float(np.arctan2(sign * np.sqrt(max(0.0, 1.0 - D**2)), D))
            gamma = float(np.arctan2(z_plane, r_plane))
            beta = float(np.arctan2(
                self.a3 * np.sin(q3_kin),
                self.a2 + self.a3 * np.cos(q3_kin),
            ))
            q2 = gamma - beta
            q3 = q3_kin - self.q3_offset   # convertir a convencion de servo
            out.append(np.array([_wrap(q1), _wrap(q2), _wrap(q3)], dtype=float))
        return out

    @staticmethod
    def _extract_wrist(R36: np.ndarray) -> tuple[float, float, float]:
        R36 = np.asarray(R36, dtype=float).reshape(3, 3)
        q5 = float(np.arctan2(
            np.hypot(R36[0, 2], R36[1, 2]),
            R36[2, 2],
        ))

        if abs(np.sin(q5)) < 1e-6:
            # Muneca alineada (q5=0/pi): un solo grado de libertad combinado.
            # Formula analitica (cinematica_inversa_esferica): q4=0 fijo,
            # q6 = atan2(-R36[1,0], R36[0,0]).
            q4 = 0.0
            q6 = float(np.arctan2(-R36[1, 0], R36[0, 0]))
        else:
            q4 = float(np.arctan2(-R36[1, 2], -R36[0, 2]))
            q6 = float(np.arctan2(-R36[2, 1], R36[2, 0]))
        return q4, q5, q6

    @staticmethod
    def _equivalent_wrist(q: np.ndarray) -> np.ndarray:
        alt = np.asarray(q, dtype=float).copy()
        alt[3] += np.pi
        alt[4] = -alt[4]
        alt[5] += np.pi
        return np.array([_wrap(v) for v in alt], dtype=float)

    @staticmethod
    def _closest(candidates: list[np.ndarray], q_init: np.ndarray | None) -> np.ndarray:
        if q_init is None:
            return candidates[0]
        qi = np.asarray(q_init, dtype=float).reshape(6)
        return min(
            candidates,
            key=lambda q: float(np.linalg.norm(_wrap_vec(np.asarray(q) - qi))),
        )

    def ik(
        self,
        Td: np.ndarray,
        elbow: str = "down",
        q_init: np.ndarray | None = None,
        *_,
        **__,
    ) -> np.ndarray:
        """
        Analytic inverse kinematics for a full desired pose Td = [R | p].

        Extra positional/solver arguments are accepted for compatibility with
        the previous numerical solver API.
        """
        Td = np.asarray(Td, dtype=float).reshape(4, 4)
        R_des = Td[:3, :3]
        p_des = Td[:3, 3]

        candidates: list[np.ndarray] = []
        for q123 in self._ik_position_candidates(p_des, elbow):
            q1, q2, q3 = q123
            R03 = self._R03_from_angles(q1, q2, q3)
            R36 = R03.T @ R_des
            q4, q5, q6 = self._extract_wrist(R36)
            q = np.array([q1, q2, q3, q4, q5, q6], dtype=float)
            q = np.array([_wrap(v) for v in q], dtype=float)
            candidates.append(q)
            candidates.append(self._equivalent_wrist(q))

        return self._closest(candidates, q_init)

    def pose_error(self, q: np.ndarray, Rd: np.ndarray, pd: np.ndarray) -> np.ndarray:
        T = self.fk(q)["T06"]
        e_p = np.asarray(pd, dtype=float).reshape(3) - T[:3, 3]
        e_o = rot_log(np.asarray(Rd, dtype=float).reshape(3, 3) @ T[:3, :3].T)
        return np.concatenate([e_p, e_o])
