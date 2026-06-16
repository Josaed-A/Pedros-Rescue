"""
kinematics.py
=============
Cinematica directa e inversa para brazo 6-DOF tipo UR5.
Cadena de ejes: Z (base), Y (hombro), Y (codo), Y (munieca pitch),
Z (munieca yaw), X (munieca roll) — la herramienta sale del ultimo eje.
Biblioteca pura Python/NumPy — sin dependencias de ROS.

Usada por cinematica_node.py (que la expone como topics/servicios ROS).

CINEMATICA INVERSA — METODO NUMERICO POR JACOBIANO
--------------------------------------------------
Implementacion basada en https://github.com/dimitris-anastasiou/cartesian-control-IK
(control cartesiano + IK por Jacobiano, estilo Columbia Robotics).

Se abandono la IK geometrica analitica (descomposicion Euler Y-Z-X). En este
brazo las articulaciones de pitch (q2,q3,q4 sobre Y) acoplan POSICION y
ORIENTACION (igual que un UR5): el cabeceo phi del antebrazo que coloca L3 no
es libre para la orientacion. La descomposicion analitica forzaba ese
acoplamiento y producia saltos discontinuos de la munieca (el efecto "munieca
rara") y errores al mover en Z, porque cada pose se resolvia de forma
independiente y la rama de munieca podia voltearse entre pasos.

La IK numerica resuelve el error de pose 6D completo con minimos cuadrados
amortiguados (damped least squares / Levenberg-Marquardt):

    dq = Jᵀ (J Jᵀ + λ²I)⁻¹ · e_pose

Sembrando el solver con la configuracion ACTUAL del brazo (q_init), la
solucion se mantiene cerca de la anterior => movimiento continuo y suave en
trayectorias cartesianas (incl. desplazamientos en Z) y sin volteos de
munieca. Para llamadas aisladas se usan reinicios aleatorios.
"""

import numpy as np
from dataclasses import dataclass


# ─────────────────────────────────────────────────────────────
#  Transformaciones homogeneas
# ─────────────────────────────────────────────────────────────

def rotz(t: float) -> np.ndarray:
    c, s = np.cos(t), np.sin(t)
    return np.array([[c,-s,0,0],[s,c,0,0],[0,0,1,0],[0,0,0,1]], float)

def roty(t: float) -> np.ndarray:
    c, s = np.cos(t), np.sin(t)
    return np.array([[c,0,s,0],[0,1,0,0],[-s,0,c,0],[0,0,0,1]], float)

def rotx(t: float) -> np.ndarray:
    c, s = np.cos(t), np.sin(t)
    return np.array([[1,0,0,0],[0,c,-s,0],[0,s,c,0],[0,0,0,1]], float)

def transl(x=0., y=0., z=0.) -> np.ndarray:
    T = np.eye(4, dtype=float)
    T[0,3], T[1,3], T[2,3] = x, y, z
    return T

def make_T(R: np.ndarray, p: np.ndarray) -> np.ndarray:
    T = np.eye(4, dtype=float)
    T[:3,:3], T[:3,3] = R, p
    return T

def _wrap(a: float) -> float:
    return (a + np.pi) % (2 * np.pi) - np.pi


def _wrap_vec(a: np.ndarray) -> np.ndarray:
    """Envuelve cada componente a (-pi, pi]. Para medir cercania angular."""
    return (np.asarray(a, float) + np.pi) % (2 * np.pi) - np.pi


# ─────────────────────────────────────────────────────────────
#  Orientacion
# ─────────────────────────────────────────────────────────────

def rot_log(R: np.ndarray) -> np.ndarray:
    """
    Log-map de SO(3): devuelve el vector de rotacion (eje·angulo) de R.
    Usado como error de orientacion en la IK numerica.
    """
    cos_a = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    angle = np.arccos(cos_a)
    if angle < 1e-9:
        return np.zeros(3)
    if angle > np.pi - 1e-6:
        # Cerca de pi: sin(angle)~0; extraer el eje de la diagonal de R.
        A = (R + np.eye(3)) / 2.0
        axis = np.sqrt(np.clip(np.diag(A), 0.0, 1.0))
        # Fijar signos relativos con los terminos fuera de la diagonal.
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
    w = np.array([R[2, 1] - R[1, 2],
                  R[0, 2] - R[2, 0],
                  R[1, 0] - R[0, 1]]) / (2.0 * np.sin(angle))
    return w * angle


def rot_to_rpy(R: np.ndarray) -> tuple[float, float, float]:
    """Extrae Roll, Pitch, Yaw (ZYX intrinsic) de una matriz 3x3. Grados."""
    sy = np.sqrt(R[0,0]**2 + R[1,0]**2)
    singular = sy < 1e-6
    if not singular:
        roll  = np.arctan2( R[2,1],  R[2,2])
        pitch = np.arctan2(-R[2,0],  sy)
        yaw   = np.arctan2( R[1,0],  R[0,0])
    else:
        roll  = np.arctan2(-R[1,2],  R[1,1])
        pitch = np.arctan2(-R[2,0],  sy)
        yaw   = 0.0
    return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)


# ─────────────────────────────────────────────────────────────
#  Parametros geometricos del brazo
# ─────────────────────────────────────────────────────────────

@dataclass
class ArmParams:
    base_height : float = 0.10
    L1          : float = 0.36
    L2          : float = 0.36
    L3          : float = 0.10
    tool_length : float = 0.2


# ─────────────────────────────────────────────────────────────
#  Cinematica 6-DOF tipo UR5
# ─────────────────────────────────────────────────────────────

class Arm6DOF:
    def __init__(self, p: ArmParams):
        self.p = p

    def max_reach(self) -> float:
        return self.p.L1 + self.p.L2 + self.p.L3 + self.p.tool_length

    def fk(self, q: np.ndarray) -> dict:
        """
        Cinematica directa.  q: array [q1..q6] en radianes.
        Retorna 'T06' (4x4 del efector) y 'points' (6,3 posicion de cada eje;
        q5 y q6 comparten el centro de munieca).
        """
        q1, q2, q3, q4, q5, q6 = q
        A01 = rotz(q1) @ transl(0, 0, self.p.base_height)
        A02 = A01 @ roty(q2 - np.pi/2) @ transl(self.p.L1, 0, 0)
        A03 = A02 @ roty(q3) @ transl(self.p.L2, 0, 0)
        A04 = A03 @ roty(q4) @ transl(self.p.L3, 0, 0)
        A05 = A04 @ rotx(q5)                                    # Muñeca_Y (eje X)
        # Muñeca_R (eje Z) — la pinza sale COAXIAL al eje de roll (rota sobre el,
        # apuntando +X horizontal en calibracion): offset a lo largo de -Z local.
        A06 = A05 @ rotz(q6) @ transl(0, 0, -self.p.tool_length)
        pts = np.array([np.zeros(3), A01[:3,3], A02[:3,3],
                        A03[:3,3], A04[:3,3], A06[:3,3]])
        return {"T06": A06, "points": pts}

    # ─────────────────────────────────────────────────────────────
    #  Cadena de transformadas y Jacobiano geometrico
    # ─────────────────────────────────────────────────────────────

    def fk_chain(self, q: np.ndarray) -> list[np.ndarray]:
        """
        Devuelve las transformadas acumuladas [A01, A02, A03, A04, A05, A06].
        A06 es la pose del efector (= fk()['T06']).
        """
        q1, q2, q3, q4, q5, q6 = q
        A01 = rotz(q1) @ transl(0, 0, self.p.base_height)
        A02 = A01 @ roty(q2 - np.pi/2) @ transl(self.p.L1, 0, 0)
        A03 = A02 @ roty(q3) @ transl(self.p.L2, 0, 0)
        A04 = A03 @ roty(q4) @ transl(self.p.L3, 0, 0)
        A05 = A04 @ rotx(q5)                                    # Muñeca_Y (eje X)
        # Muñeca_R (eje Z) — pinza coaxial al eje de roll (offset -Z local).
        A06 = A05 @ rotz(q6) @ transl(0, 0, -self.p.tool_length)
        return [A01, A02, A03, A04, A05, A06]

    def jacobian(self, q: np.ndarray) -> np.ndarray:
        """
        Jacobiano geometrico 6x6 del efector, expresado en el frame base.
        Filas 0-2: velocidad lineal; filas 3-5: velocidad angular.

        Para cada articulacion revoluta i con eje 'axis_i' (en base) que pasa
        por el punto 'o_i':
            J_v[:,i] = axis_i × (o_ee − o_i)
            J_w[:,i] = axis_i
        El frame en el que actua cada giro y su eje local:
            q1: base   eje Z | q2: A01 eje Y | q3: A02 eje Y
            q4: A03    eje Y | q5: A04 eje X | q6: A05 eje Z
        """
        A01, A02, A03, A04, A05, A06 = self.fk_chain(q)
        frames     = [np.eye(4), A01, A02, A03, A04, A05]
        axes_local = [[0, 0, 1], [0, 1, 0], [0, 1, 0],
                      [0, 1, 0], [1, 0, 0], [0, 0, 1]]
        o_ee = A06[:3, 3]
        J = np.zeros((6, 6))
        for i in range(6):
            Ri   = frames[i][:3, :3]
            o_i  = frames[i][:3, 3]
            axis = Ri @ np.asarray(axes_local[i], float)
            J[0:3, i] = np.cross(axis, o_ee - o_i)
            J[3:6, i] = axis
        return J

    # ─────────────────────────────────────────────────────────────
    #  Cinematica inversa numerica (damped least squares)
    # ─────────────────────────────────────────────────────────────

    def pose_error(self, q: np.ndarray, Rd: np.ndarray,
                   pd: np.ndarray) -> np.ndarray:
        """Error de pose 6D [e_pos(3); e_rot(3)] en frame base."""
        T = self.fk_chain(q)[-1]
        e_p = pd - T[:3, 3]
        e_o = rot_log(Rd @ T[:3, :3].T)
        return np.concatenate([e_p, e_o])

    def _refine(self, q0: np.ndarray, Rd: np.ndarray, pd: np.ndarray,
                max_iters: int, tol: float, lam: float
                ) -> tuple[np.ndarray, float]:
        """Itera damped-least-squares desde q0. Devuelve (q, err_final)."""
        I6 = np.eye(6)
        q  = np.asarray(q0, float).copy()
        for _ in range(max_iters):
            e   = self.pose_error(q, Rd, pd)
            if np.linalg.norm(e) < tol:
                break
            J  = self.jacobian(q)
            dq = J.T @ np.linalg.solve(J @ J.T + (lam ** 2) * I6, e)
            step = np.linalg.norm(dq)
            if step > 0.5:                 # limitar el paso (evita saltos)
                dq *= 0.5 / step
            q = q + dq
        q = np.array([_wrap(v) for v in q])
        return q, float(np.linalg.norm(self.pose_error(q, Rd, pd)))

    def ik(self, Td: np.ndarray, elbow: str = "down",
           q_init: np.ndarray | None = None,
           max_iters: int = 200, tol: float = 1e-5,
           lam: float = 0.05, accept: float = 5e-4) -> np.ndarray:
        """
        Cinematica inversa numerica por Jacobiano (Levenberg-Marquardt).

        Td        : 4x4 del efector deseado.
        elbow     : "down"/"up" — solo sesga la semilla por defecto.
        q_init    : configuracion semilla (rad). Si se da, se intenta primero
                    y se PRIORIZA la solucion cercana a ella => continuidad y
                    suavidad en trayectorias cartesianas (sin volteos).
        accept    : umbral de error (m/rad) bajo el cual una solucion se
                    considera valida; entre las validas se elige la mas cercana
                    a q_init.
        Retorna q [q1..q6] en radianes.

        Itera:  dq = Jᵀ (J Jᵀ + λ²I)⁻¹ · e_pose   hasta ‖e‖ < tol.
        Si la semilla no alcanza el objetivo, reintenta con configuraciones
        aleatorias (solo para llamadas en frio, no para seguimiento).
        """
        Rd, pd = Td[:3, :3], Td[:3, 3]
        el = 0.6 if elbow != "up" else -0.6

        candidatos: list[tuple[np.ndarray, float]] = []

        # 1) Semilla de continuidad (config actual). Si converge, se devuelve
        #    de inmediato: garantiza seguimiento suave sin saltos de munieca.
        if q_init is not None:
            q, err = self._refine(q_init, Rd, pd, max_iters, tol, lam)
            candidatos.append((q, err))
            if err < accept:
                return q

        # 2) Semilla sesgada por el codo (llamada en frio).
        q, err = self._refine(np.array([0.0, 0.3, el, 0.3, 0.0, 0.0]),
                              Rd, pd, max_iters, tol, lam)
        candidatos.append((q, err))

        # 3) Reinicios aleatorios solo si aun no hay solucion valida.
        #    (El seguimiento cartesiano nunca llega aqui: la semilla de
        #     continuidad ya devolvio antes. Solo afecta llamadas en frio.)
        if min(e for _, e in candidatos) >= accept:
            rng = np.random.default_rng(12345)
            for _ in range(24):
                q, err = self._refine(rng.uniform(-np.pi, np.pi, 6),
                                      Rd, pd, max_iters, tol, lam)
                candidatos.append((q, err))
                if err < accept:
                    break

        err_min = min(e for _, e in candidatos)

        # Si hay semilla, preferir continuidad: entre las soluciones dentro de
        # una banda de 5 mm respecto a la mejor, elegir la mas cercana a q_init.
        if q_init is not None:
            qi = np.asarray(q_init, float)
            banda = [(q, e) for q, e in candidatos if e <= err_min + 5e-3]
            return min(banda, key=lambda c: np.linalg.norm(
                _wrap_vec(c[0] - qi)))[0]

        return min(candidatos, key=lambda c: c[1])[0]