"""Canonical 6R model. Metres/radians; zero pose vertical.
Rz(q1) Tz(h) Ry(-q2) Tz(L1) Ry(-q3) Tz(L2) Ry(-q4) Tz(L3)
Rz(q5) Tx(e56) Rx(q6). Both camera and tool are fixed to frame 6.
No hardware calibration, servo ticks or ROS dependencies belong here.
"""
from dataclasses import dataclass
import numpy as np

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



@dataclass
class ArmParams:
    base_height: float
    L1: float
    L2: float
    L3: float
    tool_length: float
    e56: float
    camera_xyz: tuple
    camera_rpy: tuple
    control_frame: str

    def __post_init__(self):
        if len(self.camera_xyz)!=3 or len(self.camera_rpy)!=3:
            raise ValueError('camera_xyz/camera_rpy requieren tres componentes')
        values = [self.base_height, self.L1, self.L2, self.L3,
                  self.tool_length, self.e56, *self.camera_xyz, *self.camera_rpy]
        if not np.all(np.isfinite(values)) or min(self.L1, self.L2) <= 0 or self.L3 < 0:
            raise ValueError("Geometria invalida: longitudes en metros, L1/L2 positivas")
        if self.control_frame not in ("tool", "camera"):
            raise ValueError("control_frame debe ser tool o camera")


def validate_pose(T):
    T = np.asarray(T, float).reshape(4, 4)
    R = T[:3, :3]
    if (not np.all(np.isfinite(T)) or not np.allclose(T[3], [0,0,0,1])
            or not np.allclose(R.T @ R, np.eye(3), atol=1e-7)
            or abs(np.linalg.det(R)-1) > 1e-7):
        raise ValueError("Pose invalida: se requiere una transformacion rigida finita")
    return T


class Arm6DOF:
    def __init__(self, p: ArmParams, limits=None):
        self.p = p
        self.limits = np.asarray(limits if limits is not None else
                                 [(-np.pi, np.pi)]*6, float)
        if self.limits.shape != (6,2) or not np.all(np.isfinite(self.limits)) or np.any(self.limits[:,0] >= self.limits[:,1]):
            raise ValueError("Limites invalidos")

    def max_reach(self):
        p = self.p
        return p.base_height+p.L1+p.L2+p.L3+abs(p.e56)+max(abs(p.tool_length), np.linalg.norm(p.camera_xyz))

    def mount(self, frame=None):
        if (frame or self.p.control_frame) == "tool":
            return transl(self.p.tool_length,0,0)
        r,p,y = self.p.camera_rpy
        return transl(*self.p.camera_xyz) @ rotz(y) @ roty(p) @ rotx(r)

    def fk(self, q):
        q = np.asarray(q, float).reshape(6)
        if not np.all(np.isfinite(q)):
            raise ValueError("Angulos no finitos")
        p = self.p
        T = rotz(q[0]) @ transl(z=p.base_height)
        points = [np.zeros(3), T[:3,3].copy()]
        origins = [np.zeros(3)]; axes = [np.array([0.,0.,1.])]; chain = [T.copy()]
        for i,L in enumerate((p.L1,p.L2,p.L3)):
            origins.append(T[:3,3].copy()); axes.append(T[:3,:3] @ [0,-1,0])
            T = T @ roty(-q[i+1]) @ transl(z=L)
            points.append(T[:3,3].copy()); chain.append(T.copy())
        origins.append(T[:3,3].copy()); axes.append(T[:3,:3] @ [0,0,1])
        T = T @ rotz(q[4]) @ transl(x=p.e56); chain.append(T.copy())
        origins.append(T[:3,3].copy()); axes.append(T[:3,:3] @ [1,0,0])
        T = T @ rotx(q[5]); chain.append(T.copy())
        tool = T @ self.mount("tool"); camera = T @ self.mount("camera")
        active = camera if p.control_frame == "camera" else tool
        points.append(tool[:3,3].copy())
        return dict(T06=active, flange=T, tool=tool, camera=camera,
                    points=np.array(points), origins=np.array(origins), axes=np.array(axes), chain=chain)

    def fk_chain(self, q):
        return self.fk(q)["chain"]

    def jacobian(self, q):
        f = self.fk(q)
        return np.vstack((np.cross(f["axes"], f["T06"][:3,3]-f["origins"]).T, f["axes"].T))

    def pose_error(self, q, Rd, pd):
        T = self.fk(q)["T06"]
        return np.r_[np.asarray(pd)-T[:3,3], rot_log(np.asarray(Rd) @ T[:3,:3].T)]

    def ik(self, Td, elbow="down", q_init=None):
        Td = validate_pose(Td)
        seed = np.zeros(6) if q_init is None else np.asarray(q_init,float).reshape(6)
        if not np.all(np.isfinite(seed)):
            raise ValueError("Semilla invalida")
        err = self.pose_error(seed, Td[:3,:3], Td[:3,3])
        if np.linalg.norm(err) < 1e-9 and np.all(seed >= self.limits[:,0]) and np.all(seed <= self.limits[:,1]):
            return seed.copy()
        m = self.mount(); R6 = Td[:3,:3] @ m[:3,:3].T
        w = Td[:3,3] - R6 @ (m[:3,3]+[self.p.e56,0,0])
        rho = np.hypot(*w[:2]); p=self.p
        b = np.arctan2(w[1],w[0]); vertical = np.arctan2(R6[1,0],R6[0,0])
        bases = [b,b+np.pi] if rho>1e-9 else [seed[0],b,vertical,vertical+np.pi,*np.linspace(-np.pi,np.pi,181)]
        solutions=[]
        for q1 in bases:
            B = rotz(-q1)[:3,:3] @ R6
            c5=np.hypot(B[0,0],B[2,0]); r=np.cos(q1)*w[0]+np.sin(q1)*w[1]; z=w[2]-p.base_height
            if c5>1e-9:
                phi=np.arctan2(B[2,0],B[0,0])+np.pi/2
                q5=np.arctan2(B[1,0],c5); q6=np.arctan2(-B[1,2],B[1,1])
                orientations=[(phi,q5,q6),(phi+np.pi,np.pi-q5,q6+np.pi)]
            else:
                q5=np.sign(B[1,0])*np.pi/2
                theta=np.arctan2(z,r); dist=np.hypot(r,z); boundary=[]
                if dist*p.L3>1e-12:
                    for radius in (p.L1+p.L2,abs(p.L1-p.L2)):
                        c=(dist*dist+p.L3*p.L3-radius*radius)/(2*dist*p.L3)
                        if abs(c)<=1+1e-9:
                            a=np.arccos(np.clip(c,-1,1)); boundary.extend([theta-a,theta+a])
                orientations=[]
                for phi in [np.pi/2+sum(seed[1:4]),theta,*boundary,*np.linspace(-np.pi,np.pi,181)]:
                    C=(roty(-(phi-np.pi/2)) @ rotz(q5))[:3,:3].T @ B
                    orientations.append((phi,q5,np.arctan2(C[2,1],C[1,1])))
            for phi,q5,q6 in orientations:
                u=r-p.L3*np.cos(phi); v=z-p.L3*np.sin(phi)
                D=(u*u+v*v-p.L1*p.L1-p.L2*p.L2)/(2*p.L1*p.L2)
                if abs(D)>1+1e-9: continue
                for sign in ((-1,1) if elbow=="up" else (1,-1)):
                    q3=sign*np.arccos(np.clip(D,-1,1))
                    theta=np.arctan2(v,u)-np.arctan2(p.L2*np.sin(q3),p.L1+p.L2*np.cos(q3))
                    q=np.array([q1,theta-np.pi/2,q3,phi-theta-q3,q5,q6])
                    lo=np.ceil((self.limits[:,0]-q-1e-10)/(2*np.pi)); hi=np.floor((self.limits[:,1]-q+1e-10)/(2*np.pi))
                    if np.any(lo>hi): continue
                    q += 2*np.pi*np.clip(np.round((seed-q)/(2*np.pi)),lo,hi)
                    e=self.pose_error(q,Td[:3,:3],Td[:3,3])
                    if np.linalg.norm(e[:3])<1e-6 and np.linalg.norm(e[3:])<1e-6:
                        solutions.append(q)
            # Keep preceding base angle when it belongs to the singular family.
            if rho<1e-9 and solutions: break
        if not solutions:
            raise ValueError("Sin solucion exacta dentro de limites (familias singulares muestreadas)")
        return min(solutions,key=lambda q: np.linalg.norm(q-seed)).copy()

    def relative_target(self, q, delta, frame="tool"):
        if frame not in ("base","tool","camera"):
            raise ValueError("Marco desconocido")
        delta=np.asarray(delta,float).reshape(3)
        f=self.fk(q); T=f["T06"].copy()
        R=np.eye(3) if frame=="base" else f[frame][:3,:3]
        T[:3,3] += R @ delta
        return validate_pose(T)

    def relative_basis(self, active_R, frame):
        """Axes for a displacement, independent from the point being controlled."""
        if frame=='base': return np.eye(3)
        if frame not in ('tool','camera'): raise ValueError('Marco desconocido')
        return np.asarray(active_R) @ self.mount()[:3,:3].T @ self.mount(frame)[:3,:3]

    def component_target(self, xyz, beta, q5, q6):
        """Legacy service adapter. beta=q2+q3+q4; all angles radians.

        q1 inferred from target azimuth is only a target-orientation convention;
        for arbitrary fixed orientations use the full-pose interface instead.
        """
        q1=np.arctan2(xyz[1],xyz[0])
        R=(rotz(q1)@roty(-beta)@rotz(q5)@rotx(q6))[:3,:3] @ self.mount()[:3,:3]
        return make_T(R,xyz)
