"""Shared offline planner. No partial plan is returned on failure.

Cartesian samples use exact IK seeded by the preceding solution. The actual
linear joint interpolation is checked at quarter points and refined. These
are discrete geometric checks, not collision/dynamic safety guarantees.
"""
from dataclasses import dataclass
import numpy as np
from .kinematics import make_T, validate_pose
from .cartesian_controller import slerp


@dataclass
class Plan:
    t: np.ndarray
    q: np.ndarray

    def sample(self, t):
        return np.array([np.interp(t, self.t, self.q[:, i]) for i in range(6)])


def plan_segment(arm, start, target, duration=3.0, kind='cartesian', steps=100):
    target = validate_pose(target)
    start = np.asarray(start, float).reshape(6)
    if (not np.isfinite(duration) or not 0.2 <= duration <= 120
            or not np.all(np.isfinite(start))
            or np.any(start < arm.limits[:, 0]) or np.any(start > arm.limits[:, 1])):
        raise ValueError('Inicio fuera de limites o duracion fuera de [0.2,120] s')
    if kind not in ('cartesian', 'joint'):
        raise ValueError('Tipo de trayectoria desconocido')
    steps = max(50, min(int(steps), 3000))
    initial = arm.fk(start)['T06']
    goal = arm.ik(target, q_init=start)

    def desired(t):
        u = t / duration
        s = 10*u**3 - 15*u**4 + 6*u**5
        return make_T(slerp(initial[:3,:3], target[:3,:3], s),
                      initial[:3,3]*(1-s) + target[:3,3]*s)

    if kind == 'joint':
        # Dense quintic samples, with bounded spacing for interpolation/playback.
        steps = max(steps, int(np.ceil(1.875*np.max(abs(goal-start))/np.radians(1))))
        ts = np.linspace(0, duration, steps+1)
        u = ts / duration
        s = 10*u**3 - 15*u**4 + 6*u**5
        return Plan(ts, start[None,:] + s[:,None]*(goal-start)[None,:])

    times = [0.0]
    joints = [start.copy()]
    calls = 0

    def interval(t0, q0, t1, depth=0):
        nonlocal calls
        calls += 1
        if calls > 16000 or len(times) > 6000:
            raise ValueError('Trayectoria demasiado compleja; cambie postura o destino')
        try:
            q1 = arm.ik(desired(t1), q_init=q0)
        except ValueError as exc:
            raise ValueError(f'IK imposible al {100*t1/duration:.2f}%: {exc}') from exc
        refine = np.max(abs(q1-q0)) > np.radians(2)
        for a in (0.25, 0.5, 0.75):
            T = desired(t0+(t1-t0)*a)
            e = arm.pose_error(q0+(q1-q0)*a, T[:3,:3], T[:3,3])
            refine |= np.linalg.norm(e[:3]) > 5e-5 or np.linalg.norm(e[3:]) > np.radians(0.02)
        if refine:
            if depth >= 12:
                raise ValueError('Salto de rama o singularidad: trayectoria rechazada')
            mid = (t0+t1)/2
            qm = interval(t0, q0, mid, depth+1)
            return interval(mid, qm, t1, depth+1)
        times.append(t1)
        joints.append(q1)
        return q1

    q0 = start.copy()
    for t in np.linspace(0, duration, steps+1)[1:]:
        q0 = interval(times[-1], q0, float(t))
    return Plan(np.asarray(times), np.asarray(joints))


def plan_waypoints(arm, start, waypoints, dt=0.05):
    """Legacy station waypoints (p,R,n); each segment duration = n*dt."""
    if len(waypoints) < 2 or not np.isfinite(dt) or dt <= 0:
        raise ValueError('Waypoints o intervalo invalidos')
    p, R, _ = waypoints[0]
    e = arm.pose_error(start, R, p)
    if np.linalg.norm(e[:3]) > 1e-4 or np.linalg.norm(e[3:]) > 1e-3:
        raise ValueError('El primer waypoint no coincide con la pose actual')
    ts = [0.0]; qs = [np.asarray(start).copy()]
    for p, R, n in waypoints[1:]:
        segment = plan_segment(arm, qs[-1], make_T(R, p), max(0.2, n*dt), steps=n)
        ts.extend((segment.t[1:]+ts[-1]).tolist())
        qs.extend(segment.q[1:])
    return Plan(np.asarray(ts), np.asarray(qs))
