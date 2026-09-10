"""SO(3) interpolation and tracking tolerances; planning lives in motion.py."""
import numpy as np
from .kinematics import rot_log


def slerp(R0, R1, t):
    v=rot_log(np.asarray(R0).T @ R1)
    angle=np.linalg.norm(v)
    if angle<1e-12: return np.asarray(R0).copy()
    x,y,z=v/angle
    K=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
    a=float(t)*angle
    return R0 @ (np.eye(3)+np.sin(a)*K+(1-np.cos(a))*(K@K))


def within_tolerance(pos_err, rot_err, tol_pos, tol_rot):
    if not np.all(np.isfinite([pos_err,rot_err,tol_pos,tol_rot])): return False
    return (tol_pos<=0 or pos_err<=tol_pos) and (tol_rot<=0 or rot_err<=tol_rot)
