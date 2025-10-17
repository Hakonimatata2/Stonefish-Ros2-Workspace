import numpy as np
from math import sin, cos, atan2, asin, pi

def Rx(r):
    return np.array([[1,0,0],[0,cos(r),-sin(r)],[0,sin(r),cos(r)]], dtype=float)

def Ry(p):
    return np.array([[cos(p),0,sin(p)],[0,1,0],[-sin(p),0,cos(p)]], dtype=float)

def Rz(y):
    return np.array([[cos(y),-sin(y),0],[sin(y),cos(y),0],[0,0,1]], dtype=float)

def rpy_to_R_zyx(roll, pitch, yaw):
    return Rz(yaw) @ Ry(pitch) @ Rx(roll)

def R_to_rpy_zyx(R):
    sp = -R[2,0]
    sp = np.clip(sp, -1.0, 1.0)
    pitch = asin(sp)

    if abs(abs(pitch) - pi/2) < 1e-6:
        roll = 0.0
        yaw = atan2(-R[0,1], R[1,1])
    else:
        roll = atan2(R[2,1], R[2,2])
        yaw  = atan2(R[1,0], R[0,0])
    return roll, pitch, yaw

def apply_T(T, p_xyz, rpy):
    """T: 4x4, p_xyz: (x,y,z), rpy: (r,p,y) -> (p_out, rpy_out)"""
    p = np.array(p_xyz, dtype=float).reshape(3,1)
    R_in = rpy_to_R_zyx(*rpy)
    R_T = T[:3,:3]
    t_T = T[:3, 3:4]

    p_out = (R_T @ p + t_T).reshape(3)
    R_out = R_T @ R_in
    r_out, p_out_ang, y_out = R_to_rpy_zyx(R_out)
    return p_out, (r_out, p_out_ang, y_out)


def invert_T(T):
    """
    Inverts a 4x4 homogenious transformation matrix.
    """
    R = T[:3, :3]
    t = T[:3, 3]
    T_inv = np.eye(4)
    T_inv[:3, :3] = R.T
    T_inv[:3, 3] = -R.T @ t
    return T_inv