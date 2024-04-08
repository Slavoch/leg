import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import block_diag
import casadi as cs
import sympy as sp


def rotx(ang):
    mat = np.array(
        [
            [1, 0, 0],
            [0, np.cos(ang), -np.sin(ang)],
            [0, np.sin(ang), np.cos(ang)],
        ]
    )
    return block_diag(mat, 1)


def roty(ang):
    sin = np.sin
    cos = np.cos
    mat = np.array(
        [
            [cos(ang), 0, sin(ang)],
            [0, 1, 0],
            [-sin(ang), 0, cos(ang)],
        ]
    )
    return block_diag(mat, 1)


def rotz(ang):
    sin = np.sin
    cos = np.cos
    mat = np.array(
        [
            [cos(ang), -sin(ang), 0],
            [sin(ang), cos(ang), 0],
            [0, 0, 1],
        ]
    )
    return np.block([mat, 1])


def trans(v):
    res = np.eye(4)
    res[:3, -1] = v
    return res


def to_decart(A):
    return A[:3, -1]


def kinematics(angles):
    origin_p = -np.array([0.12552, -0.012418, -0.171634])
    hip_p = -np.array([-0.086434, -0.012369, -0.171631])
    thigh_p = -np.array([0.004233, -0.032538, -0.171631])
    foot_p = -np.array([-0.27037, -0.147629, -0.050396])
    tip_p = -np.array([0.025909, -0.090243, -0.045146])

    origin = trans(origin_p)
    # origin -> hip
    R = rotx(angles[0])
    T = trans(hip_p - origin_p)
    origin_hip = origin @ T @ R

    # hip -> thigh
    R = roty(-angles[1])
    T = trans(thigh_p - hip_p)
    origin_thigh = origin_hip @ T @ R

    # thigh -> foot
    R = roty(angles[2])
    T = trans(foot_p - thigh_p)
    origin_sock = origin_thigh @ T @ R

    # foot -> tip
    T = trans(tip_p - foot_p)
    origin_tip = origin_sock @ T @ R

    return [
        to_decart(origin_hip),
        to_decart(origin_thigh),
        to_decart(origin_sock),
        to_decart(origin_tip),
    ]
