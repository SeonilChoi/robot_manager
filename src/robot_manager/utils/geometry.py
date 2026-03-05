import numpy as np

from robot_manager.core import Pose

def transformation_matrix(pose: Pose) -> np.ndarray:
    x, y, z = pose.position
    R, P, Y = pose.orientation

    Rz = np.array([[np.cos(Y), -np.sin(Y), 0],
                   [np.sin(Y),  np.cos(Y), 0],
                   [        0,          0, 1]])

    Ry = np.array([[ np.cos(P), 0, np.sin(P)],
                   [         0, 1,         0],
                   [-np.sin(P), 0, np.cos(P)]])

    Rx = np.array([[1,         0,          0],
                   [0, np.cos(R), -np.sin(R)],
                   [0, np.sin(R),  np.cos(R)]])

    R = Rz @ Ry @ Rx
    p = np.array([x, y, z]).reshape(3, 1)

    return np.r_[np.c_[R, p], [[0, 0, 0, 1]]]