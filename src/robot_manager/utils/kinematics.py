import numpy as np

def transformation_matrix(a: float, alpha: float, d: float, theta: float) -> np.ndarray:
    return np.array([[                np.cos(theta),                -np.sin(theta),              0,                  a],
                     [np.sin(theta) * np.cos(alpha), np.cos(theta) * np.cos(alpha), -np.sin(alpha), -d * np.sin(alpha)],
                     [np.sin(theta) * np.sin(alpha), np.cos(theta) * np.sin(alpha),  np.cos(alpha),  d * np.cos(alpha)],
                     [0, 0, 0, 1]])

def forward_kinematics(dh_parameters: np.ndarray) -> np.ndarray:
    n = dh_parameters.shape[0]
    points = np.zeros((n, 3))
    
    T = np.eye(4)
    for i in range(n):
        T = T @ transformation_matrix(dh_parameters[i, 0], dh_parameters[i, 1], dh_parameters[i, 2], dh_parameters[i, 3])
        points[i, :] = T[:3, 3]
    return points
