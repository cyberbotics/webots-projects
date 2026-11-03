import numpy as np


def ik_step(J, V, damping=0.05):
    return J.T @ np.linalg.solve(J @ J.T + damping**2 * np.eye(6), V)
