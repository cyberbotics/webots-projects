import numpy as np


def quintic_position(q_start, q_end, T, t):
    tau = t / T
    s = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
    return np.asarray(q_start) + s * (np.asarray(q_end) - np.asarray(q_start))


def quintic_duration(q_start, q_end, dq_max):
    return float(np.max(15 * np.abs(np.asarray(q_end) - np.asarray(q_start)) / (8 * np.asarray(dq_max))))
