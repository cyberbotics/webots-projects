import numpy as np


def fit_line(times, positions):
    A = np.column_stack([np.ones_like(times), times - times[-1]])
    (p0, v), *_ = np.linalg.lstsq(A, positions, rcond=None)
    return p0, v
