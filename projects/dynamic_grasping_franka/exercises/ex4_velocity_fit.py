"""
Exercise 4 · Estimating the ball's velocity from noisy positions
================================================================

The camera gives positions p_k at times t_k.  Fit the straight line

    p(t) = p0 + v · (t − t_last)

by least squares: build the matrix A with one row [1, t_k − t_last] per
sample and solve  A · [p0; v] ≈ P  with np.linalg.lstsq.
(p0 is then the position at the *last* sample time.)

Check your answer:   python3 exercises/check.py 4
Reference solution:  controllers/dynamic_grasping/dynamic_grasping.py → fit_ball_motion
"""

import numpy as np


def fit_line(times, positions):
    """times: (N,) array, positions: (N, 3) array.
    Return (p0, v): position at the last sample time and velocity, both (3,)."""
    # TODO
    raise NotImplementedError
