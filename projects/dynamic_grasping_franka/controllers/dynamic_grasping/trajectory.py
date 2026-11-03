"""
trajectory.py
=============
Quintic (minimum-jerk) joint-space trajectories.

Every joint moves along the straight line from q_start to q_end, timed by

    s(τ) = 10 τ³ − 15 τ⁴ + 6 τ⁵ ,   τ = t / T  ∈ [0, 1]
    q(t) = q_start + s(τ) · (q_end − q_start)

s goes smoothly from 0 to 1 with zero velocity *and* zero acceleration at
both ends, so the arm starts and stops without a jerk.

How long should the motion take?  The peak of ds/dt is 15 / (8 T) (at
τ = ½), so joint j's peak speed is 15 |Δq_j| / (8 T).  Keeping it under the
joint's speed limit gives the shortest admissible duration

    T_min = max_j 15 |Δq_j| / (8 · dq_max_j)
"""

import numpy as np

from config import DQ_MAX, MIN_SEGMENT_TIME


def quintic_duration(q_start, q_end, speed_scale=1.0):
    """Shortest quintic duration that respects every joint speed limit.

    ``speed_scale`` (0 … 1] lets you run the arm slower than its limits.
    """
    if not speed_scale > 0:
        raise ValueError(f"speed_scale must be positive, got {speed_scale}")
    dq = np.abs(np.asarray(q_end) - np.asarray(q_start))
    T = np.max(15.0 * dq / (8.0 * DQ_MAX * speed_scale))
    return max(float(T), MIN_SEGMENT_TIME)


class QuinticTrajectory:
    """A rest-to-rest quintic from q_start to q_end lasting ``duration`` s."""

    def __init__(self, q_start, q_end, duration):
        self.q_start = np.array(q_start, dtype=float)
        self.q_end = np.array(q_end, dtype=float)
        self.duration = float(duration)

    def sample(self, t):
        """Desired (position, velocity) at time t since the start."""
        T = self.duration
        tau = np.clip(t / T, 0.0, 1.0)
        s = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
        ds = (30 * tau**2 - 60 * tau**3 + 30 * tau**4) / T
        delta = self.q_end - self.q_start
        return self.q_start + s * delta, ds * delta

    def finished(self, t):
        return t >= self.duration
