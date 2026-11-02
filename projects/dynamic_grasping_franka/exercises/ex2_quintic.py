"""
Exercise 2 · A quintic (minimum-jerk) trajectory
================================================

(a) Position along the quintic at time t (0 ≤ t ≤ T):

        τ = t / T
        s = 10 τ³ − 15 τ⁴ + 6 τ⁵
        q(t) = q_start + s · (q_end − q_start)

(b) The shortest duration that keeps every joint below its speed limit.
    The peak of ds/dt is 15 / (8T), so joint j needs
        T_j = 15 · |Δq_j| / (8 · dq_max_j)
    and the trajectory needs the largest of these.

Check your answer:   python3 exercises/check.py 2
Reference solution:  controllers/dynamic_grasping/trajectory.py
"""

import numpy as np


def quintic_position(q_start, q_end, T, t):
    """Joint angles at time t of a quintic from q_start to q_end lasting T seconds."""
    # TODO
    raise NotImplementedError


def quintic_duration(q_start, q_end, dq_max):
    """Shortest T (seconds) so that no joint exceeds dq_max (array, rad/s)."""
    # TODO
    raise NotImplementedError
