"""
Exercise 5 · How hard must the gripper squeeze?
===============================================

Two fingers hold a ball by friction.  Each finger presses with normal
force N and can transmit at most μ·N of friction.

(a) Holding still, the friction must carry the weight:
        2 μ N ≥ m g          →   N_min = m g / (2 μ)

(b) While lifting with upward acceleration a, the fingers must carry
    m (g + a) instead.

Check your answer:   python3 exercises/check.py 5
Reference solution:  controllers/dynamic_grasping/gripper.py → min_grip_force
"""

G = 9.81


def min_grip_force(mass, mu):
    """Smallest per-finger normal force that holds a still ball."""
    # TODO
    raise NotImplementedError


def holds_while_lifting(force, mass, mu, acceleration):
    """True if a per-finger force keeps the ball while it accelerates upwards."""
    # TODO
    raise NotImplementedError
