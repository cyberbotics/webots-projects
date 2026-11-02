"""
Exercise 1 · Forward kinematics with the Product of Exponentials
================================================================

Write the function that gives the gripper pose for joint angles q:

    T(q) = exp([S1] q1) · exp([S2] q2) · … · exp([S7] q7) · M

You can use:
    S_LIST       6×7 array, column i is the screw axis of joint i+1
    M_HOME       4×4 gripper pose when all joints are 0
    exp_twist(S, theta)   4×4 matrix exp([S] theta)   (already written)

Check your answer:   python3 exercises/check.py 1
Reference solution:  controllers/dynamic_grasping/kinematics.py → forward_kinematics
"""

import numpy as np

from kinematics import S_LIST, M_HOME, exp_twist   # noqa: F401  (provided for you)


def forward_kinematics(q):
    """Return the 4×4 pose of the gripper for the 7 joint angles in q."""
    # TODO: start from the identity, multiply by exp_twist(S_LIST[:, i], q[i])
    #       for every joint, and finish with M_HOME.
    raise NotImplementedError
