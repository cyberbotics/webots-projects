"""
Exercise 3 · One step of inverse kinematics (damped least squares)
==================================================================

Given the Jacobian J (6×7) and the twist error V (6-vector: how far the
gripper is from where it should be), one damped least-squares step is

    Δq = Jᵀ (J Jᵀ + λ² I)⁻¹ V

The damping λ keeps Δq small when J is nearly singular.

Tip: use np.linalg.solve(A, b) rather than inverting A.

Check your answer:   python3 exercises/check.py 3
Reference solution:  controllers/dynamic_grasping/kinematics.py → ik_from_seed
"""

import numpy as np


def ik_step(J, V, damping=0.05):
    """Return the joint update Δq (7-vector) for Jacobian J and twist error V."""
    # TODO
    raise NotImplementedError
