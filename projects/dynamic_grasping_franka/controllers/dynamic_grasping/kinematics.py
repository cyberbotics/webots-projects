"""
kinematics.py
=============
Forward and inverse kinematics of the Panda with screw theory
(the Product-of-Exponentials formula), written with NumPy only so every
line can be read and stepped through.

Forward kinematics
------------------
    T(q) = exp([S1] q1) · exp([S2] q2) · … · exp([S7] q7) · M

* ``M``  — pose of the tool centre point (TCP) when every joint is at 0.
* ``Si`` — screw axis of joint i, expressed in the base frame at q = 0:
           Si = (ωi, vi) with ωi the rotation axis and vi = −ωi × pi
           for any point pi on the axis.

Inverse kinematics
------------------
Damped least squares (Levenberg–Marquardt) on the body twist error:

    V  = log( T(q)⁻¹ · T_desired )             6-vector "how far am I"
    Δq = Jᵀ (J Jᵀ + λ² I)⁻¹ V                  one damped Newton step
    q ← clip(q + Δq, Q_MIN, Q_MAX)

The damping λ keeps the step bounded near singular configurations.
"""

import numpy as np

from config import Q_MIN, Q_MAX, DQ_MAX

# ─────────────────────────────────────────────────────────────────────────────
#  The kinematic model (metres).  Same numbers as the URDF / EduPanda.proto.
# ─────────────────────────────────────────────────────────────────────────────

#                 j1     j2      j3     j4      j5     j6      j7
S_LIST = np.array([
    [0.0,   0.0,   0.0,   0.0,    0.0,   0.0,   0.0],     # ωx
    [0.0,   1.0,   0.0,  -1.0,    0.0,  -1.0,   0.0],     # ωy
    [1.0,   0.0,   1.0,   0.0,    1.0,   0.0,  -1.0],     # ωz
    [0.0,  -0.333, 0.0,   0.649,  0.0,   1.033, 0.0],     # vx
    [0.0,   0.0,   0.0,   0.0,    0.0,   0.0,   0.088],   # vy
    [0.0,   0.0,   0.0,  -0.0825, 0.0,   0.0,   0.0],     # vz
])

# TCP pose at q = 0: 8.8 cm forward, 82.6 cm up, gripper pointing down and
# rotated 45° about its axis (the Franka Hand is mounted that way).
M_HOME = np.array([
    [0.70710678,  0.70710678,  0.0, 0.088],
    [0.70710678, -0.70710678,  0.0, 0.0],
    [0.0,         0.0,        -1.0, 0.826],
    [0.0,         0.0,         0.0, 1.0],
])


# ─────────────────────────────────────────────────────────────────────────────
#  Small Lie-group toolbox (rotation / rigid-motion exponentials and logs)
# ─────────────────────────────────────────────────────────────────────────────

def skew(w):
    """3-vector → 3×3 skew-symmetric matrix [w] so that [w] p = w × p."""
    return np.array([[0.0, -w[2], w[1]],
                     [w[2], 0.0, -w[0]],
                     [-w[1], w[0], 0.0]])


def exp_twist(S, theta):
    """exp([S] θ) for a unit screw S = (ω, v)  →  4×4 transform (Rodrigues)."""
    w, v = S[:3], S[3:]
    T = np.eye(4)
    if np.linalg.norm(w) < 1e-12:            # prismatic joint: pure translation
        T[:3, 3] = v * theta
        return T
    W = skew(w)
    s, c = np.sin(theta), np.cos(theta)
    T[:3, :3] = np.eye(3) + s * W + (1 - c) * W @ W
    T[:3, 3] = (np.eye(3) * theta + (1 - c) * W + (theta - s) * W @ W) @ v
    return T


def log_transform(T):
    """4×4 transform → 6-vector twist (ω θ, v θ), inverse of exp_twist."""
    R, p = T[:3, :3], T[:3, 3]
    cos_theta = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    theta = np.arccos(cos_theta)
    if theta < 1e-9:                          # (almost) pure translation
        return np.concatenate([np.zeros(3), p])
    if np.pi - theta < 1e-6:                  # 180° turn: pick the axis from R
        k = np.argmax(np.diag(R))
        w = R[:, k] + np.eye(3)[k]
        w = w / np.linalg.norm(w)
    else:
        w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]]) / (2 * np.sin(theta))
    W = skew(w)
    G_inv = (np.eye(3) / theta - 0.5 * W
             + (1.0 / theta - 0.5 / np.tan(theta / 2.0)) * W @ W)
    return np.concatenate([w * theta, G_inv @ p * theta])


def adjoint(T):
    """6×6 adjoint map: converts twists from one frame to another."""
    R, p = T[:3, :3], T[:3, 3]
    Ad = np.zeros((6, 6))
    Ad[:3, :3] = R
    Ad[3:, 3:] = R
    Ad[3:, :3] = skew(p) @ R
    return Ad


def inverse_transform(T):
    R, p = T[:3, :3], T[:3, 3]
    Ti = np.eye(4)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ p
    return Ti


# ─────────────────────────────────────────────────────────────────────────────
#  Forward kinematics and Jacobian
# ─────────────────────────────────────────────────────────────────────────────

def forward_kinematics(q):
    """TCP pose (4×4) for joint angles q."""
    T = np.eye(4)
    for i in range(7):
        T = T @ exp_twist(S_LIST[:, i], q[i])
    return T @ M_HOME


def space_jacobian(q):
    """6×7 Jacobian in the base frame: column i = joint i's screw axis,
    moved to where the joints before it have carried it."""
    J = np.zeros((6, 7))
    T = np.eye(4)
    for i in range(7):
        J[:, i] = adjoint(T) @ S_LIST[:, i]
        T = T @ exp_twist(S_LIST[:, i], q[i])
    return J


def fk_and_body_jacobian(q):
    """TCP pose and the 6×7 Jacobian expressed in the TCP frame,
    J_b = Ad(T⁻¹) J_s, computed in a single pass over the joints."""
    J = np.zeros((6, 7))
    T = np.eye(4)
    for i in range(7):
        J[:, i] = adjoint(T) @ S_LIST[:, i]
        T = T @ exp_twist(S_LIST[:, i], q[i])
    T = T @ M_HOME
    return T, adjoint(inverse_transform(T)) @ J


def body_jacobian(q):
    return fk_and_body_jacobian(q)[1]


def tcp_position(q):
    return forward_kinematics(q)[:3, 3]


# ─────────────────────────────────────────────────────────────────────────────
#  Inverse kinematics
# ─────────────────────────────────────────────────────────────────────────────

def ik_from_seed(T_desired, q_seed, max_iters=100, damping=0.05,
                 tol_pos=1e-4, tol_rot=1e-3):
    """Damped-least-squares IK started from one seed.

    Returns (q, converged, iterations, history) where history holds the
    position error (m) after every iteration — handy for plotting.
    Gives up early when the error has stopped shrinking (usually a joint
    stuck at its limit), which is what makes random restarts cheap.
    """
    q = np.clip(np.array(q_seed, dtype=float), Q_MIN, Q_MAX)
    history = []
    for it in range(max_iters):
        T, J = fk_and_body_jacobian(q)
        V = log_transform(inverse_transform(T) @ T_desired)   # body twist error
        pos_err = np.linalg.norm(T_desired[:3, 3] - T[:3, 3])
        rot_err = np.linalg.norm(V[:3])
        history.append(pos_err)
        if pos_err < tol_pos and rot_err < tol_rot:
            return q, True, it, history
        if it >= 20 and history[-1] > 0.95 * history[-11]:  # stalled
            break
        dq = J.T @ np.linalg.solve(J @ J.T + damping ** 2 * np.eye(6), V)
        q = np.clip(q + dq, Q_MIN, Q_MAX)
    return q, False, max_iters, history


def solve_ik(T_desired, q_seed, n_random_seeds=8, rng=None):
    """IK with random restarts.  The seed is tried first; random seeds are
    only used if it fails.  Among the solutions found, the one that the arm
    can reach fastest from ``q_seed`` is returned.

    Returns (q, success, info) with ``info`` = dict(iterations, history).
    """
    rng = rng if rng is not None else np.random.default_rng()
    q, ok, iters, hist = ik_from_seed(T_desired, q_seed)
    if ok:
        return q, True, {"iterations": iters, "history": hist, "seeds_tried": 1}

    best, best_cost, info = None, np.inf, {"iterations": iters, "history": hist}
    for k in range(n_random_seeds):
        q, ok, iters, hist = ik_from_seed(T_desired, rng.uniform(Q_MIN, Q_MAX))
        if not ok:
            continue
        cost = np.max(np.abs(q - q_seed) / DQ_MAX)   # slowest joint's travel time
        if cost < best_cost:
            best, best_cost = q, cost
            info = {"iterations": iters, "history": hist}
    info["seeds_tried"] = 1 + n_random_seeds
    return (best, True, info) if best is not None else (q_seed, False, info)
