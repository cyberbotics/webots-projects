"""
Unit tests for the algorithm modules.  They need only NumPy (no Webots):

    cd dynamic_grasping_franka
    python3 -m pytest tests -q
"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "controllers" / "dynamic_grasping"))

import config as C                                              # noqa: E402
from kinematics import (forward_kinematics, space_jacobian, exp_twist,   # noqa: E402
                        log_transform, solve_ik)
from trajectory import QuinticTrajectory, quintic_duration     # noqa: E402
from intercept_planner import plan_intercept, grasp_poses      # noqa: E402


def dh_forward_kinematics(q):
    """Independent reference: Franka's published (modified) DH table."""
    a = [0, 0, 0, 0.0825, -0.0825, 0, 0.088]
    d = [0.333, 0, 0.316, 0, 0.384, 0, 0]
    alpha = [0, -np.pi / 2, np.pi / 2, np.pi / 2, -np.pi / 2, np.pi / 2, np.pi / 2]
    T = np.eye(4)
    for i in range(7):
        ca, sa, ct, st = np.cos(alpha[i]), np.sin(alpha[i]), np.cos(q[i]), np.sin(q[i])
        T = T @ np.array([[ct, -st, 0, a[i]],
                          [st * ca, ct * ca, -sa, -d[i] * sa],
                          [st * sa, ct * sa, ca, d[i] * ca],
                          [0, 0, 0, 1]])
    flange_to_tcp = np.eye(4)
    flange_to_tcp[2, 3] = 0.107 + 0.1
    c, s = np.cos(-np.pi / 4), np.sin(-np.pi / 4)
    flange_to_tcp[:2, :2] = [[c, -s], [s, c]]
    return T @ flange_to_tcp


def test_fk_matches_dh_table():
    rng = np.random.default_rng(1)
    for _ in range(50):
        q = rng.uniform(C.Q_MIN, C.Q_MAX)
        assert np.allclose(forward_kinematics(q), dh_forward_kinematics(q), atol=1e-6)


def test_home_pose_points_down():
    T = forward_kinematics(C.Q_HOME)
    assert T[2, 2] < -0.99          # TCP z axis ≈ world −z


def test_exp_log_roundtrip():
    rng = np.random.default_rng(2)
    S = np.concatenate([[0, 0, 1.0], rng.normal(size=3)])
    T = exp_twist(S, 0.7)
    assert np.allclose(log_transform(T), S * 0.7, atol=1e-9)


def test_jacobian_by_finite_differences():
    q = C.Q_HOME + 0.1
    J = space_jacobian(q)
    eps = 1e-6
    T0 = forward_kinematics(q)
    for i in range(7):
        dq = np.zeros(7)
        dq[i] = eps
        T1 = forward_kinematics(q + dq)
        # spatial twist: [V] = Ṫ T⁻¹
        Vmat = (T1 - T0) / eps @ np.linalg.inv(T0)
        V = np.array([Vmat[2, 1], Vmat[0, 2], Vmat[1, 0], *Vmat[:3, 3]])
        assert np.allclose(V, J[:, i], atol=1e-4)


def test_ik_reaches_random_reachable_poses():
    rng = np.random.default_rng(3)
    for _ in range(20):
        q_true = rng.uniform(C.Q_MIN, C.Q_MAX)
        T = forward_kinematics(q_true)
        q, ok, _ = solve_ik(T, C.Q_HOME, rng=rng)
        assert ok
        assert np.allclose(forward_kinematics(q)[:3, 3], T[:3, 3], atol=1e-3)


def test_quintic_boundaries_and_speed_limit():
    q0, q1 = C.Q_HOME, C.Q_HOME + np.array([1.0, -0.5, 0.3, 0.2, 0.0, 0.4, -1.0])
    T = quintic_duration(q0, q1)
    traj = QuinticTrajectory(q0, q1, T)
    p, v = traj.sample(0.0)
    assert np.allclose(p, q0) and np.allclose(v, 0)
    p, v = traj.sample(T)
    assert np.allclose(p, q1) and np.allclose(v, 0)
    peak = max(np.max(np.abs(traj.sample(t)[1]) / C.DQ_MAX) for t in np.linspace(0, T, 400))
    assert peak <= 1.0 + 1e-6


def test_planner_finds_intercept_for_rolling_ball():
    p0 = np.array([0.5, -0.8, C.BALL_RADIUS])
    v = np.array([0.0, 0.3, 0.0])
    plan, candidates = plan_intercept(p0, v, C.Q_HOME)
    assert plan is not None
    assert candidates[-1]["result"] == "accepted"
    # The gripper really is at the predicted ball position at the grasp pose.
    p_tcp = forward_kinematics(plan["q_grasp"])[:3, 3]
    assert np.allclose(p_tcp[:2], plan["p_intercept"][:2], atol=2e-3)
    # The arm arrives before the ball does.
    assert plan["T_approach"] + plan["T_descend"] < plan["t_intercept"]


def test_planner_static_ball():
    plan, _ = plan_intercept(np.array([0.5, 0.2, C.BALL_RADIUS]), np.zeros(3), C.Q_HOME)
    assert plan is not None


def test_grasp_orientation_follows_velocity():
    _, T = grasp_poses(np.array([0.5, 0.0, 0.03]), np.array([0.0, 1.0, 0.0]))
    assert np.allclose(T[:3, 0], [0, 1, 0])     # TCP x along the ball's motion
    assert np.allclose(T[:3, 2], [0, 0, -1])    # pointing down
