"""
intercept_planner.py
====================
"Where and when should the gripper meet the ball?"

The ball slides on a frictionless floor, so it moves in a straight line:

    p_ball(t) = p_now + v · t

The planner scans candidate meeting times t = Δt, 2Δt, 3Δt, … and, for
each one, asks four questions.  The first candidate that passes them all
is the plan.

  1. REACH      Is the predicted ball position inside the arm's workspace?
  2. IK         Can the arm put its gripper above the ball (pre-grasp) and
                around the ball (grasp) with the right orientation?
                A parallel gripper is symmetric: turned by 180° about its own
                axis it grasps just as well, so both versions are tried and
                the faster one is kept.
  3. TIME       Arm motion time + gripper closing time + safety margin ≤ t ?
                (the arm has to be there *before* the ball)
  4. → ACCEPT   Return the joint targets and segment durations.

Every candidate is recorded with the reason it was rejected, so the robot
window can show the whole search.
"""

import numpy as np

import config as C
from kinematics import solve_ik
from trajectory import quintic_duration


def predict_ball(p_now, v, t):
    """Straight-line prediction of the ball centre after t seconds."""
    return p_now + v * t


def grasp_orientation(v):
    """Gripper orientation for a ball moving with velocity v.

    * TCP z axis points straight down (top grasp).
    * TCP x axis points along the ball's direction of travel, so the two
      fingers (which close along TCP y) sit on either side of its path and
      the ball slides *between* them.
    """
    v_xy = np.array([v[0], v[1], 0.0])
    speed = np.linalg.norm(v_xy)
    x = v_xy / speed if speed > 1e-3 else np.array([1.0, 0.0, 0.0])
    z = np.array([0.0, 0.0, -1.0])
    y = np.cross(z, x)
    return np.column_stack([x, y, z])


def grasp_poses(p_ball, v, flipped=False):
    """(T_pregrasp, T_grasp) 4×4 TCP targets for a ball at p_ball.

    ``flipped`` turns the gripper by 180° about its vertical axis (same
    grasp, different joint angles — joint 7 differs by about π).
    """
    R = grasp_orientation(v)
    if flipped:
        R = R @ np.diag([-1.0, -1.0, 1.0])
    T_grasp = np.eye(4)
    T_grasp[:3, :3] = R
    T_grasp[:3, 3] = [p_ball[0], p_ball[1], C.GRASP_HEIGHT]
    T_pre = T_grasp.copy()
    T_pre[2, 3] += C.PREGRASP_HEIGHT
    return T_pre, T_grasp


def in_workspace(p):
    """Quick geometric reachability test for a downward-pointing gripper.

    The wrist centre is WRIST_OFFSET straight above the TCP; it must be close
    enough to the shoulder for the upper arm + forearm to reach it, and the
    TCP must stay outside the robot's base column.  This costs a few
    microseconds and saves running IK on hopeless candidates.
    """
    wrist = np.asarray(p) + np.array([0.0, 0.0, C.WRIST_OFFSET])
    return (np.linalg.norm(wrist - C.SHOULDER) <= C.WRIST_REACH
            and np.linalg.norm(p[:2]) >= C.MIN_RADIUS_XY)


def best_grasp_ik(p, v, q_now, speed_scale, rng):
    """IK for both gripper orientations; returns the option the arm reaches
    fastest as (T_pre, T_grasp, q_pre, q_grasp, T1, T2), or None."""
    best = None
    for flipped in (False, True):
        T_pre, T_grasp = grasp_poses(p, v, flipped)
        q_pre, ok, _ = solve_ik(T_pre, q_now, n_random_seeds=2, rng=rng)
        if not ok:
            continue
        q_grasp, ok, _ = solve_ik(T_grasp, q_pre, n_random_seeds=2, rng=rng)
        if not ok:
            continue
        T1 = quintic_duration(q_now, q_pre, speed_scale)      # approach
        T2 = quintic_duration(q_pre, q_grasp, speed_scale)    # descend
        if best is None or T1 + T2 < best[4] + best[5]:
            best = (T_pre, T_grasp, q_pre, q_grasp, T1, T2)
    return best


def plan_intercept(p_now, v, q_now, speed_scale=1.0, rng=None):
    """Forward-scan intercept search.

    Returns (plan, candidates).  ``plan`` is None when no candidate works,
    otherwise a dict with the intercept time/position, joint targets and
    segment durations.  ``candidates`` is a list of dicts describing every
    time that was tried.
    """
    rng = rng if rng is not None else np.random.default_rng(0)
    candidates = []
    t = C.PLAN_DT
    while t <= C.PLAN_HORIZON:
        p = predict_ball(p_now, v, t)
        entry = {"t": round(t, 3), "p": [round(x, 4) for x in p]}
        candidates.append(entry)

        T_pre, T_grasp = grasp_poses(p, v)

        # 1. REACH ───────────────────────────────────────────────────────────
        if not (in_workspace(T_pre[:3, 3]) and in_workspace(T_grasp[:3, 3])):
            entry["result"] = "out of reach"
            t += C.PLAN_DT
            continue

        # 2. IK ──────────────────────────────────────────────────────────────
        ik = best_grasp_ik(p, v, q_now, speed_scale, rng)
        if ik is None:
            entry["result"] = "no IK solution"
            t += C.PLAN_DT
            continue
        T_pre, T_grasp, q_pre, q_grasp, T1, T2 = ik

        # 3. TIME ────────────────────────────────────────────────────────────
        needed = T1 + T2 + C.GRIPPER_CLOSE_TIME + C.SAFETY_MARGIN
        entry["needed"] = round(needed, 3)
        if needed > t:
            entry["result"] = "too late"
            t += C.PLAN_DT
            continue

        # 4. ACCEPT ──────────────────────────────────────────────────────────
        entry["result"] = "accepted"
        plan = {
            "t_intercept": t,
            "p_intercept": p,
            "T_pregrasp": T_pre,
            "T_grasp": T_grasp,
            "q_pregrasp": q_pre,
            "q_grasp": q_grasp,
            "T_approach": T1,
            "T_descend": T2,
            "slack": t - needed + C.SAFETY_MARGIN,
        }
        return plan, candidates

    return None, candidates
