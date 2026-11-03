"""
check.py — tests your exercise solutions against the robot's own code.

    python3 exercises/check.py        check every exercise
    python3 exercises/check.py 3      check exercise 3 only
    python3 exercises/check.py --folder=DIR   check the files in DIR (e.g. a student's copy)

The lesson window's "Exercises" tab runs the same checks, with
``--json``: one JSON line per exercise, in a separate process, so that a
mistake such as an endless loop cannot freeze the simulation.
"""

import importlib.util
import json
import sys
import traceback
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "controllers" / "dynamic_grasping"))

import config as C                                         # noqa: E402
import kinematics as ref_kin                               # noqa: E402
from trajectory import QuinticTrajectory                   # noqa: E402


def _rng():
    return np.random.default_rng(42)


# ─────────────────────────────────────────────────────────────────────────────
#  One checker per exercise: raise AssertionError with a helpful hint
# ─────────────────────────────────────────────────────────────────────────────

def check_1(m):
    for q in _rng().uniform(C.Q_MIN, C.Q_MAX, (20, 7)):
        T = np.asarray(m.forward_kinematics(q))
        assert T.shape == (4, 4), "Return a 4×4 matrix."
        expected = ref_kin.forward_kinematics(q)
        assert np.allclose(T[:3, 3], expected[:3, 3], atol=1e-6), \
            "The position is wrong. Multiply the exponentials in joint order 1…7 and end with M_HOME (on the right)."
        assert np.allclose(T, expected, atol=1e-6), "The position is right but the orientation is not."


def check_2(m):
    q0, q1 = C.Q_HOME, C.Q_HOME + np.array([0.8, -0.4, 0.3, 0.5, -0.2, 0.4, -1.0])
    T = 1.7
    ref = QuinticTrajectory(q0, q1, T)
    for t in np.linspace(0, T, 9):
        got = np.asarray(m.quintic_position(q0, q1, T, t))
        assert np.allclose(got, ref.sample(t)[0], atol=1e-9), \
            f"quintic_position is wrong at t = {t:.2f} s. Check τ = t/T and s = 10τ³ − 15τ⁴ + 6τ⁵."
    got = float(m.quintic_duration(q0, q1, C.DQ_MAX))
    expected = float(np.max(15 * np.abs(q1 - q0) / (8 * C.DQ_MAX)))
    assert abs(got - expected) < 1e-9, \
        "quintic_duration is wrong. For each joint 15·|Δq|/(8·dq_max), then take the largest."


def check_3(m):
    rng = _rng()
    for _ in range(10):
        J = rng.normal(size=(6, 7))
        V = rng.normal(size=6)
        lam = 0.05
        expected = J.T @ np.linalg.solve(J @ J.T + lam ** 2 * np.eye(6), V)
        got = np.asarray(m.ik_step(J, V, lam))
        assert got.shape == (7,), "Δq must have 7 entries (one per joint)."
        assert np.allclose(got, expected, atol=1e-9), \
            "Not quite. Δq = Jᵀ · solve(J Jᵀ + λ² I, V). Is the damping squared?"


def check_4(m):
    rng = _rng()
    times = np.arange(0.0, 0.2, 0.016) + 3.0
    v_true = np.array([0.2, -0.3, 0.0])
    p_last = np.array([0.6, 0.1, 0.03])
    positions = p_last + np.outer(times - times[-1], v_true) + rng.normal(0, 0.002, (len(times), 3))
    p0, v = (np.asarray(x) for x in m.fit_line(times, positions))
    A = np.column_stack([np.ones_like(times), times - times[-1]])
    (e_p0, e_v), *_ = np.linalg.lstsq(A, positions, rcond=None)
    assert np.allclose(v, e_v, atol=1e-6), \
        "The velocity is wrong. Each row of A is [1, t_k − t_last]; solve with np.linalg.lstsq(A, positions)."
    assert np.allclose(p0, e_p0, atol=1e-6), "The velocity is right; p0 should be the position at the last sample time."


def check_5(m):
    for mass, mu in ((0.1, 1.5), (0.5, 0.8), (2.0, 0.3)):
        expected = mass * 9.81 / (2 * mu)
        got = float(m.min_grip_force(mass, mu))
        assert abs(got - expected) < 1e-6, "min_grip_force: two fingers, each gives μ·N, together they carry m·g."
    assert m.holds_while_lifting(0.5, 0.1, 1.5, 2.0), \
        "0.5 N should hold a 0.1 kg ball lifted at 2 m/s² (needed: 0.1·(9.81+2)/3 = 0.39 N)."
    assert not m.holds_while_lifting(0.35, 0.1, 1.5, 2.0), \
        "0.35 N is enough to hold still (0.33 N) but not while accelerating at 2 m/s² (0.39 N needed)."


def check_6(m):
    c = np.array([0.6, -0.1, 2.5])
    R = np.array([[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]])   # looking straight down
    W, H, fov = 320, 240, 1.2
    f = (W / 2) / np.tan(fov / 2)
    for (u, v) in ((160.0, 120.0), (204.65, 102.35), (10.0, 230.0)):
        ray = R @ np.array([f, -(u + 0.5 - W / 2), -(v + 0.5 - H / 2)])
        expected = c + (0.03 - c[2]) / ray[2] * ray
        got = np.asarray(m.pixel_to_floor(u, v, W, H, fov, c, R, 0.03))
        assert np.allclose(got, expected, atol=1e-6), \
            f"Wrong point for pixel ({u}, {v}). Remember the minus signs: image right is camera −y, image down is camera −z."


EXERCISES = [
    (1, "ex1_forward_kinematics", "Forward kinematics (PoE)", check_1),
    (2, "ex2_quintic", "Quintic trajectory", check_2),
    (3, "ex3_ik_step", "One IK step (damped least squares)", check_3),
    (4, "ex4_velocity_fit", "Velocity from noisy positions", check_4),
    (5, "ex5_grip_force", "Grip force and friction", check_5),
    (6, "ex6_pixel_to_floor", "From pixel to floor position", check_6),
]


def load(module_name, folder):
    """Load an exercise file afresh, so the latest edits are always used."""
    spec = importlib.util.spec_from_file_location(module_name, Path(folder) / f"{module_name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def check_one(n, module_name, title, checker, folder=HERE):
    """Check one exercise; returns a dict (number, title, status, message)."""
    try:
        checker(load(module_name, folder))
        return {"n": n, "title": title, "status": "pass", "message": "Correct!"}
    except NotImplementedError:
        return {"n": n, "title": title, "status": "todo",
                "message": f"Not started: open exercises/{module_name}.py"}
    except AssertionError as e:
        return {"n": n, "title": title, "status": "fail", "message": str(e)}
    except Exception as e:                               # a bug in the student's code
        last = traceback.extract_tb(e.__traceback__)[-1]
        return {"n": n, "title": title, "status": "error",
                "message": f"{type(e).__name__}: {e} (line {last.lineno})"}


def selected(numbers=None):
    return [ex for ex in EXERCISES if not numbers or ex[0] in numbers]


def run(numbers=None, folder=HERE):
    """Run the checks; returns a list of dicts (number, title, status, message)."""
    return [check_one(*ex, folder=folder) for ex in selected(numbers)]


if __name__ == "__main__":
    args = sys.argv[1:]
    as_json = "--json" in args
    folders = [a.split("=", 1)[1] for a in args if a.startswith("--folder=")]
    folder = Path(folders[-1]) if folders else HERE
    wanted = {int(a) for a in args if not a.startswith("--")} or None
    icons = {"pass": "✔", "todo": "·", "fail": "✘", "error": "!"}
    if as_json:
        # First the list of exercises, then one line per result as soon as it
        # is known: if an exercise never finishes, the caller still gets the
        # results before it and knows which one is stuck.
        print(json.dumps({"exercises": [[n, title] for n, _, title, _ in selected(wanted)]}), flush=True)
    for ex in selected(wanted):
        r = check_one(*ex, folder=folder)
        if as_json:
            print(json.dumps(r), flush=True)
        else:
            print(f"{icons[r['status']]} Exercise {r['n']}: {r['title']:<38} {r['message']}")
