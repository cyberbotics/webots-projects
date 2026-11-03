"""The exercise checker: blank templates are 'todo', reference solutions pass."""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "exercises"))

import check   # noqa: E402


def test_templates_are_reported_as_not_started():
    assert {r["status"] for r in check.run()} == {"todo"}


def test_reference_solutions_pass():
    results = check.run(folder=ROOT / "tests" / "exercise_solutions")
    assert [r["status"] for r in results] == ["pass"] * 6, results


def test_wrong_answer_gets_a_hint(tmp_path):
    (tmp_path / "ex5_grip_force.py").write_text(
        "def min_grip_force(mass, mu):\n    return mass * 9.81 / mu\n"
        "def holds_while_lifting(f, m, mu, a):\n    return True\n")
    [r] = check.run([5], folder=tmp_path)
    assert r["status"] == "fail" and "two fingers" in r["message"]


def test_endless_loop_times_out_without_freezing(tmp_path, monkeypatch):
    """The window runs the checker in a separate process with a time limit."""
    import time
    import shutil
    sys.path.insert(0, str(ROOT / "tests" / "fake_webots"))
    sys.path.insert(0, str(ROOT / "controllers" / "dynamic_grasping"))
    import dynamic_grasping as dg
    for f in (ROOT / "tests" / "exercise_solutions").glob("ex*.py"):
        shutil.copy(f, tmp_path)
    (tmp_path / "ex3_ik_step.py").write_text("def ik_step(J, V, lam):\n    while True:\n        pass\n")
    monkeypatch.setattr(dg, "EXERCISE_TIMEOUT", 3.0)
    run = dg.ExerciseRun([f"--folder={tmp_path}"])
    while (results := run.poll()) is None:      # the simulation would keep stepping here
        time.sleep(0.05)
    assert [r["status"] for r in results] == ["pass", "pass", "error", "error", "error", "error"]
    assert "endless loop" in results[2]["message"] and "Not checked" in results[3]["message"]


def test_checker_output_with_student_prints():
    sys.path.insert(0, str(ROOT / "tests" / "fake_webots"))
    sys.path.insert(0, str(ROOT / "controllers" / "dynamic_grasping"))
    import dynamic_grasping as dg
    out = ('{"exercises": [[1, "FK"]]}\nhello from my code\n'
           '{"n": 1, "title": "FK", "status": "pass", "message": "Correct!"}\n')
    assert dg.ExerciseRun.parse(out, "", False)[0]["status"] == "pass"
