"""
Runs the real controller against the kinematic fake in ``fake_webots/`` to
check the state machine end to end (launch → … → CAUGHT, teaching-mode
pauses, reset, IK playground).  Physics is *not* tested here.
"""

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tests" / "fake_webots"))
sys.path.insert(0, str(ROOT / "controllers" / "dynamic_grasping"))

from dynamic_grasping import DynamicGraspingDemo   # noqa: E402


def make_demo():
    demo = DynamicGraspingDemo()
    demo.perception = "simulator"
    demo.q_des = demo.q()
    demo.q_hold = demo.q()
    return demo


def run_until(demo, predicate, max_time=30.0):
    sup = demo.robot
    while sup.time < max_time:
        sup.step(demo.dt_ms)
        message = sup.wwiReceiveText()
        while message:
            demo.handle_message(message)
            message = sup.wwiReceiveText()
        demo.step_state()
        demo.control_arm()
        if predicate():
            return True
    return False


def send(demo, **msg):
    demo.robot.inbox.append(json.dumps(msg))


def test_catch_rolling_ball():
    demo = make_demo()
    assert run_until(demo, lambda: demo.state == "IDLE")
    send(demo, cmd="launch", x=0.5, y=-0.8, speed=0.3, heading=90)
    assert run_until(demo, lambda: demo.state in ("CAUGHT", "MISSED", "DROPPED", "NO_PLAN"))
    assert demo.state == "CAUGHT", [m for m in demo.robot.outbox if '"log"' in m][-3:]


def test_teaching_mode_pauses_and_resumes():
    demo = make_demo()
    run_until(demo, lambda: demo.state == "IDLE")
    send(demo, cmd="teach", on=True)
    send(demo, cmd="launch", x=0.5, y=-0.8, speed=0.3, heading=90)
    pauses = []
    for _ in range(6):
        if not run_until(demo, lambda: demo.state in ("PAUSED", "CAUGHT", "MISSED", "DROPPED")):
            break
        if demo.state != "PAUSED":
            break
        pauses.append(demo.paused_next)
        ball_before = list(demo.robot.ball.p)
        run_until(demo, lambda: False, max_time=demo.robot.time + 1.0)   # sit paused for 1 s
        assert ball_before == list(demo.robot.ball.p)                    # ball is frozen
        send(demo, cmd="next")
        current = demo.paused_next
        # Some phases (PLAN) finish within one step and pause again at once.
        run_until(demo, lambda: demo.state != "PAUSED" or demo.paused_next != current)
    assert pauses == ["PLAN", "APPROACH", "DESCEND", "WAIT", "LIFT"]
    run_until(demo, lambda: demo.state in ("CAUGHT", "MISSED", "DROPPED"))
    assert demo.state == "CAUGHT"


def test_teaching_mode_with_camera_keeps_predictions_consistent():
    demo = make_demo()
    demo.perception = "camera"
    run_until(demo, lambda: demo.state == "IDLE")
    send(demo, cmd="teach", on=True)
    send(demo, cmd="launch", x=0.5, y=-0.8, speed=0.4, heading=90)
    for _ in range(5):
        run_until(demo, lambda: demo.state in ("PAUSED", "CAUGHT", "MISSED", "DROPPED"))
        if demo.state != "PAUSED":
            break
        current = demo.paused_next
        run_until(demo, lambda: False, max_time=demo.robot.time + 2.0)    # long pause
        send(demo, cmd="next")
        run_until(demo, lambda: demo.state != "PAUSED" or demo.paused_next != current)
    run_until(demo, lambda: demo.state in ("CAUGHT", "MISSED", "DROPPED"), max_time=60)
    assert demo.state == "CAUGHT"
    assert abs(demo.v_est[1] - 0.4) < 0.01          # pauses did not distort the velocity


def test_camera_ignores_stale_frames_after_relaunch():
    """Webots renders camera frames every 16 ms: right after the ball is moved,
    the newest frame can still show it at its old place (found in Webots)."""
    # Try every launch moment within the camera's 16 ms frame cycle (4 steps).
    for offset in range(4):
        demo = make_demo()
        demo.perception = "camera"
        run_until(demo, lambda: demo.state == "IDLE")
        run_until(demo, lambda: False, max_time=demo.robot.time + offset * 0.004 + 1e-6)
        send(demo, cmd="launch", x=0.8, y=-0.6, speed=0.3, heading=130)
        run_until(demo, lambda: demo.state in ("PLAN", "APPROACH", "NO_PLAN"))
        assert abs(demo.v_est[0] + 0.1928) < 0.03 and abs(demo.v_est[1] - 0.2298) < 0.03, \
            f"offset {offset}: velocity estimate {demo.v_est} polluted by a stale frame"


def test_catch_with_camera_perception():
    demo = make_demo()
    demo.perception = "camera"
    run_until(demo, lambda: demo.state == "IDLE")
    send(demo, cmd="launch", x=0.8, y=-0.6, speed=0.3, heading=130)
    assert run_until(demo, lambda: demo.state in ("CAUGHT", "MISSED", "DROPPED", "NO_PLAN"))
    assert demo.state == "CAUGHT"
    # the fitted velocity is close to the true one (0.3 m/s at 130°)
    assert abs(demo.v_est[0] - (-0.1928)) < 0.01 and abs(demo.v_est[1] - 0.2298) < 0.01


def test_camera_back_projection_is_exact():
    demo = make_demo()
    demo.robot.ball.p = __import__("numpy").array([0.78, -0.58, 0.03])
    demo.robot.devices["overhead_camera"].snapshot = demo.robot.ball.p.copy()   # render a frame
    d = demo.camera.detect()
    assert d is not None and abs(d["p"][0] - 0.78) < 0.004 and abs(d["p"][1] + 0.58) < 0.004


def test_unreachable_ball_reports_no_plan():
    demo = make_demo()
    run_until(demo, lambda: demo.state == "IDLE")
    send(demo, cmd="launch", x=1.5, y=-1.0, speed=0.3, heading=90)
    assert run_until(demo, lambda: demo.state == "NO_PLAN")


def test_reset_and_ik_playground():
    demo = make_demo()
    run_until(demo, lambda: demo.state == "IDLE")
    send(demo, cmd="launch", x=0.55, y=0.1, speed=0.0, heading=0)
    run_until(demo, lambda: demo.state == "CAUGHT")
    send(demo, cmd="reset")
    assert run_until(demo, lambda: demo.state == "IDLE")
    send(demo, cmd="ik_goal", x=0.4, y=0.3, z=0.3, yaw=30)
    assert run_until(demo, lambda: demo.state == "IK_MOVE")
    assert run_until(demo, lambda: demo.state == "IDLE")
    ik = [json.loads(m) for m in demo.robot.outbox if '"type": "ik"' in m][-1]
    assert ik["ok"] and ik["error"] < 1e-3


def test_bad_messages_are_reported_not_fatal():
    demo = make_demo()
    run_until(demo, lambda: demo.state == "IDLE")
    for bad in ("not json", "[1, 2]", '{"cmd": "launch", "x": 0.5}',
                '{"cmd": "grip_force", "value": "strong"}', '{"cmd": "arm_speed", "value": NaN}'):
        demo.robot.inbox.append(bad)
    send(demo, cmd="arm_speed", value=0)          # would divide by zero in quintic_duration
    send(demo, cmd="grip_force", value=-3)
    run_until(demo, lambda: not demo.robot.inbox)
    warnings = [json.loads(m) for m in demo.robot.outbox if '"Ignored a bad window message' in m]
    assert len(warnings) == 7
    assert demo.speed_scale == 1.0 and demo.gripper.grip_force > 0
    send(demo, cmd="launch", x=0.5, y=-0.8, speed=0.3, heading=90)   # still works afterwards
    assert run_until(demo, lambda: demo.state == "CAUGHT")


def test_launch_inside_the_base_is_refused():
    demo = make_demo()
    run_until(demo, lambda: demo.state == "IDLE")
    send(demo, cmd="launch", x=0.1, y=0.05, speed=0.3, heading=0)
    run_until(demo, lambda: not demo.robot.inbox)
    assert demo.state == "IDLE"
    assert any("inside the robot's base column" in m for m in demo.robot.outbox)


def test_nothing_is_streamed_before_the_window_says_hello():
    demo = make_demo()
    run_until(demo, lambda: demo.state == "IDLE")
    assert not demo.window_connected and not demo.camera_view
    send(demo, cmd="hello", camera_view=False)
    run_until(demo, lambda: not demo.robot.inbox)
    assert demo.window_connected and not demo.camera_view
    send(demo, cmd="camera_view", on=True)
    run_until(demo, lambda: not demo.robot.inbox)
    assert demo.camera_view
