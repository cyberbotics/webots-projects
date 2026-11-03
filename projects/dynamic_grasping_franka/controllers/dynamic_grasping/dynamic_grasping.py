"""
dynamic_grasping.py — Webots controller
=======================================
A Franka Panda catches a ball that slides across the floor.

The controller is a state machine.  Each state is one idea you can study on
its own (the robot window explains them as they happen):

    IDLE ─launch─► OBSERVE ─► PLAN ─► APPROACH ─► DESCEND ─► WAIT ─► CLOSE ─► LIFT ─► CAUGHT
                  (perception) (where/when?) (quintic)  (quintic) (timing) (gripper)  (check)

    OBSERVE  find the ball (camera or simulator) and estimate its velocity → perception.py
    PLAN     forward-scan intercept search          → intercept_planner.py
    APPROACH move above the meeting point           → trajectory.py
    DESCEND  correct the meeting point with the latest tracking, then lower the
             open gripper around the ball's path
    WAIT     close exactly when the ball arrives    → gripper.py
    CLOSE    squeeze with a limited force
    LIFT     raise the ball and check it is still held

Joint control (one law for every joint, every time step):

    dq_cmd = dq_desired + KP · (q_desired − q_measured)

i.e. velocity feed-forward from the trajectory plus a proportional
correction; Webots' motors then apply whatever torque (within limits)
achieves dq_cmd.

The robot window ("grasping_window") talks to this file through JSON
messages: see ``handle_message`` (window → controller) and the ``send_*``
helpers (controller → window).

Optional controllerArgs (in the world file):
    --launch=x,y,speed,heading   launch this ball automatically once the arm is home
    --grip-force=N               start with this grip force instead of the default
    --exit-on-result             quit Webots when the catch ends (for automated checks)
    --script=file.json           play a list of window commands (see command_script.py)
    --perception=camera|simulator  where ball positions come from (default: config.py)
    --record-movie=file.mp4      record the 3D view of the first launch, until shortly after its result
"""

import json
import math
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
from controller import Supervisor

import config as C
from command_script import CommandScript
from gripper import Gripper, min_grip_force
from intercept_planner import plan_intercept
from kinematics import forward_kinematics, solve_ik
from perception import OverheadCamera
from scene_markers import SceneMarkers
from trajectory import QuinticTrajectory, quintic_duration

PAUSE_POINTS = ("PLAN", "APPROACH", "DESCEND", "WAIT", "LIFT")   # teaching-mode stops
TELEMETRY_PERIOD = 0.02    # s
TRAIL_PERIOD = 0.04        # s
CAMERA_STREAM_PERIOD = 0.25  # s between camera images sent to the robot window
MOVIE_TAIL = 1.0           # s of video kept after the result, so the movie shows it
EXERCISE_TIMEOUT = 10.0    # s (wall clock) before the exercise checker is stopped
CHECK_SCRIPT = Path(__file__).resolve().parents[2] / "exercises" / "check.py"


class DynamicGraspingDemo:

    def __init__(self):
        self.robot = Supervisor()
        self.dt_ms = int(self.robot.getBasicTimeStep())
        self.dt = self.dt_ms / 1000.0

        # Arm motors in velocity mode (position = ∞ disables Webots' own
        # position loop, so our control law is the only one acting).
        self.motors = [self.robot.getDevice(n) for n in C.JOINT_NAMES]
        self.sensors = []
        for m in self.motors:
            m.setPosition(float("inf"))
            m.setVelocity(0.0)
            s = m.getPositionSensor()
            s.enable(self.dt_ms)
            self.sensors.append(s)
        # The tighter of the motor limits and Franka's datasheet limits.
        self.v_limits = np.minimum([m.getMaxVelocity() for m in self.motors], C.DQ_MAX)

        self.gripper = Gripper(self.robot, self.dt_ms)
        self.tcp_gps = self.robot.getDevice("tcp_gps")
        self.tcp_gps.enable(self.dt_ms)

        self.ball = self.robot.getFromDef("BALL")
        self.camera = OverheadCamera(self.robot)
        self.perception = C.PERCEPTION
        self.markers = SceneMarkers(self.robot)
        self.rng = np.random.default_rng()

        # Learner-adjustable settings
        self.speed_scale = 1.0
        self.teach_mode = False

        # State
        self.state = "START"
        self.state_time = 0.0
        self.traj = None
        self.q_hold = None
        self.plan = None
        self.candidates = []
        self.observations = []
        self.v_est = np.zeros(3)
        self.paused_next = None
        self.frozen_ball_velocity = None
        self.last_telemetry = -1.0
        self.last_trail = -1.0
        self.last_camera_sample = -1.0
        self.last_camera_stream = -1.0
        self.p_est = np.zeros(3)        # ball position estimated at time t_est
        self.t_est = 0.0
        self.t_meet = 0.0               # absolute time the plan meets the ball
        self.launch = {"x": 0.5, "y": -0.8, "speed": 0.3, "heading": 90.0}
        self.auto_launch = None
        self.exit_on_result = False
        self.script = None
        self.quit_time = None
        self.movie_file = None
        self.movie_state = None         # None → "recording" → "encoding" → "done"
        self.movie_stop_time = None
        self.window_connected = False   # stream to the robot window only once it said hello
        self.camera_view = False        # ... and camera images only while the Camera tab is shown
        self.exercise_run = None
        for arg in sys.argv[1:]:
            if arg.startswith("--launch="):
                self.auto_launch = [float(v) for v in arg.split("=", 1)[1].split(",")]
            elif arg.startswith("--grip-force="):
                self.gripper.grip_force = float(arg.split("=", 1)[1])
            elif arg == "--exit-on-result":
                self.exit_on_result = True
            elif arg.startswith("--record-movie="):
                self.movie_file = arg.split("=", 1)[1]
            elif arg.startswith("--perception="):
                self.perception = arg.split("=", 1)[1]
            elif arg.startswith("--script="):
                self.script = CommandScript(arg.split("=", 1)[1])

    # ═════════════════════════════════════════════════════════════════════════
    #  Low-level helpers
    # ═════════════════════════════════════════════════════════════════════════
    def q(self):
        return np.array([s.getValue() for s in self.sensors])

    def ball_position(self):
        return np.array(self.ball.getPosition())

    def tcp_measured(self):
        return np.array(self.tcp_gps.getValues())

    def send(self, msg_type, **data):
        data["type"] = msg_type
        self.robot.wwiSendText(json.dumps(data, default=_to_json))

    def log(self, text, level="info"):
        print(text)
        self.send("log", text=text, level=level, t=round(self.robot.getTime(), 2))

    def set_state(self, new_state, allow_pause=True):
        """Change state; in teaching mode, stop at the interesting boundaries."""
        if allow_pause and self.teach_mode and new_state in PAUSE_POINTS:
            self.paused_next = new_state
            self.freeze_ball()
            self.state = "PAUSED"
            self.q_hold = self.q()
            self.send("phase", phase="PAUSED", next=new_state)
            self.robot.setLabel(0, f"Paused - next: {new_state}   (press Next in the robot window)",
                                0.01, 0.01, 0.07, 0xFFD000, 0.0, "Arial")
            return
        self.state = new_state
        self.state_time = self.robot.getTime()
        self.send("phase", phase=new_state)
        self.robot.setLabel(0, f"Phase: {new_state}", 0.01, 0.01, 0.08, 0xFFFFFF, 0.0, "Arial")

    def freeze_ball(self):
        self.frozen_ball_velocity = self.ball.getVelocity()
        self.frozen_at = self.robot.getTime()
        self.ball.setVelocity([0.0] * 6)

    def unfreeze_ball(self):
        if self.frozen_ball_velocity is not None:
            self.ball.setVelocity(self.frozen_ball_velocity)
            self.frozen_ball_velocity = None
            # The ball stood still during the pause: shift the prediction too.
            pause = self.robot.getTime() - self.frozen_at
            self.t_est += pause
            self.t_meet += pause
            # Old camera frames too, or the line fit would see a ball that "stood still".
            self.observations = [(t + pause, p) for t, p in self.observations]

    def elapsed(self):
        return self.robot.getTime() - self.state_time

    def start_motion(self, q_target, duration=None):
        q_now = self.q()
        if duration is None:
            duration = quintic_duration(q_now, q_target, self.speed_scale)
        self.traj = QuinticTrajectory(q_now, q_target, duration)
        self.q_hold = np.array(q_target)

    # ═════════════════════════════════════════════════════════════════════════
    #  Joint control law
    # ═════════════════════════════════════════════════════════════════════════
    def control_arm(self):
        q = self.q()
        if self.traj is not None and self.state in ("APPROACH", "DESCEND", "LIFT", "GO_HOME", "IK_MOVE"):
            q_des, dq_des = self.traj.sample(self.elapsed())
        else:
            q_des, dq_des = self.q_hold, np.zeros(7)
        dq_cmd = dq_des + C.KP_JOINT * (q_des - q)
        dq_cmd = np.clip(dq_cmd, -0.99 * self.v_limits, 0.99 * self.v_limits)
        for motor, v in zip(self.motors, dq_cmd):
            motor.setVelocity(float(v))
        self.q_des = q_des

    # ═════════════════════════════════════════════════════════════════════════
    #  Scene setup
    # ═════════════════════════════════════════════════════════════════════════
    def place_ball(self, x, y, velocity=(0.0, 0.0, 0.0)):
        self.ball.getField("translation").setSFVec3f([x, y, C.BALL_RADIUS])
        self.ball.getField("rotation").setSFRotation([0, 0, 1, 0])
        if np.linalg.norm(velocity) < 1e-9:
            self.ball.resetPhysics()          # make it sit perfectly still
        else:
            self.ball.setVelocity([velocity[0], velocity[1], 0.0, 0.0, 0.0, 0.0])

    def do_launch(self, x, y, speed, heading_deg):
        if self.state != "IDLE":
            self.log("Press Reset first so the arm returns home, then launch.", "warn")
            return
        if math.hypot(x, y) < C.MIN_RADIUS_XY:
            self.log(f"Launch refused: ({x:.2f}, {y:.2f}) is inside the robot's base column. "
                     f"Start the ball at least {C.MIN_RADIUS_XY:.2f} m from the base.", "warn")
            return
        if speed < 0:
            self.log("Launch refused: the speed cannot be negative.", "warn")
            return
        self.launch = {"x": x, "y": y, "speed": speed, "heading": heading_deg}
        h = np.radians(heading_deg)
        v = np.array([np.cos(h), np.sin(h), 0.0]) * speed
        self.markers.clear()
        self.plan = None
        self.candidates = []
        self.place_ball(x, y, v)
        self.observations = []
        self.tracked_frames = 0
        self.gripper.open()
        self.log(f"Ball launched from ({x:.2f}, {y:.2f}) at {speed:.2f} m/s, heading {heading_deg:.0f}°")
        self.start_movie()
        self.set_state("OBSERVE")

    def do_reset(self):
        if self.state == "PAUSED":
            self.unfreeze_ball()
        self.gripper.open()
        self.markers.clear()
        self.place_ball(self.launch["x"], self.launch["y"])
        self.plan = None
        self.start_motion(C.Q_HOME)
        self.robot.setLabel(1, "", 0, 0, 0.1, 0, 0, "Arial")
        self.set_state("GO_HOME")

    # ═════════════════════════════════════════════════════════════════════════
    #  States
    # ═════════════════════════════════════════════════════════════════════════
    def step_state(self):
        st = self.state
        t = self.elapsed()

        if st == "START":
            self.gripper.open()
            self.place_ball(self.launch["x"], self.launch["y"])
            self.start_motion(C.Q_HOME, duration=3.0)
            self.set_state("GO_HOME")

        elif st in ("GO_HOME", "IK_MOVE"):
            if self.traj.finished(t):
                self.set_state("IDLE")
                if self.auto_launch:
                    launch, self.auto_launch = self.auto_launch, None
                    self.do_launch(*launch)

        elif st == "OBSERVE":
            # Perception: collect ball positions, then fit p(t) = p0 + v t.
            # Supervisor changes (placing the ball) take effect one step later,
            # and a camera frame is only rendered every camera period, so the
            # first images can still show the ball at its old place.  Skip them.
            settle = 2 * self.dt + (2 * self.camera.period if self.perception == "camera" else 0.0)
            if t < settle:
                return
            sample = self.measure_ball()
            if sample is not None:
                self.observations.append((self.robot.getTime(), sample))
            if t >= C.OBSERVE_TIME and len(self.observations) >= 3:
                self.fit_ball_motion()
                self.markers.predicted_path(self.predicted_ball(), self.v_est, C.PLAN_HORIZON)
                self.set_state("PLAN")
            elif t >= C.OBSERVE_TIMEOUT:
                self.finish("MISSED", "The camera could not see the ball (is it under the arm "
                                      "or outside the image?).")

        elif st == "PLAN":
            p_now = self.predicted_ball()       # what the robot believes, not the truth
            q_now = self.q()
            self.plan, self.candidates = plan_intercept(p_now, self.v_est, q_now,
                                                        self.speed_scale, self.rng)
            self.markers.candidates(self.candidates)
            self.send_plan(p_now)
            if self.plan is None:
                self.log("No intercept found: the ball never passes where the arm can be in time.", "warn")
                self.robot.setLabel(1, "NO FEASIBLE INTERCEPT", 0.35, 0.45, 0.15, 0xFF5050, 0, "Arial")
                self.set_state("NO_PLAN")
                self.schedule_movie_stop()
                if self.exit_on_result:
                    self.quit_later(1)
                return
            p = self.plan
            self.log(f"Intercept at t = {p['t_intercept']:.2f} s, position "
                     f"({p['p_intercept'][0]:.3f}, {p['p_intercept'][1]:.3f}); "
                     f"approach {p['T_approach']:.2f} s + descend {p['T_descend']:.2f} s, "
                     f"slack {p['slack']:.2f} s")
            self.t_meet = self.robot.getTime() + p["t_intercept"]   # absolute meeting time
            self.markers.frame(p["T_pregrasp"])
            self.markers.frame(p["T_grasp"])
            self.markers.start_trail(self.tcp_measured())
            self.set_state("APPROACH")
            if self.state == "APPROACH":
                self.start_motion(p["q_pregrasp"], p["T_approach"])

        elif st == "APPROACH":
            if self.traj.finished(t):
                self.set_state("DESCEND")
                if self.state == "DESCEND":
                    self.start_descend()

        elif st == "DESCEND":
            if self.traj.finished(t):
                self.log(f"Tracking: the ball estimate was refreshed from {self.tracked_frames} "
                         f"more measurements while the arm moved")
                self.set_state("WAIT")

        elif st == "WAIT":
            # Close when the ball's arrival time equals the fingers' travel time.
            # The gripper hides the ball from the camera now, so use the prediction.
            ball, tcp = self.predicted_ball(), self.tcp_measured()
            speed = np.linalg.norm(self.v_est[:2])
            if speed < 1e-3:
                self.start_close()
                return
            direction = self.v_est[:2] / speed
            distance_along_path = float(np.dot(tcp[:2] - ball[:2], direction))
            time_to_arrival = distance_along_path / speed
            if time_to_arrival <= self.gripper.closing_time():
                self.start_close()
            elif distance_along_path < -0.05 or t > 10.0:
                self.finish("MISSED", "The ball passed before the gripper closed.")

        elif st == "CLOSE":
            fingers = self.gripper.positions()
            if min(fingers) < C.BALL_RADIUS - 0.01:    # closed past the ball's surface
                self.finish("MISSED", "Fingers closed on nothing — the ball was not between them.")
            elif t > 0.25:                       # fingers have settled on the ball
                self.log(f"Fingers stopped at {fingers[0]*100:.1f} / {fingers[1]*100:.1f} cm, "
                         f"squeezing with {np.mean(self.gripper.forces()):.1f} N each")
                self.set_state("LIFT")
                if self.state == "LIFT":
                    self.start_lift()

        elif st == "LIFT":
            if self.traj.finished(t):
                ball, tcp = self.ball_position(), self.tcp_measured()
                if np.linalg.norm(ball - tcp) < 0.05 and ball[2] > 0.1:
                    self.finish("CAUGHT", "Ball caught and lifted!")
                else:
                    self.finish("DROPPED", "The ball slipped out — try a larger grip force.")

        elif st == "PAUSED":
            pass                                  # waiting for "next"

    # ── perception helpers ──────────────────────────────────────────────────
    def measure_ball(self, near=None):
        """One ball position measurement, or None (no new frame / not visible)."""
        if self.perception != "camera":
            return self.ball_position()
        now = self.robot.getTime()
        if now - self.last_camera_sample < self.camera.period - 1e-6:
            return None                           # no new camera frame yet
        self.last_camera_sample = now
        detection = self.camera.detect(near)
        return None if detection is None else detection["p"]

    def track_ball(self):
        """Keep watching the ball while the arm moves (APPROACH/DESCEND/WAIT).

        New frames are added and the line is re-fitted on the latest
        TRACK_WINDOW seconds, so the closing moment is timed from fresh data
        instead of a long extrapolation.  Frames are ignored once the ball is
        close to the gripper, where the hand hides it from the camera.
        """
        sample = self.measure_ball(near=self.predicted_ball())
        if sample is None:
            return
        if np.linalg.norm(sample[:2] - self.tcp_measured()[:2]) < C.TRACK_MIN_DISTANCE:
            return
        # Gating: ignore a measurement far from where the ball should be
        # (something else orange-ish, a half-hidden ball, …).
        if np.linalg.norm(sample[:2] - self.predicted_ball()[:2]) > C.TRACK_GATE:
            return
        now = self.robot.getTime()
        self.observations.append((now, sample))
        recent = [o for o in self.observations if o[0] >= now - C.TRACK_WINDOW]
        if len(recent) >= 3:
            self.fit_ball_motion(recent, quiet=True)
            self.tracked_frames += 1

    def fit_ball_motion(self, observations=None, quiet=False):
        """Least-squares line through the samples: position and velocity."""
        observations = observations if observations is not None else self.observations
        times = np.array([o[0] for o in observations])
        pos = np.array([o[1] for o in observations])
        A = np.column_stack([np.ones_like(times), times - times[-1]])
        coef, *_ = np.linalg.lstsq(A, pos, rcond=None)
        self.p_est, self.v_est, self.t_est = coef[0], coef[1], times[-1]
        self.p_est[2] = C.BALL_RADIUS
        self.v_est[2] = 0.0
        if quiet:
            return
        source = "camera frames" if self.perception == "camera" else "simulator positions"
        error_mm = np.linalg.norm(self.predicted_ball()[:2] - self.ball_position()[:2]) * 1000
        self.log(f"Estimated ball velocity: ({self.v_est[0]:.3f}, {self.v_est[1]:.3f}) m/s "
                 f"from {len(times)} {source}; position error {error_mm:.1f} mm")

    def predicted_ball(self):
        """Where the robot believes the ball is now (straight-line model)."""
        return self.p_est + self.v_est * (self.robot.getTime() - self.t_est)

    def start_close(self):
        self.gripper.close()
        self.log(f"Closing gripper (grip force {self.gripper.grip_force:.1f} N, "
                 f"minimum needed {min_grip_force():.2f} N)")
        self.set_state("CLOSE")

    def start_descend(self):
        """Correct the grasp pose with the latest ball estimate, then descend.

        The plan was made from 0.2 s of observation.  Since then the camera
        kept tracking the ball, so its predicted position at the meeting time
        is more accurate now; a small sideways error in the first estimate of
        the direction would otherwise leave the ball off-centre in the gripper.
        """
        plan = self.plan
        target = plan["T_grasp"].copy()
        p_meet = self.p_est + self.v_est * (self.t_meet - self.t_est)
        shift = np.linalg.norm(p_meet[:2] - plan["p_intercept"][:2])
        q_grasp = plan["q_grasp"]
        if 1e-4 < shift < C.MAX_CORRECTION:
            target[:2, 3] = p_meet[:2]
            q_new, ok, _ = solve_ik(target, plan["q_grasp"], n_random_seeds=0, rng=self.rng)
            if ok:
                q_grasp = q_new
                self.log(f"Correction before descending: meeting point moved by {shift * 1000:.0f} mm "
                         f"using the latest tracking")
        self.start_motion(q_grasp, max(plan["T_descend"], quintic_duration(self.q(), q_grasp, self.speed_scale)))

    def start_lift(self):
        T = forward_kinematics(self.q())
        T[2, 3] = C.LIFT_HEIGHT
        q_lift, ok, _ = solve_ik(T, self.q(), rng=self.rng)
        if not ok:
            q_lift = self.plan["q_pregrasp"]
        self.start_motion(q_lift, max(1.0, quintic_duration(self.q(), q_lift, 0.5)))

    # ── movie (--record-movie): one recording, from the first launch until
    #    MOVIE_TAIL seconds after its result ─────────────────────────────────
    def start_movie(self):
        if self.movie_file and self.movie_state is None:
            self.robot.movieStartRecording(self.movie_file, 1280, 720, 0, 95, 1, False)
            self.movie_state = "recording"

    def schedule_movie_stop(self):
        if self.movie_state == "recording" and self.movie_stop_time is None:
            self.movie_stop_time = self.robot.getTime() + MOVIE_TAIL

    def update_movie(self):
        if self.movie_state == "recording" and self.movie_stop_time is not None \
                and self.robot.getTime() >= self.movie_stop_time:
            self.robot.movieStopRecording()
            self.movie_state = "encoding"
        elif self.movie_state == "encoding" and self.robot.movieIsReady():
            self.movie_state = "done"
            if self.robot.movieFailed():
                self.log(f"Movie recording failed ({self.movie_file}).", "warn")
            else:
                self.log(f"Movie saved: {self.movie_file}")

    def quit_later(self, code):
        """Quit after a short delay so the last messages reach the console."""
        self.quit_code, self.quit_time = code, self.robot.getTime() + 0.2
        if self.movie_state == "recording":
            self.quit_time += MOVIE_TAIL              # let the movie show the result

    def stop_movie_and_quit(self):
        if self.movie_state == "recording":
            self.movie_stop_time = self.robot.getTime()
        while self.movie_state in ("recording", "encoding"):   # wait for the encoder
            self.update_movie()
            if self.movie_state == "encoding" and self.robot.step(self.dt_ms) == -1:
                return
        self.robot.simulationQuit(self.quit_code)

    def finish(self, outcome, text):
        self.q_hold = self.q()
        self.traj = None
        colors = {"CAUGHT": 0x40FF60, "MISSED": 0xFF5050, "DROPPED": 0xFF9030}
        self.robot.setLabel(1, outcome, 0.4, 0.45, 0.2, colors[outcome], 0, "Arial")
        self.log(text, "success" if outcome == "CAUGHT" else "warn")
        self.state = outcome
        self.send("phase", phase=outcome)
        self.schedule_movie_stop()
        if self.exit_on_result:
            self.quit_later(0 if outcome == "CAUGHT" else 1)

    # ═════════════════════════════════════════════════════════════════════════
    #  Robot-window messages
    # ═════════════════════════════════════════════════════════════════════════
    def handle_message(self, text):
        """Apply one window (or script) command; a bad message is reported, never fatal."""
        try:
            msg = json.loads(text)
            if not isinstance(msg, dict):
                raise ValueError("expected a JSON object")
            self.apply_command(msg)
        except (ValueError, KeyError, TypeError) as e:
            self.log(f"Ignored a bad window message ({type(e).__name__}: {e}): {text[:120]}", "warn")

    def apply_command(self, msg):
        cmd = msg.get("cmd")
        if cmd == "hello":
            self.window_connected = True
            self.camera_view = bool(msg.get("camera_view", False))
            self.send_hello()
        elif cmd == "camera_view":
            self.camera_view = bool(msg["on"])
            self.last_camera_stream = -1.0            # send a fresh image right away
        elif cmd == "launch":
            self.do_launch(*number(msg, "x", "y", "speed", "heading"))
        elif cmd == "reset":
            self.do_reset()
        elif cmd == "teach":
            self.teach_mode = bool(msg["on"])
            if not self.teach_mode and self.state == "PAUSED":
                self.resume()
        elif cmd == "next" and self.state == "PAUSED":
            self.resume()
        elif cmd == "grip_force":
            [force] = number(msg, "value")
            if force <= 0:
                raise ValueError(f"grip force must be positive, got {force}")
            self.gripper.grip_force = force
        elif cmd == "arm_speed":
            [scale] = number(msg, "value")
            if not 0 < scale <= 1:                    # 0 would make every motion last forever
                raise ValueError(f"arm speed must be in (0, 1], got {scale}")
            self.speed_scale = scale
        elif cmd == "markers":
            self.markers.enabled = bool(msg["on"]) and self.markers.children is not None
            if not self.markers.enabled:
                self.markers.clear()
        elif cmd == "perception" and msg.get("value") in ("camera", "simulator"):
            self.perception = msg["value"]
            self.log(f"Ball positions now come from the {msg['value']}.")
        elif cmd == "check_exercises":
            if self.exercise_run is None:
                self.exercise_run = ExerciseRun()
        elif cmd == "ik_goal":
            self.do_ik_goal(msg)

    def resume(self):
        self.unfreeze_ball()
        nxt, self.paused_next = self.paused_next, None
        self.set_state(nxt, allow_pause=False)
        # Motions that begin at this boundary start now.
        if nxt == "APPROACH":
            self.start_motion(self.plan["q_pregrasp"], self.plan["T_approach"])
        elif nxt == "DESCEND":
            self.start_descend()
        elif nxt == "LIFT":
            self.start_lift()

    def do_ik_goal(self, msg):
        if self.state != "IDLE":
            self.log("The IK playground works when the arm is idle (press Reset).", "warn")
            return
        x, y, z = number(msg, "x", "y", "z")
        yaw = np.radians(number({"yaw": 0.0, **msg}, "yaw")[0])
        T = np.eye(4)
        T[:3, :3] = np.array([[np.cos(yaw), np.sin(yaw), 0],
                              [np.sin(yaw), -np.cos(yaw), 0],
                              [0, 0, -1]])                       # pointing down, rotated by yaw
        T[:3, 3] = [x, y, z]
        q_sol, ok, info = solve_ik(T, self.q(), rng=self.rng)
        self.send("ik", ok=ok, q=q_sol, iterations=info["iterations"],
                  history=info["history"][:100], seeds=info.get("seeds_tried", 1),
                  error=float(np.linalg.norm(forward_kinematics(q_sol)[:3, 3] - T[:3, 3])))
        self.markers.clear()
        self.markers.frame(T)
        if ok:
            self.log(f"IK solved in {info['iterations']} iterations; moving there.")
            self.start_motion(q_sol)
            self.set_state("IK_MOVE")
        else:
            self.log("IK failed: that pose is outside the reachable workspace.", "warn")

    def send_hello(self):
        self.send("hello",
                  state=self.state,
                  teach=self.teach_mode,
                  grip_force=self.gripper.grip_force,
                  min_grip_force=min_grip_force(),
                  speed_scale=self.speed_scale,
                  q_min=C.Q_MIN, q_max=C.Q_MAX, dq_max=C.DQ_MAX,
                  shoulder=C.SHOULDER, wrist_offset=C.WRIST_OFFSET,
                  wrist_reach=C.WRIST_REACH, min_radius=C.MIN_RADIUS_XY,
                  ball_radius=C.BALL_RADIUS, launch=self.launch,
                  perception=self.perception)
        if self.candidates:
            self.send_plan(None)

    def send_plan(self, p_now):
        p = self.plan
        self.send("plan",
                  p_now=p_now, v=self.v_est,
                  candidates=self.candidates,
                  plan=None if p is None else {
                      "t_intercept": p["t_intercept"], "p_intercept": p["p_intercept"],
                      "T_approach": p["T_approach"], "T_descend": p["T_descend"],
                      "slack": p["slack"], "q_pregrasp": p["q_pregrasp"], "q_grasp": p["q_grasp"]})

    def send_camera(self):
        """Camera image + what the detector found, for the Camera tab."""
        has_estimate = self.state not in ("IDLE", "OBSERVE", "GO_HOME", "IK_MOVE")
        detection = self.camera.detect(self.predicted_ball() if has_estimate else None)
        self.send("camera", image=self.camera.thumbnail(), perception=self.perception,
                  detection=detection, ball_true=np.round(self.ball_position(), 4),
                  predicted_px=self.camera.world_to_pixel(self.predicted_ball()) if has_estimate else None,
                  predicted=np.round(self.predicted_ball(), 4) if has_estimate else None,
                  expected_diameter_px=self.camera.expected_ball_diameter_px(),
                  image_size=[self.camera.width, self.camera.height])

    def send_telemetry(self):
        q = self.q()
        tcp = self.tcp_measured()
        tcp_fk = forward_kinematics(q)[:3, 3]
        self.send("telemetry",
                  t=round(self.robot.getTime(), 3),
                  state=self.state,
                  q=np.round(q, 4), q_des=np.round(self.q_des, 4),
                  tcp=np.round(tcp, 4), fk_error_mm=round(float(np.linalg.norm(tcp - tcp_fk)) * 1000, 2),
                  ball=np.round(self.ball_position(), 4),
                  fingers=np.round(self.gripper.positions(), 4),
                  finger_force=np.round(self.gripper.forces(), 2))

    # ═════════════════════════════════════════════════════════════════════════
    #  Main loop
    # ═════════════════════════════════════════════════════════════════════════
    def run(self):
        self.q_des = np.zeros(7)
        self.q_hold = np.zeros(7)
        while self.robot.step(self.dt_ms) != -1:
            message = self.robot.wwiReceiveText()
            while message:
                self.handle_message(message)
                message = self.robot.wwiReceiveText()

            if self.script:
                self.script.update(self)
            if self.state in ("APPROACH", "DESCEND", "WAIT"):
                self.track_ball()
            self.step_state()
            self.control_arm()
            self.gripper.update()

            now = self.robot.getTime()
            self.update_movie()
            if self.quit_time is not None and now >= self.quit_time:
                self.quit_time = None
                self.stop_movie_and_quit()
            if self.exercise_run is not None:
                results = self.exercise_run.poll()
                if results is not None:
                    self.exercise_run = None
                    self.send("exercises", results=results)
            # Stream to the robot window only once it is open (it says hello),
            # and the camera image only while the Camera tab is visible.
            if self.window_connected and now - self.last_telemetry >= TELEMETRY_PERIOD:
                self.last_telemetry = now
                self.send_telemetry()
            if self.window_connected and self.camera_view and \
                    now - self.last_camera_stream >= CAMERA_STREAM_PERIOD:
                self.last_camera_stream = now
                self.send_camera()
            if now - self.last_trail >= TRAIL_PERIOD and self.state in (
                    "APPROACH", "DESCEND", "WAIT", "CLOSE", "LIFT"):
                self.last_trail = now
                self.markers.extend_trail(self.tcp_measured())


class ExerciseRun:
    """Runs exercises/check.py in its own process, so that student code (an
    endless loop, a crash) can never freeze or kill the simulation.  The
    simulation keeps running meanwhile; ``poll`` is called every step."""

    def __init__(self, extra_args=()):
        self.started = time.monotonic()
        try:
            self.proc = subprocess.Popen([sys.executable, str(CHECK_SCRIPT), "--json", *extra_args],
                                         stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            self.error = None
        except OSError as e:
            self.proc, self.error = None, f"Could not start the checker: {e}"

    def poll(self):
        """None while the checker runs, then its list of results."""
        if self.proc is None:
            return [{"n": 0, "title": "checker", "status": "error", "message": self.error}]
        timed_out = time.monotonic() - self.started > EXERCISE_TIMEOUT
        if self.proc.poll() is None and not timed_out:
            return None
        if self.proc.poll() is None:
            self.proc.kill()
        out, err = self.proc.communicate()
        return self.parse(out, err, timed_out)

    @staticmethod
    def parse(out, err, timed_out):
        exercises, results = [], {}
        for line in out.splitlines():           # skip anything the student's code printed
            try:
                item = json.loads(line)
            except ValueError:
                continue
            if isinstance(item, dict) and "exercises" in item:
                exercises = item["exercises"]
            elif isinstance(item, dict) and {"n", "title", "status", "message"} <= item.keys():
                results[item["n"]] = item
        if not exercises:                       # the checker itself failed to start
            last = err.strip().splitlines()[-1] if err.strip() else "no output"
            return [{"n": 0, "title": "checker", "status": "error", "message": last}]
        checked, first_missing = [], True
        for n, title in exercises:
            if n in results:
                checked.append(results[n])
                continue
            if not timed_out:
                message = "The checker stopped before this exercise."
            elif first_missing:                 # the exercise that never finished
                message = f"Timed out after {EXERCISE_TIMEOUT:.0f} s: is there an endless loop?"
            else:
                message = "Not checked: the exercise before it never finished."
            checked.append({"n": n, "title": title, "status": "error", "message": message})
            first_missing = False
        return checked


def number(msg, *keys):
    """The message's values for these keys, as finite floats (else ValueError)."""
    values = [float(msg[k]) for k in keys]
    if not all(math.isfinite(v) for v in values):
        raise ValueError(f"{', '.join(keys)} must be finite numbers")
    return values


def _to_json(obj):
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    if isinstance(obj, np.generic):
        return obj.item()
    raise TypeError(type(obj))


if __name__ == "__main__":
    DynamicGraspingDemo().run()
