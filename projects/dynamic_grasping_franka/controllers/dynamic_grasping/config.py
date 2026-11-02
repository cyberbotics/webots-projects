"""
config.py
=========
Every number the demo depends on, in one place.  Change a value here (or
through the robot window) and re-run to see what happens; nothing else in
the code hides a magic constant.
"""

import numpy as np

# ── Robot: Franka Emika Panda (7 revolute joints) ────────────────────────────
N_JOINTS = 7
JOINT_NAMES = [f"panda_joint{i}" for i in range(1, 8)]
FINGER_NAMES = ["panda_finger::left", "panda_finger::right"]

Q_MIN = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
Q_MAX = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])
DQ_MAX = np.array([2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61])   # rad/s

# "Ready" pose: elbow bent, gripper pointing straight down.
Q_HOME = np.array([0.0, -0.3, 0.0, -2.0, 0.0, 1.7, 0.785])

# Workspace test used by the planner.  With the gripper pointing down, the
# wrist centre sits WRIST_OFFSET above the TCP and must stay within
# WRIST_REACH of the shoulder (joint 2) — upper arm + forearm ≈ 0.79 m.
SHOULDER = np.array([0.0, 0.0, 0.333])
WRIST_OFFSET = 0.207   # flange (0.107) + hand to TCP (0.1)
WRIST_REACH = 0.79
MIN_RADIUS_XY = 0.25   # keep away from the robot's own base column

# ── Joint controller: velocity feed-forward + proportional feedback ──────────
#    dq_cmd = dq_desired + KP * (q_desired - q_measured)
KP_JOINT = 20.0        # 1/s

# ── Gripper (Franka Hand, each finger travels 0 … 4 cm) ──────────────────────
FINGER_OPEN = 0.04         # m, fully open
FINGER_SPEED = 0.2         # m/s, max finger speed in the Webots model
GRIP_FORCE_DEFAULT = 20.0  # N, squeezing force per finger
FRICTION_COEFF = 1.5       # finger-ball Coulomb friction (see world file)
FINGER_COUPLING = 2000.0   # N/m, keeps the two fingers symmetric (gripper.update)

# ── Ball ──────────────────────────────────────────────────────────────────────
BALL_RADIUS = 0.03         # m
BALL_MASS = 0.1            # kg

# ── Grasp geometry ────────────────────────────────────────────────────────────
PREGRASP_HEIGHT = 0.12     # m above the grasp pose
GRASP_HEIGHT = BALL_RADIUS # TCP at ball centre height
LIFT_HEIGHT = 0.25         # m, TCP height after the catch

# ── Intercept planner ─────────────────────────────────────────────────────────
PLAN_DT = 0.05             # s, spacing of candidate intercept times
PLAN_HORIZON = 8.0         # s, look this far into the future
GRIPPER_CLOSE_TIME = 0.2   # s, time budget reserved for closing the fingers
SAFETY_MARGIN = 0.15       # s, extra slack so the arm is early, never late
MIN_SEGMENT_TIME = 0.4     # s, shortest allowed arm motion

# ── Perception (ball tracking) ────────────────────────────────────────────────
PERCEPTION = "camera"      # "camera" (overhead camera) or "simulator" (exact positions)
OBSERVE_TIME = 0.2         # s of ball positions used to estimate its velocity
OBSERVE_TIMEOUT = 1.0      # s, give up if the camera cannot see the ball
CAMERA_PERIOD_MS = 16      # camera frame period (about 60 frames per second)
TRACK_WINDOW = 1.5         # s, keep re-fitting on the latest frames while the arm moves
TRACK_MIN_DISTANCE = 0.25  # m, stop using frames once the ball is this close to the gripper
                           #    (the arm, hand and their shadow start to hide it)
TRACK_GATE = 0.05          # m, reject measurements this far from the prediction
MAX_CORRECTION = 0.05      # m, largest correction of the meeting point before descending
