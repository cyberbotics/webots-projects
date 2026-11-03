"""
A tiny stand-in for Webots' ``controller`` module so the state machine can
be exercised without Webots (``tests/test_state_machine.py``).

It is *kinematic*, not a physics engine: joints follow their velocity
commands exactly, the ball slides in a straight line, the fingers stop
when they touch the ball, and a gripped ball follows the gripper.  Good
enough to check the logic and timing of the controller — not the contact
physics, which only real Webots can tell you about.
"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "controllers" / "dynamic_grasping"))
from kinematics import forward_kinematics  # noqa: E402

BALL_RADIUS = 0.03


class _Sensor:
    def __init__(self, motor):
        self.motor = motor

    def enable(self, _):
        pass

    def getValue(self):
        return self.motor.position


class _Motor:
    def __init__(self, max_velocity, max_force=87.0, position=0.0):
        self.position = position
        self.target = position
        self.velocity = 0.0
        self.max_velocity = max_velocity
        self.max_force = max_force
        self.available_force = max_force
        self.sensor = _Sensor(self)

    def setPosition(self, p):
        self.target = p

    def setVelocity(self, v):
        assert abs(v) <= self.max_velocity + 1e-9, "velocity above motor limit"
        self.velocity = v

    def setAvailableForce(self, f):
        self.available_force = f

    def getMaxForce(self):
        return self.max_force

    def getMaxVelocity(self):
        return self.max_velocity

    def enableForceFeedback(self, _):
        pass

    def getForceFeedback(self):
        return self.available_force if getattr(self, "blocked", False) else 0.0

    def getPositionSensor(self):
        return self.sensor


class _GPS:
    def __init__(self, sup):
        self.sup = sup

    def enable(self, _):
        pass

    def getValues(self):
        return list(forward_kinematics(self.sup.q())[:3, 3])


class _Camera:
    """Overhead camera: draws the ball as an orange disc on a grey floor."""
    WIDTH, HEIGHT, FOV = 320, 240, 1.2
    POSITION = np.array([0.6, -0.1, 2.5])
    ROTATION = np.array([[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]])   # Ry(+90°)

    PERIOD = 0.016

    def __init__(self, sup):
        self.sup = sup
        self._tag = self
        self.snapshot = sup.ball.p.copy()   # what the last rendered frame shows
        self.last_frame = -1.0

    def enable(self, _):
        pass

    def getWidth(self):
        return self.WIDTH

    def getHeight(self):
        return self.HEIGHT

    def getFov(self):
        return self.FOV

    def getImage(self):
        img = np.zeros((self.HEIGHT, self.WIDTH, 4), dtype=np.uint8)
        img[:, :, :3] = 128                                   # grey floor (BGR)
        f = (self.WIDTH / 2) / np.tan(self.FOV / 2)
        rel = self.ROTATION.T @ (self.snapshot - self.POSITION)     # camera frame
        if rel[0] > 0:
            u = self.WIDTH / 2 - f * rel[1] / rel[0] - 0.5
            v = self.HEIGHT / 2 - f * rel[2] / rel[0] - 0.5
            r = f * BALL_RADIUS / rel[0]
            vv, uu = np.mgrid[0:self.HEIGHT, 0:self.WIDTH]
            disc = (uu - u) ** 2 + (vv - v) ** 2 <= r ** 2
            img[disc, 0], img[disc, 1], img[disc, 2] = 40, 150, 240   # orange in BGR
        return img.tobytes()


class _DeviceNode:
    def __init__(self, camera):
        self.camera = camera

    def getPosition(self):
        return list(self.camera.POSITION)

    def getOrientation(self):
        return list(self.camera.ROTATION.ravel())


class _Field:
    def __init__(self, getter=None, setter=None):
        self.getter, self.setter = getter, setter
        self.items = []

    def setSFVec3f(self, v):
        self.setter(v)

    def setSFRotation(self, _):
        pass

    # MF children of the MARKERS group
    def getCount(self):
        return len(self.items)

    def removeMF(self, i):
        self.items.pop(i)

    def importMFNodeFromString(self, i, s):
        self.items.append(s)

    def getMFNode(self, i):
        return _Node(self)


class _Node:
    """Generic node; for markers it fakes the trail's nested fields."""

    def __init__(self, owner=None):
        self.owner = owner

    def getField(self, name):
        if name in ("geometry", "coord"):
            return self
        if name in ("point", "coordIndex"):
            return _TrailField()
        return _Field()

    def getSFNode(self):
        return self


class _TrailField:
    def getCount(self):
        return 1

    def insertMFVec3f(self, i, v):
        pass

    def insertMFInt32(self, i, v):
        pass


class _Ball:
    def __init__(self, sup):
        self.sup = sup
        self.p = np.array([0.5, -0.8, BALL_RADIUS])
        self.v = np.zeros(6)
        self.held = False

    def getPosition(self):
        return list(self.p)

    def getVelocity(self):
        return list(self.v)

    def setVelocity(self, v):
        self.v = np.array(v, dtype=float)

    def resetPhysics(self):
        self.v = np.zeros(6)
        self.held = False

    def getField(self, name):
        if name == "translation":
            return _Field(setter=lambda p: setattr(self, "p", np.array(p, dtype=float)))
        return _Field()


class Supervisor:
    def __init__(self):
        names = [f"panda_joint{i}" for i in range(1, 8)]
        vmax = [2.175] * 4 + [2.61] * 3
        self.devices = {n: _Motor(v) for n, v in zip(names, vmax)}
        self.fingers = [_Motor(0.2, 100.0), _Motor(0.2, 100.0)]
        self.devices["panda_finger::left"], self.devices["panda_finger::right"] = self.fingers
        self.devices["tcp_gps"] = _GPS(self)
        self.ball = _Ball(self)
        self.devices["overhead_camera"] = _Camera(self)
        self.markers = _Node()
        self.markers_children = _Field()
        self.time = 0.0
        self.inbox, self.outbox = [], []
        self.labels = {}

    def q(self):
        return np.array([self.devices[f"panda_joint{i}"].position for i in range(1, 8)])

    # ── Robot API ────────────────────────────────────────────────────────────
    def getBasicTimeStep(self):
        return 4.0

    def getTime(self):
        return self.time

    def getDevice(self, name):
        return self.devices[name]

    def getFromDevice(self, device):
        return _DeviceNode(device)

    def simulationQuit(self, code):
        self.quit_code = code

    def getFromDef(self, name):
        if name == "BALL":
            return self.ball
        if name == "MARKERS":
            group = _Node()
            group.getField = lambda _: self.markers_children
            return group
        return None

    def wwiSendText(self, text):
        self.outbox.append(text)

    def wwiReceiveText(self):
        return self.inbox.pop(0) if self.inbox else None

    def setLabel(self, i, text, *args):
        self.labels[i] = text

    def step(self, ms):
        dt = ms / 1000.0
        self.time += dt
        camera = self.devices["overhead_camera"]
        if self.time - camera.last_frame >= camera.PERIOD - 1e-9:
            # Like Webots: a frame shows the scene *before* this step's changes
            camera.snapshot, camera.last_frame = self.ball.p.copy(), self.time
        for i in range(1, 8):
            m = self.devices[f"panda_joint{i}"]
            m.position += m.velocity * dt
        T = forward_kinematics(self.q())
        tcp = T[:3, 3]
        rel = self.ball.p - tcp
        across = abs(float(rel @ T[:3, 1]))           # distance along the finger axis
        between = np.linalg.norm(rel - (rel @ T[:3, 1]) * T[:3, 1]) < 0.012
        for f in self.fingers:
            step = np.clip(f.target - f.position, -f.max_velocity * dt, f.max_velocity * dt)
            new = f.position + step
            f.blocked = between and step < 0 and new <= BALL_RADIUS + across * 0 and f.available_force > 0
            f.position = max(new, BALL_RADIUS) if f.blocked or (between and new < BALL_RADIUS) else new
        squeezed = all(getattr(f, "blocked", False) for f in self.fingers)
        if squeezed and self.fingers[0].available_force > 0.33:
            self.ball.held = True
        if self.ball.held:
            self.ball.p = tcp.copy()
            self.ball.v = np.zeros(6)
        else:
            self.ball.p = self.ball.p + self.ball.v[:3] * dt
        return 0
