"""
gripper.py
==========
The Franka Hand: two fingers, each driven by a Webots LinearMotor that
travels from 0 (closed) to 0.04 m (open).

How the grasp works
-------------------
To hold a ball of mass m by friction, each finger must press with a normal
force N such that the two friction forces carry the weight:

    2 · μ · N ≥ m · g      →      N_min = m · g / (2 μ)

Closing is done in one command that behaves like two phases:

  * the finger target is set *inside* the ball (it can never get there),
  * while the finger moves freely it travels at full speed (position phase),
  * once it touches the ball the position error cannot shrink, so the motor
    saturates at its force limit — which we set to the desired grip force
    with ``setAvailableForce`` (force phase).

Keeping the fingers symmetric
-----------------------------
On the real Franka Hand the two fingers are coupled mechanically, so they
always move together.  In the Webots model each finger has its own motor:
if the ball is a little off-centre, one finger pushes first, the ball slides
on the frictionless floor, and the pair can drift sideways until a finger
slips past the ball.  ``update()`` therefore couples them in software: the
finger that is more open gets more force, the other one less,

    F_left  = F + K · (x_left − x_right)
    F_right = F − K · (x_left − x_right)

so the average squeeze is still F, but any drift is pushed back.
"""

import config as C


def min_grip_force(mass=C.BALL_MASS, mu=C.FRICTION_COEFF, g=9.81):
    """Smallest per-finger normal force that keeps the ball from slipping."""
    return mass * g / (2.0 * mu)


class Gripper:
    def __init__(self, robot, time_step):
        self.fingers = [robot.getDevice(name) for name in C.FINGER_NAMES]
        self.sensors = []
        for motor in self.fingers:
            motor.enableForceFeedback(time_step)
            sensor = motor.getPositionSensor()
            sensor.enable(time_step)
            self.sensors.append(sensor)
        self.grip_force = C.GRIP_FORCE_DEFAULT
        self.squeezing = False

    # ── commands ─────────────────────────────────────────────────────────────
    def open(self):
        self.squeezing = False
        for motor in self.fingers:
            motor.setAvailableForce(motor.getMaxForce())
            motor.setVelocity(C.FINGER_SPEED)
            motor.setPosition(C.FINGER_OPEN)

    def close(self):
        """Start closing: aim 1.5 cm inside the ball, force-limited."""
        target = max(C.BALL_RADIUS - 0.015, 0.0)
        self.squeezing = True
        for motor in self.fingers:
            motor.setAvailableForce(self.grip_force)
            motor.setVelocity(C.FINGER_SPEED)
            motor.setPosition(target)

    def update(self):
        """Call every step: software coupling of the two fingers."""
        if not self.squeezing:
            return
        left, right = self.positions()
        correction = C.FINGER_COUPLING * (left - right)
        for motor, force in zip(self.fingers, (self.grip_force + correction,
                                               self.grip_force - correction)):
            motor.setAvailableForce(float(min(max(force, 0.0), motor.getMaxForce())))

    # ── measurements ─────────────────────────────────────────────────────────
    def positions(self):
        return [s.getValue() for s in self.sensors]

    def forces(self):
        """Force each finger motor is currently applying (N)."""
        return [abs(m.getForceFeedback()) for m in self.fingers]

    def closing_time(self):
        """Time for an open finger to reach the ball surface at full speed."""
        return (C.FINGER_OPEN - C.BALL_RADIUS) / C.FINGER_SPEED
