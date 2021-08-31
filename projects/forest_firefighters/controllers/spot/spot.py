import math
from controller import Robot, Keyboard

class Spot (Robot):
    # Constants, empirically found.
    K_VERTICAL_THRUST = 68.5  # with this thrust, the drone lifts.
    K_VERTICAL_OFFSET = 0.6   # Vertical offset where the robot actually targets to stabilize itself.
    K_VERTICAL_P = 3.0        # P constant of the vertical PID.
    K_ROLL_P = 50.0           # P constant of the roll PID.
    K_PITCH_P = 30.0;         # P constant of the pitch PID.

    target_altitude = 20

    def __init__(self):
        Robot.__init__(self)

        self.timeStep = int(self.getBasicTimeStep())

        motor_list = {  
            "front left shoulder abduction motor",  "front left shoulder rotation motor",  "front left elbow motor",
            "front right shoulder abduction motor", "front right shoulder rotation motor", "front right elbow motor",
            "rear left shoulder abduction motor",   "rear left shoulder rotation motor",   "rear left elbow motor",
            "rear right shoulder abduction motor",  "rear right shoulder rotation motor",  "rear right elbow motor"
        }
        self.motors = dict()
        for motor_name in motor_list:
            motor = self.getDevice(motor_name)
            self.motors[motor_name] = motor

    def run(self):
        while self.step(self.timeStep) != -1:

            # Move the left front leg (basic example)
            time = int(self.getTime())
            leg = self.motors["front left shoulder rotation motor"]
            if time % 2 == 0:
                leg.setPosition(1)
            else:
                leg.setPosition(0)

robot = Spot()
robot.run()





