from controller import Robot


class Spot (Robot):
    NUMBER_OF_LEDS = 8
    NUMBER_OF_JOINTS = 12
    NUMBER_OF_CAMERAS = 5

    def __init__(self):
        Robot.__init__(self)

        self.time_step = int(self.getBasicTimeStep())

        # keyboard
        self.keyboard = self.getKeyboard()
        self.keyboard.enable(10 * self.time_step)

        self.water_to_drop = 0

        motor_list = [
            "front left shoulder abduction motor",  "front left shoulder rotation motor",  "front left elbow motor",
            "front right shoulder abduction motor", "front right shoulder rotation motor", "front right elbow motor",
            "rear left shoulder abduction motor",   "rear left shoulder rotation motor",   "rear left elbow motor",
            "rear right shoulder abduction motor",  "rear right shoulder rotation motor",  "rear right elbow motor"
        ]
        self.motors = []
        for motor_name in motor_list:
            motor = self.getDevice(motor_name)
            self.motors.append(motor)

        # Display manual control message.
        print("You throw water with your computer keyboard by pressing 'D'.")

    def robotStep(self):
        if self.step(self.time_step) != -1:

            key = self.keyboard.getKey()

            # throw the water from the robot
            if key == ord('D'):
                self.water_to_drop += 1
            elif self.water_to_drop > 0:
                self.setCustomData(str(self.water_to_drop))
                self.water_to_drop = 0
            else:
                self.setCustomData(str(0))

    def movementDecomposition(self, target, duration):
        n_steps_to_achieve_target = int(duration * 1000 / self.time_step)
        step_difference = []
        current_position = []

        for i in range(self.NUMBER_OF_JOINTS):
            current_position.append(self.motors[i].getTargetPosition())
            step_difference.append(
                (target[i] - current_position[i]) / n_steps_to_achieve_target)

        for _ in range(n_steps_to_achieve_target):
            for j in range(self.NUMBER_OF_JOINTS):
                current_position[j] += step_difference[j]
                self.motors[j].setPosition(current_position[j])
                self.robotStep()

    def lieDown(self, duration):
        motors_target_pos = [-0.40, -0.99, 1.59,  # Front left leg
                             0.40,  -0.99, 1.59,  # Front right leg
                             -0.40, -0.99, 1.59,  # Rear left leg
                             0.40,  -0.99, 1.59]  # Rear right
        self.movementDecomposition(motors_target_pos, duration)

    def standUp(self, duration):
        motors_target_pos = [-0.1, 0, 0,  # Front left leg
                             0.1,  0, 0,  # Front right leg
                             -0.1, 0, 0,  # Rear left leg
                             0.1,  0, 0]  # Rear right
        self.movementDecomposition(motors_target_pos, duration)

    def sitDown(self, duration):
        motors_target_pos = [-0.20, -0.40, -0.19,  # Front left leg
                             0.20,  -0.40, -0.19,  # Front right leg
                             -0.40, -0.90, 1.18,   # Rear left leg
                             0.40,  -0.90, 1.18]   # Rear right
        self.movementDecomposition(motors_target_pos, duration)

    def run(self):      
        while self.getTime() < 40:
            self.lieDown(1)
            self.standUp(1)
            self.sitDown(1)


robot = Spot()
robot.run()
