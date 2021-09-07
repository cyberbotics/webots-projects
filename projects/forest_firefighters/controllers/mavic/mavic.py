from controller import Robot, Keyboard


def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)


class Mavic (Robot):
    # Constants, empirically found.
    K_VERTICAL_THRUST = 68.5  # with this thrust, the drone lifts.
    K_VERTICAL_OFFSET = 0.6   # Vertical offset where the robot actually targets to stabilize itself.
    K_VERTICAL_P = 3.0        # P constant of the vertical PID.
    K_ROLL_P = 50.0           # P constant of the roll PID.
    K_PITCH_P = 30.0          # P constant of the pitch PID.

    target_altitude = 20

    def __init__(self):
        Robot.__init__(self)

        self.timeStep = int(self.getBasicTimeStep())

        # keyboard
        self.keyboard = self.getKeyboard()
        self.keyboard.enable(10 * self.timeStep)

        self.water_to_drop = 0

        # Get and enable devices.
        self.camera = self.getDevice("camera")
        self.camera.enable(self.timeStep)
        self.imu = self.getDevice("inertial unit")
        self.imu.enable(self.timeStep)
        self.gps = self.getDevice("gps")
        self.gps.enable(self.timeStep)
        self.gyro = self.getDevice("gyro")
        self.gyro.enable(self.timeStep)

        self.front_left_motor = self.getDevice("front left propeller")
        self.front_right_motor = self.getDevice("front right propeller")
        self.rear_left_motor = self.getDevice("rear left propeller")
        self.rear_right_motor = self.getDevice("rear right propeller")
        motors = [self.front_left_motor, self.front_right_motor, self.rear_left_motor, self.rear_right_motor]
        for motor in motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(1)

        # Display manual control message.
        print("You can control the drone with your computer keyboard:")
        print("- 'D': drop water")
        print("- 'up': move forward.")
        print("- 'down': move backward.")
        print("- 'right': turn right.")
        print("- 'left': turn left.")
        print("- 'shift + up': increase the target altitude.")
        print("- 'shift + down': decrease the target altitude.")

    def run(self):
        while self.step(self.timeStep) != -1:
            # Read sensors
            roll, pitch, _ = self.imu.getRollPitchYaw()
            _, _, altitude = self.gps.getValues()
            roll_acceleration, pitch_acceleration, _ = self.gyro.getValues()

            roll_disturbance = 0
            pitch_disturbance = 0
            yaw_disturbance = 0

            key = self.keyboard.getKey()

            # Drop the water from the drone
            if key == ord('D'):
                self.water_to_drop += 1
            elif self.water_to_drop > 0:
                self.setCustomData(str(self.water_to_drop))
                self.water_to_drop = 0
            else:
                self.setCustomData(str(0))

            # Movement
            if key == Keyboard.LEFT:
                yaw_disturbance = 1.3
            elif key == Keyboard.RIGHT:
                yaw_disturbance = -1.3
            elif key == Keyboard.UP:
                pitch_disturbance = -2
            elif key == Keyboard.DOWN:
                pitch_disturbance = 2
            elif key == Keyboard.UP + Keyboard.SHIFT:
                self.target_altitude += 0.05
                print(f"target altitude: {self.target_altitude} [m]\n")
            elif key == Keyboard.DOWN + Keyboard.SHIFT:
                self.target_altitude -= 0.05
                print(f"target altitude: {self.target_altitude} [m]\n")

            roll_input = self.K_ROLL_P * clamp(roll, -1, 1) + roll_acceleration + roll_disturbance
            pitch_input = self.K_PITCH_P * clamp(pitch, -1, 1) + pitch_acceleration + pitch_disturbance
            yaw_input = yaw_disturbance
            clamped_difference_altitude = clamp(self.target_altitude - altitude + self.K_VERTICAL_OFFSET, -1, 1)
            vertical_input = self.K_VERTICAL_P * pow(clamped_difference_altitude, 3.0)

            front_left_motor_input = self.K_VERTICAL_THRUST + vertical_input - yaw_input + pitch_input - roll_input
            front_right_motor_input = self.K_VERTICAL_THRUST + vertical_input + yaw_input + pitch_input + roll_input
            rear_left_motor_input = self.K_VERTICAL_THRUST + vertical_input + yaw_input - pitch_input - roll_input
            rear_right_motor_input = self.K_VERTICAL_THRUST + vertical_input - yaw_input - pitch_input + roll_input

            self.front_left_motor.setVelocity(front_left_motor_input)
            self.front_right_motor.setVelocity(-front_right_motor_input)
            self.rear_left_motor.setVelocity(-rear_left_motor_input)
            self.rear_right_motor.setVelocity(rear_right_motor_input)


robot = Mavic()
robot.run()
