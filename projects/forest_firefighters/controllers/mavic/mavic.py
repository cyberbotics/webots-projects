import math
from controller import Robot, Camera, IMU

TIME_STEP = 32

robot = Robot()

# Get and enable devices.
camera = robot.getDevice("camera")
camera.enable(TIME_STEP)
imu = robot.getDevice("inertial unit")
imu.enable(TIME_STEP)
gps = robot.getDevice("gps")
gps.enable(TIME_STEP)
compass = robot.getDevice("compass")
compass.enable(TIME_STEP)
gyro = robot.getDevice("gyro")
gyro.enable(TIME_STEP)


front_left_motor = robot.getDevice("front left propeller");
front_right_motor = robot.getDevice("front right propeller");
rear_left_motor = robot.getDevice("rear left propeller");
rear_right_motor = robot.getDevice("rear right propeller");
motors = {front_left_motor, front_right_motor, rear_left_motor, rear_right_motor};
for motor in motors: 
    motor.setPosition(float('inf'))
    motor.setVelocity(1.0)

# Wait one second.
while robot.step(TIME_STEP) != -1:
    if robot.getTime() > 1:
        break

# Constants, empirically found.
k_vertical_thrust = 68.5  # with this thrust, the drone lifts.
k_vertical_offset = 0.6   # Vertical offset where the robot actually targets to stabilize itself.
k_vertical_p = 3.0        # P constant of the vertical PID.
k_roll_p = 50.0           # P constant of the roll PID.
k_pitch_p = 30.0;         # P constant of the pitch PID.

while robot.step(TIME_STEP) != -1:
    roll = imu.getRollPitchYaw() + math.PI / 2.0
    const double pitch = wb_inertial_unit_get_roll_pitch_yaw(imu)[1];
    const double altitude = wb_gps_get_values(gps)[1];
    const double roll_acceleration = wb_gyro_get_values(gyro)[0];
    const double pitch_acceleration = wb_gyro_get_values(gyro)[1];


    front_right_motor.setVelocity(-150)
    rear_left_motor.setVelocity(-150)
    front_left_motor.setVelocity(150)
    rear_right_motor.setVelocity(150)