from controller import Robot
import sys, random, optparse
#import transform3d
try:
    import numpy as np
    from numpy import NaN, nan
except ImportError:
    sys.exit("Warning: 'numpy' module not found. Please check the Python modules installation instructions " +
             "at 'https://www.cyberbotics.com/doc/guide/using-python'.")
try:
    import cv2
except ImportError:
    sys.exit("Warning: 'cv2' module not found. Please check the Python modules installation instructions " +
             "at 'https://www.cyberbotics.com/doc/guide/using-python'.")

def clamp(value, value_min, value_max):
    return min(max(value, value_min), value_max)

class Mavic (Robot):
    # Constants, empirically found.
    K_VERTICAL_THRUST = 68.5  # with this thrust, the drone lifts.
    K_VERTICAL_OFFSET = 0.6   # Vertical offset where the robot actually targets to stabilize itself.
    K_VERTICAL_P = 3.0        # P constant of the vertical PID.
    K_ROLL_P = 50.0           # P constant of the roll PID.
    K_PITCH_P = 30.0          # P constant of the pitch PID.

    MAX_YAW_DISTURBANCE = 0.4
    MAX_PITCH_DISTURBANCE = -1
    target_precison = 0.5 #precision between the target position and the robot position in meters

    def __init__(self):
        Robot.__init__(self)

        self.timeStep = int(self.getBasicTimeStep())

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
        self.camera_pitch_motor = self.getDevice("camera pitch")
        self.camera_pitch_motor.setPosition(1.55) #vertical PoV
        motors = [self.front_left_motor, self.front_right_motor, self.rear_left_motor, self.rear_right_motor]
        for motor in motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(1)

        self.curr_pos = 6*[0] # X,Y,Z, yaw, pitch, roll
        self.target_pos = [0,0,0]
        self.target_index = 0

        self.fire_pos = [0,0]
        self.FireDetectionStatus = False
        self.WaterDropStatus = False
        

    def get_image_from_camera(self):
        """
        Take an image from the camera device and prepare it for OpenCV processing:
        - convert data type,
        - convert to RGB format (from BGRA), and
        - rotate & flip to match the actual image.
        Returns:
            image of the camera
        """
        img = self.camera.getImageArray()
        img = np.asarray(img, dtype=np.uint8)
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2RGB)
        img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
        return cv2.flip(img, 1)

    def set_position(self, pos):
        """
        Set a new absolut position of the robot
        Parameters:
            pos (list): [X,Y,Z, yaw, pitch, roll] current absolut position and angles
        """
        self.curr_pos = pos

    def move_to_target(self, local_coord, verbose_movement=False, verbose_target=True):
        """
        Move the robot to the given coordinates
        Parameters: 
            local_coord (list): X,Y coordinates
            verbose_movement (bool): whether to print remaning angle and distance or not
            verbose_target (bool): whether to print targets or not
        Returns:
            yaw_disturbance (float): yaw disturbance (negative value to go on the right)
            pitch_disturbance (float): pitch disturbance (negative value to go forward)
        """

        if self.target_pos[0:2] == [0,0]: #Initialisation
            self.target_pos[0:2] = local_coord[0]
            if verbose_target: print("First target: ", self.target_pos[0:2])

        if all([abs(x1 - x2) < self.target_precison for (x1, x2) in zip(self.target_pos, self.curr_pos[0:2])]): #if the robot is at the position with a precision of target_precison

            if self.FireDetectionStatus:
                if self.fire_pos == [0,0]:             
                    self.water_to_drop = 25
                    if verbose_target: print("Water dropped on fire target: {} at position {}".format(self.target_pos[0:2], self.curr_pos[0:2]))
                    self.FireDetectionStatus = False 
                
            else:
                self.target_index += 1
                if self.target_index > len(local_coord)-1: self.target_index=0
                self.target_pos[0:2] = local_coord[self.target_index]
                if verbose_target: print("Target reached! New target: ", self.target_pos[0:2])
        
        #this will be in ]-pi;pi]
        self.target_pos[2] = np.arctan2(self.target_pos[1] - self.curr_pos[1], self.target_pos[0] - self.curr_pos[0])
        #this is now in ]-2pi;2pi[
        angle_left = self.target_pos[2] - self.curr_pos[5]
        #Normalize turn angle to ]-pi;pi]
        angle_left = (angle_left + 2*np.pi) % (2*np.pi)
        if (angle_left > np.pi):
            angle_left -= 2*np.pi

        #turn the robot to the left or to the right according the value and the sign of angle_left
        yaw_disturbance = self.MAX_YAW_DISTURBANCE*angle_left/(2*np.pi)
        pitch_disturbance = clamp(np.log10(abs(angle_left)), self.MAX_PITCH_DISTURBANCE, 0.1) #non proportional and decruising function

        if verbose_movement: 
            distance_left = np.sqrt(((self.target_pos[0] - self.curr_pos[0]) **2) + ((self.target_pos[1] - self.curr_pos[1]) **2))
            print("remaning angle: {:.4f}, remaning distance: {:.4f}".format(angle_left, distance_left))
        return yaw_disturbance, pitch_disturbance

    def NaiveApproach(self,coord_fire):
        """
        Naive approch to move the robot above the fire. Closed loop to move the robot towards to the fire by step until to be above it.
        Parameters: 
            coord_fire (list): x,y image coordinates of the fire
        Returns:
            X,Y (float): X,Y position target 
        """

        resolutionX, resolutionY = self.camera.getWidth(), self.camera.getHeight()
        X, Y = self.curr_pos[0:2]
        x_img = coord_fire[0] 
        y_img = coord_fire[1]
        yaw = self.curr_pos[5]
        
        self.fire_pos = [0,0]
        if abs(x_img-resolutionX/2) > 40:
            self.fire_pos[0] = np.sign(x_img-resolutionX/2)
        if abs(y_img-resolutionY/2) > 40:
            self.fire_pos[1] = np.sign(y_img-resolutionY/2)

        self.fire_pos[1] *= -np.sign(yaw)
        if not (-np.pi/2 < yaw < np.pi/2): self.fire_pos[0] *= -1

        X += self.fire_pos[0]*abs(x_img-resolutionX/2)/50
        Y += self.fire_pos[1]*abs(y_img-resolutionY/2)/50

        return X,Y

    def ComputeMonoplotting(self, coord_fire, verbose=True):
        """
        /!\ Still need to be fixed
        Compute the world coordinates from the image coordinates of the fire.
        Monoplotting based on https://stackoverflow.com/questions/68195056/finding-the-real-world-coordinates-of-an-object-from-a-camera
        and http://sar.kangwon.ac.kr/etc/rs_note/rsnote/cp9/cp9-6.htm
        Parameters: 
            coord_fire (list): x,y image coordinates of the fire
            verbose (bool): whether to print status messages or not
        Returns:
            X,Y (float): X,Y position target 
        """
        resolution = self.camera.getWidth(), self.camera.getHeight()
        roll, pitch, yaw = self.curr_pos[3:6]

        Z_obj = 32

        #intrinsic properties of the camera
        cx, cy, cz = self.curr_pos[0:3] #principal point in the image ~= real world coordinates of the camera

        fieldOfView = self.camera.getFov() #0.78398rad FOV = 2*arctan(x/2f)
        f = resolution[0]/(2*np.tan(fieldOfView*0.5))

        #diag_img = np.sqrt(resolution[0]**2 + resolution[1]**2) #diagonal of the film or sensor in pixel
        #fd = diag_img/(2*np.tan(fieldOfView/2)) #f = focal length

        x_img = coord_fire[0]
        y_img = coord_fire[1]

        #3. Rotation matrix along X,Y,Z axis
        R = transforms3d.euler.euler2mat(roll, pitch, yaw)
        if verbose: print("rotational matrix: ", R)

        #4. Inversed colinearity equation
        xc = (Z_obj-cz)*(R[0,0]*x_img + R[0,1]*y_img - R[0,2]*f)
        yc = (Z_obj-cz)*(R[1,0]*x_img + R[1,1]*y_img - R[1,2]*f)
        zc = (R[2,0]*x_img + R[2,1]*y_img - R[2,2]*f)
        if verbose: print("colinearity equation",xc,yc,zc)

        X = xc/zc
        Y = yc/zc
        if verbose: print("coord distance from the robot to the fire: {:.2f},{:.2f}".format(X,Y))

        X += cx
        Y += cy

        return X, Y

    def FireDetection(self, verbose=True):
        """
        Detect the smoke and return the fire coordinate in the image
        Parameters: 
            verbose (bool): whether to print status messages or not
        Returns:
            coord_fire (list):x,y image coordinates of the fire
        """
        img = self.get_image_from_camera()
        # Segment the image by color in HSV color space
        hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        # Range of the smoke
        smoke_lower = np.array([0,0,168])
        smoke_upper = np.array([172,111,255])

        mask_fire = cv2.inRange(hsv, smoke_lower, smoke_upper)

        fire_ratio =np.round((cv2.countNonZero(mask_fire))/(img.size/3)*100, 2)
        if fire_ratio > 0.15: #higher the fire ratio, higher the number of fire in the image
            
            # detect the contours on the binary image using cv2.CHAIN_APPROX_NONE
            contours, _ = cv2.findContours(image=mask_fire, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)

            # Approximate contours to polygons + get circles
            contours_poly = [None]*len(contours)
            centers = [None]*len(contours)
            radius = [None]*len(contours)
            radius_max = 0
            for i, c in enumerate(contours):
                contours_poly[i] = cv2.approxPolyDP(c, 3, True)
                centers[i], radius[i] = cv2.minEnclosingCircle(contours_poly[i])
                if radius[i] > 3 and radius[i] > radius_max: #we keep only the biggest circle and > 3
                        coord_fire = centers[i]
                        radius_max = radius[i]
                        if verbose: print("fire detected, coordinates {}".format(centers[i]))   

            if verbose:
                drawing = img.copy()
                # Draw polygonal contour + circles
                for i in range(len(contours)):
                    color = (random.randint(0,256), random.randint(0,256), random.randint(0,256))
                    cv2.drawContours(drawing, contours_poly, i, color)
                    cv2.circle(drawing, (int(centers[i][0]), int(centers[i][1])), int(radius[i]), color, 2)
                cv2.imwrite("FireDetection.jpg", drawing)
            return coord_fire

    def run(self):
        t=self.getTime()
        t1=self.getTime()
        t2 = self.getTime()

        roll_disturbance = 0
        pitch_disturbance = 0
        yaw_disturbance = 0

        #we add controller args to waypoints and target_altitude variables
        opt_parser = optparse.OptionParser()
        opt_parser.add_option("--patrol_coords", default="11 11, 11 21, 21 21,21 11", help="Specify the patrol coordinates in the format [x1 y1, x2 y2, ...]")
        opt_parser.add_option("--target_altitude", default=42, type=float, help="target altitude of the robot in meters")
        options, _ = opt_parser.parse_args()

        point_list = options.patrol_coords.split(',')
        number_of_waypoints = len(point_list)
        waypoints = []
        for i in range(0, number_of_waypoints):
            waypoints.append([])
            waypoints[i].append(float(point_list[i].split()[0]))
            waypoints[i].append(float(point_list[i].split()[1]))

        target_altitude = options.target_altitude

        while self.step(self.timeStep) != -1:

            # Read sensors
            roll, pitch, yaw = self.imu.getRollPitchYaw()
            Xpos, Ypos, altitude = self.gps.getValues()
            roll_acceleration, pitch_acceleration, _ = self.gyro.getValues()
            self.set_position([Xpos, Ypos, altitude, roll, pitch, yaw])

            # Drop the water from the drone
            if self.water_to_drop > 0:
                self.WaterDropStatus = True
                self.setCustomData(str(self.water_to_drop))
                self.water_to_drop = 0
            else:
                self.setCustomData(str(0))

            if altitude > target_altitude - 1:
                # Motion
                if self.getTime() - t1 > 0.1:
                    yaw_disturbance, pitch_disturbance = self.move_to_target(waypoints)
                    t1=self.getTime()
                # fire detection, and go above the fire
                if self.getTime() - t > 1:      
                    if not self.WaterDropStatus:
                        coord_fire = self.FireDetection()
                        if coord_fire:
                            self.FireDetectionStatus = True
                            X,Y= self.NaiveApproach(coord_fire) #self.ComputeMonoplotting(coord_fire)
                            self.target_pos[0:2] = [X, Y] #update the target position with the ones of the fire
                    t = self.getTime()

                if not self.WaterDropStatus:
                    t2 = self.getTime()
                if self.getTime() - t2 > 15: #wait 15 times to avoid detection of the dropping water as smoke
                    self.WaterDropStatus = False

            roll_input = self.K_ROLL_P * clamp(roll, -1, 1) + roll_acceleration + roll_disturbance
            pitch_input = self.K_PITCH_P * clamp(pitch, -1, 1) + pitch_acceleration + pitch_disturbance
            yaw_input = yaw_disturbance
            clamped_difference_altitude = clamp(target_altitude - altitude + self.K_VERTICAL_OFFSET, -1, 1)
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
