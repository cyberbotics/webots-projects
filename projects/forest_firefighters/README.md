# Forest Firefighters

This simulation features a small forest wildfire and a few firefighter robots: a couple of drones and legged robots equipped with cameras.
Users can run the wildfire simulation and program the behavior of the robots to fight the fire.
They can also improve the models of the robot and even design new ones.

**Warning**: This project works only with Webots R2021b.


![fire](https://user-images.githubusercontent.com/12995815/131650395-876f5ce5-ecdc-4eb7-83bc-a86f94709e32.png)
Connect at www.forestfirehub.com for more information about this project.


# Demo Videos
This video shows a wildfire propagating into one tree until the tree is completely burnt:

https://user-images.githubusercontent.com/1264964/130241653-c0fd0966-1ce2-41d1-aeae-452a187b95be.mp4

This video shows a Mavic Pro 2 that fights against a wildfire (speed x2):

https://user-images.githubusercontent.com/12995815/131981332-7b0c6105-5910-4962-8e67-b953e10be9d6.mp4

This video shows two autonomous drones patroling over the forest, detecting and dropping water on wildfires:

https://user-images.githubusercontent.com/48200998/135884718-92a00c63-9f73-4557-96ce-58445ba5eb83.mp4


# Creating Your Own Firefighter Robots

In order to contribute to this simulation by developing your own robot models and program their behavior you will have to follow these steps:

1. download and install [Webots](https://cyberbotics.com) on your computer.
2. install [python 3.8](https://www.python.org/downloads/) on your computer if not already installed.
3. follow the [Webots Tutorials](https://cyberbotics.com/doc/guide/tutorials) to get familiar with the software.
4. [clone](https://docs.github.com/en/github/creating-cloning-and-archiving-repositories/cloning-a-repository-from-github/cloning-a-repository) this repository on your computer: `git clone https://github.com/cyberbotics/webots-projects.git` or download it from [here](https://github.com/cyberbotics/webots-projects/archive/refs/heads/master.zip) and unzip it.
5. run the simulation by opening the [forest_firefighter.wbt](worlds/forest_firefighter.wbt) world file in Webots.
6. edit the robot controllers to change their behavior.
7. create new robot models (or improve existing ones) and use them in the simulation.

For steps 6 and 7, please refer to the Webots [User Guide](https://cyberbotics.com/doc/guide/index) and [Reference Manual](https://cyberbotics.com/doc/reference/index) to get started with robot programming and robot modeling.


# Technical Details

## Forest Generation
An efficient and customizable forest can be generated using the proto `protos/UnevenForest.proto`.
The size of the trees, the dimension and the density of the forest are all parameters of the proto. To do so, you need to delete the current solid `uneven forest`, add the proto `UnevenForest`, change its parameters as you want, Right-Click on the proto and select `Convert Root to Base Node(s)` to make the tree protos accessible for the Supervisor.

Half burned forest using the forest generator:
![burned](https://user-images.githubusercontent.com/12995815/131650414-fb5fe445-c74d-4c89-bc05-562f6a304ef3.png)

## Fire
The fire starts randomly at one of the trees of the forest.
It propagates randomly and according to the wind to other trees close to it.
The fire of a tree stops naturally once the tree is completely burned.
If some water dropped by a robot goes sufficiently close to a tree on fire and if the fire is not too big compared to the amount of water dropped, it extinguishes the fire instantly.
The fire is run by a supervisor which can be found in [controllers/fire/fire.py](controllers/fire/fire.py).
Some fire parameters can be changed in this file: 

- `Fire.MAX_PROPAGATION`: the maximum distance that the fire can propagate in meter.

- `Fire.MAX_EXTINCTION`: the maximum distance from which water can extinguish the fire.
  
- `Fire.FIRE_DURATION`: the duration of a fire to consume a tree completely.
  
- `Wind.INTENSITY_EVOLVE`: the speed at which the intensity of the wind evolve.
   
- `Wind.ANGLE_EVOLVE`: the speed at which the angle of the wind evolve.
  
- `Wind.RANDOM_EVOLUTION`: true if there the wind evolves randomly, false otherwise.
   
- `Tree.ROBUSTNESS_VARIATION`: the variation in the robustness of the trees (if a tree is more robust, it is less likely to burn).

## Wind Window
The Wind window allows the user to modify the intensity and the direction of the wind during a simulation.
The random evolution of the wind can activated/deactivated.

<div align="center"><img src="https://user-images.githubusercontent.com/12995815/131666969-df9b520f-338d-42fc-a533-8e0a161edcc1.png"></div>

## Mavic Pro 2 Demo Controller
The Mavic controller is a simple example controller.
The keyboard can be used to move the robot and drop water on the forest fire.

![mavic](https://user-images.githubusercontent.com/12995815/131667338-302ff820-19ff-4736-8195-b48d1c55a3ad.png)

## Spot Demo Controller
The Spot controller is a basic controller that just moves the legs of the robot. The keyboard can also be used to throw water on the forest fire. 

## Autonomous Mavic Controller

The autonomous Mavic drone moves between chosen coordinates above a forest in order to patrol and detect a fire.
The altitude and world coordinates can be chosen through the controller args.
Each time a fire is detected, the drone goes above the fire and drop a quantity of water to stop the fire.
The project is using OpenCV to detect the smoke color range, and a naive approach to move above the fire (moves until the fire is in the center of the image).
An alternative approach could be to use monoplotting to compute the world coordinates from the image coordinates of the fire (for example based on [this method](http://sar.kangwon.ac.kr/etc/rs_note/rsnote/cp9/cp9-6.htm)).

It is also possible to add multiple drones to increase chances of detecting a fire.
Please note that the fire is starting only when the drone has reached an altitude of 40m.

To use the project, please install OpenCV `pip install opencv-python`.
