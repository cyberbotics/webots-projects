# Forest Firefighters

This simulation features a small forest wildfire and a few firefighter robots: a couple of drones and legged robots equipped with cameras.
Users can run the wildfire simulation and program the behavior of the robots to fight the fire.
They can also improve the models of the robot and even design new ones.
![fire](https://user-images.githubusercontent.com/12995815/131650395-876f5ce5-ecdc-4eb7-83bc-a86f94709e32.png)
Connect at www.forestfirehub.com for more information about this project.


## Forest generation
An efficient and customizable forest can be generated using the proto `protos/UnevenForest.proto`.
The size of the trees, the dimension and the density of the forest are all parameters of the proto.

Half burned forest using the forest generator:
![burned](https://user-images.githubusercontent.com/12995815/131650414-fb5fe445-c74d-4c89-bc05-562f6a304ef3.png)


## Fire
The fire starts randomly at one of the trees of the forest.
It propagates randomly and according to the wind to other trees close to it.
The fire of a tree stops naturally once the tree is completely burned.
If some water dropped by a robot goes sufficiently close to a tree on fire and if the fire is not too big compare to the amount of water dropped, it extinguishes the fire instantly.
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


## Mavic Pro 2 controller
The Mavic controller is a simple example controller.
The keyboard can be used to move the robot and drop water on the forest fire.

![mavic](https://user-images.githubusercontent.com/12995815/131667338-302ff820-19ff-4736-8195-b48d1c55a3ad.png)


## Spot controller  
The Spot controller is a basic controller that just moves the legs of the robot. The keyboard can also be used to throw water on the forest fire 


## Demo videos
This video shows a wildfire propagating into one tree until the tree is completely burnt:

https://user-images.githubusercontent.com/1264964/130241653-c0fd0966-1ce2-41d1-aeae-452a187b95be.mp4

This video shows a Mavic Pro 2 that fights against a wildfire (speed x2):

https://user-images.githubusercontent.com/12995815/131981332-7b0c6105-5910-4962-8e67-b953e10be9d6.mp4

