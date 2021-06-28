######################################################
## Spawner - ConveyorStation
######################################################

# Conveyor Station

It can have an adjustable number of platforms (by setting platformNumber).

The name of the belt motors are for each platform:

Left motor: motorNamePrefix_i_left
Right motor: motorNamePrefix_i_right


Where motorNamePrefix is a settable field of ConveyorStation.proto and i=[1..platformNumber]
If there is only one platform the name are simply (motorNamePrefix_left, motorNamePrefix_right).


The name of the distance sensor is for each platform: sensorNamePrefix_i.


# Spawner

Assumes that platformNumber=2 (platform 1 is for the deposit of boxes and platform 2 for the spawning).

The box_creator controller handles the REST requests. The syntax for the request is:

Method: POST
Address: http://0.0.0.0:10002/addObject
BODY (application/json): {"objectId":25, "values":{"typeId":1,"size":"0.25 0.2 0.3"}}


Where objectId is just an indivudal id for the box to spawn, typeId is the type of box to spawn (matching the definition of the predefined types in boxtypes.json), and size determines the size of the box in meters (/!\ for the moment y is the vertical dimension).

The main controller of the ConveyorStation is for the OPC_UA interface, for the moment it is only motor_station_example which simply turns on the motor of the two platforms. --> An updated version (draft, not tested) of the OPC_UA_STATION controller with the new names of components is available (Spawner_OPC_UA_STATION).
