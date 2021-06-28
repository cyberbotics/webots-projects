# SeRoNet Spawner
![spawner_small](https://user-images.githubusercontent.com/38250944/123643493-b8748b80-d824-11eb-9a5a-6d17e9b63180.png)


This project enables to use a REST API to spawn different kind of boxes. It comes with two worlds: the first is a demo world (`spawner_demo.wbt`) where the different controllers are already implemented. The second, `spawner_seronet.wbt`, is meant to be used with the SeRoNet-Tooling-Collection software, so all the controller (robot, conveyor belts, etc.) are `<extern>` (except the inner controller of the `ConveyorStation`, `box_creator`, which handles the REST API and is independent).

## Configuration

Both worlds make use of a `ConveyorStation` with 2 `ConveyorStationPlatform`: one will output the box requested through the REST API, while the other will be the place where robots deposit boxes to remove.

In general though, the `ConveyorStation` can have an adjustable number of `ConveyorStationPlatform` (by setting `platformNumber`). The name of the components for the `i`th `ConveyorStationPlatform` is:


* Left motor: `motorNamePrefix`_`i`_left
* Right motor: `motorNamePrefix`_`i`_right
* Distance sensor: `sensorNamePrefix`_`i`

If there is only one `ConveyorStationPlatform` the name are simply (`motorNamePrefix`_left, `motorNamePrefix`_right, `sensorNamePrefix`).


# Dependencies
* cpprestsdk: https://github.com/microsoft/cpprestsdk

Only for the SeRoNet example:
* SeRoNet-Tooling-Collection: https://xito.one
* Open62541CppWrapper: https://github.com/Servicerobotics-Ulm/Open62541CppWrapper
* OpcUaDeviceRepository: https://github.com/Servicerobotics-Ulm/OpcUaDeviceRepository

## How to run
Here are the instructions to run these examples after having installed the aforementionned dependencies.

### Opening world
`spawner_demo.wbt` can be simply opened in the Webots interface, while `spawner_seronet.wbt` must be opened through the SeRoNet-Tooling-Collection so that the `<extern>` controller are linked.
### REST API request
Once the 

Method: POST
Address: http://0.0.0.0:10002/addObject
Body (application/json): {"objectId":25, "values":{"typeId":1,"size":"0.25 0.2 0.3"}}


Where objectId is just an indivudal id for the box to spawn, typeId is the type of box to spawn (matching the definition of the predefined types in boxtypes.json), and size determines the size of the box in meters (/!\ for the moment y is the vertical dimension).

The main controller of the ConveyorStation is for the OPC_UA interface, for the moment it is only motor_station_example which simply turns on the motor of the two platforms. --> An updated version (draft, not tested) of the OPC_UA_STATION controller with the new names of components is available (Spawner_OPC_UA_STATION).
