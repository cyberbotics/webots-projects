# SeRoNet Spawner
![spawner_small](https://user-images.githubusercontent.com/38250944/123643493-b8748b80-d824-11eb-9a5a-6d17e9b63180.png)


This project enables to use a REST API to spawn different kind of boxes, such as `CardboardBox` or `PlasticCrate`.
It comes with two worlds: the first is a demo world (`spawner_demo.wbt`) where the different controllers are already implemented.
The second, `spawner_seronet.wbt`, is meant to be used with the SeRoNet-Tooling-Collection software, so all the controllers (robot, conveyor belts, etc.) are defined as `<extern>` (except the inner controller of the `ConveyorStation`, `box_creator`, which handles the REST API and is independent).

## Configuration

Both worlds make use of a `ConveyorStation` with 2 `ConveyorStationPlatform`: one is the place where robots deposit boxes to remove, while the other outputs the box requested by the REST API.

In general though, the `ConveyorStation` can have an adjustable number of `ConveyorStationPlatform` (by setting `platformNumber`).
The name of the components for the `i`th `ConveyorStationPlatform` is:


* Left motor: `motorNamePrefix`_`i`_left
* Right motor: `motorNamePrefix`_`i`_right
* Distance sensor: `sensorNamePrefix`_`i`

If there is only one `ConveyorStationPlatform` the names are simply (`motorNamePrefix`_left, `motorNamePrefix`_right, `sensorNamePrefix`). In our case, the platform number 1 is the removal place and number 2 the spawning place.


# Dependencies
* [cpprestsdk](https://github.com/microsoft/cpprestsdk): `sudo apt install  libcpprest-dev`
* [nlohmann](https://json.nlohmann.me/): `sudo apt install nlohmann-json3-dev`

Only for the SeRoNet example:
* SeRoNet-Tooling-Collection: https://xito.one
* Open62541CppWrapper: https://github.com/Servicerobotics-Ulm/Open62541CppWrapper
* OpcUaDeviceRepository: https://github.com/Servicerobotics-Ulm/OpcUaDeviceRepository

## How to run
Here are the instructions to run these examples after having installed the aforementionned dependencies and downloaded this folder.

### Opening world
`spawner_demo.wbt` can be simply opened in the Webots interface, while `spawner_seronet.wbt` must be opened through the SeRoNet-Tooling-Collection so that the `<extern>` controller are linked.
### REST API request
Once the world is running, use a REST Client to send the following request:
* Method: POST
* Address: http://0.0.0.0:10002/addObject
* Body (application/json): { "objectId": 25, "values": { "typeId": 1, "size": "0.25 0.2 0.3" } }

#### Request parameters
The different parameters of the request above are
* `objectId`: the indivudal identifiant (integer) to give to the box.
* values:
  * `typeId`: the type of box to spawn (see **Edit box types** section below)
  * `size`: the size of the box in meters (LxHxW)

### Edit box types
`typeId`, the type of box to spawn, search for the corresponding predefined types in `controllers/box_creator/utils/boxtypes.json` that can be edited or completed.

## Additional information for SeRonet-Tooling-Collection
The main controller of the ConveyorStation is for the OPC_UA interface.
An updated version (draft, not tested) of the OPC_UA_STATION controller with the new names of components, `Spawner_OPC_UA_STATION`, is set as default for the `spawner_seronet.wbt` world.
The existing OPC_UA motor commands are applied to the conveyor belt of the remover part of the `ConveyorStation`.
The motor of the spawner part are initialized in speed control but the interface with OPC_UA must be implemented.
