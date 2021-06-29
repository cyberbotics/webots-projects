/*
 * Copyright 1996-2021 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:  This controller is used to spawn/remove boxes in an industrial
 * environment using a REST server to add them to the simulation.
 */

#include "utils/Box.hh"
#include "utils/CoordinationRESTServer.hh"
#include <cmath>
#include <iostream>
#include <list>
#include <nlohmann/json.hpp>
#include <string>
#include <webots/Supervisor.hpp>

#define SPAWN_TIME_MARGIN 0.05
#define MAX_DISAPPEAR_DISTANCE 0.2

using namespace std;
using namespace webots;
using json = nlohmann::json;

Supervisor *controller = new Supervisor();
CoordinationRESTServer *server = new CoordinationRESTServer();
const int basicTimeStep = controller->getBasicTimeStep();
Node *station = controller->getSelf()->getParentNode();
Node *rootNode = controller->getRoot();
Field *rootChildrenField = rootNode->getField("children");

int main() {
  // Get the location of the deposit station (where objects disappear).
  Node *spawnBoxPoint = station->getFromProtoDef("SpawnBoxPoint");
  Node *removeBoxPoint = station->getFromProtoDef("RemoveBoxPoint");
  if (!spawnBoxPoint || !removeBoxPoint) {
    cerr << "Couldn't find DEF SpawnBoxPoint and/or DEF RemoveBoxPoint in "
            "PROTO file"
         << endl;
    return 0;
  }
  double spawnLocation[3];
  double removeLocation[3];
  double stationRotation[4];
  const double *spawnLocationPointer = spawnBoxPoint->getPosition();
  const double *removeLocationPointer = removeBoxPoint->getPosition();
  const double *stationRotationPointer =
      station->getField("rotation")->getSFRotation();
  for (int i = 0; i < 3; i++) {
    spawnLocation[i] = spawnLocationPointer[i];
    removeLocation[i] = removeLocationPointer[i];
  }
  for (int i = 0; i < 4; i++) {
    stationRotation[i] = stationRotationPointer[i];
  }

  // Get the definition of the different types of items to instantiate.
  const string boxTypesFilePath = "utils/boxtypes.json";
  ifstream boxTypesFile(boxTypesFilePath);
  if (boxTypesFile.fail()) {
    cerr << "Couldn't find JSON file (" << boxTypesFilePath << ")" << endl;
    return 0;
  }

  json items;
  try {
    items = json::parse(boxTypesFile);
  } catch (json::exception &ex) {
    cerr << "Error while parsing JSON file (" << boxTypesFilePath
         << "): " << endl
         << ex.what() << endl;
    return 0;
  }

  int nTypes = items["boxType"].size();
  if (nTypes == 0) {
    cerr << "There must be at least one boxType defined in JSON file ("
         << boxTypesFilePath << "): " << endl;
    return 0;
  }

  BoxType *boxTypes[nTypes];

  // Assign each of these types to a BoxType object.
  for (int i = 0; i < nTypes; ++i) {
    json typeDefinition = items.at("boxType").at(to_string(i));
    string proto = typeDefinition["proto"];
    std::vector<double> rotation_vector = typeDefinition["rotation"];
    double *rotation = &rotation_vector[0];
    std::vector<double> size_vector = typeDefinition["size"];
    double *size = &size_vector[0];
    boxTypes[i] = new BoxType(proto, rotation, size);
  }

  // Declare list of items and setup the server.
  list<Box> boxes;
  server->setBoxTypes(boxTypes);
  server->setBoxes(&boxes);
  server->start();

  while (controller->step(basicTimeStep) != -1) {
    for (list<Box>::iterator boxIterator = boxes.begin();
         boxIterator != boxes.end(); ++boxIterator) {
      // Spawn boxes in the list that are not yet spawned.
      if (!(boxIterator->isBoxSpawned())) {
        boxIterator->spawnBox(rootChildrenField, spawnLocation,
                              stationRotation);
      }
      // Remove boxes that are at the deposit station.
      if (boxIterator->isBoxSpawned() && !(boxIterator->isBoxRemoved())) {
        const double *boxLocation = boxIterator->getBoxPosition(controller);
        if (boxLocation) {
          double distance = 0;
          for (int j = 0; j < 3; ++j) {
            distance += pow(removeLocation[j] - boxLocation[j], 2);
          }
          distance = sqrt(distance);
          if (distance < MAX_DISAPPEAR_DISTANCE) {
            boxIterator->removeBox(controller);
          }
        }
      }
    }
  }

  // Quit simulation.
  server->stop();
  controller->simulationQuit(EXIT_SUCCESS);
  delete controller;
  return 0;
}
