#include "Box.hh"
#include <iostream>
using namespace std;

Box::Box(BoxType *boxType, int id, double *size) : boxType(boxType), id(id) {
  double *chosenSize = size ? size : this->boxType->size;
  for (int i = 0; i < 3; i++) {
    this->size[i] = chosenSize[i];
  }
  this->boxType->ids.push_back(this->id);
}

void Box::spawnBox(Field *field, const double *spawnLocation,
                   const double *stationRotation) {
  // Add an offset to the height so that bottom of object is on the belt.
  const double spawnLocationHeight =
      (this->boxType->proto == "PlasticCrate") ? 0 : 0 + (this->size[1] / 2);

  const string defName = this->boxType->protoCaps + "_" + to_string(this->id);
  const string translationString =
      " translation " + to_string(spawnLocation[0]) + " " +
      to_string(spawnLocation[1]) + " " +
      to_string(spawnLocation[2] + spawnLocationHeight);
  const string rotationString = " rotation " +
                                to_string(this->boxType->rotation[0]) + " " +
                                to_string(this->boxType->rotation[1]) + " " +
                                to_string(this->boxType->rotation[2]) + " " +
                                to_string(this->boxType->rotation[3]);
  const string sizeString = " size " + to_string(this->size[0]) + " " +
                            to_string(this->size[1]) + " " +
                            to_string(this->size[2]);
  string importString;

  if (stationRotation[3] == 0) {
    importString = "DEF " + defName + " " + this->boxType->proto + " { " +
                   translationString + rotationString + sizeString + " mass 1" +
                   " }";
  } else {
    // Handle case where the station is rotated.
    const string translationStringRotatedCase =
        " translation " + to_string(spawnLocation[0]) + " " +
        to_string(spawnLocation[1]) + " " + to_string(spawnLocation[2]);
    const string rotationStringRotatedCase =
        " rotation " + to_string(stationRotation[0]) + " " +
        to_string(stationRotation[1]) + " " + to_string(stationRotation[2]) +
        " " + to_string(stationRotation[3]);

    importString = "DEF TRANSFORM_" + defName + " Transform { " +
                   translationStringRotatedCase + rotationStringRotatedCase +
                   " children [ " + "DEF " + defName + " " +
                   this->boxType->proto + " { translation 0 0 " +
                   to_string(spawnLocationHeight) + rotationString +
                   sizeString + " mass 1" + " }" + "] }";
  }

  // Creates the element in Webots.
  field->importMFNodeFromString(0, importString);
  this->isSpawned = true;
  cout << "Spawned " << defName << endl;
}

void Box::removeBox(Supervisor *supervisor) {
  const string defName = this->boxType->protoCaps + "_" + to_string(this->id);
  Node *boxToRemove = supervisor->getFromDef(this->boxType->protoCaps + "_" +
                                             to_string(this->id));
  boxToRemove->remove();
  this->isRemoved = true;
  cout << "Removed " << defName << endl;
}

const double *Box::getBoxPosition(Supervisor *supervisor) const {
  Node *boxToLocate = supervisor->getFromDef(this->boxType->protoCaps + "_" +
                                             to_string(this->id));
  if (boxToLocate)
    return boxToLocate->getPosition();
  return nullptr;
}
