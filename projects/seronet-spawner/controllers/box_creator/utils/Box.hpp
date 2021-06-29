#ifndef BOX_HPP_
#define BOX_HPP_

#include "BoxType.hpp"
#include <webots/Supervisor.hpp>
using namespace webots;

class Box {
public:
  Box(BoxType *boxType, int id, double *size);
  void spawnBox(Field *field, const double *spawnLocation,
                const double *stationRotation);
  void removeBox(Supervisor *supervisor);
  bool isBoxSpawned() const { return this->isSpawned; }
  bool isBoxRemoved() const { return this->isRemoved; }
  const double *getBoxPosition(Supervisor *supervisor) const;
  const int getId() const { return this->id; };

private:
  BoxType *boxType;
  const int id;
  double size[3];
  bool isSpawned = false;
  bool isRemoved = false;
};

#endif
