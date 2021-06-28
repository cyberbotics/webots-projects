#include "BoxType.hh"
#include <algorithm>

BoxType::BoxType(string proto, double *rotation, double *size) : proto(proto) {
  for (int i = 0; i < 4; i++) {
    this->rotation[i] = rotation[i];
  }
  for (int i = 0; i < 3; i++) {
    this->size[i] = size[i];
  }
  this->protoCaps = proto;
  transform(this->protoCaps.begin(), this->protoCaps.end(), this->protoCaps.begin(), ::toupper);
}
