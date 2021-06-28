#ifndef BOXTYPE_HH_
#define BOXTYPE_HH_

#include <list>
#include <string>
using namespace std;

class BoxType {
public:
  BoxType(string proto, double *rotation, double *size);

private:
  const string proto;
  string protoCaps;
  double rotation[4];
  double size[3];
  list<int> ids;
  
  friend class Box;
};

#endif
