/*
 * CoordinationRESTServer.hh
 *
 *  Created on: Aug 13, 2020
 *      Author: alexej
 */

#ifndef SMARTSOFT_SRC_COORDINATIONRESTSERVER_HH_
#define SMARTSOFT_SRC_COORDINATIONRESTSERVER_HH_

#include "Box.hh"
#include "cpprest/http_listener.h"

class CoordinationRESTServer {
private:
  web::http::experimental::listener::http_listener rest_listener;

  // some clients perform a "pre-flight" check by calling the options method which is handled here
  void handle_options(web::http::http_request request);

  // this callback method handles the REST API POST requests
  void handle_post(web::http::http_request message);

  // callback method that handles the specify call for adding a new object
  web::http::status_code handle_add_new_object(const web::json::value &input);

  // types of instantiable box and list of boxes to spawn
  BoxType **boxTypes;
  list<Box> *boxes;

public:
  CoordinationRESTServer() = default;
  virtual ~CoordinationRESTServer() = default;

  // starts the internal REST server listening thread
  int start();
  // stops the internal REST server listening thread
  int stop(const bool &wait_till_stopped = true);

  // setters
  void setBoxTypes(BoxType **boxTypes) {this->boxTypes = boxTypes;}
  void setBoxes(list<Box> *boxes) {this->boxes = boxes;}
};

#endif /* SMARTSOFT_SRC_COORDINATIONRESTSERVER_HH_ */
