/*
 * CoordinationRESTServer.cc
 *
 *  Created on: Aug 13, 2020
 *      Author: alexej
 */

#include "CoordinationRESTServer.hh"
#include <iostream>
#include <nlohmann/json.hpp>

using namespace web;
using nljson = nlohmann::json;
using namespace http;
using namespace utility;
using namespace http::experimental::listener;

int CoordinationRESTServer::start() {
  // address the server will be listening on, (0.0.0.0) means all local
  // addresses
  utility::string_t address = std::string("http://0.0.0.0:10002");
  uri_builder uri(address);

  // start server at given address
  auto address_uri_string = uri.to_uri().to_string();
  rest_listener = http_listener(address_uri_string);

  // bind the handle_options callback method to the server
  rest_listener.support(methods::OPTIONS,
                        std::bind(&CoordinationRESTServer::handle_options, this,
                                  std::placeholders::_1));

  // bind the handle_post callback method to the server
  rest_listener.support(methods::POST,
                        std::bind(&CoordinationRESTServer::handle_post, this,
                                  std::placeholders::_1));

  // trigger server startup and wait until it has fully initialized
  rest_listener.open().wait();

  std::cout << "REST server is listening at address: " << address << std::endl;

  return 0;
}

int CoordinationRESTServer::stop(const bool &wait_till_stopped) {
  if (wait_till_stopped == true) {
    // trigger server shutdown and wait until it is down
    rest_listener.close().wait();
  } else {
    rest_listener.close();
  }
  return 0;
}

// some clients perform a "pre-flight" check by calling the options method which
// is handled here
void CoordinationRESTServer::handle_options(web::http::http_request request) {
  http_response response(status_codes::OK);
  response.headers().add(U("Allow"), U("GET, POST, OPTIONS"));
  response.headers().add(U("Access-Control-Allow-Origin"), U("*"));
  response.headers().add(U("Access-Control-Allow-Methods"),
                         U("GET, POST, OPTIONS"));
  response.headers().add(U("Access-Control-Allow-Headers"), U("Content-Type"));
  request.reply(response);
}

// this callback method handles the REST API POST requests
void CoordinationRESTServer::handle_post(web::http::http_request request) {
  std::cout << "[handle_post] request uri: " << request.relative_uri().path()
            << std::endl;
  // this is the resource URI path (e.g. /states)
  std::vector<std::string> resource_uri_path =
      uri::split_path(request.relative_uri().path());

  if (resource_uri_path.size() < 1) {
    std::cout << "[handle_get] Error false ResourceURI" << std::endl;
    // error code 400: bad request
    http_response response(status_codes::BadRequest);
    response.headers().add(U("Access-Control-Allow-Origin"), U("*"));
    request.reply(response);
    return;
  }

  std::string resource = resource_uri_path[0];

  // we need to specify the CORS header correctly
  http_response response(status_codes::OK);
  response.headers().add(U("Access-Control-Allow-Origin"), U("*"));

  // extract the json value from the request object
  auto jvalue = request.extract_json().get();

  if (resource == "addObject")
    response.set_status_code(handle_add_new_object(jvalue));
  else  // error code 404: resource not found
    response.set_status_code(web::http::status_codes::NotFound);

  // reply with the response object
  request.reply(response);
}

// callback method that handles the specific call for adding a new object
http::status_code
CoordinationRESTServer::handle_add_new_object(const web::json::value &input) {
  std::cout << "[handle_add_new_object] input:" << input << std::endl;

  std::vector<double> size_vector;
  try {
    nljson nlInput = nljson::parse(input.serialize());
    int objectId = nlInput["objectId"];
    nljson values = nlInput["values"];
    int typeId = values["typeId"];

    auto isSizeDefined = values.find("size");
    if (isSizeDefined != values.end()) {
      const std::vector<double> temporary_size_vector = values["size"];
      std::copy(temporary_size_vector.begin(), temporary_size_vector.end(),
                std::back_inserter(size_vector));
    }

    Box requestedBox(boxTypes[typeId], objectId, &size_vector[0]);
    boxes->push_back(requestedBox);
    std::cout << "Server list boxes: " << boxes->size() << std::endl;
  } catch (json::json_exception &ex) {
    std::cerr << "Addobject: Error in request syntax (" << ex.what() << ")"
              << endl;
    return 0;
  }

  bool addOK = true;
  if (addOK) {
    return status_codes::OK;
  }

  // error code 500: internal server error
  return status_codes::InternalError;
}
