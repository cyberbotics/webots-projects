// OPC_UA server for an production station with conveyor belt and signal lights
// is used by SeRoNet Tooling system, THU Ulm 2021, Thomas Feldmeier

/* to compile this:
 cd build
 cmake ..
 make
 cp OPC_UA_Station ..
 */

// https://github.com/Servicerobotics-Ulm/OpcUaDeviceRepository/tree/master/OPCUAProductionStation:
// pictures below for arguments
// https://github.com/Servicerobotics-Ulm/OpcUaDeviceRepository/blob/master/OPCUAProductionStation/OpcUaProductionStation.cc:
// see createServerSpace()
#include "OpcUaGenericServer.hh"

#include <chrono>
#include <mutex>
#include <thread>

#include <webots/DistanceSensor.hpp>
#include <webots/LED.hpp>
#include <webots/Motor.hpp>
#include <webots/Robot.hpp>

using namespace OPCUA;
using namespace webots;
using namespace std;

bool redLed = false;
bool orangeLed = false;
bool greenLed = false;
bool isBoxPresent = false;
int motorSpeed = 0;
int motorTimeout = 0;
Robot *robot;
mutex wait_isBoxPresent;
mutex wait_timeout;
int port;

class MyServer : public GenericServer {
private:
  thread thr;
  bool cancelled;

  void handleOnWrite(const string &browseName, const Variant &value) {
    if (browseName == "LED_RED")
      redLed = value;
    if (browseName == "LED_YELLOW")
      orangeLed = value;
    if (browseName == "LED_GREEN")
      greenLed = value;
  }

  void handleOnRead(const string &browseName, Variant &value) {
    if (browseName == "LED_RED")
      value = redLed;
    if (browseName == "LED_YELLOW")
      value = orangeLed;
    if (browseName == "LED_GREEN")
      value = greenLed;
    if (browseName == "isBoxPresent")
      value = isBoxPresent;
  }

  virtual void handleMethodCall(const string &browseName,
                                const vector<Variant> &inputs,
                                vector<Variant> &outputs) override {
    if (inputs.size() == 1) {
      if (browseName == "loadbox")
        motorSpeed = 1;
      else if (browseName == "start_unloading")
        motorSpeed = -1;
      else if (browseName == "stop_unloading")
        motorSpeed = 0;
      else
        return;
      cout << "OPC_UA_Station:" << port << " call " << browseName << "("
           << (int)(inputs[0]) << ")"
           << " : motorSpeed = " << motorSpeed << endl;
      // bug: integer overflow if simulation runs for 596 hours
      // but int32 is probably atomic to read/write, so no synchronisation
      // needed this way
      motorTimeout = robot->getTime() * 1000 + 1000 * (int)(inputs[0]);
    } else
      cout << "OPC_UA_Station:" << port << " call with " << inputs.size()
           << " arguments?" << endl;
  }

public:
  MyServer(const string &rootObjectName)
      : GenericServer(rootObjectName, 1, port) {
    cancelled = false;
  }
  virtual ~MyServer() { stopThread(); }

  void startThread() {
    // spawn new thread
    thr = thread(&MyServer::execution, this);
  }

  void stopThread() {
    cancelled = true;
    thr.join();
  }

  void execution() {
    while (!cancelled) {
      //      redLed = !redLed;
      //      writeVariable("LED_RED", redLed);
      this_thread::sleep_for(chrono::milliseconds(2000));
    }
  }

  virtual bool createServerSpace() override {
    // arguments: string:name, *:type, bool:readOnly
    // the second argument 'type': its value is not used, only its type
    if (!addVariableNode("isBoxPresent", false, true) ||
        !addVariableNode("LED_RED", false, false) ||
        !addVariableNode("LED_YELLOW", false, false) ||
        !addVariableNode("LED_GREEN", false, false)) {
      cout << "failed adding a node" << endl;
      return false;
    }

    // add the method loadbox
    map<string, Variant> loadboxInputArguments;
    loadboxInputArguments["timeout"] = 0;

    map<string, Variant> loadboxOutputArguments;
    loadboxOutputArguments["result"] = string();

    if (addMethodNode("loadbox", loadboxInputArguments,
                      loadboxOutputArguments) != true) {
      cout << "failed adding loadbox" << endl;
      return false;
    }

    // add the method start_unloading
    map<string, Variant> start_unloadingInputArguments;
    start_unloadingInputArguments["XtimeoutX"] = 0;

    map<string, Variant> start_unloadingOutputArguments;
    start_unloadingOutputArguments["result"] = string();

    if (addMethodNode("start_unloading", start_unloadingInputArguments,
                      start_unloadingOutputArguments) != true) {
      cout << "failed adding start_unloading" << endl;
      return false;
    }

    // add the method stop_unloading
    map<string, Variant> stop_unloadingInputArguments;
    stop_unloadingInputArguments["XtimeoutX"] = 0;

    map<string, Variant> stop_unloadingOutputArguments;
    stop_unloadingOutputArguments["result"] = string();

    if (addMethodNode("stop_unloading", stop_unloadingInputArguments,
                      stop_unloadingOutputArguments) != true) {
      cout << "failed adding stop_unloading" << endl;
      return false;
    }

    return true;
  }
};

void start_server(string s) {
  MyServer server(s);
  server.startThread();
  server.run();
}

int main(int argc, char *argv[]) {
  std::cout << "OPC_UA_Station - main " << std::endl;
  robot = new Robot();

  // OPC UA server
  if (argc != 3) {
    std::cout << "OPC_UA_Station needs 2 controller args: Name and Port"
              << std::endl;
    return -1;
  }
  port = atoi(argv[2]);
  cout << "OPC_UA_Station " << argv[1] << ":" << port << endl;
  thread first_thread(start_server, argv[1]);

  // webots
  int timeStep = (int)(robot->getBasicTimeStep() + 0.5);
  LED *redLedDevice = robot->getLED("red_led");
  LED *orangeLedDevice = robot->getLED("orange_led");
  LED *greenLedDevice = robot->getLED("green_led");
  // Motor *motorDevice0 = robot->getMotor("belt_motor0");
  // Motor *motorDevice1 = robot->getMotor("belt_motor1");
  Motor *motorRemoverLeft = robot->getMotor("belt_motor_1_left");
  Motor *motorRemoverRight = robot->getMotor("belt_motor_1_right");
  Motor *motorSpawnerLeft = robot->getMotor("belt_motor_2_left");
  Motor *motorSpawnerRight = robot->getMotor("belt_motor_2_right");

  // DistanceSensor *distanceDevice = robot->getDistanceSensor("isBoxPresent");
  DistanceSensor *distanceDeviceRemover =
      robot->getDistanceSensor("isBoxPresent_1");
  DistanceSensor *distanceDeviceSpawner =
      robot->getDistanceSensor("isBoxPresent_2");

  // motorDevice0->setPosition(INFINITY);
  // motorDevice1->setPosition(INFINITY);
  motorRemoverLeft->setPosition(INFINITY);
  motorRemoverRight->setPosition(INFINITY);
  motorSpawnerLeft->setPosition(INFINITY);
  motorSpawnerRight->setPosition(INFINITY);

  // distanceDevice->enable(timeStep);
  distanceDeviceSpawner->enable(timeStep);

  while (robot->step(timeStep) != -1) {
    isBoxPresent = distanceDeviceRemover->getValue() > 0.3;
    if (motorTimeout > 0 && robot->getTime() * 1000 >= motorTimeout) {
      cout << "OPC_UA_Station:" << port << " timeout, motor stopped" << endl;
      motorTimeout = -1;
      motorSpeed = 0;
    }
    if (motorSpeed > 0 && isBoxPresent) {
      cout << "OPC_UA_Station:" << port << " box arrived, motor stopped"
           << endl;
      motorSpeed = 0;
    }
    motorRemoverLeft->setVelocity(motorSpeed * 0.3);
    motorRemoverRight->setVelocity(motorSpeed * 0.3);
    redLedDevice->set(redLed);
    greenLedDevice->set(isBoxPresent);
    orangeLedDevice->set(motorSpeed != 0);

    // Add a similar code for controlling the conveyor belt where the boxes are
    // spawned.
  }
  delete robot;
  return 0;
}
