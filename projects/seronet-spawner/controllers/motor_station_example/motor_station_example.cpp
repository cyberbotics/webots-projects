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
 * Description:  This controller is used to make the ConveyorStationBelt of the ConveyorStation move.
 */

#include <webots/Motor.hpp>
#include <webots/Robot.hpp>

#define SPAWN_TIME_MARGIN 0.05

using namespace webots;

int main() {
  Robot *robot = new Robot();
  int timeStep = robot->getBasicTimeStep();
  Motor *motorsIn[2];
  char motorInNames[2][20] = {"belt_motor_1_left", "belt_motor_1_right"};
  Motor *motorsOut[2];
  char motorOutNames[2][20] = {"belt_motor_2_left", "belt_motor_2_right"};

  for (int i = 0; i < 2; i++) {
    motorsIn[i] = robot->getMotor(motorInNames[i]);
    motorsOut[i] = robot->getMotor(motorOutNames[i]);
    motorsIn[i]->setPosition(INFINITY);
    motorsOut[i]->setPosition(INFINITY);
    motorsIn[i]->setVelocity(0.15);
    motorsOut[i]->setVelocity(-0.15);
  }

  while (robot->step(timeStep) != -1) {
  }

  delete robot;
  return 0;
}
