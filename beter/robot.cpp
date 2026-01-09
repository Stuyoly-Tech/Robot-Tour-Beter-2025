#include "simplePursuit.h"
#include "controller.h"
#include "robot.h"

#include "config.h"

Robot::Robot(simplePursuit *iSimplePursuit, Controller *iController, HWCDC *pDebugSerial) {
  robotSimplePursuit = iSimplePursuit;
  robotController = iController;
  debugSerial = pDebugSerial;
}

void Robot::init() {
  robotController->init(PI / 2);
  pathMode = 0;
  state = 0;
}

void Robot::init(int iPathMode) {
  init();
  pathMode = iPathMode;
}

void Robot::update() {
  float dist;
  float theta;
  float deltaTheta;
  float vx;
  int steps;
  //Serial.println(state);
  switch (state) {
    case 0:
      break;

    //deciding forward movement
    case 1:
      dist = robotSimplePursuit->getCurrentGoalPointDist(robotController->position);
      if (robotSimplePursuit->atLastPoint()) {
        dist -= DIST_TO_DOWEL;
        robotController->setVx(robotSimplePursuit->getEndVx(micros() / pow(10, 6) - t_0, dist));
        robotController->lastPoint = true;
      } else {
        robotController->setVx(MAX_VEL);
      }
      robotController->moveX(dist);
      state = 2;
      break;
    //actually moving
    case 2:
      if (robotController->state == 0) {
        if (robotSimplePursuit->atLastPoint()) {
          state = 0;
        } else {
          state = 3;
        }
      }
      robotController->update();
      break;

    //deciding turns
    case 3:
      robotSimplePursuit->nextPoint();
      if (robotSimplePursuit->isTurn()) {
        deltaTheta = PI;
        //debugSerial->println(deltaTheta);

        while (deltaTheta > PI) {
          deltaTheta -= TWO_PI;
        }
        while (deltaTheta < -PI) {
          deltaTheta += TWO_PI;
        }
        robotController->turnTheta(robotController->theta);
        state = 4;
        robotSimplePursuit->nextPoint();
      } else {
        if (robotSimplePursuit->atLastPoint()) {
          robotController->lastPoint = true;
        }
        theta = robotSimplePursuit->getTheta(robotController->position);
        Serial.print("next target theta: ");
        Serial.println(theta);
        deltaTheta = theta - robotController->theta;
        //debugSerial->println(deltaTheta);

        while (deltaTheta > PI) {
          deltaTheta -= TWO_PI;
        }
        while (deltaTheta < -PI) {
          deltaTheta += TWO_PI;
        }

        //correct heading first;
        if (abs(abs(deltaTheta) - PI) < 0.1) {
          robotController->turnTheta(theta - PI);
          state = 5;
        }

        else {
          robotController->turnTheta(theta);
          state = 4;
        }
      }
      break;

    case 4:
      if (robotController->state == 0) {
        state = 1;
      } else {
        //("UPDATE");
        robotController->update();
      }
      break;

    case 5:
      //go backwards
      dist = robotSimplePursuit->getCurrentGoalPointDist(robotController->position);

      if (robotSimplePursuit->atLastPoint()) {
        dist += DIST_TO_DOWEL;
        robotController->setVx(robotSimplePursuit->getEndVx(micros() / pow(10, 6) - t_0, dist));
        robotController->lastPoint = true;
      } else {
        //vx = (2*robotController->mmToSteps(dist)*robotSimplePursuit->getAvgVx(micros() - start_us));
        //vx = vx / (robotController->mmToSteps(dist) + 2*(robotController->mmToSteps(dist)/maxAx));
        robotController->setVx(MAX_VEL);
      }
      robotController->moveX(-dist);
      state = 2;

      //robotController->update();
      break;

    default:
      state = 0;
      break;
  }
}

void Robot::startPath() {
  t_0 = micros() / pow(10, 6);
  state = 1;
}

float Robot::stopPath() {
  state = 0;
  return (micros() / pow(10, 6)) - t_0;
}

int Robot::getState() {
  return state;
}

float Robot::sgn(float n) {
  return (n < 0) ? -1 : 1;
}