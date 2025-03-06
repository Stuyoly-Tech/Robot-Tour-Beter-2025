#include "simplePursuit.h"
#include "controller.h"
#include "robot.h"

robot::robot
( 
  simplePursuit *iSimplePursuit, controller *iController,
  double iMaxAx, double iMaxAngAx, double iMaxAngVx,
  double iCenterToDowel
)
{
  robotSimplePursuit = iSimplePursuit;
  robotController = iController;
  maxAx = iMaxAx;
  maxAngAx = iMaxAngAx;
  maxAngVx = iMaxAngVx;
  centerToDowel = iCenterToDowel;
}

void robot::init() {
  robotController->init(PI/2);
  robotController->setMaxAx(maxAx);
  robotController->setMaxAngVx(maxAngVx);
  robotController->setMaxAngAx(maxAngAx);
  pathMode = 0;
  STATE = 0;
}

void robot::init(int iPathMode) {
  init();
  pathMode = iPathMode;
}

void robot::update() {
  double dist;
  double theta;
  double deltaTheta;
  double vx;
  Serial.println(STATE);
  switch (STATE) {
    case 0:
      break;

    //deciding forward movement 
    case 1:
      dist = robotSimplePursuit->getCurrentGoalPointDist();
      if (robotSimplePursuit->atLastPoint()) {
        dist -= centerToDowel;
      }
      //vx = robotSimplePursuit->getAvgVx();
      vx = (2*robotController->mmToSteps(dist)*robotSimplePursuit->getAvgVx(micros()-start_us));
      vx = vx / (robotController->mmToSteps(dist) + 2*(robotController->mmToSteps(dist)/maxAx));
      robotController->setMaxVx(vx);
      robotController->moveX(dist);
      STATE = 2;
      break;
    //actually moving
    case 2:
      if (robotController->getState() == 0) {
        if (robotSimplePursuit->atLastPoint()) {
          STATE = 0;
        }
        else {
          STATE = 3;
        }
      }
      robotController->update();
      break;

    //deciding turns
    case 3:
      robotSimplePursuit->nextPoint();
      theta = robotSimplePursuit->getTheta();
      deltaTheta = robotSimplePursuit->getTheta() - robotController->getTargetTheta();
    
      while (deltaTheta > PI) {
        deltaTheta -= TWO_PI;
      }
      while (deltaTheta < -PI) {
        deltaTheta += TWO_PI;
      }
  
      //correct heading first;
      if (((abs(deltaTheta) == PI) && (pathMode == 1)) && !(robotSimplePursuit->isAGate())) {
        robotController->setTheta(robotController->getTargetTheta());
        STATE = 5;
      }
  
      else {
        theta = robotSimplePursuit->getTheta();
        robotController->setTheta(theta);
        STATE = 4;
      }
    break;

    case 4:  
      if (robotController->getState() == 0) {
        STATE = 1;
      }
      else {
        Serial.println("UPDATE");
        robotController->update();
      }
      break;

    case 5:
      //go backwards
      if (robotController->getState() == 0) {
        dist = robotSimplePursuit->getCurrentGoalPointDist();

        if (robotSimplePursuit->atLastPoint()) {
          dist += centerToDowel;
          
        }

        //vx = robotSimplePursuit->getAvgVx();
        vx = (2*robotController->mmToSteps(dist)*robotSimplePursuit->getAvgVx(micros() - start_us));
        vx = vx / (robotController->mmToSteps(dist) + 2*(robotController->mmToSteps(dist)/maxAx));
        robotController->setMaxVx(vx);
        robotController->moveX(-dist);
        STATE = 2;
      }
      robotController->update();
      break;
      
    default:
      STATE = 0;
      break;
  }
}

void robot::startPath() {
  STATE = 1;
  start_us = micros();
}

double robot::stopPath() {
  STATE = 0;
  return (micros() - start_us)/pow(10, 6);
}

uint8_t robot::getState() {
  return STATE;
}

double robot::sgn(double n) {
  return (n < 0) ? -1 : 1;
}
