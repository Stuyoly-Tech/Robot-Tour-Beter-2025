#include "simplePursuit.h"
#include "controller.h"
#include "robot.h"

#include "config.h"

Robot::Robot(simplePursuit *iSimplePursuit, controller *iController) {
  robotSimplePursuit = iSimplePursuit;
  robotController = iController;
}

void Robot::init() {
  robotController->init(PI/2);
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
      dist = robotSimplePursuit->getCurrentGoalPointDist();
      if (robotSimplePursuit->atLastPoint()) {
        dist -= DIST_TO_DOWEL;
      }
      vx = robotSimplePursuit->getAvgVx();
      robotController->setVx(vx);
      robotController->moveX(dist);
      state = 2;
      break;
    //actually moving
    case 2:
      if (robotController->state == 0) {
        if (robotSimplePursuit->atLastPoint()) {
          state = 0;
        }
        else {
          state = 3;
        }
      }
      robotController->update();
      break;

    //deciding turns
    case 3:
      robotSimplePursuit->nextPoint();
      theta = robotSimplePursuit->getTheta();
      deltaTheta = robotSimplePursuit->getTheta() - robotController->thetaSetPoint;
    
      while (deltaTheta > PI) {
        deltaTheta -= TWO_PI;
      }
      while (deltaTheta < -PI) {
        deltaTheta += TWO_PI;
      }
  
      //correct heading first;
      if (((abs(deltaTheta) == PI) && (pathMode == 1)) && !(robotSimplePursuit->isAGate())) {
        robotController->turnTheta(robotController->thetaSetPoint);
        state = 5;
      }
  
      else {
        theta = robotSimplePursuit->getTheta();
        robotController->turnTheta(theta);
        state = 4;
      }
    break;

    case 4:  
      if (robotController->state == 0) {
        state = 1;
      }
      else {
        //Serial.println("UPDATE");
        robotController->update();
      }
      break;

    case 5:
      //go backwards
      if (robotController->state == 0) {
        dist = robotSimplePursuit->getCurrentGoalPointDist();

        if (robotSimplePursuit->atLastPoint()) {
          dist += DIST_TO_DOWEL; 
        }

        vx = robotSimplePursuit->getAvgVx();
        //vx = (2*robotController->mmToSteps(dist)*robotSimplePursuit->getAvgVx(micros() - start_us));
        //vx = vx / (robotController->mmToSteps(dist) + 2*(robotController->mmToSteps(dist)/maxAx));
        robotController->setVx(vx);
        robotController->moveX(-dist);
        state = 2;
      }
      robotController->update();
      break;
      
    default:
      state = 0;
      break;
  }
}

void Robot::startPath() {
  t_0 = micros()/pow(10, 6);
  state = 1;
}

float Robot::stopPath() {
  state = 0;
  return (micros()/pow(10, 6)) - t_0;
}

int Robot::getstate() {
  return state;
}

float Robot::sgn(float n) {
  return (n < 0) ? -1 : 1;
}
