#include <mutex>

#include <Arduino.h>
#include <AccelStepper.h>
#include "controller.h"
#include <BMI160Gen.h>

#define RIGHT_OFF 1.00

controller::controller
(
  double iWheelRadius, double iTrackWidth,
  AccelStepper *iStepperL, AccelStepper *iStepperR,
  uint32_t iStepsPerRev, uint32_t iTurnInterval,
  uint32_t iIntervalIMUus, std::mutex *iSteppersEngaged_mtx,
  void (*iEngageSteppers)(void * parameter),
  TaskHandle_t *iEngageSteppersHandle,
  double iHighPassFreq
) 
{
  wheelRadius = iWheelRadius;
  trackWidth = iTrackWidth;
  stepperL = iStepperL;
  stepperR = iStepperR;
  stepsPerRev = iStepsPerRev;
  turnInterval = iTurnInterval;
  intervalIMUus = iIntervalIMUus;
  steppersEngaged_mtx = iSteppersEngaged_mtx;
  engageSteppers = iEngageSteppers;
  engageSteppersHandle = iEngageSteppersHandle;
  highPassFreq = iHighPassFreq;
}

void controller::init(double iTheta) {
  init();
  theta = iTheta;
  targetTheta = theta;
}

void controller::init() {
  //init steppers
  steppersEngaged_mtx->lock();
  stepperL->setPinsInverted(true);
  stepperL->setMinPulseWidth(2);
  stepperR->setMinPulseWidth(2);
  steppersEngaged_mtx->unlock();
  //init values
  maxVx = 0;
  maxAx = 0;
  maxAngVx = 0;
  oldus = micros();
  oldIMUus = micros();
  STATE = 0;
  theta = 0;
  targetTheta = 0;
}

void controller::update() {
  updateTheta();
  double deltaTheta = targetTheta - theta;
  //Serial.println(deltaTheta);
  //Serial.println(theta*180/PI);
  switch (STATE) {
    case 0:
      break;
      
    case 1:
      //stepperL->run();
      //stepperR->run();
      if (steppersEngaged_mtx->try_lock()) {
        STATE = 0;
        steppersEngaged_mtx->unlock();
      }
      break;
      
    //deciding turn
    case 2:
      if (deltaTheta > PI) {
         deltaTheta -= TWO_PI;
      }
      else if (deltaTheta < -PI) {
        deltaTheta += TWO_PI;
      }
      if (abs(targetTheta - theta) < 0.001) {
        steppersEngaged_mtx->lock();
        stepperL->setCurrentPosition(stepperL->targetPosition());
        stepperR->setCurrentPosition(stepperR->targetPosition());
        steppersEngaged_mtx->unlock();
        STATE = 0;
      }
      else {
        steppersEngaged_mtx->lock();
        stepperL->setCurrentPosition(stepperL->targetPosition());
        stepperR->setCurrentPosition(stepperR->targetPosition());
        stepperL->move(mmToSteps(-0.5*trackWidth*deltaTheta));
        stepperR->move(mmToSteps(0.5*trackWidth*deltaTheta));
        steppersEngaged_mtx->unlock();
        xTaskCreatePinnedToCore(engageSteppers, "engageSteppers Task", 10000, NULL, 1, engageSteppersHandle, 1);
        delay(20);    
        STATE = 3;
      }
      break;
        
    //actually turning
    case 3:
      if (steppersEngaged_mtx->try_lock()) {
          if (!stepperL->isRunning() && !stepperR->isRunning()) {
            STATE = 2;
          }
          //Serial.println("Done with movement");
          steppersEngaged_mtx->unlock();
      }      
      break;
      
    default:
      STATE = 0;
      break;
  }
}

void controller::updateTheta() {
  //micros() - oldIMUus > intervalIMUus
  if (micros() - oldIMUus > intervalIMUus) {
    double angVel = ((BMI160.getRotationZ() * 1000.0) / 32768.0) * PI/180;
    //Serial.print("angvel: ");
    //Serial.println(angVel, 10);
    double interval = micros() - oldIMUus;
    double dtheta = angVel*(interval/pow(10, 6));
    if (abs(angVel) > highPassFreq) {
      theta += dtheta;
    }
    while (theta > PI) {
      theta -= TWO_PI;
    }
    while (theta < -PI) {
      theta += TWO_PI;
    }
    oldIMUus = micros();
  }
}

long controller::mmToSteps(double mm) {
  return (long)((mm/(TWO_PI*wheelRadius)) * stepsPerRev);
}

double controller::stepsTomm(long steps) {
  return (steps/stepsPerRev)*TWO_PI*wheelRadius;
}

void controller::setMaxVx(double newVx) {
  maxVx = newVx;
}

void controller::setMaxAx(double newAx) {
  maxAx = newAx;
}

void controller::setMaxAngAx(double newAngAx) {
  maxAngAx = newAngAx;
}

void controller::setMaxAngVx(double newAngVx) {
  maxAngVx = newAngVx;
}

void controller::moveX(double dist) {
  if (dist == 0) {
    return;
  }
  steppersEngaged_mtx->lock();
  //set accel and vel
  stepperL->setAcceleration(mmToSteps(maxAx));
  stepperR->setAcceleration(mmToSteps(maxAx*RIGHT_OFF));
  stepperL->setMaxSpeed(mmToSteps(maxVx));
  stepperR->setMaxSpeed(mmToSteps(maxVx*RIGHT_OFF));
  stepperL->setSpeed(mmToSteps(maxVx));
  stepperR->setSpeed(mmToSteps(maxVx*RIGHT_OFF));
  //set wheel positions
  stepperL->move(mmToSteps(dist));
  stepperR->move(mmToSteps(dist));
  steppersEngaged_mtx->unlock();
  xTaskCreatePinnedToCore(engageSteppers, "engageSteppers Task", 10000, NULL, 1, engageSteppersHandle, 1);
  STATE = 1;
}


void controller::setTheta(double newTheta) {
  //set accel and vel
  steppersEngaged_mtx->lock();
  stepperL->setAcceleration(mmToSteps(maxAngAx));
  stepperR->setAcceleration(mmToSteps(maxAngAx*RIGHT_OFF));
  stepperL->setMaxSpeed(mmToSteps(maxAngVx));
  stepperR->setMaxSpeed(mmToSteps(maxAngVx*RIGHT_OFF));
  stepperL->setSpeed(mmToSteps(maxAngVx));
  stepperR->setSpeed(mmToSteps(maxAngVx*RIGHT_OFF));
  //set wheel positions
  targetTheta = newTheta;
  while (targetTheta > PI) {
      targetTheta -= TWO_PI;
  }
  while (targetTheta < -PI) {
      targetTheta += TWO_PI;
  }
  steppersEngaged_mtx->unlock();
  xTaskCreatePinnedToCore(engageSteppers, "engageSteppers Task", 10000, NULL, 1, engageSteppersHandle, 1);
  oldus = micros();
  STATE = 2;
}

double controller::getMaxVx() {
  return maxVx;
}

double controller::getMaxAx() {
  return maxAx;
}

double controller::getMaxAngVx() {
  return maxAngVx;
}

double controller::getMaxAngAx() {
  return maxAngAx;
}

double controller::getTheta() {
  return theta;
}

double controller::getTargetTheta() {
  return targetTheta;
}

void controller::initTheta(double newTheta) {
  //theta = newTheta;
  return;
}

int controller::getState() {
  return STATE;
}
