#include <mutex>
#include <Arduino.h>
#include <AccelStepper.h>
#include <SparkFun_BMI270_Arduino_Library.h>

#include "utils.h"
#include "config.h"
#include "controller.h"


controller::Controller(
  AccelStepper* pStepperL, AccelStepper* pStepperR,
  std::mutex *iSteppersEngaged_mtx, void (*iEngageSteppers)(void * parameter),
  TaskHandle_t *iEngageSteppersHandle, 
  BMI270* pImu0, BMI270* pImu1
) 
{
  stepperL = pStepperL;
  stepperR = pStepperR;
  steppersEngaged_mtx = iSteppersEngaged_mtx;
  engageSteppers = iEngageSteppers;
  engageSteppersHandle = iEngageSteppersHandle;
  imu0 = pImu0;
  imu1 = pImu1;
}

void Controller::init(float iTheta) {
  init();
  theta = iTheta;
  thetaSetPoint = iTheta;
}

void controller::init() {
  //init steppers
  steppersEngaged_mtx->lock();
  stepperL->setPinsInverted(true);
  stepperL->setMinPulseWidth(2);
  stepperR->setMinPulseWidth(2);
  steppersEngaged_mtx->unlock();

  //init values
  state = 0;
  theta = 0;
  thetaSetPoint = 0;
  vx = 0;
  t_0 = micros()/pow(10, 6);
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
      } else if (deltaTheta < -PI) {
        deltaTheta += TWO_PI;
      }
      if (abs(targetTheta - theta) < 0.001) {
        steppersEngaged_mtx->lock();
        stepperL->setCurrentPosition(stepperL->targetPosition());
        stepperR->setCurrentPosition(stepperR->targetPosition());
        steppersEngaged_mtx->unlock();
        STATE = 0;
      } else {
        steppersEngaged_mtx->lock();
        stepperL->setCurrentPosition(stepperL->targetPosition());
        stepperR->setCurrentPosition(stepperR->targetPosition());
        stepperL->move(mmToSteps(-0.5 * trackWidth * deltaTheta));
        stepperR->move(mmToSteps(0.5 * trackWidth * deltaTheta));
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
    imu1.getSensorData();
    imu2.getSensorData();
    double angVel = (imu1.data.gyroZ + imu2.data.gyroZ) * PI / 360;
    //Serial.print("angvel: ");
    //Serial.println(angVel, 10);
    double interval = micros() - oldIMUus;
    double dtheta = angVel * (interval / pow(10, 6));
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

void Controller::moveX(float dist) {
  if (dist != 0) {
    int maxV = mm_to_steps(MAX_VEL, WHEEL_RADIUS, STEPS_PER_REV);
    int maxA = mm_to_steps(MAX_ACC, WHEEL_RADIUS, STEPS_PER_REV);
    int steps = mm_to_steps(dist, WHEEL_RADIUS, STEPS_PER_REV);

    //Lock steppers
    steppersEngaged_mtx->lock();

    //set accel and vel
    stepperL->setAcceleration(maxA);
    stepperR->setAcceleration(maxA);
    stepperL->setMaxSpeed(maxV);
    stepperR->setMaxSpeed(maxV);
    stepperL->setSpeed(maxV);
    stepperR->setSpeed(maxV);

    //set wheel positions
    stepperL->move(steps);
    stepperR->move(steps);

    //Unlock steppers
    steppersEngaged_mtx->unlock();

    //Create task
    xTaskCreatePinnedToCore(engageSteppers, "engageSteppers Task", 10000, NULL, 1, engageSteppersHandle, 1);

    //Update state
    STATE = 1;
  }
}


void Controller::turnTheta(float targetTheta) {
  int maxOmega = mm_to_steps(MAX_ANG_VEL, WHEEL_RADIUS, STEPS_PER_REV)*WHEEL_RADIUS;
  int maxAlpha = mm_to_steps(MAX_ANG_ACC, WHEEL_RADIUS, STEPS_PER_REV)*WHEEL_RADIUS;

  //lock steppers 
  steppersEngaged_mtx->lock();

  //set accel and vel
  stepperL->setAcceleration(maxAlpha);
  stepperR->setAcceleration(maxAlpha);
  stepperL->setMaxSpeed(maxOmega);
  stepperR->setMaxSpeed(maxOmega);
  stepperL->setSpeed(maxOmega);
  stepperR->setSpeed(maxOmega);

  //set wheel positions
  thetaSetPoint = targetTheta;
  
  //Adjust to turn the right direction
  while (thetaSetPoint > PI) {
    thetaSetPoint -= TWO_PI;
  }
  while (thetaSetPoint < -PI) {
    thetaSetPoint += TWO_PI;
  }
  
  //Unlock
  steppersEngaged_mtx->unlock();

  //Create task
  xTaskCreatePinnedToCore(engageSteppers, "engageSteppers Task", 10000, NULL, 1, engageSteppersHandle, 1);

  //Update time
  t_0 = micros()/pow(10, 6);

  //Change state
  state = 2;
}

void Controller::setVx(float newVx) {
  vx = newVx;
}

void Controller::getState() {
  return state;
}
