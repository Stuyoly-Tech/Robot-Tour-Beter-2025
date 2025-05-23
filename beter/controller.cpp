#include <mutex>
#include <Arduino.h>
#include <AccelStepper.h>
#include <BMI160Gen.h>

#include "utils.h"
#include "config.h"
#include "controller.h"


Controller::Controller(
  AccelStepper* pStepperL, AccelStepper* pStepperR,
  std::mutex* iSteppersEngaged_mtx, void (*iEngageSteppers)(void* parameter),
  TaskHandle_t* iEngageSteppersHandle,
  uint32_t IntervalIMUs,
  HWCDC* pDebugSerial) {
  stepperL = pStepperL;
  stepperR = pStepperR;
  steppersEngaged_mtx = iSteppersEngaged_mtx;
  engageSteppers = iEngageSteppers;
  engageSteppersHandle = iEngageSteppersHandle;
  IntervalIMUs = iIntervalIMUs;
  debugSerial = pDebugSerial;
}

void Controller::init(float iTheta) {
  init();
  theta = iTheta;
  thetaSetPoint = iTheta;
}

void Controller::init() {
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
  t_0 = micros() / pow(10, 6);
}


void Controller::update() {
  updateTheta();
  float deltaTheta = thetaSetPoint - theta;
  //debugSerial->println(deltaTheta);
  //debugSerial->println(theta);

  switch (state) {
    case 0:
      break;

    case 1:
      //stepperL->run();
      //stepperR->run();
      if (steppersEngaged_mtx->try_lock()) {
        state = 0;
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
      if (abs(thetaSetPoint - theta) < 0.0025) {
        steppersEngaged_mtx->lock();
        stepperL->setCurrentPosition(stepperL->targetPosition());
        stepperR->setCurrentPosition(stepperR->targetPosition());
        steppersEngaged_mtx->unlock();
        state = 0;
      } else {
        int steps = mm_to_steps(0.5 * TRACK_WIDTH * deltaTheta, WHEEL_RADIUS, STEPS_PER_REV);
        steppersEngaged_mtx->lock();
        stepperL->setCurrentPosition(stepperL->targetPosition());
        stepperR->setCurrentPosition(stepperR->targetPosition());
        stepperL->move(-steps);
        stepperR->move(steps);
        steppersEngaged_mtx->unlock();
        xTaskCreatePinnedToCore(engageSteppers, "engageSteppers Task", 10000, NULL, 1, engageSteppersHandle, 1);
        state = 3;
      }
      break;

    //actually turning
    case 3:
      if (steppersEngaged_mtx->try_lock()) {
        if (!stepperL->isRunning() && !stepperR->isRunning()) {
          state = 2;
        }
        //Serial.println("Done with movement");
        steppersEngaged_mtx->unlock();
      }
      break;

    default:
      state = 0;
      break;
  }
}

void Controller::updateTheta() {
  float t_now = micros() / pow(10, 6);
  if (t_now - t_0 > IMU_UPDATE_PERIOD) {
    //Update theta
    double omega = 1 * (((BMI160.getRotationZ() * 1000.0) / 32768.0) * PI/180 + COUNTER_BIAS);

    //debugSerial->printf("OMEGA: %f\n", omega);

    float dTheta = (omega)*(t_now - t_0);

    //Filter
    if (abs(omega) > HIGH_PASS_FREQ) {
      theta += dTheta;
    }

    //Rescale theta
    while (theta > PI) {
      theta -= TWO_PI;
    }
    while (theta < -PI) {
      theta += TWO_PI;
    }

    t_0 = t_now;
  }
}

void Controller::moveX(float dist) {
  if (dist != 0) {
    int maxV = mm_to_steps(vx, WHEEL_RADIUS, STEPS_PER_REV);
    int maxA = mm_to_steps(MAX_ACC, WHEEL_RADIUS, STEPS_PER_REV);
    int steps = mm_to_steps(dist, WHEEL_RADIUS, STEPS_PER_REV);

    //Lock steppers
    steppersEngaged_mtx->lock();

    //set accel and vel
    stepperL->setAcceleration(maxA);
    stepperR->setAcceleration(maxA*RIGHT_OFF);
    stepperL->setMaxSpeed(maxV);
    stepperR->setMaxSpeed(maxV*RIGHT_OFF);
    stepperL->setSpeed(maxV);
    stepperR->setSpeed(maxV*RIGHT_OFF);

    //set wheel positions
    stepperL->move(steps);
    stepperR->move(steps*RIGHT_OFF);

    //Unlock steppers
    steppersEngaged_mtx->unlock();

    //Create task
    xTaskCreatePinnedToCore(engageSteppers, "engageSteppers Task", 10000, NULL, 1, engageSteppersHandle, 1);

    //Update state
    state = 1;
  }
}


void Controller::turnTheta(float targetTheta) {
  int maxOmega = mm_to_steps(MAX_ANG_VEL * WHEEL_RADIUS, WHEEL_RADIUS, STEPS_PER_REV);
  int maxAlpha = mm_to_steps(MAX_ANG_ACC * WHEEL_RADIUS, WHEEL_RADIUS, STEPS_PER_REV);

  //lock steppers
  steppersEngaged_mtx->lock();

  //set accel and vel
  stepperL->setAcceleration(maxAlpha);
  stepperR->setAcceleration(maxAlpha*RIGHT_OFF);
  stepperL->setMaxSpeed(maxOmega);
  stepperR->setMaxSpeed(maxOmega*RIGHT_OFF);
  stepperL->setSpeed(maxOmega);
  stepperR->setSpeed(maxOmega*RIGHT_OFF);

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
  t_0 = micros() / pow(10, 6);

  //Change state
  state = 2;
}

void Controller::setVx(float newVx) {
  vx = newVx;
}