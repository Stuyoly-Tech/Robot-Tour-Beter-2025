#include <mutex>
#include <Arduino.h>
#include <AccelStepper.h>
#include <SparkFun_BMI270_Arduino_Library.h>
#include <ArduinoEigenDense.h>
using namespace Eigen;

#include "utils.h"
#include "config.h"
#include "controller.h"


Controller::Controller(
  AccelStepper* pStepperL, AccelStepper* pStepperR,
  std::mutex* iSteppersEngaged_mtx, void (*iEngageSteppers)(void* parameter),
  TaskHandle_t* iEngageSteppersHandle,
  BMI270* pImu,
  HWCDC* pDebugSerial) {
  stepperL = pStepperL;
  stepperR = pStepperR;
  steppersEngaged_mtx = iSteppersEngaged_mtx;
  engageSteppers = iEngageSteppers;
  engageSteppersHandle = iEngageSteppersHandle;
  imu = pImu;
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
  position = Vector2f(0, 0);
  vx = 0;
  t_0 = micros() / pow(10, 6);
  dt = 0;
  lastPosL = 0;
  lastPosR = 0;
  lastPoint = false;
  stepperL->setCurrentPosition(0);
  stepperR->setCurrentPosition(0);
}

void Controller::gyroInit() {
  //Configure IMUs
  delay(1000);
  bmi2_sens_config gyroConfig;
  gyroConfig.type = BMI2_GYRO;
  gyroConfig.cfg.gyr.odr = BMI2_GYR_ODR_200HZ;
  gyroConfig.cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;
  gyroConfig.cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;
  gyroConfig.cfg.gyr.range = BMI2_GYR_RANGE_500;
  gyroConfig.cfg.gyr.noise_perf = BMI2_PERF_OPT_MODE;

  imu->setConfig(gyroConfig);


  imu->beginI2C(IMU_ADDRESS);
  imu->performComponentRetrim();
  imu->performGyroOffsetCalibration();

  delay(1000);

  float sum = 0;
  t_0 = micros() / pow(10, 6);
  //Begin reading
  for (int i = 0; i < 1000;) {
    float t = micros() / pow(10, 6);
    if (t - t_0 > IMU_UPDATE_PERIOD) {
      imu->getSensorData();
      float omega = (imu->data.gyroZ) * PI / 180.0;
      sum += omega;
      t_0 = t;
      i++;
    }
  }
  gyroOffset = sum / 1000;

  delay(1000);
}


void Controller::update() {
  //Serial.println(dt);
  updateTheta();

  float deltaTheta = thetaSetPoint - theta;
  //debugSerial->println(deltaTheta);
  //debugSerial->
  //Serial.println(theta, 10);

  switch (state) {
    case 0:
      break;

    case 1:
      stepperL->run();

      stepperR->run();

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
      if ((abs(deltaTheta) < 0.1 && !lastPoint) || abs(deltaTheta) < 0.0001) {
        steppersEngaged_mtx->lock();
        //stepperL->setCurrentPosition(stepperL->targetPosition());
        //stepperR->setCurrentPosition(stepperR->targetPosition());
        steppersEngaged_mtx->unlock();
        state = 0;
      } else {
        int steps = mm_to_steps(0.5 * TRACK_WIDTH * deltaTheta, WHEEL_RADIUS, STEPS_PER_REV);
        steppersEngaged_mtx->lock();
        //stepperL->setCurrentPosition(stepperL->targetPosition());
        //stepperR->setCurrentPosition(stepperR->targetPosition());
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
  dt = micros() / pow(10, 6) - t_0;
  if (dt > IMU_UPDATE_PERIOD) {
    t_0 = micros() / pow(10, 6);
    //Serial.println(theta, 10);
    //Serial.println(position(0));
    //Serial.println(position(1));

    //Update theta
    imu->getSensorData();

    //debugSerial->
    //Serial.printf("bihh IMU: %f\n", imu->data.gyroZ);

    float omega = (imu->data.gyroZ) * PI / 180.0;
    //float omega0 = PI / 772.77777775;  //increasing: 772.75  Decreasing: 772.8
    //Serial.println(imu->data.gyroZ);

    //debugSerial->
    //Serial.printf("OMEGA: %f\n", omega);

    float dTheta = (omega - gyroOffset) * (dt);

    theta += dTheta;

    //Rescale theta
    while (theta > PI) {
      theta -= TWO_PI;
    }
    while (theta < -PI) {
      theta += TWO_PI;
    }
    updatePosition(dTheta);
  }
}

void Controller::updatePosition(float dTheta) {
  //Serial.print("x: ");
  //Serial.println(position(0));
  //Serial.print("y: ");
  //Serial.println(position(1));

  float dxL = stepperL->currentPosition() - lastPosL;
  float dxR = stepperR->currentPosition() - lastPosR;
  float thetaMid = theta - dTheta / 2;
  position(0) += steps_to_mm(dxL) * cos(thetaMid) / 2;
  position(0) += steps_to_mm(dxR) * cos(thetaMid) / 2;
  position(1) += steps_to_mm(dxL) * sin(thetaMid) / 2;
  position(1) += steps_to_mm(dxR) * sin(thetaMid) / 2;
  lastPosL = stepperL->currentPosition();
  lastPosR = stepperR->currentPosition();
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
    stepperR->setAcceleration(maxA * RIGHT_OFF);
    stepperL->setMaxSpeed(maxV);
    stepperR->setMaxSpeed(maxV * RIGHT_OFF);
    stepperL->setSpeed(maxV);
    stepperR->setSpeed(maxV * RIGHT_OFF);

    //set wheel positions
    stepperL->move(steps);
    stepperR->move(steps * RIGHT_OFF);

    //Unlock steppers
    steppersEngaged_mtx->unlock();

    //Create task
    xTaskCreatePinnedToCore(engageSteppers, "engageSteppers Task", 10000, NULL, 1, engageSteppersHandle, 1);

    //Update state
    state = 1;
  }
}


void Controller::turnTheta(float targetTheta, bool SLOW) {
  int maxOmega = mm_to_steps(MAX_ANG_VEL * WHEEL_RADIUS, WHEEL_RADIUS, STEPS_PER_REV);
  int maxAlpha = mm_to_steps(MAX_ANG_ACC * WHEEL_RADIUS, WHEEL_RADIUS, STEPS_PER_REV);
  if (SLOW) {
    maxOmega = mm_to_steps(SLOW_ANG_VEL * WHEEL_RADIUS, WHEEL_RADIUS, STEPS_PER_REV);
    maxAlpha = mm_to_steps(SLOW_ANG_ACC * WHEEL_RADIUS, WHEEL_RADIUS, STEPS_PER_REV);
  }

  //lock steppers
  steppersEngaged_mtx->lock();

  //set accel and vel
  stepperL->setAcceleration(maxAlpha);
  stepperR->setAcceleration(maxAlpha * RIGHT_OFF);
  stepperL->setMaxSpeed(maxOmega);
  stepperR->setMaxSpeed(maxOmega * RIGHT_OFF);
  stepperL->setSpeed(maxOmega);
  stepperR->setSpeed(maxOmega * RIGHT_OFF);

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

  //Change state
  state = 2;
}

void Controller::doArc(int arcType) {
  float outerR = 250 + TRACK_WIDTH / 2;
  float innerR = 250 - TRACK_WIDTH / 2;
  float ratio = innerR / outerR;
  steppersEngaged_mtx->lock();
  if (arcType == 1) {
    stepperL->setAcceleration(mm_to_steps(MAX_ACC_ARC, WHEEL_RADIUS, STEPS_PER_REV));
    stepperR->setAcceleration(mm_to_steps(MAX_ACC_ARC, WHEEL_RADIUS, STEPS_PER_REV) * ratio);
    stepperL->setMaxSpeed(mm_to_steps(MAX_VEL_ARC, WHEEL_RADIUS, STEPS_PER_REV));
    stepperR->setMaxSpeed(mm_to_steps(MAX_VEL_ARC, WHEEL_RADIUS, STEPS_PER_REV) * ratio);
    stepperL->setSpeed(mm_to_steps(MAX_VEL_ARC, WHEEL_RADIUS, STEPS_PER_REV));
    stepperR->setSpeed(mm_to_steps(MAX_VEL_ARC, WHEEL_RADIUS, STEPS_PER_REV) * ratio);
    stepperR->move(mm_to_steps(innerR * PI / 2, WHEEL_RADIUS, STEPS_PER_REV));
    stepperL->move(mm_to_steps(outerR * PI / 2, WHEEL_RADIUS, STEPS_PER_REV));
  }
  if (arcType == 2) {
    stepperR->setAcceleration(mm_to_steps(MAX_ACC_ARC, WHEEL_RADIUS, STEPS_PER_REV));
    stepperL->setAcceleration(mm_to_steps(MAX_ACC_ARC, WHEEL_RADIUS, STEPS_PER_REV) * ratio);
    stepperR->setMaxSpeed(mm_to_steps(MAX_VEL_ARC, WHEEL_RADIUS, STEPS_PER_REV));
    stepperL->setMaxSpeed(mm_to_steps(MAX_VEL_ARC, WHEEL_RADIUS, STEPS_PER_REV) * ratio);
    stepperR->setSpeed(mm_to_steps(MAX_VEL_ARC, WHEEL_RADIUS, STEPS_PER_REV));
    stepperL->setSpeed(mm_to_steps(MAX_VEL_ARC, WHEEL_RADIUS, STEPS_PER_REV) * ratio);
    stepperL->move(mm_to_steps(innerR * PI / 2, WHEEL_RADIUS, STEPS_PER_REV));
    stepperR->move(mm_to_steps(outerR * PI / 2, WHEEL_RADIUS, STEPS_PER_REV));
  }

  steppersEngaged_mtx->unlock();

  xTaskCreatePinnedToCore(engageSteppers, "engageSteppers Task", 10000, NULL, 1, engageSteppersHandle, 1);

  state = 1;
}

void Controller::setVx(float newVx) {
  vx = newVx;
}

float Controller::getGyroZ() {
  imu->getSensorData();
  return imu->data.gyroZ;  // degrees per second
}

void Controller::setPos(Vector2f p) {
  position = p;
}
