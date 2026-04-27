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
  lastAccX = 0;
  lastAccY = 0;
  lastPosL = 0;
  lastPosR = 0;
  vel_x = 0;
  vel_y = 0;
  thetaMid = 0;
  lastPoint = false;
  bottle = false;
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
  gyroConfig.cfg.acc.range = BMI2_ACC_RANGE_2G;
  gyroConfig.cfg.acc.odr = BMI2_ACC_ODR_200HZ;

  imu->setConfig(gyroConfig);


  imu->beginI2C(IMU_ADDRESS);
  imu->performComponentRetrim();
  imu->performGyroOffsetCalibration();

  delay(1000);

  float gyro_sum = 0;
  float x_sum = 0;
  float y_sum = 0;
  t_0 = micros() / pow(10, 6);
  //Begin reading
  for (int i = 0; i < 1000;) {
    float t = micros() / pow(10, 6);
    if (t - t_0 > IMU_UPDATE_PERIOD) {
      imu->getSensorData();
      float omega = (imu->data.gyroZ) * PI / 180.0;
      float xacc = (imu->data.accelX);
      float yacc = (imu->data.accelY);
      gyro_sum += omega;
      x_sum += xacc;
      y_sum += yacc;
      t_0 = t;
      i++;
    }
  }
  gyroOffset = gyro_sum / 1000;
  ACC_X_OFFSET = -x_sum / 1000;
  ACC_Y_OFFSET = -y_sum / 1000;

  imu_position(0) = position(0) + IMU_X;
  imu_position(1) = position(1) + IMU_Y;
  omega = 0;

  delay(1000);
}


void Controller::update() {
  //Serial.println(dt);

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
        Serial.println(position(0));
        Serial.println(position(1));
        state = 0;
        steppersEngaged_mtx->unlock();
        //vel_x = 0;
        //vel_y = 0;
      }
      break;

    //deciding turn
    case 2:
      if (deltaTheta > PI) {
        deltaTheta -= TWO_PI;
      } else if (deltaTheta < -PI) {
        deltaTheta += TWO_PI;
      }
      if ((abs(deltaTheta) < 0.1 && !lastPoint) || abs(deltaTheta) < 0.001) {
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
        //lastPosL = 0;
        //lastPosR = 0;
        //stepperL->setCurrentPosition(0);
        //stepperR->setCurrentPosition(0);
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
  updateTheta();
}

void Controller::updateTheta() {
  Serial.println(theta);
  dt = micros() / pow(10, 6) - t_0;
  //updatePosition(0, 0);
  if (dt > IMU_UPDATE_PERIOD) {
    t_0 = micros() / pow(10, 6);
    //Serial.println(theta, 10);
    //Serial.println(position(0));
    //Serial.println(position(1));

    //Update theta
    imu->getSensorData();

    //debugSerial->
    //Serial.printf("bihh IMU: %f\n", imu->data.gyroZ);

    omega = (imu->data.gyroZ) * PI / 180.0 - gyroOffset;
    //float omega0 = PI / 772.77777775;  //increasing: 772.75  Decreasing: 772.8
    //Serial.println(imu->data.gyroZ);

    //debugSerial->
    //Serial.printf("OMEGA: %f\n", omega);

    float dTheta = omega * (dt);
    //thetaMid = theta + dTheta / 2;
    theta += dTheta;

    //Rescale theta
    while (theta > PI) {
      theta -= TWO_PI;
    }
    while (theta < -PI) {
      theta += TWO_PI;
    }
    updatePosition();
  }
}


void Controller::updatePosition() {


  //Serial.println(position(0));
  //Serial.println(position(1));
  float dL = stepperL->currentPosition() - lastPosL;
  float dR = stepperR->currentPosition() - lastPosR;
  lastPosL = stepperL->currentPosition();
  lastPosR = stepperR->currentPosition();
  float dS = (dL + dR) / 2;
  Serial.println(dL);
  Serial.println(dR);
  Serial.println(dS);
  position(0) += steps_to_mm(dS) * cos(theta);
  position(1) += steps_to_mm(dS) * sin(theta);
  /*
  Serial.print("x: ");
  Serial.println(position(0));
  Serial.print("y: ");
  Serial.println(position(1));
  // using motor positions
  float dxL = stepperL->speed() * dt;
  float dxR = stepperR->speed() * dt;
  float thetaMid = theta;
  position(0) += steps_to_mm(dxL) * cos(thetaMid) / 2;
  position(0) += steps_to_mm(dxR) * cos(thetaMid) / 2;
  position(1) += steps_to_mm(dxL) * sin(thetaMid) / 2;
  position(1) += steps_to_mm(dxR) * sin(thetaMid) / 2;
  Serial.println(dxL);
  //lastPosL = stepperL->currentPosition();
  //lastPosR = stepperR->currentPosition();
  */

  /* using imu
  float acc_x = (imu->data.accelX - ACC_X_OFFSET) * g;
  float acc_y = (imu->data.accelY - ACC_Y_OFFSET) * g;
  if(abs(acc_x) < ACC_MIN){
    acc_x = 0;
  }
  if(abs(acc_y) < ACC_MIN){
    acc_y = 0;
  }
  
  Serial.print("ACC_X: ");
  Serial.println(acc_x);
  Serial.print("ACC_Y: ");
  Serial.println(acc_y);
  
  vel_x += (acc_x) * dt;
  vel_y += (acc_y) * dt;
  float dx = dt*(vel_x - dt*(acc_x + lastAccX) /2);
  float dy = dt*(vel_y - dt*(acc_y + lastAccY) /2);
  imu_position(0) += dx;
  imu_position(1) += dy;
  lastAccX = acc_x;
  lastAccY = acc_y;
  position(0) = imu_position(0) + (IMU_X * cos(theta));
  position(1) = imu_position(1) + (IMU_Y * sin(theta));
  */
}

void Controller::moveX(float dist) {
  if (dist != 0) {
    int maxV = mm_to_steps(vx, WHEEL_RADIUS, STEPS_PER_REV);
    int maxA = mm_to_steps(MAX_ACC, WHEEL_RADIUS, STEPS_PER_REV);
    int steps = mm_to_steps(dist, WHEEL_RADIUS, STEPS_PER_REV);

    //Lock steppers
    steppersEngaged_mtx->lock();
    //lastPosL = stepperL->currentPosition();
    //lastPosR = stepperR->currentPosition();

    //set accel and vel
    stepperL->setAcceleration(maxA);
    stepperR->setAcceleration(maxA * RIGHT_OFF);
    stepperL->setMaxSpeed(maxV);
    stepperR->setMaxSpeed(maxV * RIGHT_OFF);

    //lastPosL = 0;
    //lastPosR = 0;
    //stepperL->setCurrentPosition(0);
    //stepperR->setCurrentPosition(0);

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
  if(!bottle){
    return;
  }
  if(bottle){
    maxOmega = mm_to_steps(BOTTLE_ANG_VEL * WHEEL_RADIUS, WHEEL_RADIUS, STEPS_PER_REV);
    maxAlpha = mm_to_steps(BOTTLE_ANG_ACC * WHEEL_RADIUS, WHEEL_RADIUS, STEPS_PER_REV);
  }
  if (SLOW) {
    maxOmega = mm_to_steps(SLOW_ANG_VEL * WHEEL_RADIUS, WHEEL_RADIUS, STEPS_PER_REV);
    maxAlpha = mm_to_steps(SLOW_ANG_ACC * WHEEL_RADIUS, WHEEL_RADIUS, STEPS_PER_REV);
  }

  //lock steppers
  steppersEngaged_mtx->lock();
  //lastPosL = stepperL->currentPosition();
  //lastPosR = stepperR->currentPosition();
  //set accel and vel
  stepperL->setAcceleration(maxAlpha);
  stepperR->setAcceleration(maxAlpha * RIGHT_OFF);
  stepperL->setMaxSpeed(maxOmega);
  stepperR->setMaxSpeed(maxOmega * RIGHT_OFF);

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
    stepperR->move(mm_to_steps(innerR * PI / 2, WHEEL_RADIUS, STEPS_PER_REV));
    stepperL->move(mm_to_steps(outerR * PI / 2, WHEEL_RADIUS, STEPS_PER_REV));
    position(0) += 250 * sqrt(2) * cos(theta - (PI / 4));
    position(1) += 250 * sqrt(2) * sin(theta - (PI / 4));
    lastPosL += mm_to_steps(outerR * PI / 2, WHEEL_RADIUS, STEPS_PER_REV);
    lastPosR += mm_to_steps(innerR * PI / 2, WHEEL_RADIUS, STEPS_PER_REV);
  }
  if (arcType == 2) {
    stepperR->setAcceleration(mm_to_steps(MAX_ACC_ARC, WHEEL_RADIUS, STEPS_PER_REV));
    stepperL->setAcceleration(mm_to_steps(MAX_ACC_ARC, WHEEL_RADIUS, STEPS_PER_REV) * ratio);
    stepperR->setMaxSpeed(mm_to_steps(MAX_VEL_ARC, WHEEL_RADIUS, STEPS_PER_REV));
    stepperL->setMaxSpeed(mm_to_steps(MAX_VEL_ARC, WHEEL_RADIUS, STEPS_PER_REV) * ratio);
    stepperL->move(mm_to_steps(innerR * PI / 2, WHEEL_RADIUS, STEPS_PER_REV));
    stepperR->move(mm_to_steps(outerR * PI / 2, WHEEL_RADIUS, STEPS_PER_REV));
    position(0) += 250 * sqrt(2) * cos(theta + (PI / 4));
    position(1) += 250 * sqrt(2) * sin(theta + (PI / 4));
    lastPosL += mm_to_steps(innerR * PI / 2, WHEEL_RADIUS, STEPS_PER_REV);
    lastPosR += mm_to_steps(outerR * PI / 2, WHEEL_RADIUS, STEPS_PER_REV);
  }

  steppersEngaged_mtx->unlock();

  xTaskCreatePinnedToCore(engageSteppers, "engageSteppers Task", 10000, NULL, 1, engageSteppersHandle, 1);


  state = 1;
}

void Controller::toggleBottle(){
  bottle = !bottle;
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