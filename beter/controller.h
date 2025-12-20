#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <mutex>
#include <Arduino.h>
#include <AccelStepper.h>
#include <SparkFun_BMI270_Arduino_Library.h>
#include <HardwareSerial.h>
#include <ArduinoEigenDense.h>
using namespace Eigen;

#include "utils.h"
#include "config.h"

class Controller {
  private:
    HWCDC* debugSerial;

    //IMU
    BMI270 *imu;

    //Motors
    AccelStepper *stepperL;
    AccelStepper *stepperR;

    std::mutex *steppersEngaged_mtx;
    void (*engageSteppers)(void * parameter);
    TaskHandle_t *engageSteppersHandle;

    float vx;
    float gyroOffset;
    

    //Control variables (mostly for P controller on theta)
    float t_0;
    float dt;
    float lastPosL;
    float lastPosR;
  public:
    Vector2f position;
    int state;
    float theta;
    float thetaSetPoint;
    

    Controller(
      AccelStepper* pStepperL, AccelStepper* pStepperR,
      std::mutex *iSteppersEngaged_mtx, void (*iEngageSteppers)(void * parameter),
      TaskHandle_t *iEngageSteppersHandle, 
      BMI270* pImu,
      HWCDC* pDebugSerial
    );

    void init(float iTheta);
    void init();
    void gyroInit();
    float getGyroZ();

    void update();
    void updateTheta();
    void updatePosition();

    void moveX(float dist);
    void turnTheta(float targetTheta);

    void setVx(float newVx);

    void setPos(Vector2f p);
};

#endif
