#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <mutex>
#include <Arduino.h>
#include <AccelStepper.h>
#include <BMI160Gen.h>
#include <HardwareSerial.h>

#include "utils.h"
#include "config.h"

class Controller {
  private:
    HWCDC* debugSerial;

    //IMU
    uint32_t iIntervalIMUs;

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
  
  public:

    int state;
    float theta;
    float thetaSetPoint;
    

    Controller(
      AccelStepper* pStepperL, AccelStepper* pStepperR,
      std::mutex *iSteppersEngaged_mtx, void (*iEngageSteppers)(void * parameter),
      TaskHandle_t *iEngageSteppersHandle,
      uint32_t IntervalIMUs,
      HWCDC* pDebugSerial
    );

    void init(float iTheta);
    void init();
    void gyroInit();

    void update();
    void updateTheta();

    void moveX(float dist);
    void turnTheta(float targetTheta);

    void setVx(float newVx);
};

#endif