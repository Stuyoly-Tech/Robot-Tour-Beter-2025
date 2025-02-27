#ifndef controller_h
#define controller_h

#include <mutex>

#include <Arduino.h>
#include <AccelStepper.h>
#include <BMI160Gen.h>

class controller {
  private:
    //int mm
    double wheelRadius;
    double trackWidth;

    //All in mm/s
    double maxVx;
    double maxAx;

    //Rad/s
    double maxAngVx;
    double maxAngAx;
    
    uint32_t oldus;
    uint32_t oldIMUus;
    uint32_t intervalIMUus;
    
    volatile double theta;
    double targetTheta;
    
    AccelStepper *stepperL;
    AccelStepper *stepperR;
    uint32_t stepsPerRev;

    uint32_t turnInterval;

    double highPassFreq;
    
    //0 is idle
    //1 is moving
    int STATE;

    std::mutex *steppersEngaged_mtx;
    void (*engageSteppers)(void * parameter);
    TaskHandle_t *engageSteppersHandle;

  public:
    controller(
      double iWheelRadius, double iTrackWidth,
      AccelStepper *iStepperL, AccelStepper *iStepperR,
      uint32_t iStepsPerRev, uint32_t iTurnInterval,
      uint32_t iIntervalIMUussd, std::mutex *iSteppersEngaged_mtx,
      void (*iEngageSteppers)(void * parameter),
      TaskHandle_t *iEngageSteppersHandle,
      double iHighPassFreq
    );

    void init(double iTheta);
    void init();
    void update();

    void setMaxVx(double newVx);
    void setMaxAx(double newAx);
    void setMaxAngAx(double newAngAx);
    void setMaxAngVx(double newAngVx);

    //Motion profiled
    void moveX(double dist);
    void setTheta(double newTheta);
    void updateTheta();
    void initTheta(double newTheta);

    double getMaxVx();
    double getMaxAx();
    double getMaxAngAx();
    double getMaxAngVx();

    double getTheta();
    double getTargetTheta();

    int getState();

    long mmToSteps(double mm);
    double stepsTomm(long steps);
};

#endif
