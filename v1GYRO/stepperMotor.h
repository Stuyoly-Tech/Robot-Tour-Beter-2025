//NOT USED

#include <Arduino.h>

#ifndef stepperMotor_h
#define stepperMotor_h

class stepperMotor {
  private:
    int DIR_PIN;
    int STEP_PIN;
    
    bool directionBias;
    
    double stepsPerSecond;
    
    uint32_t delayus;
    long totalSteps;
    
    bool enabled;

    //for timing
    uint32_t oldus;
    
    
  public:
    stepperMotor(int iDIR_PIN, int iSTEP_PIN);

    void init();
    void enable();
    void disable();
    bool update();

    void setDirectionBias(bool direction);
    void setStepsPerSecond(double newStepsPerSecond);

    bool getDirectionBias();
    double getStepsPerSecond();
    long getTotalSteps();
      
};

#endif
