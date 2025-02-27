//NOT USED

#include <Arduino.h>
#include "stepperMotor.h"

stepperMotor::stepperMotor(int iDIR_PIN, int iSTEP_PIN) {
  STEP_PIN = iSTEP_PIN;
  DIR_PIN = iDIR_PIN;
}

void stepperMotor::init() {
  directionBias = false;
  stepsPerSecond = 0;
  delayus = 0;
  totalSteps = 0;
  enabled = false;
  oldus = micros();
  
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);
}

void stepperMotor::enable() {
  enabled = true;
}

void stepperMotor::disable() {
  enabled = false;
}

bool stepperMotor::update() {
  if (enabled && (micros() - oldus > delayus)) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(2);
    digitalWrite(STEP_PIN, LOW);
    if (stepsPerSecond != 0) {
      totalSteps += (stepsPerSecond < 0) ? -1 : 1;
    }
    oldus = micros();
    return true;
  }
  else {
    digitalWrite(STEP_PIN, LOW);
    return false;
  }
}

void stepperMotor::setDirectionBias(bool direction) {
  directionBias = direction;
}

void stepperMotor::setStepsPerSecond(double newStepsPerSecond) {
  stepsPerSecond = newStepsPerSecond;
  digitalWrite(DIR_PIN, LOW^directionBias);
  if (newStepsPerSecond < 0) {
    digitalWrite(DIR_PIN, HIGH^directionBias);
  }
  delayus = (uint32_t)(1000000 / abs(stepsPerSecond)) - 2;
}

bool stepperMotor::getDirectionBias() {
  return directionBias;
}

double stepperMotor::getStepsPerSecond() {
  return stepsPerSecond;
}

long stepperMotor::getTotalSteps() {
  return totalSteps;
}
