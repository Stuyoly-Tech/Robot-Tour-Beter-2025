#include <Arduino.h>
#include <ArduinoEigenDense.h>
using namespace Eigen;

#include "simplePursuit.h"
#include "config.h"
#include "utils.h"

float simplePursuit::getDist(Vector2f p1, Vector2f p2) {
  return sqrt(pow((p2(0) - p1(0)), 2) + pow((p2(1) - p1(1)), 2));
}

//path will never be larger than 100
void simplePursuit::init(
  Vector2f *iPath, int iPathSize,
  Vector2f *iGates, int iGateSize,
  float iTargetTime,
  float iFinalOffsetY, float iFinalOffsetX) {
  path = iPath;
  pathSize = iPathSize;
  gates = iGates;
  gateSize = iGateSize;
  targetTime = iTargetTime;
  finalOffsetX = iFinalOffsetX;
  finalOffsetY = iFinalOffsetY;

  prevPointIndex = 0;
  currentGoalPointIndex = 1;

  path[pathSize - 1] = Vector2f(path[pathSize - 1](0) + finalOffsetX, path[pathSize - 1](1) + finalOffsetY);

  //Calculate pathTotalDist and avgVx
  endVx = 0;
  pathTotalDist -= DIST_TO_DOWEL;

  for (int i = 1; i < pathSize; i++) {
    pathTotalDist += getDist(path[i], path[i - 1]);
  }
}

int simplePursuit::getPathIndexCount() {
  return currentGoalPointIndex;
}

void simplePursuit::nextPoint() {
  pathTotalDist -= getDist(path[prevPointIndex], path[currentGoalPointIndex]);
  if (currentGoalPointIndex < pathSize) {
    prevPointIndex++;
    currentGoalPointIndex++;
  }
}

boolean simplePursuit::atLastPoint() {
  return (currentGoalPointIndex == pathSize - 1);
}

boolean simplePursuit::isAGate() {
  for (int i = 0; i < gateSize; i++) {
    if (path[currentGoalPointIndex] == gates[i]) {
      return true;
    }
  }
  return false;
}

float simplePursuit::getCurrentGoalPointDist() {
  return getDist(path[currentGoalPointIndex], path[prevPointIndex]);
}

float simplePursuit::getTheta() {
  return atan2(
    path[currentGoalPointIndex](1) - path[prevPointIndex](1),
    path[currentGoalPointIndex](0) - path[prevPointIndex](0));
}

float simplePursuit::getEndVx(float t, float d) {
  float remTime = targetTime - t;
  float a = steps_to_mm(MAX_ACC, WHEEL_RADIUS, STEPS_PER_REV);
  //Serial.println(a);
  //Serial.println(d);
  //Serial.println(remTime);
  if (a * a * remTime * remTime - 4 * a * d > 0) {
    endVx = (a * remTime - sqrt(a * a * remTime * remTime - 4 * a * d)) / 2;
  }
  else{
    endVx = dist / remTime;
  }
  //Serial.println(endVx);
  if ((endVx > MAX_VEL) || (remTime < 0)) {
    endVx = MAX_VEL;
    //Serial.println("SET MAX VX");
  }
  //Serial.println("SET END VX");
  return endVx;
}
