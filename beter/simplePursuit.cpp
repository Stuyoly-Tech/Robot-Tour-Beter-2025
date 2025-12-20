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
  float iTargetTime,
  float iFinalOffsetY, float iFinalOffsetX) {
  path = iPath;
  pathSize = iPathSize;
  targetTime = iTargetTime;
  finalOffsetX = iFinalOffsetX;
  finalOffsetY = iFinalOffsetY;
  currentGoalPointIndex = 1;

  path[pathSize - 1] = Vector2f(path[pathSize - 1](0) + finalOffsetX, path[pathSize - 1](1) + finalOffsetY);

  //Calculate pathTotalDist and avgVx
  endVx = 0;
}

int simplePursuit::getPathIndexCount() {
  return currentGoalPointIndex;
}

void simplePursuit::nextPoint() {
  if (currentGoalPointIndex < pathSize) {
    currentGoalPointIndex++;
  }
}

boolean simplePursuit::atLastPoint() {
  return (currentGoalPointIndex == pathSize - 1);
}

float simplePursuit::getCurrentGoalPointDist(Vector2f p) {
  return getDist(path[currentGoalPointIndex], p);
}

float simplePursuit::getTheta(Vector2f p) {
  return atan2(
    path[currentGoalPointIndex](1) - p(1),
    path[currentGoalPointIndex](0) - p(0)
  );
}

float simplePursuit::getEndVx(float t, float d) {
  float remTime = targetTime - t;
  float a = MAX_ACC;
  Serial.println(d);
  Serial.println(t);
  if ((a * remTime - a * sqrt(sq(remTime) - 4 * d / a)) / 2 < MAX_VEL) {
    Serial.println((a * remTime - a * sqrt(sq(remTime) - 4 * d / a)) / 2);
    return (a * remTime - a * sqrt(sq(remTime) - 4 * d / a)) / 2;
  } else {
    return MAX_VEL;
  }
}