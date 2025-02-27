#include <Arduino.h>
#include <ArduinoEigenDense.h>
using namespace Eigen;

#include "simplePursuit.h"

simplePursuit::simplePursuit(double iLimitVx, double iCenterToDowel) {
  limitVx = iLimitVx;
  centerToDowel = iCenterToDowel;
}

double simplePursuit::getDist(Vector2d p1, Vector2d p2) {
  return sqrt(pow((p2(0)-p1(0)), 2) + pow((p2(1)-p1(1)), 2));
}

//path will never be larger than 100
void simplePursuit::init(Vector2d *iPath, uint8_t iPathSize, Vector2d *iGates, uint8_t iGateSize, double iTargetTime, double iFinalOffsetY, double iFinalOffsetX) {
  path = iPath;
  pathSize = iPathSize;
  gates = iGates;
  gateSize = iGateSize;
  targetTime = iTargetTime;
  finalOffsetX = iFinalOffsetX;
  finalOffsetY = iFinalOffsetY;
  
  prevPointIndex = 0;
  currentGoalPointIndex = 1;
  
  path[pathSize] = Vector2d(path[pathSize-1](0)+finalOffsetX, path[pathSize-1](1)+finalOffsetY);

  //Calculate pathTotalDist and avgVx
  avgVx = 0;
  pathTotalDist -= centerToDowel;
  for (uint8_t i=1; i<pathSize+1; i++) {
    pathTotalDist += getDist(path[i], path[i-1]);
  }
}

uint8_t simplePursuit::getPathIndexCount() {
  return currentGoalPointIndex;
}

void simplePursuit::nextPoint() {
  pathTotalDist -= getDist(path[prevPointIndex], path[currentGoalPointIndex]);
  if (currentGoalPointIndex < pathSize) {
    prevPointIndex++;
    currentGoalPointIndex++;
  }
  //decrease path distance
}

boolean simplePursuit::atLastPoint() {
  return (currentGoalPointIndex == pathSize);
}

boolean simplePursuit::isAGate() {
  for (uint8_t i=0; i<gateSize; i++) {
    if (path[currentGoalPointIndex] == gates[i]) {
      return true;
    }
  }
  return false;
}

double simplePursuit::getCurrentGoalPointDist() {
  return getDist(path[currentGoalPointIndex], path[prevPointIndex]);
}

double simplePursuit::getTheta() {
  return atan2(
    path[currentGoalPointIndex](1)-path[prevPointIndex](1), 
    path[currentGoalPointIndex](0)-path[prevPointIndex](0)
    );
}

double simplePursuit::getAvgVx(uint32_t elapsedTime) {
  double remTime = targetTime - (elapsedTime/pow(10, 6));
  avgVx = pathTotalDist/remTime;
  if ((avgVx > limitVx) || (remTime < 0)) {
    avgVx = limitVx;
  }
  return avgVx;
}
