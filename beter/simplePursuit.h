#ifndef simplePursuit_h
#define simplePursuit_h

#include <Arduino.h>
#include <ArduinoEigenDense.h>

using namespace Eigen;

//Gets points and tells distances/required headings

class simplePursuit {
  private:
    Vector2f *path;
    Vector2f *gates;

    int pathSize;
    int gateSize;
    int prevPointIndex;
    int currentGoalPointIndex;

    float centerToDowel; 
    float pathTotalDist; 
    float targetTime;

    float finalOffsetX;
    float finalOffsetY;

    float avgVx;

    //Time alloted for each turn, used to calculate avgVx

    //in us
    uint32_t turnInterval;

    float limitVx;
    float limitOmega;
    
    float getDist(Vector2f p1, Vector2f p2);
    
  public:
    simplePursuit(float iLimitVx, float iLimitOmega, float iCenterToDowel);
    
    void init(
      Vector2f *iPath, int iPathSize, 
      Vector2f *iGates, int iGateSize, 
      float iTargetTime, 
      float iFinalOffsetY, float iFinalOffsetX
    );

    //get index of path
    int getPathIndexCount();

    //True if not the end of the path, false if it is
    void nextPoint();

    //Stuff
    boolean atLastPoint();
    boolean isAGate();

    //Distance needed to be traveled from point a to b
    float getCurrentGoalPointDist();

    //Requried heading to go from point a to b
    float getTheta();

    //Average speed needed to complete track on time
    float getAvgVx(uint32_t elapsedTime);
};

#endif
