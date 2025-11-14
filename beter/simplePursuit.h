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
    Vector2f *bottles;

    int pathSize;
    int gateSize;
    int bottleSize;
    int prevPointIndex;
    int currentGoalPointIndex;

    float centerToDowel; 
    float pathTotalDist; 
    float targetTime;

    float finalOffsetX;
    float finalOffsetY;

    float endVx;

    //Time alloted for each turn, used to calculate avgVx

    float limitVx;
    float limitOmega;
    
    float getDist(Vector2f p1, Vector2f p2);
    
  public:
    //simplePursuit();
    
    void init(
      Vector2f *iPath, int iPathSize, 
      Vector2f *iGates, int iGateSize, 
      Vector2f *iBottles, int iBottleSize, 
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
    float getEndVx(float t, float d);

    //Bottle Stuff
    boolean isABottle();
};

#endif