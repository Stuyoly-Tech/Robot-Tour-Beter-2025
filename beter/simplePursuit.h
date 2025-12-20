#ifndef simplePursuit_h
#define simplePursuit_h

#include <Arduino.h>
#include <ArduinoEigenDense.h>

using namespace Eigen;

//Gets points and tells distances/required headings

class simplePursuit {
  private:
    Vector2f *path;

    int pathSize;
    int currentGoalPointIndex;

    float centerToDowel; 
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
      float iTargetTime, 
      float iFinalOffsetY, float iFinalOffsetX
    );

    //get index of path
    int getPathIndexCount();

    //True if not the end of the path, false if it is
    void nextPoint();

    //Stuff
    boolean atLastPoint();

    //Distance needed to be traveled from point a to b
    float getCurrentGoalPointDist(Vector2f p);

    //Requried heading to go from point a to b
    float getTheta(Vector2f p);

    //Average speed needed to complete track on time
    float getEndVx(float t, float d);
};

#endif