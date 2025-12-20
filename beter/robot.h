#ifndef robot_h
#define robot_h

#include <HardwareSerial.h>

#include "simplePursuit.h"
#include "controller.h"

#include "config.h"

class Robot {
  private:
    simplePursuit *robotSimplePursuit;
    Controller *robotController; //robot controller

    HWCDC* debugSerial;
    
    //0 for idle
    //1 for deciding forward
    //2 for moving
    //3 for deciding turn
    //4 for turning
    int state;

    float t_0;

    //getting distance from end point
    float centerToDowel;
    int pathMode;

    float sgn(float n);
    
  public:
    Robot(simplePursuit *iSimplePursuit, Controller *iController, HWCDC* pDebugSerial);
      
    void init();
    void init(int iPathMode);
    void update();
    void setPos(Vector2f r);
    void updatePos();
    void startPath();
    float stopPath();
    int getState();

};


#endif