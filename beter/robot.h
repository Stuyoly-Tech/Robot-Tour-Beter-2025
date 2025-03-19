#ifndef robot_h
#define robot_h

#include "simplePursuit.h"
#include "controller.h"

class robot {
  private:
    simplePursuit *robotSimplePursuit;
    controller *robotController; //robot controller
    
    //0 for idle
    //1 for deciding forward
    //2 for moving
    //3 for deciding turn
    //4 for turning
    uint8_t STATE;
    uint32_t start_us;

    //getting distance from end point
    float centerToDowel;

    float maxAx;
    float maxAngAx;
    float maxAngVx;

    int pathMode;

    float sgn(float n);
    
  public:
    robot( 
      simplePursuit *iSimplePursuit, controller *iController,
      float iMaxAx, float iMaxAngAx, float iMaxAngVx,
      float iCenterToDowel
      );
      
    void init();
    void init(int iPathMode);
    void update();
    void startPath();
    float stopPath();

    uint8_t getState();

};


#endif
