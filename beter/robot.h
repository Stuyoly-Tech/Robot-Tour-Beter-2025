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
    int state;

    float t_0;

    //getting distance from end point
    float centerToDowel;
    int pathMode;

    float sgn(float n);
    
  public:
    robot(simplePursuit *iSimplePursuit, controller *iController);
      
    void init();
    void init(int iPathMode);
    void update();
    void startPath();
    float stopPath();

     getState();

};


#endif
