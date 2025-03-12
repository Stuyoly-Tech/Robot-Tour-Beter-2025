#ifndef menu_h
#define menu_h

#include "CONFIG.h"
#include <AccelStepper.h>
#include "controller.h"
#include "simplePursuit.h"
#include "robot.h"

//OFFSET VARS
double TIME_OFFSET;
double FINAL_OFFSET_Y;
double FINAL_OFFSET_X;

void displayScreen(int state);//display screen
void init();//init robot
void run();//run robot
int getOffsetType();//read position of rotary encoder
void saveOffset(int offsetType);//
void resetOffset(int offsetType);//

#endif