#ifndef menu_h
#define menu_h

#include "Arduino.h"
#include "beter.ino"
#include "CONFIG.h"
#include <AccelStepper.h>
#include "controller.h"
#include "simplePursuit.h"
#include "robot.h"


//Button related
uint8_t BTN_PINS[];
bool BTN_PREV_STATES[];
bool BTN_STATE(uint8_t index);

//OFFSET VARS
double TARGET_TIME;
double FINAL_OFFSET_Y;
double FINAL_OFFSET_X;

void displayScreen(int state);//display screen
void init();//init robot
void run();//run robot
int getOffsetType();//read position of rotary encoder
void saveOffset(int offsetType);//
void resetOffset(int offsetType);//

#endif