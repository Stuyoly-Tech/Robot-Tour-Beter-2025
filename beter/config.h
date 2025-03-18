#ifdef CONFIG__H
#define CONFIG__H

#define VREF_VOLTAGE 0.9447
#define DAC_ADDRESS 0x0D
#define STEPS_PER_REV 200*8//microsteps are controlled by dipswitch

//Screen
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1 
#define SCREEN_ADDRESS 0x3C
#define SCREEN_WIRE &Wire

//BMI160 IMU
#define IMU_ADDRESS 0x68
#define IMU2_ADDRESS 0x69
#define IMU_UPDATE_US 0.10*pow(10, 3)

//Robot physical dimensions
//in mm
#define WHEEL_RADIUS 40
#define TRACK_WIDTH 116
#define DIST_TO_DOWEL 33.175

//Controller tuning values

#define MAX_ACCEL 700
#define MAX_VX 8000

#define MAX_ANG_ACCEL 300
#define MAX_ANG_VEL 325

#define HIGH_PASS_FREQ 13*pow(10, -3)//11.2051109*pow(10, -3) //Best value found so far
//2.5*pow(10, -6)
//Not used
#define TURN_US 2500*pow(10, 3)

//PATH INFO
#define PATH_FILE "/path.txt"

//STATE DEFINITIONS
#define INIT -1
#define IDLE 0
#define READY 1
#define RUNNING 2
#define END_RUN 3
#define STOPPED 4
#define SD_ERROR 5
#define FILE_ERROR 6
#define IMU_ERROR 7
#define ADJUST_MENU 8
#define ADJUST_X 9
#define ADJUST_Y 10
#define ADJUST_TIME 11

#endif