#ifndef CONFIG____H
#define CONFIG____H

//DAC
#define VREF_VOLTAGE 0.9447
#define DAC_ADDRESS 0xD//0b00011011

#define STEPS_PER_REV 200*32 //microsteps are controlled by dipswitch

//Screen
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1 
#define SCREEN_ADDRESS 0x3C
#define SCREEN_WIRE &Wire

//BMI270 IMU
#define IMU_ADDRESS 0x68
#define IMU_UPDATE_PERIOD 1.0/200

//Robot physical dimensions
//in mm
#define WHEEL_RADIUS 40.6
#define TRACK_WIDTH 146.2
#define DIST_TO_DOWEL 85
#define RIGHT_OFF 1

//Kinematics

//in mm/s
#define MAX_ACC 500
#define MAX_VEL 2000

//in rad/sec
#define MAX_ANG_ACC 10 //5
#define MAX_ANG_VEL 20 //10

//Filter for angular velocity drift
#define HIGH_PASS_FREQ 0.0001//0.0001
#define COUNTER_BIAS 0.000008 // right bias 0.0001 left bias 0.0001
//BEST VALUE: 0.0003
//0 best value
//2.5*pow(10, -6)

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
#define TESTING_TURNS 12

#endif
