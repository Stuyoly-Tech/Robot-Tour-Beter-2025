#ifndef CONFIG____H
#define CONFIG____H

//DAC
#define VREF_VOLTAGE 0.9447
#define DAC_ADDRESS 0xD//0b00011011

#define STEPS_PER_REV 200*8 //microsteps are controlled by dipswitch

//Screen
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1 
#define SCREEN_ADDRESS 0x3C
#define SCREEN_WIRE &Wire

//BMI160 IMU
#define IMU0_ADDRESS 0x68
#define IMU1_ADDRESS 0x69
#define IMU_UPDATE_PERIOD 1.0/200

//Robot physical dimensions
//in mm
#define WHEEL_RADIUS 59.67/2
#define TRACK_WIDTH 116.95
#define DIST_TO_DOWEL 33.175

//Kinematics

//in mm/s
#define MAX_ACC 1
#define MAX_VEL 1

//in rad/sec
#define MAX_ANG_ACC 6
#define MAX_ANG_VEL 12

//Filter for angular velocity drift
#define HIGH_PASS_FREQ 0.00266
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

#endif