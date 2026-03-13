#ifndef CONFIG____H
#define CONFIG____H

//DAC
#define DAC_ADDRESS 0xD  //0b00011011

#define STEPS_PER_REV (200 * 8)  //microsteps are controlled by dipswitch

//Screen
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
#define SCREEN_WIRE &Wire

//BMI270 IMU
#define IMU_ADDRESS 0x69
#define IMU_UPDATE_PERIOD (1.0 / 200)
#define g 9807
#define ACC_MIN 1000

//Robot physical dimensions
//in mm
#define WHEEL_RADIUS 40.6
#define TRACK_WIDTH 149  //146.2 is real 151 under, 151.2 over steer
#define DIST_TO_DOWEL 90.90431
#define IMU_X 24.21250
#define IMU_Y -1.75000
#define RIGHT_OFF 1

//Kinematics

/*ACCURATE SET OF FOUR VALUES:
ACCELERATION: 1500
VELOCITY: 2000
MAX ANGULAR ACCELERATION: 15
MAX ANGULAR VELOCITY: 15 unsure need to do a full 180 turn test
*/
//in mm/s
#define MAX_ACC 2500
#define MAX_VEL 1000

#define MAX_ACC_ARC 2500
#define MAX_VEL_ARC 1000

//in rad/sec
#define MAX_ANG_ACC 20
#define MAX_ANG_VEL 20

#define BOTTLE_ANG_ACC 10
#define BOTTLE_ANG_VEL 10

#define SLOW_ANG_ACC 3
#define SLOW_ANG_VEL 3

//Filter for angular velocity drift
#define HIGH_PASS_FREQ 0.00000  //0.0001
#define COUNTER_BIAS 0.00000    // right bias 0.0001 left bias 0.0001
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
