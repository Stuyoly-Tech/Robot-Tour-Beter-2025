//Hardware connections

//Buttons for input
#define BTN_0 1 //INIT | x
#define BTN_1 2 //RUN | TIME ADJUST
#define BTN_2 3 //ADJUSTMENTS | X ADJUST
#define BTN_3 4 //LASERS | Y ADJUST

//Rotary Encoder
#define INCR_BTN 5
#define INCR_A 15
#define INCR_B 16


//Motors
#define STEP_L 10
#define DIR_L 11

#define STEP_R 12
#define DIR_R 13

#define STEP_ENABLE 17

#define STEPS_PER_REV 200*8//microsteps are controlled by dipswitch

//Screen
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1 
#define SCREEN_ADDRESS 0x3C
#define SCREEN_WIRE &Wire

//LASERS
#define LASERS 45

//LEDS
#define LED_0 6
#define LED_1 7

//SD Card
#define SD_CS 41
#define SD_CLK 38
#define SD_MOSI 40
#define SD_MISO 39

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