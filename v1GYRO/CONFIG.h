//Hardware connections

//Buttons for input
#define BTN_0 34
#define BTN_1 35

//Motors
#define STEP_L 26
#define DIR_L 25
#define FAULT_L 32

#define STEP_R 13
#define DIR_R 27
#define FAULT_R 33

#define STEP_ENABLE 16

#define STEPS_PER_REV 6400

//Oled
#define OLED_ADD 0x3C
#define OLED_RST -1

//LEDS
#define LED_0 16
#define LED_1 17

//SD Card
#define SD_CS 5

//oled
#define I2C_ADDRESS 0x3C

//BMI160 IMU
#define IMU_GND 4
#define IMU_ADDRESS 0x68
#define IMU_UPDATE_US 0.10*pow(10, 3)

//Robot physical dimensions
//in mm
#define WHEEL_RADIUS 40
#define TRACK_WIDTH 146.2
#define DIST_TO_DOWEL 85

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
#define ERROR 5
#define SD_ERROR 6
#define FILE_ERROR 7
#define IMU_ERROR 8
