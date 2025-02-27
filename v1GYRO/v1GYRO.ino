#include <mutex>
#include <esp_task_wdt.h>

#include "CONFIG.h"
#include <AccelStepper.h>
#include "controller.h"
#include "simplePursuit.h"
#include "robot.h"

#include <ArduinoEigenDense.h>
using namespace Eigen;

#include "SD.h"
#include "FS.h"
#include "SPI.h"

#include <BMI160Gen.h>

#include <Wire.h>
#include "SSD1306Ascii.h"
#include "SSD1306AsciiWire.h"

//FSM State
int STATE;

//Path globals
Vector2d PATH[100]; //= {Vector2d(0, 0), Vector2d(0, 300), Vector2d(300, 300), Vector2d(300, 0), Vector2d(0, 0)};
uint8_t PATH_SIZE;

//Gates
Vector2d GATES[7];
uint8_t GATE_SIZE;

//Paramters
double TARGET_TIME;
double FINAL_OFFSET_Y;
double FINAL_OFFSET_X;
int PATH_MODE; 

//SD Methods
boolean loadPathFromSD(fs::FS &fs);

//Button related
uint8_t BTN_PINS[] = {BTN_0, BTN_1};
bool BTN_PREV_STATES[] = {LOW, LOW};
bool BTN_STATE(uint8_t index);

SSD1306AsciiWire oled;

AccelStepper stepperL(AccelStepper::DRIVER, STEP_L, DIR_L);
AccelStepper stepperR(AccelStepper::DRIVER, STEP_R, DIR_R);

//Mutexes for stepper instances
std::mutex steppersEngaged_mtx;

//Multicore tasks for engaging steppers
void engageSteppers(void * parameter);
TaskHandle_t engageSteppersHandle = NULL;

controller robotController
(
  WHEEL_RADIUS, TRACK_WIDTH,
  &stepperL, &stepperR,
  STEPS_PER_REV, TURN_US,
  IMU_UPDATE_US, &steppersEngaged_mtx,
  &engageSteppers, &engageSteppersHandle,
  HIGH_PASS_FREQ
);

simplePursuit robotSimplePursuit(MAX_VX, DIST_TO_DOWEL);

robot Robot
(
  &robotSimplePursuit, &robotController,
  MAX_ACCEL, MAX_ANG_ACCEL, MAX_ANG_VEL, 
  DIST_TO_DOWEL
);

void setup() {
  //for steppers
  //xTaskCreate(engageSteppers, "engageSteppers Task", 10000, NULL, 1, &engageSteppersHandle);

  //start init
  STATE = INIT;

  Serial.begin(115200);

  Wire.begin();
  Wire.setClock(400000L);

  //init pins
  pinMode(BTN_0, INPUT);
  pinMode(BTN_1, INPUT);

  pinMode(FAULT_L, INPUT);
  pinMode(FAULT_R, INPUT);

  pinMode(STEP_ENABLE, OUTPUT);
  digitalWrite(STEP_ENABLE, HIGH);

  pinMode(LED_0, OUTPUT);
  pinMode(LED_1, OUTPUT);

  pinMode(IMU_GND, OUTPUT);
  digitalWrite(IMU_GND, LOW);

  //oled init
  oled.begin(&Adafruit128x32, I2C_ADDRESS, OLED_RST);
  oled.set2X();
  oled.setFont(Adafruit5x7);
  oled.clear();
  oled.println("INIT...");

  //gyro init
  if (!BMI160.begin(BMI160GenClass::I2C_MODE, Wire, IMU_ADDRESS)) {
    STATE = IMU_ERROR;
  }
  //BMI160.setGyroRate(11);
  delay(500);
  oled.clear();
  oled.println("INIT GYRO");
  oled.set1X();
  oled.print("DO NOT MOVE");
  delay(500);
  BMI160.setFullScaleGyroRange(1); //1000 deg/s
  BMI160.autoCalibrateGyroOffset();
  delay(500);

  //SD begin
  if (SD.begin(SD_CS) == 0) {
    STATE = SD_ERROR;
    oled.clear();
    oled.set2X();
    oled.println("SD ERROR");
    oled.set1X();
    oled.print("Check connections");

  }

  //load Paths
  if (!loadPathFromSD(SD)) {
    STATE = FILE_ERROR;
    oled.clear();
    oled.set2X();
    oled.println("FILE ERROR");
    oled.set1X();
    oled.println("NO CARD or");
    oled.println("BAD FILE");
  }

  if (STATE == INIT) {
    STATE = IDLE;
    oled.clear();
    oled.set2X();
    oled.println("IDLE");
  }

  if (PATH_MODE == 2) {
    delay(2000);
    robotController.init();
    Robot.init(1); 
    oled.clear();
    oled.set1X();
    oled.println("TESTING TURN");
    digitalWrite(STEP_ENABLE, LOW);
    for (int i=0; i<50; i++) {
      robotController.setTheta(PI/2);
      while (robotController.getState() != 0) {
        robotController.update();
      }
      delay(500);
      robotController.setTheta(0);
      while (robotController.getState() != 0) {
        robotController.update();
      }
      delay(500);
      robotController.setTheta(PI);
      while (robotController.getState() != 0) {
        robotController.update();
      }
      delay(500);
      robotController.setTheta(0);
      while (robotController.getState() != 0) {
        robotController.update();
      }
      delay(500);
    }
  }
  
  if (PATH_MODE == 3) {
    delay(2000);
    robotController.init();
    robotController.setTheta(PI/2);
    Robot.init(1); 
    oled.clear();
    oled.set1X();
    oled.println("TESTING FORW");
    digitalWrite(STEP_ENABLE, LOW);
    robotController.setMaxVx(MAX_VX);
    for (int i=0; i<50; i++) {
      robotController.moveX(300);
      while (robotController.getState() != 0) {
        robotController.update();
      }
      delay(500);
      robotController.moveX(-300);
      while (robotController.getState() != 0) {
        robotController.update();
      }
      delay(500);
    }
  }
  
}

void loop() { 
  switch (STATE) {
    case INIT:
      break;
    case IDLE:
      if (BTN_STATE(1)) {
        Robot.init(PATH_MODE);
        robotSimplePursuit.init(PATH, PATH_SIZE, GATES, GATE_SIZE, TARGET_TIME, FINAL_OFFSET_Y, FINAL_OFFSET_X);
        STATE = READY;
        digitalWrite(STEP_ENABLE, LOW);
        oled.clear();
        oled.set1X();
        oled.println("READY");
        oled.print("target_t: "); oled.println(TARGET_TIME);
        oled.print("x_off: "); oled.println(FINAL_OFFSET_X);
        oled.print("y_off: "); oled.print(FINAL_OFFSET_Y);
      }
      break;

    case READY:
      if (BTN_STATE(1)) {
        Robot.init(PATH_MODE);
        STATE = READY;
        digitalWrite(STEP_ENABLE, LOW);
        oled.clear();
        oled.println("READY");
        oled.print("target_t: "); oled.println(TARGET_TIME);
        oled.print("x_off: "); oled.println(FINAL_OFFSET_X);
        oled.print("y_off: "); oled.print(FINAL_OFFSET_Y);
      }
      if (BTN_STATE(0)) {
        Robot.startPath();
        STATE = RUNNING;
        digitalWrite(STEP_ENABLE, LOW);
        oled.clear();
        oled.set2X();
        oled.println("RUNNING");
        robotController.initTheta(PI/2);
      }
      break;

    case RUNNING:
      Robot.update();
      if (BTN_STATE(1)) {
        STATE = STOPPED;
        digitalWrite(STEP_ENABLE, HIGH);
        oled.clear();
        oled.set2X();
        oled.println("STOPPED");
        oled.set1X();
        oled.print("current_t: "); oled.print(Robot.stopPath());
      }
      if (Robot.getState() == 0) {
        STATE = END_RUN;
        oled.clear();
        oled.set2X();
        oled.println("RUN ENDED");
        oled.set1X();
        oled.print("elapsed_t: "); oled.print(Robot.stopPath());
      }
      break;

    case END_RUN:
      if (BTN_STATE(0)) {
        STATE = IDLE;
        digitalWrite(STEP_ENABLE, HIGH);
        oled.clear();
        oled.set2X();
        oled.println("IDLE");
      }
      break;

    case STOPPED:
      if (BTN_STATE(0)) {
        STATE = IDLE;
        oled.clear();
        oled.set2X();
        oled.println("IDLE");
      }
      break;

    case ERROR:
      break;
    case SD_ERROR:
      break;
    case FILE_ERROR:
      break;
    case IMU_ERROR:
      break;
    default:
      STATE = IDLE;
    }
}

void engageSteppers(void * parameter) {
  //esp_task_wdt_init(300, false);
  steppersEngaged_mtx.lock();
  while (stepperL.run() && stepperR.run());
  stepperL.setCurrentPosition(stepperL.targetPosition());
  stepperR.setCurrentPosition(stepperR.targetPosition()); 
  steppersEngaged_mtx.unlock();
  vTaskDelete(NULL);
}

boolean loadPathFromSD(fs::FS &fs) {
  /*
     FILE FORMAT

     TARGET TIME:
     50.00
     PATH:
     A1
     B2
     ...
  */
  File file = fs.open(PATH_FILE);
  if (!file) {
    Serial.println("no_file!");
    return false;
  }
  PATH_SIZE = 0;
  char buff[10];

  //read in the mode for path following
  while (file.available()) {
    if (file.read() == '\n') {
      break;
    }
  }
  for (int i = 0; i < 1; i++) {
    buff[i] = file.read();
  }
  
  PATH_MODE = atoi(buff);
  file.read();

  //read in the final offset y

  //skip first line until newline is reached
  while (file.available()) {
    if (file.read() == '\n') {
      break;
    }
  }
  for (int i = 0; i < 6; i++) {
    buff[i] = file.read();
  }
  FINAL_OFFSET_Y = atof(buff);
  file.read();

  //read in the final offset x

  //skip first line until newline is reached
  while (file.available()) {
    if (file.read() == '\n') {
      break;
    }
  }
  for (int i = 0; i < 6; i++) {
    buff[i] = file.read();
  }
  FINAL_OFFSET_X = atof(buff);
  file.read();

  //read in target time

  //skip first line until newline is reached
  while (file.available()) {
    if (file.read() == '\n') {
      break;
    }
  }
  for (int i = 0; i < 5; i++) {
    buff[i] = file.read();
  }
  TARGET_TIME = atof(buff);
  file.read();

  //skip line
  while (file.available()) {
    if (file.read() == '\n') {
      break;
    }
  }

  //read in gate number
  buff[0] = '0';
  buff[1] = file.read();
  GATE_SIZE = atoi(buff);
  file.read();
  
  //Skip line
  while (file.available()) {
    if (file.read() == '\n') {
      break;
    }
  }

  //Read in Gates
  for (byte i=0; i < GATE_SIZE; i++) {
    buff[0] = file.read();
    buff[1] = file.read();
    //coords
    double pX, pY;
    switch (buff[0]) {
        case 'A':
          pX = 250;
          break;
        case 'B':
          pX = 750;
          break;
        case 'C':
          pX = 1250;
          break;
        case 'D':
          pX = 1750;
          break;
        case 'E':
          pX = 2250;
          break;
        default:
          Serial.println("bad_gate!");
          return false;
      }
      switch (buff[1]) {
        case '1':
          pY = 250;
          break;
        case '2':
          pY = 750;
          break;
        case '3':
          pY = 1250;
          break;
        case '4':
          pY = 1750;
          break;
        case '5':
          pY = 2250;
          break;
        default:
          Serial.println("bad_gate!");
          return false;
      }
      
     GATES[i] = Vector2d(pX, pY);

     //Skip to next line
     while (file.available()) {
      if (file.read() == '\n') {
        break;
      }
    }
  }

  //Skip line
  while (file.available()) {
    if (file.read() == '\n') {
      break;
    }
  }
  bool firstDone = false;
  //Read in paths
  while (file.available()) {
    buff[0] = file.read();
    buff[1] = file.read();
    //coords
    double pX, pY;
    switch (buff[0]) {
      case 'A':
        pX = 250;
        break;
      case 'B':
        pX = 750;
        break;
      case 'C':
        pX = 1250;
        break;
      case 'D':
        pX = 1750;
        break;
      case 'E':
        pX = 2250;
          break;
      default:
        Serial.println("bad_path!");
        return false;
    }
    switch (buff[1]) {
      case '1':
        pY = 250;
        break;
      case '2':
        pY = 750;
        break;
      case '3':
        pY = 1250;
      break;
      case '4':
        pY = 1750;
        break;
        case '5':
        pY = 2250;
        break;
      default:
        Serial.println("bad_path!");
        return false;
    }
    if(!firstDone){
      PATH[PATH_SIZE] = Vector2d(pX, -DIST_TO_DOWEL);
      PATH_SIZE++;
      firstDone = true;
    }
    PATH[PATH_SIZE] = Vector2d(pX, pY);
    PATH_SIZE++;
    while (file.available()) {
      if (file.read() == '\n') {
        break;
      }
    }
  }
  return true;
}

bool BTN_STATE(uint8_t index) {
  bool buttonstate = digitalRead(BTN_PINS[index]);
  if (buttonstate != BTN_PREV_STATES[index]) {
    BTN_PREV_STATES[index] = buttonstate;
    if (buttonstate == HIGH) {
      return true;
    }
  }
  return false;
}
