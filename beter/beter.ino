#include <mutex>
#include <esp_task_wdt.h>

#include "CONFIG.h"
#include <AccelStepper.h>
#include "controller.h"
#include "simplePursuit.h"
#include "robot.h"

#include <ArduinoEigenDense.h>
using namespace Eigen;
#include "FS.h"
#include "SPI.h"
#include "SD.h"

#include "SparkFun_BMI270_Arduino_Library.h"

#include <Wire.h>
#include <Adafruit_SSD1306.h>

//FSM State
int STATE;

//Path globals
Vector2d PATH[100];  //= {Vector2d(0, 0), Vector2d(0, 300), Vector2d(300, 300), Vector2d(300, 0), Vector2d(0, 0)};
uint8_t PATH_SIZE;

//Gates
Vector2d GATES[7];
uint8_t GATE_SIZE;

//Paramters
double TARGET_TIME;

int PATH_MODE;

double TIME_INCREMENT = 0.1;
double DIST_INCREMENT = 1;
int TICKS = 0;
double FINAL_OFFSET_X;
double FINAL_OFFSET_Y;
double TIME_OFFSET;
double TEMP_OFFSET;

uint8_t BTN_PINS[] = { BTN_0, BTN_1, BTN_2, BTN_3, INCR_BTN };
bool BTN_PREV_STATES[] = { LOW, LOW, LOW, LOW, LOW };

//SD Methods
boolean loadPathFromSD(fs::FS &fs);

Adafruit_SSD1306 SCREEN(SCREEN_WIDTH, SCREEN_HEIGHT, SCREEN_WIRE, OLED_RESET);

AccelStepper stepperL(AccelStepper::DRIVER, STEP_L, DIR_L);
AccelStepper stepperR(AccelStepper::DRIVER, STEP_R, DIR_R);

//Mutexes for stepper instances
std::mutex steppersEngaged_mtx;

//Multicore tasks for engaging steppers
void engageSteppers(void *parameter);
TaskHandle_t engageSteppersHandle = NULL;

controller robotController(
  WHEEL_RADIUS, TRACK_WIDTH,
  &stepperL, &stepperR,
  STEPS_PER_REV, TURN_US,
  IMU_UPDATE_US, &steppersEngaged_mtx,
  &engageSteppers, &engageSteppersHandle,
  HIGH_PASS_FREQ);

simplePursuit robotSimplePursuit(MAX_VX, DIST_TO_DOWEL);

robot Robot(
  &robotSimplePursuit, &robotController,
  MAX_ACCEL, MAX_ANG_ACCEL, MAX_ANG_VEL,
  DIST_TO_DOWEL);

void setup() {
  //for steppers
  //xTaskCreate(engageSteppers, "engageSteppers Task", 10000, NULL, 1, &engageSteppersHandle);

  //start init
  STATE = INIT;

  Serial.begin(115200);

  Wire.begin(17, 18);
  Wire.setClock(400000L);

  //init pins
  pinMode(BTN_0, INPUT);
  pinMode(BTN_1, INPUT);
  pinMode(BTN_2, INPUT);
  pinMode(BTN_3, INPUT);
  pinMode(INCR_BTN, INPUT);

  pinMode(STEP_ENABLE, OUTPUT);
  digitalWrite(STEP_ENABLE, HIGH);

  pinMode(LED_0, OUTPUT);
  pinMode(LED_1, OUTPUT);
  pinMode(LASERS, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(INCR_A), encoderInterruptHandlerA, RISING);

  //oled init
  SCREEN.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  SCREEN.clearDisplay();
  SCREEN.display();
  STATE = INIT;
  displayScreen(STATE);

  //gyroInit();

  //SD begin
  if (SD.begin(SD_CS) == 0) {
    STATE = SD_ERROR;
  }

  //load Paths
  if (!loadPathFromSD(SD)) {
    STATE = FILE_ERROR;
  }

  if (STATE == INIT) {
    STATE = IDLE;
    displayScreen(STATE);
  }

  if (PATH_MODE == 2) {
    testTurns();
  }

  if (PATH_MODE == 3) {
    testDist();
  }
}

void loop() {
  switch (STATE) {
    case INIT:
      break;
    case IDLE:
      if (BTN_STATE(1)) {
        STATE = INIT;
        gyroInit();
        displayScreen(STATE);
        Robot.init(PATH_MODE);
        robotSimplePursuit.init(PATH, PATH_SIZE, GATES, GATE_SIZE, TARGET_TIME + TIME_OFFSET, FINAL_OFFSET_Y, FINAL_OFFSET_X);
        STATE = READY;
        digitalWrite(STEP_ENABLE, LOW);
        displayScreen(STATE);
      }
      if (BTN_STATE(2)) {
        STATE = ADJUST_MENU;
        displayScreen(STATE);
      }
      if (BTN_STATE(3)) {
        digitalWrite(LASERS, !digitalRead(LASERS));
        digitalWrite(LED_0, !digitalRead(LED_0));
      }
      break;
    case READY:
      if (BTN_STATE(0)) {
        digitalWrite(LASERS, LOW);
        digitalWrite(LED_0, LOW);
        Robot.startPath();
        STATE = RUNNING;
        displayScreen(STATE);
        robotController.initTheta(PI / 2);
      }
      if (BTN_STATE(1)) {
        STATE = IDLE;
        digitalWrite(STEP_ENABLE, HIGH);
        displayScreen(STATE);
      }
      break;
    case RUNNING:
      Robot.update();
      if (BTN_STATE(0) || BTN_STATE(1) || BTN_STATE(2) || BTN_STATE(3) || BTN_STATE(4)) {
        STATE = STOPPED;
        displayScreen(STATE);
      }
      if (Robot.getState() == 0) {
        STATE = END_RUN;
        displayScreen(STATE);
      }
      break;
    case END_RUN:
      if (BTN_STATE(0) || BTN_STATE(1) || BTN_STATE(2) || BTN_STATE(3) || BTN_STATE(4)) {
        STATE = IDLE;
        digitalWrite(STEP_ENABLE, HIGH);
        displayScreen(STATE);
      }
      break;
    case STOPPED:
      if (BTN_STATE(0) || BTN_STATE(1) || BTN_STATE(2) || BTN_STATE(3) || BTN_STATE(4)) {
        STATE = IDLE;
        displayScreen(STATE);
      }
      break;
    case ADJUST_MENU:
      TICKS = 0;
      STATE = ADJUST_TIME;
      displayScreen(STATE);
      break;
    case ADJUST_TIME:
      TEMP_OFFSET = TICKS * TIME_INCREMENT;
      if (BTN_STATE(1)) {
        STATE = IDLE;
        displayScreen(STATE);
      }
      if (BTN_STATE(0)) {
        STATE = ADJUST_TIME;
        TICKS = 0;
        displayScreen(STATE);
      }
      if (BTN_STATE(2)) {
        STATE = ADJUST_X;
        TICKS = 0;
        displayScreen(STATE);
      }
      if (BTN_STATE(3)) {
        STATE = ADJUST_Y;
        TICKS = 0;
        displayScreen(STATE);
      }
      if (BTN_STATE(4)) {
        TIME_OFFSET = TEMP_OFFSET;
        displayScreen(STATE);
      }
      break;
    case ADJUST_X:
      TEMP_OFFSET = TICKS * DIST_INCREMENT;
      if (BTN_STATE(1)) {
        STATE = IDLE;
        displayScreen(STATE);
      }
      if (BTN_STATE(0)) {
        STATE = ADJUST_TIME;
        TICKS = 0;
        displayScreen(STATE);
      }
      if (BTN_STATE(2)) {
        STATE = ADJUST_X;
        TICKS = 0;
        displayScreen(STATE);
      }
      if (BTN_STATE(3)) {
        STATE = ADJUST_Y;
        TICKS = 0;
        displayScreen(STATE);
      }
      if (BTN_STATE(4)) {
        FINAL_OFFSET_X = TEMP_OFFSET;
        displayScreen(STATE);
      }
      break;
    case ADJUST_Y:
      TEMP_OFFSET = TICKS * DIST_INCREMENT;
      if (BTN_STATE(1)) {
        STATE = IDLE;
        displayScreen(STATE);
      }
      if (BTN_STATE(0)) {
        STATE = ADJUST_TIME;
        TICKS = 0;
        displayScreen(STATE);
      }
      if (BTN_STATE(2)) {
        STATE = ADJUST_X;
        TICKS = 0;
        displayScreen(STATE);
      }
      if (BTN_STATE(3)) {
        STATE = ADJUST_Y;
        TICKS = 0;
        displayScreen(STATE);
      }
      if (BTN_STATE(4)) {
        FINAL_OFFSET_Y = TEMP_OFFSET;
        displayScreen(STATE);
      }
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

void engageSteppers(void *parameter) {
  //esp_task_wdt_init(300, false);
  steppersEngaged_mtx.lock();
  while (stepperL.run() && stepperR.run())
    ;
  stepperL.setCurrentPosition(stepperL.targetPosition());
  stepperR.setCurrentPosition(stepperR.targetPosition());
  steppersEngaged_mtx.unlock();
  vTaskDelete(NULL);
}

//no more x y offset in SD file
boolean loadPathFromSD(fs::FS &fs) {
  /*
     FILE FORMAT
     
     PATH MODE:
     1
     TARGET TIME:
     50.00
     NUM GATES:
     4
     GATES:
     A1
     ...
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
  for (byte i = 0; i < GATE_SIZE; i++) {
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
    if (!firstDone) {
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

void testTurns() {
  delay(2000);
  robotController.init();
  Robot.init(1);
  STATE = RUNNING;
  displayScreen(STATE);
  digitalWrite(STEP_ENABLE, LOW);
  for (int i = 0; i < 50; i++) {
    robotController.setTheta(PI / 2);
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
  STATE = END_RUN;
}

void testDist() {
  delay(2000);
  robotController.init();
  robotController.setTheta(PI / 2);
  Robot.init(1);
  STATE = RUNNING;
  displayScreen(STATE);
  digitalWrite(STEP_ENABLE, LOW);
  robotController.setMaxVx(MAX_VX);
  for (int i = 0; i < 50; i++) {
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
  STATE = END_RUN;
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
//Interrupts
void encoderInterruptHandlerA() {
  if (digitalRead(INCR_A) != digitalRead(INCR_B)) {
    TICKS--;
  } else {
    TICKS++;
  }
}

void encoderInterruptHandlerB() {
  if (digitalRead(INCR_A) == digitalRead(INCR_B)) {
    TICKS--;
  } else {
    TICKS++;
  }
}

void displayScreen(int state) {
  SCREEN.clearDisplay();
  SCREEN.setTextColor(SSD1306_WHITE);
  switch (state) {
    case INIT:
      SCREEN.setTextSize(1);
      SCREEN.setCursor(1, 1);
      SCREEN.print("INIT: DON'T MOVE");
      break;
    case IDLE:
      //Button Letters
      SCREEN.setTextSize(1);
      SCREEN.setCursor(0, 0);
      SCREEN.print("E");
      SCREEN.setCursor(SCREEN_WIDTH - 10, 0);
      SCREEN.print("A");
      SCREEN.setCursor(0, SCREEN_HEIGHT - 10);
      SCREEN.print("R");
      SCREEN.setCursor(SCREEN_WIDTH - 10, SCREEN_HEIGHT - 10);
      SCREEN.print("L");

      SCREEN.setCursor((1.5 * SCREEN_WIDTH) / 8, 0);
      SCREEN.println("NOT READY");
      SCREEN.println("TARGET TIME:");
      SCREEN.println(TARGET_TIME + TIME_OFFSET, 3);
      SCREEN.println("X OFFSET(mm):");
      SCREEN.println(FINAL_OFFSET_X, 3);
      SCREEN.println("Y OFFSET(mm):");
      SCREEN.println(FINAL_OFFSET_Y, 3);
      SCREEN.println("LASERS?");
      if (digitalRead(LASERS)) {
        SCREEN.print("YES");
      } else SCREEN.print("NO");
      break;
    case READY:
      //Button Letters
      SCREEN.setTextSize(1);
      SCREEN.setCursor(0, 0);
      SCREEN.print("E");
      SCREEN.setCursor(SCREEN_WIDTH - 10, 0);
      SCREEN.print("A");
      SCREEN.setCursor(0, SCREEN_HEIGHT - 10);
      SCREEN.print("R");
      SCREEN.setCursor(SCREEN_WIDTH - 10, SCREEN_HEIGHT - 10);
      SCREEN.print("L");

      SCREEN.setCursor((1.5 * SCREEN_WIDTH) / 8, 0);
      SCREEN.println("READY");
      SCREEN.println("TARGET TIME:");
      SCREEN.println(TARGET_TIME + TIME_OFFSET, 3);
      SCREEN.println("X OFFSET(mm):");
      SCREEN.println(FINAL_OFFSET_X, 3);
      SCREEN.println("Y OFFSET(mm):");
      SCREEN.println(FINAL_OFFSET_Y, 3);
      SCREEN.println("LASERS?");
      if (digitalRead(LASERS)) {
        SCREEN.print("YES");
      } else SCREEN.print("NO");
      break;
    case RUNNING:
      SCREEN.setTextSize(3);
      SCREEN.setCursor(0, 0);
      SCREEN.print("RUNNING");
      break;
    case END_RUN:
      SCREEN.setTextSize(2);
      SCREEN.setCursor(0, 0);
      SCREEN.println("RUN ENDED");
      SCREEN.println("RUNTIME:");
      SCREEN.print(Robot.stopPath());
      break;
    case STOPPED:
      SCREEN.setTextSize(2);
      SCREEN.setCursor(0, 0);
      SCREEN.println("RUN STOPPED");
      SCREEN.println("RUNTIME:");
      SCREEN.print(Robot.stopPath());
      break;
    case SD_ERROR:
      SCREEN.setTextSize(2);
      SCREEN.setCursor(0, 0);
      SCREEN.println("SD ERROR");
      break;
    case FILE_ERROR:
      SCREEN.setTextSize(2);
      SCREEN.setCursor(0, 0);
      SCREEN.println("FILE ERROR");
      break;
    case IMU_ERROR:
      SCREEN.setTextSize(2);
      SCREEN.setCursor(0, 0);
      SCREEN.println("IMU ERROR");
      break;
    case ADJUST_X:
      //Button Letters
      SCREEN.setTextSize(1);
      SCREEN.setCursor(0, 0);
      SCREEN.print("x");
      SCREEN.setCursor(SCREEN_WIDTH - 10, 0);
      SCREEN.print("X");
      SCREEN.setCursor(0, SCREEN_HEIGHT - 10);
      SCREEN.print("T");
      SCREEN.setCursor(SCREEN_WIDTH - 10, SCREEN_HEIGHT - 10);
      SCREEN.print("Y");

      SCREEN.setTextSize(3);
      SCREEN.setCursor((1.5 * SCREEN_WIDTH) / 8, 0);
      SCREEN.println("X OFFSET");
      SCREEN.setTextSize(1);
      if (TEMP_OFFSET = FINAL_OFFSET_X) {
        SCREEN.println("SAVED");
      } else {
        SCREEN.println("UNSAVED");
      }
      SCREEN.print(TEMP_OFFSET);
      SCREEN.print("mm");
      break;
    case ADJUST_Y:
      //Button Letters
      SCREEN.setTextSize(1);
      SCREEN.setCursor(0, 0);
      SCREEN.print("x");
      SCREEN.setCursor(SCREEN_WIDTH - 10, 0);
      SCREEN.print("X");
      SCREEN.setCursor(0, SCREEN_HEIGHT - 10);
      SCREEN.print("T");
      SCREEN.setCursor(SCREEN_WIDTH - 10, SCREEN_HEIGHT - 10);
      SCREEN.print("Y");

      SCREEN.setTextSize(3);
      SCREEN.setCursor((1.5 * SCREEN_WIDTH) / 8, 0);
      SCREEN.println("Y OFFSET");
      SCREEN.setTextSize(1);
      if (TEMP_OFFSET = FINAL_OFFSET_Y) {
        SCREEN.println("SAVED");
      } else {
        SCREEN.println("UNSAVED");
      }
      SCREEN.print(TEMP_OFFSET);
      SCREEN.print("mm");
      break;

    case ADJUST_TIME:
      //Button Letters
      SCREEN.setTextSize(1);
      SCREEN.setCursor(0, 0);
      SCREEN.print("x");
      SCREEN.setCursor(SCREEN_WIDTH - 10, 0);
      SCREEN.print("X");
      SCREEN.setCursor(0, SCREEN_HEIGHT - 10);
      SCREEN.print("T");
      SCREEN.setCursor(SCREEN_WIDTH - 10, SCREEN_HEIGHT - 10);
      SCREEN.print("Y");

      SCREEN.setTextSize(3);
      SCREEN.setCursor((1.5 * SCREEN_WIDTH) / 8, 0);
      SCREEN.println("TIME OFFSET");
      SCREEN.setTextSize(1);
      if (TEMP_OFFSET = TIME_OFFSET) {
        SCREEN.println("SAVED");
      } else {
        SCREEN.println("UNSAVED");
      }
      SCREEN.print(TEMP_OFFSET);
      SCREEN.print("s");
      break;
  }
  SCREEN.display();
}