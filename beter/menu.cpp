#include "beter.ino"
#include "CONFIG.h"
#include <AccelStepper.h>
#include "controller.h"
#include "simplePursuit.h"
#include "robot.h"

uint8_t BTN_PINS[] = {BTN_0, BTN_1, BTN_2, BTN_3};
bool BTN_PREV_STATES[] = {LOW, LOW, LOW, LOW};

void displayScreen(int state){

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