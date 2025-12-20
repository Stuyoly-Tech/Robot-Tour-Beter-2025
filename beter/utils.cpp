#include <Arduino.h>
#include "utils.h"
#include "config.h"

int mm_to_steps(float d, float r, int tpr) {
    return (d/(TWO_PI*r)) * tpr;
}

float steps_to_mm(int t) {
    return ((float)t/(STEPS_PER_REV))*TWO_PI*WHEEL_RADIUS;
}