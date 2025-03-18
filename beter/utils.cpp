#include <Arduino.h>
#include "utils.h"

int meters_to_ticks(float d, float r, int tpr) {
    return (d/(TWO_PI*r)) * tpr;
}

float ticks_to_meters(int t, float r, int tpr) {
    return ((float)t/tpr)*TWO_PI*r;
}