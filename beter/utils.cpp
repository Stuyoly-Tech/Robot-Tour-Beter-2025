#include <Arduino.h>
#include "utils.h"

int mm_to_steps(float d, float r, int tpr) {
    return (d/(TWO_PI*r)) * tpr;
}

float steps_to_mm(int t, float r, int tpr) {
    return ((float)t/tpr)*TWO_PI*r;
}