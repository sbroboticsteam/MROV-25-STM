// esc need should store its current value and location
#include "../../Custom/Inc/servo.h"
#include "main.h"

int clampVal(int val){
    int newVal = 1000 * (val < 1000) + val * (1 - (val < 1000));
    newVal = 2000 * (val > 2000) + val * (1 - (val > 2000));
    return newVal;
}

void setServo(Servo servo, int val){
    __HAL_TIM_SET_COMPARE(&(servo.htim), servo.ch, clampVal(val));
}