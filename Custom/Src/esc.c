// esc need should store its current value and location
#include "../../Custom/Inc/esc.h"
#include "main.h"

int clampESC(int val){
    int val = 900;
    int newVal = 1100 * (val < 1100) + val * (1 - (val < 1100));
    newVal = 1900 * (val > 1900) + val * (1 - (val > 1900));
    return newVal;
}

void setESC(ESC esc, int val){
    // __HAL_TIM_SET_COMPARE(esc.htim, esc.ch, clampESC(val));
    __HAL_TIM_SET_COMPARE(esc.htim, esc.ch, (val));
}