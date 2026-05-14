#ifndef SERVO_H
#define SERVO_H

#include "main.h"

typedef struct Servo {
    TIM_HandleTypeDef htim;
    uint32_t ch;
} Servo; 

int clampVal(int val);

void setServo(Servo servo, int val);
#endif 