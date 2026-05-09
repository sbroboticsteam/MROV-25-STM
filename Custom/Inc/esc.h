#ifndef ESC_H
#define ESC_H

#include "main.h"

int clampESC(int val);


typedef struct ESC {
    TIM_HandleTypeDef htim;
    uint32_t ch;
} ESC; 

void setESC(ESC esc, int val);
#endif 