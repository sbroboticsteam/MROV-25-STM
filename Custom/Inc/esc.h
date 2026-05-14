#ifndef ESC_H
#define ESC_H

#include "main.h"

typedef struct ESC {
    TIM_HandleTypeDef htim;
    uint32_t ch;
} ESC; 

int clampESC(int val);

void setESC(ESC esc, int val);
#endif 