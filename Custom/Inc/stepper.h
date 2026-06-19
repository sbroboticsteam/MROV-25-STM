#ifndef STEPPER_H
#define STEPPER_H

#include "main.h"

#define MAX_FREQ 5000.0
#define CB_FREQ 20000.0


typedef struct Stepper {
    uint8_t reset_flag;

    uint32_t accumulator;
    uint32_t ispeed;

    GPIO_TypeDef * step_port;
    uint16_t step_pin;

    GPIO_TypeDef * dir_port;
    uint16_t dir_pin;
    uint8_t dir;
} Stepper;     

void initStepper(Stepper *stepper, 
    GPIO_TypeDef * step_port, 
    uint16_t step_pin, 
    GPIO_TypeDef * dir_port,
    uint16_t dir_pin);

void setStepperPeriod(Stepper *stepper, int val);

void setStepperDir(Stepper *stepper, int val);

void setStepperSpeed(Stepper *stepper, float val);

void accumulate(Stepper *stepper);
#endif 