// esc need should store its current value and location
#include "../../Custom/Inc/stepper.h"
#include "main.h"

void initStepper(Stepper *stepper, 
    GPIO_TypeDef * step_port, 
    uint16_t step_pin, 
    GPIO_TypeDef * dir_port,
    uint16_t dir_pin){
        stepper->step_port = step_port;
        stepper->step_pin = step_pin;
        stepper->dir_port = dir_port;
        stepper->dir_pin = dir_pin;

        stepper->dir = 1;
    }

void setStepperDir(Stepper *stepper, int val){
    if (!(val == 1 || val == -1)){ // there may be complications with 1 or 0
        return;
    }

    if (val == 1){
        HAL_GPIO_WritePin(stepper->dir_port, stepper->dir_pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(stepper->dir_port, stepper->dir_pin, GPIO_PIN_RESET);
    }
    stepper->dir = val;
}

void setStepperSpeed(Stepper *stepper, float val){
    //expect val to be from -1 to 1
    stepper->accumulator = 0;
    
    int sign = 1 * (val > 0) + -1 * (val < 0);
    setStepperDir(stepper, sign);

    //clamping val to 1 or -1
    // float clamped = 1.0f * (val > 1.0f) + -1.0f * (val < -1.0f) + val * ((val <= 1) * (val >= 1));
    val = sign * val;
    val = val * (val <= 1) + 1 * (val > 1); // clamping after sign change

    uint32_t ispeed = (uint32_t)(val * UINT32_MAX * (MAX_FREQ / CB_FREQ));
    stepper->ispeed = ispeed;
    
}

void accumulate(Stepper * stepper){
    if (stepper->reset_flag){
        stepper->reset_flag = 0;
        // HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(stepper->step_port, stepper->step_pin, GPIO_PIN_RESET);
    }


    //integer code
    uint32_t prev = stepper->accumulator;
    // int prev = stepper->accumulator;
    stepper->accumulator += stepper->ispeed;
    if (prev > stepper->accumulator){ //overflow occured
        // HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(stepper->step_port, stepper->step_pin, GPIO_PIN_SET);
        stepper->reset_flag = 1;
    }
}