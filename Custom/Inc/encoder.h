#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"

#define COMM_CLK_PORT GPIOB
#define COMM_CLK_PIN GPIO_PIN_15

// #define CLK_FRAME_US 20
#define CLK_FRAME_US 20000

#define NUM_ENCODERS 3

typedef struct Encoder {
    uint8_t turn;
    uint16_t pos;

    GPIO_TypeDef * clk_port;
    uint16_t clk_pin;

    GPIO_TypeDef * data_port;
    uint16_t data_pin;

} Encoder;     

typedef struct Encoder_Data {
    float angle;
    float velocity;
} Encoder_Data;

void initEncoder(Encoder *encoder, 
    GPIO_TypeDef * data_port,
    uint16_t data_pin);

float getAngle(Encoder *encoder);

#endif 