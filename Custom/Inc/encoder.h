#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"

#define COMM_CLK_PORT GPIOB
#define COMM_CLK_PIN GPIO_PIN_15

typedef struct Encoder {
    uint8_t turn;
    uint16_t pos;

    GPIO_TypeDef * clk_port;
    uint16_t clk_pin;

    GPIO_TypeDef * data_port;
    uint16_t data_pin;

} Encoder;     

void initEncoder(Encoder *encoder, 
    GPIO_TypeDef * data_port,
    uint16_t data_pin);

// float getAngle(Encoder *encoder);


#endif 