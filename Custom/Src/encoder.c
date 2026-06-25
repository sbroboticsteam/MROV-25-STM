#include "main.h"
#include "../../Custom/Inc/encoder.h"
#include <stdio.h>

void initEncoder(Encoder *encoder, 
    GPIO_TypeDef * data_port,
    uint16_t data_pin)
    {
        encoder->clk_port = COMM_CLK_PORT;
        encoder->clk_pin = COMM_CLK_PIN;

        encoder->data_port = data_port;
        encoder->data_pin = data_pin;
    }

float getAngle(Encoder *encoder){
    return (float)encoder->pos * 360.0 / 1024.0; 
}  
