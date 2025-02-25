#ifndef __MYI2C_H
#define __MYI2C_H
#include "stm32f1xx_hal.h"
#include "main.h"
#include <stdint.h>

#define SCL GPIO_PIN_4
#define SDA GPIO_PIN_3


void I2C_W_SCL(uint8_t State);
void I2C_W_SDA(uint8_t State);
uint8_t I2C_R_SDA();
void I2C_Start();
void I2C_Stop();
void I2C_W_Data(uint8_t Data);
int8_t I2C_R_Data();
uint8_t I2C_ReceiceACK();
void I2C_SendACK(uint8_t ACK);


#endif