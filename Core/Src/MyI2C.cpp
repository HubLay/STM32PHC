#include "MyI2C.h"

void I2C_W_SCL(uint8_t State){
    HAL_GPIO_WritePin(GPIOB,SCL,(GPIO_PinState)(State));
    HAL_Delay(1);
}

void I2C_W_SDA(uint8_t State){
    HAL_GPIO_WritePin(GPIOB,SDA,(GPIO_PinState)(State));
    HAL_Delay(1);       //1ms延时还是太长，可尝试用定时器改
}

uint8_t I2C_R_SDA(){
    uint8_t Bit = 0;
    Bit = HAL_GPIO_ReadPin(GPIOB,SDA);
    HAL_Delay(1);
    return Bit;
}

void I2C_Start(){
    I2C_W_SDA(1);           //防止重复启动时，前一次SDA为0，无法完成下降沿
    I2C_W_SCL(1);
    I2C_W_SDA(0);
    I2C_W_SCL(0);
}

void I2C_Stop(){
    I2C_W_SDA(0);       //确保SDA为强下拉，从而完成上升沿
    I2C_W_SCL(1);
    I2C_W_SDA(1);
}

void I2C_W_Data(uint8_t Data){
    for(uint8_t i = 0;i<8;i++){
        I2C_W_SDA(Data & (0x80 >> i));      //发送接收数据都是高位在前
        I2C_W_SCL(1);
        I2C_W_SCL(0);
    }
}

int8_t I2C_R_Data(){
    int8_t Data = 0x00;
    I2C_W_SDA(1);               //接受数据前先释放SDA，防止前一次出现SDA已经被主机强制下拉影响从机发送
                                //由于是弱上拉，所以即使SDA已被从机发数据，这里也不会被主机影响数据发送
    for(int i = 0;i<8;i++){
        I2C_W_SCL(1);
        if(I2C_R_SDA() == 1){
            Data = Data | (0x80 >> i);
        }
        I2C_W_SCL(0);
    }
    return Data;
}

uint8_t I2C_ReceiceACK(){
    I2C_W_SDA(1);
    I2C_W_SCL(1);
    uint8_t ACK = I2C_R_SDA();
    I2C_W_SCL(0);
    return ACK;
}

void I2C_SendACK(uint8_t ACK){
    I2C_W_SDA(ACK);
    I2C_W_SCL(1);
    I2C_W_SCL(0);
}