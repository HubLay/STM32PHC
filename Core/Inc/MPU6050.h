#ifndef __MPU6050_H
#define __MPU6050_H

#include <math.h>
#include "MyI2C.h"
#include "i2c.h"
#include "main.h"
#include "stdint.h"
#include "MPU6050_Reg.h"


class MPU6050
{
public:
  void MPU6050_Init(I2C_HandleTypeDef *__hi2c, uint16_t __address = 0xD0);
  uint8_t get_ID();
  void get_ACCEL();
  void get_GYRO();
  void get_Temp();
  float get_ACC();
  void MPU6050_GYRO_Calibration();
  void MPU6050_ACC_Calibration();
  float get_ACC_PitchAngle();
  float get_ACC_RollAngle();
  void Updata_TrueData();
  void Updata_ACCAngle();

  float ACCAngle[3], TrueGYRO[3],TrueACC[3];

private:
  uint16_t address;
  I2C_HandleTypeDef *hi2c;
  float Temp;
  uint8_t ID;
  float Calibration_GYRO_X,Calibration_GYRO_Y,Calibration_GYRO_Z;
  float Calibration_ACC_X,Calibration_ACC_Y,Calibration_ACC_Z;
  int16_t RawACC[3], RawGYRO[3];
  
  int16_t data_8To16(uint8_t Hdata, uint8_t Ldata);
  void MPU6050_W_Res(uint8_t adr, uint8_t data);
  uint8_t MPU6050_R_Res(uint8_t adr);
};


#endif