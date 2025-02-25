#include "MPU6050.h"

void MPU6050::MPU6050_W_Res(uint8_t adr,uint8_t data){
    I2C_Start();
    I2C_W_Data(address | 0x00);
    I2C_ReceiceACK();
    I2C_W_Data(adr);
    I2C_ReceiceACK();
    I2C_W_Data(data);
    I2C_ReceiceACK();
    I2C_Stop();

}

uint8_t MPU6050::MPU6050_R_Res(uint8_t adr){
    uint8_t Byte = 0x00;
    
    I2C_Start();
    I2C_W_Data(address | 0x00);
    I2C_ReceiceACK();
    I2C_W_Data(adr);
    I2C_ReceiceACK();

    I2C_Start();
    I2C_W_Data(address | 0x01);
    I2C_ReceiceACK();
    
    Byte = I2C_R_Data();
    I2C_SendACK(1);
    I2C_Stop();

    return Byte;
}

int16_t MPU6050::data_8To16(uint8_t Hdata,uint8_t Ldata){
    return Hdata<<8 | Ldata;
}

uint8_t Init[8] = {0x01,0x00,0x04,0x02,0x18,0x00};

void MPU6050::MPU6050_Init(I2C_HandleTypeDef *__hi2c,uint16_t __address){
    HAL_Delay(100);
    address = __address;
    hi2c = __hi2c;

    //电源管理寄存器1，取消睡眠模式，选择时钟源为X轴陀螺仪
	//电源管理寄存器2，保持默认值0，所有轴均不待机
	//采样率分频寄存器，配置采样率,100Hz
	//配置寄存器，配置DLPF,100/2 = 50
    //陀螺仪配置寄存器，选择满量程为±2000°/s
    //量程±2g
    HAL_I2C_Mem_Write(&hi2c1, address,MPU6050_PWR_MGMT_1, 1, Init + 0, 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Write(&hi2c1, address, MPU6050_PWR_MGMT_2, 1, Init + 1, 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Write(&hi2c1, address, MPU6050_SMPLRT_DIV, 1, Init + 2, 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Write(&hi2c1, address, MPU6050_CONFIG, 1, Init + 3, 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Write(&hi2c1, address, MPU6050_GYRO_CONFIG, 1, Init + 4, 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Write(&hi2c1, address, MPU6050_ACCEL_CONFIG, 1, Init + 5, 1, HAL_MAX_DELAY);
}

uint8_t MPU6050::get_ID(){
    HAL_I2C_Mem_Read(hi2c,address,MPU6050_WHO_AM_I,1,&ID,1,100);
    return ID;
}

uint8_t Hdata;
uint8_t Ldata;
uint8_t State;

void MPU6050::get_ACCEL(){
    //HAL_I2C_Mem_Read(hi2c,address,MPU6050_ACCEL_CONFIG,1,&State,1,HAL_MAX_DELAY);

    HAL_I2C_Mem_Read(hi2c,address,MPU6050_ACCEL_XOUT_H,1,&Hdata,1,HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(hi2c,address,MPU6050_ACCEL_XOUT_L,1,&Ldata,1,HAL_MAX_DELAY);
    RawACC[0] = data_8To16(Hdata,Ldata);

    HAL_I2C_Mem_Read(hi2c,address,MPU6050_ACCEL_YOUT_H,1,&Hdata,1,HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(hi2c,address,MPU6050_ACCEL_YOUT_L,1,&Ldata,1,HAL_MAX_DELAY);
    RawACC[1] = data_8To16(Hdata,Ldata);

    HAL_I2C_Mem_Read(hi2c,address,MPU6050_ACCEL_ZOUT_H,1,&Hdata,1,HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(hi2c,address,MPU6050_ACCEL_ZOUT_L,1,&Ldata,1,HAL_MAX_DELAY);
    RawACC[2] = data_8To16(Hdata,Ldata);

}    

void MPU6050::get_GYRO(){
    //uint8_t Hdata;
    //uint8_t Ldata;

    HAL_I2C_Mem_Read(hi2c,address,MPU6050_GYRO_XOUT_H,1,&Hdata,1,HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(hi2c,address,MPU6050_GYRO_XOUT_L,1,&Ldata,1,HAL_MAX_DELAY);
    RawGYRO[0] = data_8To16(Hdata,Ldata);

    HAL_I2C_Mem_Read(hi2c,address,MPU6050_GYRO_YOUT_H,1,&Hdata,1,HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(hi2c,address,MPU6050_GYRO_YOUT_L,1,&Ldata,1,HAL_MAX_DELAY);
    RawGYRO[1] = data_8To16(Hdata,Ldata);

    HAL_I2C_Mem_Read(hi2c,address,MPU6050_GYRO_ZOUT_H,1,&Hdata,1,HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(hi2c,address,MPU6050_GYRO_ZOUT_L,1,&Ldata,1,HAL_MAX_DELAY);
    RawGYRO[2] = data_8To16(Hdata,Ldata);
}

void MPU6050::get_Temp(){
    uint8_t Hdata;
    uint8_t Ldata;

    HAL_I2C_Mem_Read(hi2c,address,MPU6050_TEMP_OUT_H,1,&Hdata,1,HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(hi2c,address,MPU6050_TEMP_OUT_L,1,&Ldata,1,HAL_MAX_DELAY);
    Temp = (data_8To16(Hdata,Ldata))/340+36.53;
}

float MPU6050::get_ACC(){
    return sqrt(TrueACC[0]*TrueACC[0] + TrueACC[1]*TrueACC[1] + TrueACC[2]*TrueACC[2])*10;
}

void MPU6050::MPU6050_GYRO_Calibration(){
    int Cxx = 0, Cyy = 0, Czz = 0;
    for(int i = 0;i<200;i++){
        get_GYRO();
        Cxx+=RawGYRO[0];
        Cyy+=RawGYRO[1];
        Czz+=RawGYRO[2];
        HAL_Delay(2);       //每次读取间的固定间隔
    }
    Calibration_GYRO_X = (float)Cxx*0.060975/200;
    Calibration_GYRO_Y = (float)Cyy*0.060975/200;
    Calibration_GYRO_Z = (float)Czz*0.060975/200;
}

void MPU6050::MPU6050_ACC_Calibration(){
    int Cxx = 0, Cyy = 0, Czz = 0;
    for(int i = 0;i<200;i++){
        get_ACCEL();
        Cxx+=RawACC[0];
        Cyy+=RawACC[1];
        Czz+=RawACC[2];
        HAL_Delay(2);
    }
    Calibration_ACC_X = (float)Cxx*0.000061/200;
    Calibration_ACC_Y = (float)Cyy*0.000061/200;
    Calibration_ACC_Z = (float)Czz*0.000061/200;
}

void MPU6050::Updata_TrueData(){
    TrueACC[0] = (float)RawACC[0]*0.000061 - Calibration_ACC_X;
    TrueACC[1] = (float)RawACC[1]*0.000061 - Calibration_ACC_Y;
    TrueACC[2] = (float)(RawACC[2] + 16384)*0.000061 - Calibration_ACC_Z;

    TrueGYRO[0] = (float)RawGYRO[0]*0.060975 - Calibration_GYRO_X;
    TrueGYRO[1] = (float)RawGYRO[1]*0.060975 - Calibration_GYRO_Y;
    TrueGYRO[2] = (float)RawGYRO[2]*0.060975 - Calibration_GYRO_Z;
}

void MPU6050::Updata_ACCAngle(){
    ACCAngle[1] = -atan2(TrueACC[0],sqrt(TrueACC[2]*TrueACC[2]+TrueACC[1]*TrueACC[1]))* 57.3;
    ACCAngle[0] = atan2(TrueACC[1],TrueACC[2])* 57.3;
}

float MPU6050::get_ACC_PitchAngle(){
    return ACCAngle[1];                                //由于数据的计算方式，当pitch接近90度，CAZ趋近于0
                                                    //此时对于来自CAY的微小抖动也会引起Roll巨大变化，突变到90
                                                    //但反之Roll到90却不会影响Pitch，与其计算方式有关
                                                    //
}

float MPU6050::get_ACC_RollAngle(){
    return ACCAngle[0]; 
}