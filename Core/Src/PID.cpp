#include "PID.h"

//Mode 1 --- 直立环，以GYRO带入Kd计算。。。。、
//     2 --- 速度环，以正常Kd计算
void PID::Init(float __Kp,float __Ki,float __Kd, int8_t __Mode){
  Kp = __Kp;
  Ki = __Ki;
  Kd = __Kd;

  Mode = __Mode;
}

void PID::Set_Now(float __Now){
  Now = __Now;
}

void PID::Calculate(){
  error = Target - Now;
  // if(error > 0){
  //   Init(1.230f,0.4123f,0.0f);
  // }
  // else{
  //   Init(0.0f, 0.0f, 0.0f);
  // }
  Kp_Out = error * Kp;
  Ki_Out += error * Ki;

  //不是标准PID，为了适应平衡车直立环的改动
  if(Mode == 1){
    Kd_Out = GYRO_Y * Kd;
  }
  else if(Mode == 2){
    Kd_Out = (error-Last_error)*Kd;
  }
         

  //为平衡车速度环做的改动，类似于变速积分
  if (Mode == 2)
  {
    if (Last_error < 0 && error > 0 && (Ki_Out > 50 || Ki_Out < -50))
    {
      Ki_Out = 0;
    }
    else if (Last_error > 0 && error < 0 && (Ki_Out > 50 || Ki_Out < -50))
    {
      Ki_Out = 0;
    }
  }

  Last_error = error;
  if(Ki_Out > 1000){
    Ki_Out = 1000;
  }
  else if(Ki_Out < -1000){
    Ki_Out = -1000;
  }

  Out = Kp_Out + Ki_Out + Kd_Out;
}

void PID::Set_Target(float __Target){
  Target = __Target;
}

float PID::get_Out(){
  return Out;
}

void PID::Set_GYRO(float __GYRO_Y){
  GYRO_Y = __GYRO_Y;
}