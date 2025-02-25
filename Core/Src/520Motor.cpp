#include "520Motor.h"
#include <math.h>

void Motor_520::Init(TIM_HandleTypeDef *__TIM_Handle){
  TIM_Handle = __TIM_Handle;

  NowOmega = 0.0f;
  Last_Omega = 0.0f;
  Last_Last_Omega = 0.0f;
  Start_flag = 0;
}

void Motor_520::Omega_Updata(){
  Start_flag ++;
  Encoder = (int16_t)__HAL_TIM_GetCounter(TIM_Handle);

  NowOmega = Encoder*100.0*60.0/(4.0*11.0*30.0);          //rpm 100HZ
  Last_Last_Omega = Last_Omega;
  Last_Omega = NowOmega;

  // if(Start_flag > 2){
  //   NowOmega = (Last_Last_Omega + Last_Omega + NowOmega)/3.0f;
  // }

  __HAL_TIM_SetCounter(TIM_Handle, 0);
}

void Motor_520::Angle_Updata(float Pitch){
  PitchAngle = Pitch;
}

void Motor_520::Calculate_PID(){
  //目前只有配置平衡车的

  PID_Angle.Set_Target(2.0f);
  PID_Angle.Set_Now(PitchAngle);
  PID_Angle.Calculate();

  PID_Omega.Set_Now(NowOmega);
  PID_Omega.Set_Target(0);
  PID_Omega.Calculate();
  
  Out = PID_Angle.get_Out() + PID_Omega.get_Out();
}

float Motor_520::Get_Out(){
  return Out;
}

float Motor_520::Get_Speed(){
  return NowOmega;
}

void Motor_520::Set_Speed(float __NowOmega){
  NowOmega = __NowOmega;
}