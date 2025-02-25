#include "PID.h"
#include "main.h"

class Motor_520{
public:
  PID PID_Omega;
  PID PID_Angle;
  void Init(TIM_HandleTypeDef *__TIM_Handle);
  void Omega_Updata();
  void Angle_Updata(float Pitch);
  void Calculate_PID();
  void Set_Speed(float __NowOmega);
  float Get_Speed();
  float Get_Out();
private:
  TIM_HandleTypeDef *TIM_Handle;
  int Start_flag;
  float PitchAngle;       //车体当前俯仰角
  float NowOmega, Last_Omega, Last_Last_Omega;
  float Encoder;
  float Out;
};