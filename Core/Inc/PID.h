#include <stdint.h>

class PID{
public:
  float get_Out();
  void Set_Now(float __Now);
  void Init(float __Kp,float __Ki,float __Kd, int8_t __Mode = 2);
  void Calculate();
  void Set_Target(float __Target);

  void Set_GYRO(float __GYRO_Y);
private:
  float Now, Target;
  float error, Last_error;
  float Kp,Ki,Kd;
  float Out, Kp_Out,Ki_Out,Kd_Out;
  float GYRO_Y;

  int8_t Mode;
};