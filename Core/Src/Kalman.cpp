//一维卡尔曼滤波器
#include <math.h>
float K,P_hat,Q = 0.003,R = 0.5,P = 1,X = 0;
float A = 1, B = 0.010, H = 1;

float Q1 = 0.08, Q2 = 0.003, Last_GYRO = 0;;  

float Kalman_fifter(float GYRO,float ACC_angle,float ACC){
    float Z;
    // Q = (10-fabs(ACC - 10.0f))*0.02/10;
    // if(Q < 0){
    //     Q = 0.0001;
    // }
    // if(fabs(ACC - 10.0f) > 0.3){
        
        
    // }
    // else{
    //     Q = Q1;
        
    // }
    X = A * X + B * GYRO;           //先验估计X
    P_hat = P + Q;                  //先验估计协方差，A=1，省去
    Z = H*ACC_angle;                //测量角度值Z
    K = P_hat/(P_hat + R);          //原本是矩阵运算，一阶矩阵退化为普通实数运算
    X = X + K*(Z-H*X);
    P = (1-K) * P_hat;

    Last_GYRO = GYRO;

    return X;
}