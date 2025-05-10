#ifndef __KINEMATICS_H___
#define __KINEMATICS_H___
#include <cmath>

typedef struct{
    float tau_alpha,tau_beta;
} JacobiResult;

typedef struct{
    float pos_x,pos_z,vec_x,vec_z;
} KinematicResult;

typedef struct{
    double start,period,error_sum_x,error_sum_y;
    float kp_x,kd_x,kp_y,kd_y,ki_x,ki_y;
} VMC_Param;

typedef struct{
    float force_x,force_z;
} VMC_Result;
// motor direction follows the right-hand rule
// alpha motor is outer which points to the positive direction of axis-X
// beta motor is inner which points to the negative direction of axis-X
VMC_Result VMC_Calculate(VMC_Param* param,float target_pos_x,float target_pos_y,float now_pos_x,float now_pos_y,float now_vel_x,float now_vel_y);
JacobiResult VMC_Jacobi_Matrix(float alpha,float beta,float force_x ,float force_z);
KinematicResult Kinematic_Solution(float angle_alpha,float angle_beta,float vec_alpha,float vec_beta);
double clip(double var,double maxVal);
#endif