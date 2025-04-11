#include <cmath>

float rods[4] = {0.15,0.27,0.15,0.27};

typedef struct{
    float tau_alpha,tau_beta;
} JacobiResult;

typedef struct{
    float pos_x,pos_z,vec_x,vec_z;
} KinematicResult;

typedef struct{
    float kp_x,kd_x,kp_y,kd_y;
} VMC_Param;

typedef struct{
    float force_x,force_z;
} VMC_Result;

VMC_Result VMC_Calculate(VMC_Param* param,float target_pos_x,float target_pos_y,float now_pos_x,float now_pos_y,float now_vel_x,float now_vel_y);
JacobiResult VMC_Jacobi_Matrix(float alpha,float beta,float force_x ,float force_z);
KinematicResult Kinematic_Solution(float angle_alpha,float angle_beta,float vec_alpha,float vec_beta);