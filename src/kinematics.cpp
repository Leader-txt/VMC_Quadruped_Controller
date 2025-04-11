#include "kinematics.h"

VMC_Result VMC_Calculate(VMC_Param* param,float target_pos_x,float target_pos_y,float now_pos_x,float now_pos_y,float now_vel_x,float now_vel_y){
    VMC_Result result;
    result.force_x = param->kp_x * (target_pos_x - now_pos_x) + param->kd_x * (0 - now_vel_x);
    result.force_z = param->kp_y * (target_pos_y - now_pos_y) + param->kd_y * (0 - now_vel_y);
    return result;
}

JacobiResult VMC_Jacobi_Matrix(float alpha,float beta,float force_x ,float force_z)
{
    // 算出Phi1,Phi2,和运动学正解的一模一样
    float xb, xd, zb, zd;
    float A, B, C;
    float sin_Phi1, sin_Phi2, cos_Phi1, cos_Phi2;
    xb = rods[0] * cos(alpha);
    xd = rods[2] * cos(beta);
    zb = rods[0] * sin(alpha);
    zd = rods[2] * sin(beta);
    A = 2 * rods[1] * (xb - xd);
    B = 2 * rods[1] * (zb - zd);
    C = rods[1] * rods[1] + (xb - xd) * (xb - xd) + (zb - zd) * (zb - zd) - rods[3] * rods[3];
    cos_Phi1 = (-C * A - B * sqrt(A * A + B * B - C * C)) / (A * A + B * B);
    cos_Phi2 = (xb - xd + rods[1] * cos_Phi1) / rods[3];
    // sin_Phi1 = (-C - A * cos_Phi1) / B
    sin_Phi1 = sqrt(1-cos_Phi1*cos_Phi1);
    sin_Phi2 = (zb - zd + rods[1] * sin_Phi1) / rods[3];
    float sin_Phi1_Phi2 = sin_Phi1 * cos_Phi2 - sin_Phi2 * cos_Phi1;
    // 定义雅可比矩阵
    // [ Jacobi[0][0] Jacobi[0][1] ] [Fx]
    // [ Jacobi[1][0] Jacobi[1][1] ] [Fy]
    // Jacobi = [[[],[]],[[],[]]]
    float Jacobi[2][2];
    Jacobi[0][0] = rods[0] * sin_Phi2 * (sin(alpha) * cos_Phi1
    - sin_Phi1 * cos(alpha)) / sin_Phi1_Phi2;
    Jacobi[1][0] = rods[2] * sin_Phi1 * (sin_Phi2 * cos(beta)
    - sin(beta) * cos_Phi1) / sin_Phi1_Phi2;
    Jacobi[0][1] = -rods[0] * cos_Phi2 * (sin(alpha) * cos_Phi1
    - sin_Phi1 * cos(alpha)) / sin_Phi1_Phi2;
    Jacobi[1][1] = -rods[2] * cos_Phi1 * (sin_Phi2 * cos(beta)
    - sin(beta) * cos_Phi1) / sin_Phi1_Phi2;
    float tau_alpha = (force_x * Jacobi[0][0] + force_z * Jacobi[0][1]);
    float tau_beta = (force_x * Jacobi[1][0] + force_z * Jacobi[1][1]);
    JacobiResult result;
    result.tau_alpha = tau_alpha;
    result.tau_beta = tau_beta;
    return result;
}

KinematicResult Kinematic_Solution(float angle_alpha,float angle_beta,float vec_alpha,float vec_beta) // 其实就是Motor_Covert_To_Foot,关节坐标系到足端坐标系
{ 
    float xb, xd, zb, zd;
    float A, B, C;
    float vbx, vbz, vdx, vdz;
    float sin_Phi1, sin_Phi2, cos_Phi1, cos_Phi2, w_Phi1;

    // 根据电机转角计算足端位置
    xb = rods[0] * cos(angle_alpha);
    xd = rods[2] * cos(angle_beta);
    zb = rods[0] * sin(angle_alpha);
    zd = rods[2] * sin(angle_beta);

    A = 2 * rods[1] * (xb - xd);
    B = 2 * rods[1] * (zb - zd);
    C = rods[1] * rods[1] + (xb - xd) * (xb - xd) + (zb - zd) * (zb - zd) - rods[3] * rods[3];

    cos_Phi1 = (-C * A - B * sqrt(A * A + B * B - C * C)) / (A * A + B * B);
    cos_Phi2 = (xb - xd + rods[1] * cos_Phi1) / rods[3];

    // sin_Phi1 = (-C - A * cos_Phi1) / B
    sin_Phi1 = sqrt(1-cos_Phi1*cos_Phi1);
    sin_Phi2 = (zb - zd + rods[1] * sin_Phi1) / rods[3];
    float pos_x = xb + rods[1] * cos_Phi1;
    float pos_z = zb + rods[1] * sin_Phi1;

    // 根据电机角速度计算足端速度
    vbx = -rods[0] * vec_alpha * sin(angle_alpha);
    vbz = rods[0] * vec_alpha * cos(angle_alpha);
    vdx = -rods[2] * vec_beta * sin(angle_beta);
    vdz = rods[2] * vec_beta * cos(angle_beta);

    w_Phi1 = ((vbx - vdx) * cos_Phi2 + (vbz - vdz) * sin_Phi2) / (rods[1] * (sin_Phi1 * cos_Phi2 - sin_Phi2 * cos_Phi1));

    float vec_x = vbx - rods[1] * w_Phi1 * sin_Phi1;
    float vec_z = vbz + rods[1] * w_Phi1 * cos_Phi1;
    KinematicResult result;
    result.pos_x = pos_x;
    result.pos_z = pos_z;
    result.vec_x = vec_x;
    result.vec_z = vec_z;
    return result;
}