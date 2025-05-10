#include "pid.h"

double clip(double var,double maxVal){
    if(var < - maxVal){
        return -maxVal;
    }
    if(var > maxVal){
        return maxVal;
    }
    return var;
}

PID::PID(double p,double i,double d,double max):
    kp(p),ki(i),kd(d),integral(0),prev_error(0),max_val(max){}

double PID::calculate(double target,double now){
    double error = target - now;
    double dt_error = error - prev_error;
    prev_error = error;
    integral += error;
    double result = kp*error + ki*integral + kd*dt_error;
    return clip(result,max_val);
}