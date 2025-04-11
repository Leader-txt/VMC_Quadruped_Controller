#include "kinematics.h"
#include "rclcpp/rclcpp.hpp"
#include <thread>
#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"
#include <ctime>
#include <iostream>

MotorCmd cmd;
SerialPort* serial;
float motor_id_for_legs[4][2] = {{3,0},{2,1},{6,5},{7,4}};

MotorData SendMsg(uint id,uint mode,float tau,float kp,float kd,float q,float dq);
void control_leg(uint id);