#ifndef __FOOT_CONTROLLER_H___
#define __FOOT_CONTROLLER_H___
#include "kinematics.h"
#include "rclcpp/rclcpp.hpp"
#include <thread>
#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"
#include <chrono>
#include <iostream>
#include <mutex>
#include <cstdio>
#define _USE_MATH_DEFINES 
#define STAND_UP_ANGLE_1 66
#define STAND_UP_ANGLE_2 49
#define OUTER_MOTOR_OFFEST -(STAND_UP_ANGLE_1+STAND_UP_ANGLE_2)/180.0*M_PI
#define INNER_MOTOR_OFFEST M_PI-(STAND_UP_ANGLE_1-STAND_UP_ANGLE_2)/180.0*M_PI
std::mutex lock;
MotorCmd cmd;
SerialPort* serial;
// Front
// 2 1
// 3 0
// {outer_motor,inner_motor}
int motor_id_for_legs[4][2] = {{0,3},{1,2},{6,5},{7,4}};
int motor_dir[8] = {1,1,-1,-1,1,1,-1,-1};
void SendMsg(MotorData* data,MotorCmd* cmd);
void control_leg(uint id);
#endif