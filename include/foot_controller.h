#ifndef __FOOT_CONTROLLER_H___
#define __FOOT_CONTROLLER_H___
#include <thread>
#include <chrono>
#include <iostream>
#include <mutex>
#include <cstdio>
#include "kinematics.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"
#include "cycloid.h"
#define _USE_MATH_DEFINES 
#define STAND_UP_ANGLE_1 66
#define STAND_UP_ANGLE_2 49
#define OUTER_MOTOR_OFFEST -(STAND_UP_ANGLE_1+STAND_UP_ANGLE_2)/180.0*M_PI
#define INNER_MOTOR_OFFEST M_PI-(STAND_UP_ANGLE_1-STAND_UP_ANGLE_2)/180.0*M_PI
#define BODY_HEIGHT 0.223
#define STAND_UP_BTN 7
#define SIT_DOWN_BTN 6
#define AXES_LX 0
#define AXES_LY 1
#define TRIGGER_LEFT 2
#define AXES_RX 3
#define AXES_RY 4
#define TRIGGER_RIGHT 5

std::mutex lock;
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