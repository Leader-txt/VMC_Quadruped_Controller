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
#include "vmc_quadruped_controller/msg/move_cmd.hpp"
#include "yesense_interface/msg/euler_only.hpp"
#include "cycloid.h"
#define _USE_MATH_DEFINES 
#define STAND_UP_ANGLE_1 66
#define STAND_UP_ANGLE_2 49
#define OUTER_MOTOR_OFFEST -(STAND_UP_ANGLE_1+STAND_UP_ANGLE_2)/180.0*M_PI
#define INNER_MOTOR_OFFEST M_PI-(STAND_UP_ANGLE_1-STAND_UP_ANGLE_2)/180.0*M_PI
#define INIT_BODY_HEIGHT 0.223
#define IMU_PITCH_KD 0.01
#define CTRL_X_KD 1
#define CTRL_Y_KD 1
#define INIT_KP_X 2000
#define INIT_KI_X 0
#define INIT_KD_X 80
#define INIT_KP_Y 2000
#define INIT_KI_Y 0
#define INIT_KD_Y 80
#define BTN_A 0
#define BTN_B 1
#define BTN_X 2
#define BTN_Y 3
#define BTN_LB 4
#define BTN_RB 5
#define STAND_UP_BTN 7
#define SIT_DOWN_BTN 6
#define CTR_TYPE_BTN BTN_RB
#define USE_IMU_PITCH_BTN BTN_LB
#define JUMP_BTN BTN_A
#define JUMP_HIGH_BTN BTN_Y
#define NORMAL_GAIT_BTN BTN_X
#define LOWER_GAIT_BTN BTN_B

#define NORMAL_GAIT_PERIOD 0.4
#define NORMAL_GAIT_BODY_HEIGHT INIT_BODY_HEIGHT
#define NORMAL_GAIT_FLIGHT_PERCENT 0.5
#define NORMAL_GAIT_HEIGHT 0.1

#define LOWER_GAIT_PERIOD 0.6
#define LOWER_GAIT_BODY_HEIGHT 0.18
#define LOWER_GAIT_FLIGHT_PERCENT 0.4
#define LOWER_GAIT_HEIGHT 0.06

#define AXES_LX 0
#define AXES_LY 1
#define TRIGGER_LEFT 2
#define AXES_RX 3
#define AXES_RY 4
#define TRIGGER_RIGHT 5

// Front
// 2 1
// 3 0
// {outer_motor,inner_motor}
int motor_id_for_legs[4][2] = {{0,3},{1,2},{6,5},{7,4}};
int motor_dir[8] = {1,1,-1,-1,1,1,-1,-1};
void SendMsg(MotorData* data,MotorCmd* cmd,SerialPort* serial);
void control_leg(uint id);

#endif