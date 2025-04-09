#include "rclcpp/rclcpp.hpp"
#include <unistd.h>
#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"
#include "vmc_quadruped_controller/msg/motor_cmd.hpp"
#include "vmc_quadruped_controller/msg/motor_data.hpp"
// #include <thread>
// #include <chrono>

class Motor : public rclcpp::Node
{
    rclcpp::Publisher<vmc_quadruped_controller::msg::MotorData>::SharedPtr publisher;
    rclcpp::Subscription<vmc_quadruped_controller::msg::MotorCmd>::SharedPtr subscription;
    SerialPort* serial;
    public: Motor(): Node("motor"){
        serial = new SerialPort("/dev/ttyS1");
        publisher = this->create_publisher<vmc_quadruped_controller::msg::MotorData>("motor_data",10);
        subscription = this->create_subscription<vmc_quadruped_controller::msg::MotorCmd>(
            "motor_cmd", 10, std::bind(&Motor::motor_cmd_callback, this, std::placeholders::_1));
        // while(rclcpp::ok()){
        //     vmc_quadruped_controller::msg::MotorData data = vmc_quadruped_controller::msg::MotorData();
        //     data.id = 0;
        //     data.mode = 1;
        //     data.tau = 0.21;
        //     data.vel = 0.11;
        //     data.pos = 0.178;
        //     data.temp = 23;
        //     data.error = 0;
        //     data.force = 134;
        //     publisher->publish(data);
        //     MotorCmd cmd;
        //     MotorData data1;
        //     cmd.motorType = MotorType::GO_M8010_6;
        //     data1.motorType = MotorType::GO_M8010_6;
        //     cmd.mode = queryMotorMode(MotorType::GO_M8010_6,MotorMode::FOC);
        //     cmd.id   = 0;
        //     cmd.kp   = 0.0;
        //     cmd.kd   = 0.01;
        //     cmd.q    = 0.0;
        //     cmd.dq   = -6.28*queryGearRatio(MotorType::GO_M8010_6);
        //     cmd.tau  = 0.0;
        //     std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        // }
    }
    private:
        void motor_cmd_callback(const vmc_quadruped_controller::msg::MotorCmd::SharedPtr msg)const{
            vmc_quadruped_controller::msg::MotorData data = vmc_quadruped_controller::msg::MotorData();
            data.id = msg->id;
            data.mode = msg->mode;
            data.tau = msg->tau;
            data.vel = msg->vel;
            data.pos = msg->pos;
            data.temp = 23;
            data.error = 0;
            data.force = 2331;
            publisher->publish(data);
        }
};

int main(int argc,char* argv[]){
	rclcpp::init(argc,argv);
	rclcpp::spin(std::make_shared<Motor>());
	rclcpp::shutdown();
    return 0;
}