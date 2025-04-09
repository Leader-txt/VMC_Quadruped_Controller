#include "rclcpp/rclcpp.hpp"
#include <unistd.h>
#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"
#include "vmc_quadruped_controller/msg/motor_cmd.hpp"
#include "vmc_quadruped_controller/msg/motor_data.hpp"
#include <memory>

class Motor : public rclcpp::Node
{
    rclcpp::Publisher<vmc_quadruped_controller::msg::MotorData>::SharedPtr publisher;
    rclcpp::Subscription<vmc_quadruped_controller::msg::MotorCmd>::SharedPtr subscription;
    SerialPort* serial;
    public: Motor(): Node("motor"){
        serial = new SerialPort("/dev/ttyS7");
        publisher = this->create_publisher<vmc_quadruped_controller::msg::MotorData>("motor_data",10);
        subscription = this->create_subscription<vmc_quadruped_controller::msg::MotorCmd>(
            "motor_cmd", 10, std::bind(&Motor::motor_cmd_callback, this, std::placeholders::_1));
    }
    private:
        void motor_cmd_callback(const vmc_quadruped_controller::msg::MotorCmd::SharedPtr msg)const{
            vmc_quadruped_controller::msg::MotorData motorData = vmc_quadruped_controller::msg::MotorData();
            std::shared_ptr<MotorCmd> cmd = std::make_shared<MotorCmd>();
            std::shared_ptr<MotorData> data = std::make_shared<MotorData>();
            cmd->motorType = MotorType::GO_M8010_6;
            data->motorType = MotorType::GO_M8010_6;
            cmd->id = msg->id;
            cmd->mode = msg->mode;
            cmd->tau = msg->tau;
            cmd->kp = msg->kp;
            cmd->kd = msg->kd;
            cmd->q = msg->pos;
            cmd->dq = msg->vel;
            serial->sendRecv(cmd.get(),data.get());
            motorData.id = data->motor_id;
            motorData.mode = data->mode;
            motorData.tau = data->tau;
            motorData.vel = data->dq;
            motorData.pos = data->q;
            motorData.temp = data->temp;
            motorData.error = data->merror;
            motorData.force = data->footForce;
            publisher->publish(motorData);
        }
};

int main(int argc,char* argv[]){
	rclcpp::init(argc,argv);
	rclcpp::spin(std::make_shared<Motor>());
	rclcpp::shutdown();
    return 0;
}