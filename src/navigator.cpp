#include "navigator.h"

class Navigator : public rclcpp::Node{
    private:
        rclcpp::Subscription<yesense_interface::msg::EulerOnly>::SharedPtr imu_subscription_;
        rclcpp::Subscription<vmc_quadruped_controller::msg::AngleReq>::SharedPtr angle_subscription_;
        rclcpp::Publisher<vmc_quadruped_controller::msg::MoveCmd>::SharedPtr move_pub;
        vmc_quadruped_controller::msg::MoveCmd move_cmd;
        vmc_quadruped_controller::msg::AngleReq angle_req;
        vmc_quadruped_controller::msg::AngleCb angle_cb;
        float target_angle = 360
            ,yaw
            ,pitch
            ,roll;
        bool need_spin = false;
        PID *pid;
    public:
        Navigator()
        : Node("navigator")
        {
            // Initialize the node
            RCLCPP_INFO(this->get_logger(), "Navigator node initialized");
            // Create a subscription to the IMU data
            imu_subscription_ = this->create_subscription<yesense_interface::msg::EulerOnly>(
                "euler_only", 10, std::bind(&Navigator::imu_callback, this, std::placeholders::_1));
            angle_subscription_ = this->create_subscription<vmc_quadruped_controller::msg::AngleReq>(
                "angle_req", 10, std::bind(&Navigator::angle_callback, this, std::placeholders::_1));
            move_pub = this->create_publisher<vmc_quadruped_controller::msg::MoveCmd>("move_cmd", 10);
            pid = new PID(0.01,0,0.09,0.1);
        }
    private:
        void angle_callback(const vmc_quadruped_controller::msg::AngleReq::SharedPtr msg)
        {
            // Process the angle request
            target_angle = yaw + msg->angle;
            need_spin = true;
            RCLCPP_INFO(this->get_logger(), "Received target angle: %.2f", target_angle);
        }
        void imu_callback(const yesense_interface::msg::EulerOnly::SharedPtr msg)
        {
            // Process the IMU data
            pitch = msg->euler.pitch;
            roll = msg->euler.roll;
            yaw = msg->euler.yaw;

            // Log the IMU data
            // RCLCPP_INFO(this->get_logger(), "IMU Data: Pitch: %.2f, Roll: %.2f, Yaw: %.2f", pitch, roll, yaw);
            if(need_spin){
                move_cmd.step_x = pid->calculate(target_angle,yaw);
                move_pub->publish(move_cmd);
                RCLCPP_INFO(this->get_logger(), "step_x: %.2f", move_cmd.step_x);
                if(abs(target_angle - yaw) < 1){
                    move_cmd.step_x = 0;
                    move_pub->publish(move_cmd);
                    need_spin = false;
                }
            }
        }
};

int main(int argc,char* argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<Navigator>());
    rclcpp::shutdown();
    return 0;
}