#include "navigator.h"

class Navigator : public rclcpp::Node{
    private:
        rclcpp::Subscription<yesense_interface::msg::EulerOnly>::SharedPtr imu_subscription_;
        rclcpp::Subscription<vmc_quadruped_controller::msg::AngleReq>::SharedPtr angle_subscription_;
        rclcpp::Subscription<vmc_quadruped_controller::msg::PosReq>::SharedPtr pos_subscription_;
        rclcpp::Publisher<vmc_quadruped_controller::msg::MoveCmd>::SharedPtr move_pub;
        vmc_quadruped_controller::msg::MoveCmd move_cmd;
        vmc_quadruped_controller::msg::AngleReq angle_req;
        vmc_quadruped_controller::msg::AngleCb angle_cb;
        vmc_quadruped_controller::msg::PosReq pos_req;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        rclcpp::TimerBase::SharedPtr timer_;
        geometry_msgs::msg::TransformStamped transform;
        geometry_msgs::msg::Point target_point_in_map;
        float target_angle = 360
            ,yaw
            ,pitch
            ,roll;
        bool need_spin = false
            ,need_move = false;
        PID *pid;
    public:
        Navigator()
        : Node("navigator")
        {
            // Initialize the node
            RCLCPP_INFO(this->get_logger(), "Navigator node initialized");
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&Navigator::timer_callback, this));
            pos_subscription_ = this->create_subscription<vmc_quadruped_controller::msg::PosReq>(
                "pos_req", 10, std::bind(&Navigator::pos_callback, this, std::placeholders::_1));
            // Create a subscription to the IMU data
            // imu_subscription_ = this->create_subscription<yesense_interface::msg::EulerOnly>(
            //     "euler_only", 10, std::bind(&Navigator::imu_callback, this, std::placeholders::_1));
            // angle_subscription_ = this->create_subscription<vmc_quadruped_controller::msg::AngleReq>(
            //     "angle_req", 10, std::bind(&Navigator::angle_callback, this, std::placeholders::_1));
            // move_pub = this->create_publisher<vmc_quadruped_controller::msg::MoveCmd>("move_cmd", 10);
            pid = new PID(0.01,0,0.09,0.1);
        }
    private:
        geometry_msgs::msg::Vector3 calculate_relative_vector(
        const geometry_msgs::msg::TransformStamped& transform_me_in_map,
        const geometry_msgs::msg::Point& target_point_in_map)
        {
            // 1. 提取map到自身坐标系的变换
            tf2::Transform tf_map_to_me;
            tf2::fromMsg(transform_me_in_map.transform, tf_map_to_me);
            
            // 2. 计算自身坐标系到map的变换（求逆）
            tf2::Transform tf_me_to_map = tf_map_to_me.inverse();
            
            // 3. 将目标点转换为tf2::Vector3
            tf2::Vector3 target_vec(
                target_point_in_map.x,
                target_point_in_map.y,
                target_point_in_map.z);
            
            // 4. 将目标点转换到自身坐标系
            tf2::Vector3 target_in_me = tf_me_to_map * target_vec;
            
            // 5. 在自身坐标系中，自身位置是原点(0,0,0)
            // 所以到目标点的向量就是目标点的坐标
            geometry_msgs::msg::Vector3 relative_vector;
            relative_vector.x = target_in_me.x();
            relative_vector.y = target_in_me.y();
            relative_vector.z = target_in_me.z();
            
            return relative_vector;
        }
        
        void pos_callback(const vmc_quadruped_controller::msg::PosReq::SharedPtr msg)
        {
            // Process the angle request
            target_point_in_map.x = msg->x;
            target_point_in_map.y = msg->y;

            RCLCPP_INFO(this->get_logger(), 
                "Transform from 'map' to 'body':\n"
                "Translation: [%.2f, %.2f, %.2f]\n"
                "Rotation: [%.2f, %.2f, %.2f, %.2f]",
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w);
            RCLCPP_INFO(this->get_logger(), "Received target pos: x:%.2f y:%.2f", msg->x,msg->y);
            geometry_msgs::msg::Vector3 relative_vector = calculate_relative_vector(transform,target_point_in_map);
            RCLCPP_INFO(this->get_logger(), 
                "Relative vector from me to target in my frame: [%.2f, %.2f, %.2f]",
                relative_vector.x, relative_vector.y, relative_vector.z);
        }
        void timer_callback()
        {
            try {
                // 查找从"base_link"到"odom"的变换
                transform = 
                    tf_buffer_->lookupTransform(
                        "body",       // 目标坐标系
                        "map",  // 源坐标系
                        tf2::TimePointZero);  // 获取最新可用的变换
                
                // RCLCPP_INFO(this->get_logger(), 
                //     "Transform from 'map' to 'body':\n"
                //     "Translation: [%.2f, %.2f, %.2f]\n"
                //     "Rotation: [%.2f, %.2f, %.2f, %.2f]",
                //     transform.transform.translation.x,
                //     transform.transform.translation.y,
                //     transform.transform.translation.z,
                //     transform.transform.rotation.x,
                //     transform.transform.rotation.y,
                //     transform.transform.rotation.z,
                //     transform.transform.rotation.w);
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
            }
        }
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