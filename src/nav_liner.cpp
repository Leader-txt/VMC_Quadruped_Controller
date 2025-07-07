#include "nav_liner.h"

class Nav_liner : public rclcpp::Node{
    private:
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription;
        Path path;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        rclcpp::TimerBase::SharedPtr timer_;
        geometry_msgs::msg::TransformStamped transform;
        vmc_quadruped_controller::msg::MoveCmd move_cmd;
        rclcpp::Publisher<vmc_quadruped_controller::msg::MoveCmd>::SharedPtr move_pub;
        bool need_move = false;
        PID *pid_angle,*pid_pos;
        float step_y = 0,step_x = 0;
    public:
        Nav_liner()
        : Node("nav_liner")
        {
            // Initialize the node
            RCLCPP_INFO(this->get_logger(), "nav_liner node initialized");
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&Nav_liner::timer_callback, this));
            joy_subscription = this->create_subscription<sensor_msgs::msg::Joy>("joy",10,std::bind(&Nav_liner::joy_callback,this,std::placeholders::_1));
            
            move_pub = this->create_publisher<vmc_quadruped_controller::msg::MoveCmd>("move_cmd", 10);
            string pkg_path = ament_index_cpp::get_package_share_directory("vmc_quadruped_controller");
            pid_angle = new PID(0.01,0,0.09,0.1);
            pid_pos = new PID(1,0,0.09,0.2);
            if(!isFileExists(pkg_path + POSITION_FILE)){
                RCLCPP_INFO(get_logger(),"file not exists");
                path= {0,0,1,0};
                writePathToFile();
            }
            else{
                readPathFromFile();
            }
        }
    private:
        void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg){
            // // 打印轴和按钮状态
            // RCLCPP_INFO(this->get_logger(), "收到Joy消息:");
            
            // // 打印所有轴
            // RCLCPP_INFO(this->get_logger(), "轴:");
            // for (size_t i = 0; i < msg->axes.size(); ++i) {
            // RCLCPP_INFO(this->get_logger(), "  轴[%zu]: %.2f", i, msg->axes[i]);
            // }
            
            // // 打印所有按钮
            // RCLCPP_INFO(this->get_logger(), "按钮:");
            // for (size_t i = 0; i < msg->buttons.size(); ++i) {
            // RCLCPP_INFO(this->get_logger(), "  按钮[%zu]: %d", i, msg->buttons[i]);
            // }
            if(msg->axes[AXES_SET_POS] == 1){
                RCLCPP_INFO(get_logger(),"set start position");
                path.start_x = transform.transform.translation.x;
                path.start_y = transform.transform.translation.y;
                writePathToFile();
            }
            if(msg->axes[AXES_SET_POS] == -1){
                RCLCPP_INFO(get_logger(),"set end position");
                path.end_x = transform.transform.translation.x;
                path.end_y = transform.transform.translation.y;
                writePathToFile();
            }
            if(msg->axes[AXES_RUN] == 1){
                RCLCPP_INFO(get_logger(),"start run");
                need_move = true;
                step_y = 0;
                step_x = 0;
            }
            if(msg->axes[AXES_RUN] == -1){
                RCLCPP_INFO(get_logger(),"stop run");
                need_move = false;
                step_y = 0;
                step_x = 0;
                move_cmd.step_x = 0;
                move_cmd.step_y = 0;
                move_pub->publish(move_cmd);
            }
        }
        void timer_callback()
        {
            try {
                // 查找从"base_link"到"odom"的变换
                transform = 
                    tf_buffer_->lookupTransform(
                        "map",       // 目标坐标系
                        "body",  // 源坐标系
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
                
                // get distance from the line
                if(need_move){
                    // 判断在线的左边还是右边
                    double k = (path.end_y - path.start_y) / (path.end_x - path.start_x + 1e-7);
                    double b = path.start_y - k * path.start_x;
                    double compared_y = k * transform.transform.translation.x + b;
                    double dir = -1;
                    if(compared_y < transform.transform.translation.y){
                        RCLCPP_INFO(get_logger(),"on the left side of the line");
                        dir = 1;
                    }
                    else{
                        RCLCPP_INFO(get_logger(),"on the right side of the line");
                    }
                    double distance = abs(k * transform.transform.translation.x - transform.transform.translation.y + b) / sqrt(k * k + 1);
                    RCLCPP_INFO(get_logger(),"distance to the line: %.2f",distance);
                    // calculate the angle to the line

                    geometry_msgs::msg::Point direction_point;
                    direction_point.x = transform.transform.translation.x+1;
                    direction_point.y = transform.transform.translation.y+k;
                    geometry_msgs::msg::Vector3 relative_vector = calculate_relative_vector(transform,direction_point);

                    float angle = atan2(relative_vector.y, relative_vector.x);
                    angle = angle/M_PI * 180.0;
                    RCLCPP_INFO(get_logger(),"angle to the line %.2f",angle);
                    double target_angle = distance*30*dir;
                    RCLCPP_INFO(get_logger(),"target angle %.2f",target_angle);
                    if(abs(target_angle - angle)>5){
                        // step_x = step_x + STEP_X_KD*(-pid_angle->calculate(target_angle,angle) - step_x);
                        move_cmd.step_x = -pid_angle->calculate(target_angle,angle);
                    }
                    else{
                        // step_x = step_x + STEP_X_KD*(0-step_x);
                        move_cmd.step_x = 0;
                    }
                    // move_cmd.step_x = step_x;
                    // move_cmd.step_x = -pid_angle->calculate(target_angle,angle);
                    if(transform.transform.translation.x < path.end_x){
                        step_y = step_y + STEP_Y_KD*(-1 - step_y);
                        move_cmd.step_y = step_y;
                    }
                    else{
                        step_y = 0;
                        move_cmd.step_y = step_y;
                        move_cmd.step_x = 0;
                        need_move = false;
                    }
                    move_pub->publish(move_cmd);
                }
            }
            catch (std::exception &ex) {
                RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
            }
        }
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
        bool isFileExists(string name) {
            std::ifstream f(name.c_str());
            return f.good();
        }
        void writePathToFile() {
            string pkg_path = ament_index_cpp::get_package_share_directory("vmc_quadruped_controller");
            ofstream ofs(pkg_path + POSITION_FILE,ios::out|ios::binary);
            ofs.write((char*)&path,sizeof(Path));
            RCLCPP_INFO(get_logger(),"write path to file, start_x:%.2f start_y:%.2f end_x:%.2f end_y:%.2f",path.start_x,path.start_y,path.end_x,path.end_y);
        }
        void readPathFromFile() {
            string pkg_path = ament_index_cpp::get_package_share_directory("vmc_quadruped_controller");
            ifstream inFile(pkg_path + POSITION_FILE,ios::in|ios::binary);
            inFile.read((char*)&path,sizeof(Path));
            RCLCPP_INFO(get_logger(),"read path from file, start_x:%.2f start_y:%.2f end_x:%.2f end_y:%.2f",path.start_x,path.start_y,path.end_x,path.end_y);
        };
    };

int main(int argc,char* argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<Nav_liner>());
    rclcpp::shutdown();
    return 0;
}