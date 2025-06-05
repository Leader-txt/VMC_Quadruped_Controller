#include "nav_liner.h"

class Nav_liner : public rclcpp::Node{
    private:
        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription;
    public:
        Nav_liner()
        : Node("nav_liner")
        {
            // Initialize the node
            RCLCPP_INFO(this->get_logger(), "nav_liner node initialized");
            joy_subscription = this->create_subscription<sensor_msgs::msg::Joy>("joy",10,std::bind(&Nav_liner::joy_callback,this,std::placeholders::_1));
            if(!isFileExists(POSITION_FILE)){
                RCLCPP_INFO(get_logger(),"file not exists");
                Path path= {0,0,1,0};
                ofstream ofs(POSITION_FILE,ios::out|ios::binary);
                ofs.write((char*)&path,sizeof(Path));
            }
            else{
                ifstream inFile(POSITION_FILE,ios::in|ios::binary);
                Path path;
                inFile.read((char*)&path,sizeof(Path));
                RCLCPP_INFO(get_logger(),"file exists, start_x:%.2f start_y:%.2f end_x:%.2f end_y:%.2f",path.start_x,path.start_y,path.end_x,path.end_y);
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
            }
            if(msg->axes[AXES_SET_POS] == -1){
                RCLCPP_INFO(get_logger(),"set end position");
            }
        }
        bool isFileExists(std::string& name) {
            std::ifstream f(name.c_str());
            return f.good();
        }
    };
int main(int argc,char* argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<Nav_liner>());
    rclcpp::shutdown();
    return 0;
}