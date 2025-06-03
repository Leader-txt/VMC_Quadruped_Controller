#include "nav_liner.h"

class Nav_liner : public rclcpp::Node{

    public:
        Nav_liner()
        : Node("nav_liner")
        {
            // Initialize the node
            RCLCPP_INFO(this->get_logger(), "nav_liner node initialized");
        }
    };

int main(int argc,char* argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<Nav_liner>());
    rclcpp::shutdown();
    return 0;
}