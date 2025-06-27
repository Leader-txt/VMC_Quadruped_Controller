#include <rclcpp/rclcpp.hpp>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "pid.h"
#include "sensor_msgs/msg/joy.hpp"
#include <iostream>
#include <fstream>
#include <string>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "vmc_quadruped_controller/msg/move_cmd.hpp"

using namespace std;

typedef struct {
    double start_x;
    double start_y;
    double end_x;
    double end_y;
} Path;

std::string POSITION_FILE="/nav_liner.pos";

#define AXES_VERTICAL 7 // positive is left
#define AXES_HORIZONTAL 6 // positive is up

#define AXES_SET_POS AXES_HORIZONTAL
#define AXES_RUN AXES_VERTICAL

#define STEP_X_KD 0.05
#define STEP_Y_KD 0.03