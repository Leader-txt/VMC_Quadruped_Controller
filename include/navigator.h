#include <rclcpp/rclcpp.hpp>
#include "pid.h"
#include "yesense_interface/msg/euler_only.hpp"
#include "vmc_quadruped_controller/msg/move_cmd.hpp"
#include "vmc_quadruped_controller/msg/angle_req.hpp"
#include "vmc_quadruped_controller/msg/angle_cb.hpp"
#include <cmath>