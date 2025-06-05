cmake_minimum_required(VERSION 3.8)
project(vmc_quadruped_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set (CMAKE_CXX_STANDARD 11)
include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yesense_interface REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorCmd.msg"
  "msg/MotorData.msg"
  "msg/MoveCmd.msg"
  "msg/AngleReq.msg"
  "msg/AngleCb.msg"
  "msg/PosReq.msg"
)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
add_executable(motor src/motor.cpp)
add_executable(foots src/foot_controller.cpp src/kinematics.cpp src/cycloid.cpp)
add_executable(navigator src/navigator.cpp src/pid.cpp)
add_executable(nav_liner src/nav_liner.cpp src/pid.cpp)

ament_target_dependencies(motor rclcpp)
ament_target_dependencies(foots rclcpp sensor_msgs yesense_interface)
ament_target_dependencies(navigator rclcpp sensor_msgs yesense_interface Eigen3 tf2_ros geometry_msgs tf2_geometry_msgs)
ament_target_dependencies(nav_liner rclcpp sensor_msgs tf2_ros geometry_msgs tf2_geometry_msgs)
target_link_libraries(motor ${PROJECT_SOURCE_DIR}/lib/libUnitreeMotorSDK_Arm64.so)
target_link_libraries(foots ${PROJECT_SOURCE_DIR}/lib/libUnitreeMotorSDK_Arm64.so)

target_link_libraries(motor "${cpp_typesupport_target}")
target_link_libraries(foots "${cpp_typesupport_target}")
target_link_libraries(navigator "${cpp_typesupport_target}")
target_link_libraries(nav_liner "${cpp_typesupport_target}")
install(TARGETS
  motor
  foots
  navigator
  nav_liner
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/lib/
  DESTINATION lib/
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
