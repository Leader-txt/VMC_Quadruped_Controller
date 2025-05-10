from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
 
def generate_launch_description():

    imu_config = os.path.join(
        get_package_share_directory('yesense_std_ros2'),
        'config',
        'yesense_config.yaml',
    ),

    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            parameters=[{
                'autorepeat_rate': 0.0,
            }]
        ),
        Node(
            package='yesense_std_ros2',
            executable='yesense_node_publisher',
            name='yesense_pub',
            parameters=[imu_config],
            output='screen',
        ),
        Node(
            package='vmc_quadruped_controller',
            executable='foots',
            output='log'
        ),
        Node(
            package='vmc_quadruped_controller',
            executable='navigator',
            output='log'
        )
    ])