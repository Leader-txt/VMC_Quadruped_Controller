from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
 
def generate_launch_description():

    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            parameters=[{
                'autorepeat_rate': 0.0,
            }]
        ),
        Node(
            package='vmc_quadruped_controller',
            executable='nav_liner',
            output='log'
        )
    ])