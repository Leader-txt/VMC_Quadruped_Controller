from launch import LaunchDescription
from launch_ros.actions import Node
 
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
            executable='foots',
            output='log'
        )
    ])