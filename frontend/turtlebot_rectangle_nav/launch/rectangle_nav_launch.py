from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlebot_rectangle_nav',
            executable='rectangle_nav_node',
            name='rectangle_nav_node',
            output='screen',
        ),
    ])
