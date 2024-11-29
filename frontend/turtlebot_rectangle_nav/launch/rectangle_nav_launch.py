import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlebot_rectangle_nav',
            executable='main.py',
            name='robot_nav_node',
            output='screen'
        )
    ])
