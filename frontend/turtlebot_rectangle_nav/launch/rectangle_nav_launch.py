import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Launch the ROS 2 node
        Node(
            package='turtlebot_rectangle_nav',  # ROS package containing robot_nav_node
            executable='robot_nav_node',        # The executable for the ROS node
            name='robot_nav_node',
            output='screen',
        ),
        
        # Launch FastAPI server in a separate process (running main.py)
        ExecuteProcess(
            cmd=['python3', '../src/main.py'],  # Specify the path to your main.py file
            name='fastapi_server',
            output='screen',
        ),
    ])
