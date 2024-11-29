# Set the minimum required version of CMake
cmake_minimum_required(VERSION 3.5)

# Define the project name
project(turtlebot_rectangle_nav)

# Find and include the necessary ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# Add an executable for the ROS 2 node
add_executable(robot_nav_node src/turtlebot_rectangle_nav/robot_nav_node.py)

# Link the ROS 2 dependencies to the executable
ament_target_dependencies(robot_nav_node rclpy std_msgs)

# Install the executable to the proper directory
install(TARGETS
  robot_nav_node
  DESTINATION lib/${PROJECT_NAME}
)

# Call ament_package to generate the package build information
ament_package()
