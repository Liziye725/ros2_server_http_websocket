cmake_minimum_required(VERSION 3.5)

project(turtlebot_rectangle_nav)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add Python executables
install(PROGRAMS
  src/main.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
