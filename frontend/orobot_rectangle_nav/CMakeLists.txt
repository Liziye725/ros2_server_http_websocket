cmake_minimum_required(VERSION 3.5)

project(orobot_rectangle_nav)

find_package(ament_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)

# Add Python executables
install(PROGRAMS
  src/orobot_nav_node.py
  src/send_rectangle_coordinate.py
  src/websocket_manager.py
  src/main.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
