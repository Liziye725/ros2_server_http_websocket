FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-turtlebot3* \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init && rosdep update

COPY server_node /server_node
WORKDIR /server_node

RUN . /opt/ros/humble/setup.sh && colcon build

CMD ["bash", "-c", "source /opt/ros/humble/setup.sh && bash"]
