FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*
    
RUN pip3 install numpy fastapi uvicorn

RUN rosdep init && rosdep update


# Set the working directory in the container
WORKDIR /frontend

# Copy the project files into the container (from the host to /frontend inside the container)
COPY . /frontend

# Expose port 5001 for FastAPI WebSocket server
EXPOSE 5001

RUN . /opt/ros/humble/setup.sh && colcon build


CMD ["bash", "-c", "source /opt/ros/humble/setup.sh && bash && ros2 run orobot_rectangle_nav main"]
