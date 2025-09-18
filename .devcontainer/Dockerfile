# File: Dockerfile
FROM osrf/ros:humble-desktop-full

# Avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Install ROS2 packages for GUIs + common robots
RUN apt-get update && apt-get install -y \
    python3-pip python3-colcon-common-extensions \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    ros-humble-rqt-graph \
    ros-humble-rviz2 \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-turtlebot3-gazebo \
    build-essential git curl wget vim \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages for sensor logging + analysis
RUN pip3 install numpy pandas opencv-python matplotlib


# Default workdir
WORKDIR /workspace

# Set default shell and ROS2 source
SHELL ["/bin/bash", "-c"]