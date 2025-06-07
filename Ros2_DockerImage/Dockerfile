FROM osrf/ros:humble-desktop-full

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    git \
    && rm -rf /var/lib/apt/lists/*
# Install ROS dependencies
RUN rosdep init && \
    rosdep update
COPY SRC/ /ROS_APP/
