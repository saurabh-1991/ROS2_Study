FROM osrf/ros:humble-desktop-full

ARG USERNAME=saurabh
ARG USER_UID=1000
ARG USER_GID=${USER_UID}

RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    git \
    && rm -rf /var/lib/apt/lists/*
# Install ROS dependencies
#RUN rosdep init && \
#    rosdep update
COPY src/ /ROS_APP/
