# Use ROS2 Humble as base image
FROM ros:humble-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-rosinstall-generator \
    python3-wstool \
    python3-rosinstall \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# Create workspace directory
WORKDIR /ros2_ws

# Copy all packages to the workspace src directory
COPY automated_robot_planning/ /ros2_ws/src/automated_robot_planning/
COPY shelfino_node/ /ros2_ws/src/shelfino_node/
COPY map_pkg/ /ros2_ws/src/map_pkg/
COPY obstacles_msgs/ /ros2_ws/src/obstacles_msgs/
COPY shelfino_description/ /ros2_ws/src/shelfino_description/
COPY shelfino_gazebo/ /ros2_ws/src/shelfino_gazebo/
COPY shelfino_navigation/ /ros2_ws/src/shelfino_navigation/

# Install Python dependencies (if any requirements.txt exists)
RUN if [ -f /ros2_ws/src/map_pkg/requirements.txt ]; then \
        pip3 install -r /ros2_ws/src/map_pkg/requirements.txt; \
    fi

# Install package dependencies
RUN cd /ros2_ws && \
    rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
RUN cd /ros2_ws && \
    . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

# Source the workspace in bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Set the default command
CMD ["/bin/bash"]