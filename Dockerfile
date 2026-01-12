# Build stage for dependencies and building packages
FROM ros:humble-ros-base as builder

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# Install ROS 2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-nav2-* \
    ros-${ROS_DISTRO}-gazebo-ros2-control \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-rviz-imu-plugin \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# Create workspace directory
WORKDIR /ros2_ws

# Copy package files
COPY automated_robot_planning/ src/automated_robot_planning/
COPY shelfino_node/ src/shelfino_node/
COPY map_pkg/ src/map_pkg/
COPY obstacles_msgs/ src/obstacles_msgs/
COPY shelfino_description/ src/shelfino_description/
COPY shelfino_gazebo/ src/shelfino_gazebo/
COPY shelfino_navigation/ src/shelfino_navigation/
COPY package.xml src/
COPY CMakeLists.txt src/

# Install Python dependencies
COPY requirements.txt .
RUN pip3 install --no-cache-dir -r requirements.txt

# Install package dependencies and build
RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install --cmake-args \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

# Runtime stage
FROM ros:humble-ros-base

# Install runtime dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-nav2-* \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-gazebo-ros2-control \
    && rm -rf /var/lib/apt/lists/*

# Copy built packages and install scripts
COPY --from=builder /ros2_ws/install /ros2_ws/install
COPY --from=builder /ros2_ws/requirements.txt /ros2_ws/

# Install Python runtime dependencies
RUN pip3 install --no-cache-dir -r /ros2_ws/requirements.txt

# Source the workspace in bashrc
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Set up entrypoint
COPY ./entrypoint.sh /
RUN chmod +x /entrypoint.sh

# Set the working directory
WORKDIR /ros2_ws

# Set the entrypoint
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]