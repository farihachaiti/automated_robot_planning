# Automated Robot Planning with Docker

This document provides instructions for building and running the automated robot planning system using Docker.

## Prerequisites

- Docker installed on your system
- NVIDIA Container Toolkit (if using GPU acceleration)
- X11 server (for GUI applications like RViz and Gazebo)

## Building the Docker Image

To build the Docker image, run the following command from the project root directory:

```bash
docker build -t automated-robot-planning .
```

## Running the Container

### Basic Usage

```bash
docker run -it --rm \
  --env="DISPLAY=$DISPLAY" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --network host \
  automated-robot-planning
```

### With GPU Support (if available)

```bash
docker run -it --rm \
  --env="DISPLAY=$DISPLAY" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --gpus all \
  --network host \
  automated-robot-planning
```

### With Development Mount (for development)

This mounts your local source code into the container, allowing you to make changes without rebuilding:

```bash
docker run -it --rm \
  --env="DISPLAY=$DISPLAY" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$(pwd):/ros2_ws/src/automated_robot_planning" \
  --network host \
  automated-robot-planning
```

## Using the Container

Once inside the container, you can:

1. Verify the installation:
   ```bash
   ros2 pkg list | grep -E "(automated_robot_planning|shelfino|map_pkg|obstacles_msgs)"
   ```

2. Build the workspace (if using development mount):
   ```bash
   cd /ros2_ws && colcon build --symlink-install
   source install/setup.bash
   ```

3. Run the evacuation simulation:
   ```bash
   ros2 launch automated_robot_planning evacuation.launch.py
   ```

## Troubleshooting

### GUI Applications Not Displaying
If RViz or Gazebo windows don't appear, try these steps:
1. Ensure your X server allows connections from Docker:
   ```bash
   xhost +local:docker
   ```
2. Check that the DISPLAY environment variable is set correctly

### Network Issues
If you encounter network-related issues, try running with `--network host` as shown in the examples above.

### Performance Issues
For better performance with Gazebo, consider:
1. Using the `--gpus all` flag if you have an NVIDIA GPU
2. Limiting the resources used by Gazebo in your launch files