#Build the Docker image:

docker build -t automated-robot-planning .

#Run with additional volume mounts (if you want to persist data or share files):

docker run -it --rm \
  --env="DISPLAY=$DISPLAY" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="/home/fariha/ros2_ws/src/automated_robot_planning:/ros2_ws/src:rw" \
  --network host \
  automated-robot-planning


#Once inside the container, you can:
Check that everything is built correctly:

ros2 pkg list | grep -E "(automated_robot_planning|shelfino|map_pkg|obstacles_msgs)"

#Run your launch files:

ros2 launch automated_robot_planning evacuation.launch.py