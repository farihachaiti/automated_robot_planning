#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PythonExpression, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')
    shelfino_desc_pkg = get_package_share_directory('shelfino_description')
    shelfino_gaze_pkg = get_package_share_directory('shelfino_gazebo')
    
    # Use the hexagon world without walls
    gazebo_world_file = os.path.join(shelfino_gaze_pkg, 'worlds', 'hexagon_world', 'model.sdf')
    
    # Robot configuration
    shelfino_id = '0'
    shelfino_name = f'shelfino{shelfino_id}'
    robot_desc = os.path.join(shelfino_desc_pkg, 'models', 'shelfino_v1.xacro')
    
    # Set Gazebo model path
    os.environ["GAZEBO_MODEL_PATH"] = os.pathsep.join([
        os.path.join(shelfino_desc_pkg, 'models'),
        os.path.join(shelfino_gaze_pkg, 'worlds')
    ])
    
    return LaunchDescription([
        SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb'),
        
        # Launch Gazebo with the world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': ['-v 4', gazebo_world_file],
                'on_exit_shutdown': 'true',
                'headless': 'false',
                'pause': 'true',
                'use_sim_time': 'true',
            }.items(),
        ),
        
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=shelfino_name,
            parameters=[{
                'robot_description': Command(['xacro ', robot_desc, ' robot_id:=', shelfino_id]),
                'use_sim_time': True,
                'publish_frequency': 50.0,
            }],
            output='screen'
        ),
        
        # Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=shelfino_name,
            parameters=[{
                'robot_description': Command(['xacro ', robot_desc, ' robot_id:=', shelfino_id]),
                'use_sim_time': True,
                'rate': 50.0,
            }],
            output='screen'
        ),
        
        # Spawn the robot
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-topic', f'/{shelfino_name}/robot_description',
                '-entity', shelfino_name,
                '-robot_namespace', shelfino_name,
                '-name', shelfino_name,
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.1',
                '-Y', '0.0',
                '--wait-ms', '2000'
            ],
            output='screen'
        ),
        
        # Bridge for communication between ROS and Gazebo
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                f'/{shelfino_name}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                f'/{shelfino_name}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                f'/{shelfino_name}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                f'/{shelfino_name}/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                f'/{shelfino_name}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                f'/{shelfino_name}/tf_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            ],
            output='screen'
        ),
        
        # Static transform publisher for map to odom
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', f'{shelfino_name}/odom'],
            output='screen'
        ),
    ]) 