#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ros_gz_sim_pkg = get_package_share_directory('ros_gz_sim')
    shelfino_gaze_pkg = get_package_share_directory('shelfino_gazebo')
    
    # Use the simple test world
    gazebo_world_file = os.path.join(shelfino_gaze_pkg, 'worlds', 'test_simple.world')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            name='gazebo_world_file',
            default_value=gazebo_world_file,
            description='World used in the gazebo simulation'
        ),
        
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
    ]) 