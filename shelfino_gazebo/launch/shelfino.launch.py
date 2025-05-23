#!/usr/bin/env python3
#
# Author: Placido Falqueto   placido.falqueto [at] unitn.it
# Maintainer: Enrico Saccon  enrico.saccon [at] unitn.it

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import logging, launch.logging

def print_env(context):
    print(__file__)
    for key in context.launch_configurations.keys():
        print("\t", key, context.launch_configurations[key])
    return

def check_exists(context):
    if not os.path.exists(context.launch_configurations['gazebo_world_file']):
        raise Exception("[{}] Gazebo world file `{}` does not exist".format(__file__, context.launch_configurations['gazebo_world_file']))
        
    if context.launch_configurations['use_rviz']=="true" and \
        not os.path.exists(context.launch_configurations['rviz_config_file']):
        raise Exception("[{}] Rviz configuration `{}` does not exist".format(__file__, context.launch_configurations['use_rviz']))

    return

def generate_launch_description():
    # launch.logging.launch_config.level = logging.DEBUG
    
    ros_gz_sim_pkg    = get_package_share_directory('ros_gz_sim')  # Updated for Gazebo Harmonic
    shelfino_desc_pkg = get_package_share_directory('shelfino_description')
    shelfino_gaze_pkg = get_package_share_directory('shelfino_gazebo')
    shelfino_nav2_pkg = get_package_share_directory('shelfino_navigation')

    qt_qpa_platform = SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb')
    use_sim_time      = LaunchConfiguration('use_sim_time', default='true')
    shelfino_id       = LaunchConfiguration('shelfino_id', default='G')
    use_gui           = LaunchConfiguration('use_gui')
    use_rviz          = LaunchConfiguration('use_rviz')
    spawn_shelfino    = LaunchConfiguration('spawn_shelfino', default='true')
    rviz_config_file  = LaunchConfiguration('rviz_config_file', default=os.path.join(shelfino_desc_pkg, "rviz", "shelfino.rviz"))
    nav2_rviz_config_file = LaunchConfiguration('nav2_rviz_config_file', default=os.path.join(shelfino_nav2_pkg, 'rviz', 'shelfino_nav.rviz'))
    gazebo_world_file = LaunchConfiguration('gazebo_world_file', default=os.path.join(shelfino_gaze_pkg, 'worlds', 'hexagon_world','model.sdf'))
    
    shelfino_name = PythonExpression(["'", 'shelfino', shelfino_id, "'"])
    robot_desc = os.path.join(shelfino_desc_pkg, 'models', 'shelfino_v1.xacro')
    
    # Set Gazebo model path
    os.environ["GAZEBO_MODEL_PATH"] = os.path.join(shelfino_desc_pkg, 'models')

    def evaluate_rviz(context, *args, **kwargs):
        """
        Replace shelfinoX with the robot name in the rviz configuration file and 
        sets the rviz_config_file to the new file
        """
        if context.launch_configurations['use_rviz'] == 'true':
            rn = 'shelfino' + context.launch_configurations['shelfino_id']
            rviz_path = context.launch_configurations['rviz_config_file']
            cr_path = context.launch_configurations['nav2_rviz_config_file']
            
            print("[{}] Replacing shelfinoX with {} from file {} to file {}".format(__file__, rn, rviz_path, cr_path))

            with open(rviz_path,'r') as f_in:
                filedata = f_in.read()
                newdata = filedata.replace("shelfinoX", rn)
                with open(cr_path,'w+') as f_out:
                    f_out.write(newdata)

            context.launch_configurations['nav2_rviz_config_file'] = cr_path

        return

    return LaunchDescription([
        SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb'),
        DeclareLaunchArgument(
            name='use_sim_time', 
            default_value=use_sim_time,
            description='Flag to enable use of simulation (Gazebo) clock'
        ),
        DeclareLaunchArgument(
            name='shelfino_id',
            default_value=shelfino_id,
            description='ID of the robot'
        ),
        DeclareLaunchArgument(
            name='use_gui',
            default_value=use_gui, 
            choices=['true', 'false'],
            description='Flag to enable gazebo visualization'
        ),
        DeclareLaunchArgument(
            name='use_rviz',
            default_value=use_rviz, 
            choices=['true', 'false'],
            description='Flag to enable rviz visualization'
        ),
        DeclareLaunchArgument(
            name='spawn_shelfino',
            default_value=spawn_shelfino,
            choices=['true', 'false'],
            description='Flag to enable spawning of the robot'
        ),
        DeclareLaunchArgument(
            name='rviz_config_file',
            default_value=rviz_config_file,
            description='Path to the rviz configuration file, used only if use_rviz=true'
        ),
        DeclareLaunchArgument(
            name='gazebo_world_file',
            default_value=gazebo_world_file, # choices=['empty', 'povo', 'hexagon'],
            description='World used in the gazebo simulation'
        ),
        DeclareLaunchArgument(
            name='nav2_rviz_config_file',
            default_value=nav2_rviz_config_file,
            description='Full path to the Nav2 RVIZ config file to use'
        ),

        OpaqueFunction(function=print_env),
        OpaqueFunction(function=check_exists),
        OpaqueFunction(function=evaluate_rviz),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': ['-r -v 4 ', gazebo_world_file],  # Run immediately + verbose
                'on_exit_shutdown': 'true',  # Cleanup on exit
                'headless': 'false',  # Match your use_gui
                'pause': 'true',  # Start unpaused
                'use_sim_time': use_sim_time,  # Sync with ROS
            }.items(),
        ),

        Node(
            package='ros_ign_gazebo',
            executable='create',
            arguments=[
                '-topic', PythonExpression(["'/", shelfino_name, "/robot_description", "'"]),
                '-entity', shelfino_name,
                '-robot_namespace', shelfino_name,
                '-name', shelfino_name,
                '-x', PythonExpression(['-2.0 + float(', shelfino_id, ') + 3.0']),
                '-y', PythonExpression(['-2.0 + float(', shelfino_id, ') + 3.0']),
                '-z', '0.1',
                '-Y', '0.0'
            ],
            condition=IfCondition(spawn_shelfino),
            output='screen'
        ),
        
        # Add robot_state_publisher to publish TF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=shelfino_name,
            parameters=[{
                'robot_description': Command([
                    'xacro ', robot_desc,
                    ' robot_id:=', shelfino_id
                ]),
                'use_sim_time': True,
                'frame_prefix': '',  # Empty to let namespace handle it
            }],
            condition=IfCondition(spawn_shelfino),
            output='screen'
        ),
        
        # Bridge for Gazebo and ROS2
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                f'/{shelfino_name}/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                f'/{shelfino_name}/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                f'/{shelfino_name}/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                f'/{shelfino_name}/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                f'/{shelfino_name}/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
                '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
            ],
            condition=IfCondition(spawn_shelfino),
            output='screen'
        ),

        Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', nav2_rviz_config_file],  # Use the configured RViz file that shows all robots
        parameters=[{'use_sim_time': use_sim_time}],
        condition=UnlessCondition(PythonExpression(["'false' == '", use_rviz, "'"])),  # Only launch if use_rviz is true
        output='screen'
    ),
    ])