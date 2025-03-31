# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Author: Placido Falqueto   placido.falqueto [at] unitn.it
# Maintainer: Enrico Saccon  enrico.saccon [at] unitn.it

import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import UnlessCondition
from nav2_common.launch import RewrittenYaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package directories
    shelfino_nav2_pkg = get_package_share_directory('shelfino_navigation')
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    robot_id = LaunchConfiguration('robot_id', default='0')
    map_file = LaunchConfiguration('map_file')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    headless = LaunchConfiguration('headless', default='false')
    initial_x = LaunchConfiguration('initial_x', default='0.0')
    initial_y = LaunchConfiguration('initial_y', default='0.0')
    initial_yaw = LaunchConfiguration('initial_yaw', default='0.0')

    # Robot namespace - using proper substitution concatenation
    namespace = LaunchConfiguration('namespace', 
                                  default=['shelfino', robot_id])

    # Parameter substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'base_frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'robot_base_frame': 'base_link',
        'global_frame': 'map',
        'global_costmap.robot_base_frame': ['shelfino', robot_id, '/base_link'],
        'local_costmap.robot_base_frame': ['shelfino', robot_id, '/base_link'],
        'topic': 'scan',
        'x': initial_x,
        'y': initial_y,
        'yaw': initial_yaw,
    }

    # Rewrite parameters with namespace
    configured_params = RewrittenYaml(
        source_file=nav2_params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True
    )

    # Common node configuration
    nav2_node_config = {
        'namespace': namespace,
        'parameters': [configured_params],
        'remappings': [
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('/odom', 'odom'),
            ('/scan', 'scan'),
            ('/navigate_to_pose', 'navigate_to_pose')
        ],
        'output': 'screen',
        'respawn': True,
        'respawn_delay': 2.0
    }

    # Velocity smoother needs different cmd_vel remapping
    velocity_smoother_config = nav2_node_config.copy()
    velocity_smoother_config['remappings'] = [
        ('/cmd_vel', 'cmd_vel_nav'),
        ('/cmd_vel_smoothed', 'cmd_vel'),
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        ('/odom', 'odom'),
        ('/scan', 'scan'),
        ('/navigate_to_pose', 'navigate_to_pose')
    ]

    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        DeclareLaunchArgument(
            'robot_id',
            default_value='0',
            description='ID of the robot'),

        DeclareLaunchArgument(
            'namespace',
            default_value=['shelfino', robot_id],
            description='Robot namespace'),

        DeclareLaunchArgument(
            'map_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('shelfino_navigation'),
                'map', 'hexagon.yaml'
            ]),
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('shelfino_navigation'),
                'config', 'shelfino.yaml'
            ]),
            description='Full path to Nav2 parameters file'),

        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('shelfino_navigation'),
                'rviz', 'shelfino_nav.rviz'
            ]),
            description='Full path to Rviz config file'),

        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run without RViz if true'),

        DeclareLaunchArgument(
            'initial_x',
            default_value='0.0',
            description='Initial x position of the robot'),

        DeclareLaunchArgument(
            'initial_y',
            default_value='0.0',
            description='Initial y position of the robot'),

        DeclareLaunchArgument(
            'initial_yaw',
            default_value='0.0',
            description='Initial yaw orientation of the robot'),

        # Nodes
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'yaml_filename': map_file}
            ],
            output='screen',
            respawn=True,
            respawn_delay=2.0
        ),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            **nav2_node_config
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            **nav2_node_config
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            **nav2_node_config
        ),

        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            **velocity_smoother_config
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': True},
                {'node_names': [
                    'map_server',
                    'amcl',
                    'controller_server',
                    'planner_server',
                    'velocity_smoother'
                ]}
            ],
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=UnlessCondition(headless),
            output='screen'
        ),
    ])