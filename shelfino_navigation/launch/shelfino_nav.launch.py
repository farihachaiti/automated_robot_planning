# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Author: Placido Falqueto   placido.falqueto [at] unitn.it
# Maintainer: Enrico Saccon  enrico.saccon [at] unitn.it

import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction, RegisterEventHandler
from launch.conditions import UnlessCondition
from nav2_common.launch import RewrittenYaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessStart
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, LivelinessPolicy, HistoryPolicy

def print_env(context):
    print(__file__)
    for key in context.launch_configurations.keys():
        print("\t", key, context.launch_configurations[key])
    return

def check_exists(context):
    if not os.path.exists(context.launch_configurations['map_file']):
        raise Exception("[{}] Map file `{}` does not exist".format(__file__, context.launch_configurations['map_file']))
    
    if not os.path.exists(context.launch_configurations['nav2_params_file']):
        raise Exception("[{}] Nav2 parameters file `{}` does not exist".format(__file__, context.launch_configurations['nav2_params_file']))
    
    if context.launch_configurations['headless'] == "false" and \
        not os.path.exists(context.launch_configurations['rviz_config_file']):
        raise Exception("[{}] Rviz configuration `{}` does not exist".format(__file__, context.launch_configurations['rviz_config_file']))

    return

def evaluate_rviz(context, *args, **kwargs):
    rn = 'shelfino' + context.launch_configurations['robot_id']
    rviz_path = context.launch_configurations['rviz_config_file']
    cr_path = context.launch_configurations['nav2_rviz_config_file']
    
    with open(rviz_path,'r') as f_in:
        filedata = f_in.read()
        newdata = filedata.replace("shelfinoX", rn)
        with open(cr_path,'w+') as f_out:
            f_out.write(newdata)

    context.launch_configurations['nav2_rviz_config_file'] = cr_path
    
    return

def generate_launch_description():
    # Package directories
    shelfino_nav2_pkg = get_package_share_directory('shelfino_navigation')
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    robot_id = LaunchConfiguration('robot_id', default='0')
    map_file = LaunchConfiguration('map_file', 
        default=os.path.join(shelfino_nav2_pkg, 'maps', 'dynamic_map.yaml'))
    nav2_params_file = LaunchConfiguration('nav2_params_file', default=os.path.join(shelfino_nav2_pkg,'config', 'shelfino.yaml'))
    nav2_rviz_config_file = LaunchConfiguration('nav2_rviz_config_file', default=os.path.join(shelfino_nav2_pkg, 'rviz', 'shelfino_nav.rviz'))
    headless = LaunchConfiguration('headless', default='false')
    initial_x = LaunchConfiguration('initial_x', default='0.0')
    initial_y = LaunchConfiguration('initial_y', default='0.0')
    initial_yaw = LaunchConfiguration('initial_yaw', default='0.0')
    set_initial_pose   = LaunchConfiguration('set_initial_pose', default='true')
    # Robot namespace - using proper substitution concatenation
    namespace = LaunchConfiguration('namespace', 
                                  default=f'shelfino{robot_id}')
    param_substitutions = { 
        'use_sim_time'     : use_sim_time,
        'autostart'        : 'true',
        'base_frame_id'    : [namespace, '/base_link'],  # shelfino0/base_link
        'odom_frame_id'    : [namespace, '/odom'],      # shelfino0/odom
        'robot_base_frame' : [namespace, '/base_link'], # shelfino0/base_link
        'global_frame'     : [namespace, '/odom'],
        # Explicitly set the scan topic for the local costmap using dot notation
        'local_costmap.local_costmap.ros__parameters.obstacle_layer.scan.topic': [namespace, '/scan'],
        'x'                : initial_x,
        'y'                : initial_y,
        'yaw'              : initial_yaw,
        'set_initial_pose' : set_initial_pose,
    }

    # Rewrite parameters with namespace
    configured_params = RewrittenYaml(
        source_file=nav2_params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True
    )
    delay_robot_controller = TimerAction(
        period=5.0,  # 5 second delay
        actions=[
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                namespace=namespace,
                respawn=True,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', 'info'],
            ),
        ]
    )

    # Register the timer to start after the lifecycle manager
    register_timer = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager',
                output='screen',
                namespace=namespace,
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': True,
                    'node_names': [
                        'amcl',
                        'controller_server',
                        'planner_server',
                        'velocity_smoother',
                        'behavior_server',
                        'bt_navigator',
                    ],
                    'bond_timeout': 10.0
                }]
            ),
            on_start=[TimerAction(
                period=5.0,  # 5 second delay
                actions=[delay_robot_controller]
            )]
        )
    )


    # Map server is launched in the main evacuation.launch.py
    # to ensure a single instance for all robots

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
            default_value=map_file, 
            description='Full path to map file to load'),

        DeclareLaunchArgument(
            name='nav2_params_file',
            default_value=nav2_params_file,
            description='Full path to the ROS2 parameters file to use for all launched nodes'
        ),

        DeclareLaunchArgument(
            name='nav2_rviz_config_file',
            default_value=nav2_rviz_config_file,
            description='Full path to the Nav2 RVIZ config file to use'
        ),

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


        DeclareLaunchArgument(
            'set_initial_pose',
            default_value='true',
            description='Set initial pose of the robot'),

        # Nodes
        OpaqueFunction(function=print_env), 
        OpaqueFunction(function=check_exists),
        OpaqueFunction(function=evaluate_rviz),

  
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            namespace=namespace,
            respawn=True,
            respawn_delay=2.0,
            arguments=['--ros-args', '--log-level', 'info'],
            parameters=[configured_params, {'use_sim_time': True}],
            remappings=[
                ('map', '/map'),  # Map service in global namespace
                ('/scan', 'scan'),  # Remove leading slash for proper namespacing
                ('/odom', 'odom'),
                ('amcl_pose', 'amcl_pose'),
                ('particle_cloud', 'particle_cloud'),
                ('initialpose', 'initialpose'),
                ('map_updates', 'map_updates'),
                ('/tf', '/tf'),
                ('/tf_static', '/tf_static')
            ],
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            namespace=namespace,
            respawn=True,
            respawn_delay=2.0,
            parameters=[configured_params, {'use_sim_time': True}],
            arguments=['--ros-args', '--log-level', 'info'],
            remappings=[
                ('cmd_vel', 'cmd_vel_nav'),
                ('map', '/map'),  # Map service in global namespace
                ('/scan', 'scan'),  # Remove leading slash for proper namespacing
                ('/odom', 'odom'),
            ],
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            namespace=namespace,
            respawn=True,
            respawn_delay=2.0,
            parameters=[configured_params, {'use_sim_time': True}],
            arguments=['--ros-args', '--log-level', 'info'],
        ),

        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            namespace=namespace,
            respawn=True,
            respawn_delay=2.0,
            parameters=[configured_params, {'use_sim_time': True}],
            arguments=['--ros-args', '--log-level', 'info'],
            remappings = [
                ('cmd_vel', 'cmd_vel_nav'), 
                ('cmd_vel_smoothed', 'cmd_vel'),
            ],
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            namespace=namespace,
            respawn=True,
            respawn_delay=2.0,
            parameters=[configured_params, {'use_sim_time': True}],
            arguments=['--ros-args', '--log-level', 'info'],
        ),        

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            namespace=namespace,
            respawn=True,
            respawn_delay=2.0,
            parameters=[configured_params, {'use_sim_time': True}],
            arguments=['--ros-args', '--log-level', 'info'],
            remappings=[
                (
                    PathJoinSubstitution([
                        '/', 'odom'
                      ]),
                    PathJoinSubstitution([
                        '/', namespace, 'odom'
                      ]),
                ),
            ],
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            namespace=namespace,
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': [
                    'amcl',
                    'controller_server',
                    'planner_server',
                    'velocity_smoother',
                    'behavior_server',
                    'bt_navigator',
                ],
                'bond_timeout': 60.0,
                'bond_heartbeat_period': 2.0,
                'attempt_respawn_reconnection': True
            }]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', nav2_rviz_config_file],
            parameters=[{
                'use_sim_time': True,
            }],
            condition=UnlessCondition(headless),
            output='screen'
        ),
    ])