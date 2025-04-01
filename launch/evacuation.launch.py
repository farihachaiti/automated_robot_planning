#!/usr/bin/env python3

import os
import yaml
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression, Command
from launch.actions import (OpaqueFunction, DeclareLaunchArgument, 
                          IncludeLaunchDescription, GroupAction, 
                          ExecuteProcess, TimerAction, SetEnvironmentVariable)
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

import launch.logging
import logging

# Configure logging to write to a file
logging.basicConfig(
    filename='launch_output.log',
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)

def print_env(context):
    logging.info("Launch environment variables:")
    for key in context.launch_configurations.keys():
        logging.info(f"\t{key}: {context.launch_configurations[key]}")
    return []

def check_map(context):
    map_name = Path(context.launch_configurations['map_file'])
    logging.info(f"Checking map file: {map_name}")
    return []

def get_map_name(context):
    map_name = Path(context.launch_configurations['map_file']).stem
    context.launch_configurations['map_name'] = map_name
    logging.info(f"Map name set to: {map_name}")
    return []

def evaluate_rviz(context):
    shelfino_nav2_pkg = get_package_share_directory('shelfino_navigation')
    rviz_path = context.launch_configurations['nav2_rviz_config_file']
    cr_path = os.path.join(shelfino_nav2_pkg, 'rviz', 'shelfino') + \
        'evacuation_'+context.launch_configurations['n_shelfini']+'_nav.rviz'
    
    logging.info(f"Generating RViz configuration for {context.launch_configurations['n_shelfini']} robots...")
    
    output_config = {}
    with open(rviz_path, 'r') as f_in:
        rviz_config = yaml.load(f_in, Loader=yaml.FullLoader)
        for key in rviz_config.keys():
            if key != 'Visualization Manager':
                output_config[key] = rviz_config[key]

        output_config['Visualization Manager'] = {}
        for key in rviz_config['Visualization Manager'].keys():
            if key != 'Displays' and key != 'Tools':
                output_config['Visualization Manager'][key] = rviz_config['Visualization Manager'][key]

        displays = rviz_config['Visualization Manager']['Displays']
        output_config['Visualization Manager']['Displays'] = []
        for display in displays:
            if type(display) is not dict:
                logging.error(f"Display `{display}` is not a dictionary")
                raise Exception(f"Display `{display}` is not a dictionary")
            if "shelfinoX" in str(display):
                display_str = str(display)
                for shelfino_id in range(int(context.launch_configurations['n_shelfini'])):
                    output_config['Visualization Manager']['Displays'].append(
                        yaml.load(display_str.replace("shelfinoX", "shelfino"+str(shelfino_id)), Loader=yaml.FullLoader))
            else:
                output_config['Visualization Manager']['Displays'].append(display)

        tools = rviz_config['Visualization Manager']['Tools']
        output_config['Visualization Manager']['Tools'] = []
        for tool in tools:
            if type(tool) is not dict:
                logging.error(f"Tool `{tool}` is not a dictionary")
                raise Exception(f"Tool `{tool}` is not a dictionary")
            if "shelfinoX" in str(tool):
                tool_str = str(tool)
                for shelfino_id in range(int(context.launch_configurations['n_shelfini'])):
                    output_config['Visualization Manager']['Tools'].append(
                        yaml.load(tool_str.replace("shelfinoX", "shelfino"+str(shelfino_id)), Loader=yaml.FullLoader))
            else:
                output_config['Visualization Manager']['Tools'].append(tool)

        with open(cr_path, 'w+') as f_out:
            yaml.dump(output_config, f_out, default_flow_style=False)

    context.launch_configurations['rviz_config_file'] = cr_path
    logging.info(f"RVIZ configuration file updated: {cr_path}")
    return []

def launch_evacuation_nodes(context):
    nodes = []
    n_shelfini = int(context.launch_configurations['n_shelfini'])
    robot_desc = context.launch_configurations['robot_description']
    nav2_params_file_path = os.path.join(
        get_package_share_directory('shelfino_navigation'),
        'config',
        'shelfino.yaml'
    )

    for shelfino_id in range(n_shelfini):
        shelfino_name = f"shelfino{shelfino_id}"
        
        frame_prefix = PythonExpression([
            '"', shelfino_name, '/"'  # Results in e.g. "shelfino0/"
        ])
        # 1. Robot State Publisher
        robot_state_pub = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=shelfino_name,
            parameters=[{
                'robot_description': Command([
                    'xacro ', robot_desc,
                    ' robot_id:=', str(shelfino_id)
                ]),
                'use_sim_time': True,
                'frame_prefix': frame_prefix,
            }],
            output='screen'
        )



        # 3. Custom Controller
        robot_controller_node = Node(
            package='automated_robot_planning',
            executable='robot_controller.py',
            name='robot_controller',
            namespace=shelfino_name,
            parameters=[
                {'robot_name': shelfino_name},
                {'use_sim_time': True},
                {'robot_description': Command([
                    'xacro ', robot_desc,
                    ' robot_id:=', str(shelfino_id)
                ])},
                nav2_params_file_path
            ],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
                (f'{shelfino_name}/navigate_to_pose', 'navigate_to_pose') 
            ],
            output='screen',
            emulate_tty=True
        )

        nodes.extend([robot_state_pub, robot_controller_node])

    return nodes

def launch_rsp(context):
    nodes = []
    shelfino_desc_pkg = get_package_share_directory('shelfino_description')
    use_sim_time = context.launch_configurations['use_sim_time']
    
    for shelfino_id in range(int(context.launch_configurations['n_shelfini'])):
        rsp_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(shelfino_desc_pkg, 'launch', 'rsp.launch.py')
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time, 
                'shelfino_id': str(shelfino_id)
            }.items()
        )
        nodes.append(rsp_node)
    return nodes

def spawn_shelfini(context):
    nodes = []
    use_sim_time = context.launch_configurations['use_sim_time']
    gazebo_bridge_plugins_params_file_path = os.path.join(
    get_package_share_directory('shelfino_description'),
    'config',
    'bridge.yaml'
    )   


    for shelfino_id in range(int(context.launch_configurations['n_shelfini'])):
        shelfino_name = "shelfino" + str(shelfino_id)
        spawn_node = GroupAction([
            Node(
                package='ros_ign_gazebo',
                executable='create',
                arguments=[
                    '-topic', PythonExpression(["'/", shelfino_name, "/robot_description", "'"]),
                    '-entity', shelfino_name,
                    '-robot_namespace', shelfino_name,
                    '-x', str(shelfino_id*2.0-2.0),
                    '-y', str(shelfino_id*2.0-2.0),
                    '-z', '0.1',
                    '-Y', '0.0'
                ],
                output='screen'
            ),

                 Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                    f'/{shelfino_name}/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                    f'/{shelfino_name}/tf@gtf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                    f'/{shelfino_name}/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry'
                    f'/{shelfino_name}/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                    '--config', gazebo_bridge_plugins_params_file_path,
                ],
                output='screen'
            ),

            # Add to spawn_shelfini():
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments=['0', '0', '0', '0', '0', '0', 'map', [shelfino_name, '/odom']],
                namespace=shelfino_name,
                output='screen'
            ),

            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments=['0.1', '0', '0.2', '0', '0', '0',  # x, y, z, roll, pitch, yaw
                          f'{shelfino_name}/base_link',
                          f'{shelfino_name}/laser'],
                output='screen',
                namespace=shelfino_name
            ),

            Node(
                package='shelfino_gazebo',
                executable='destroy_shelfino',
                name='destroy_shelfino',
                output='screen',
                namespace=f'/{shelfino_name}',
                parameters=[{
                    'use_sim_time': PythonExpression(["'true' == '", use_sim_time, "'"])
    }]
            )
        ])
        nodes.append(spawn_node)
    return nodes

def launch_navigation(context):
    nodes = []
    shelfino_nav2_pkg = get_package_share_directory('shelfino_navigation')
    use_sim_time = context.launch_configurations['use_sim_time']
    map_file = context.launch_configurations['map_file']
    nav2_params_file = context.launch_configurations['nav2_params_file']
    nav2_rviz_config_file = context.launch_configurations['nav2_rviz_config_file']
    robot_desc = context.launch_configurations['robot_description']
    
    for shelfino_id in range(int(context.launch_configurations['n_shelfini'])):
        shelfino_name = "shelfino" + str(shelfino_id)
        sapf_node = GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(shelfino_nav2_pkg, 'launch', 'shelfino_nav.launch.py')
                ]),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'robot_id': str(shelfino_id),
                    'map_file': map_file,
                    'nav2_params_file': nav2_params_file,
                    'rviz_config_file': nav2_rviz_config_file,
                    'initial_x': str(shelfino_id*2.0-2.0),
                    'initial_y': str(shelfino_id*2.0-2.0),
                    'headless': 'true',
                    'namespace': shelfino_name,
                    'robot_description': robot_desc
                }.items()
            )
        ])
        nodes.append(sapf_node)
    return nodes

def generate_launch_description():
    shelfino_desc_pkg = get_package_share_directory('shelfino_description')
    shelfino_nav2_pkg = get_package_share_directory('shelfino_navigation')
    shelfino_gaze_pkg = get_package_share_directory('shelfino_gazebo')
    map_env_pkg = get_package_share_directory('map_pkg')

    nav2_params_file_path = os.path.join(shelfino_nav2_pkg, 'config', 'shelfino.yaml')
    map_env_params_file_path = os.path.join(map_env_pkg, 'config', 'map_config.yaml')

    qt_qpa_platform = SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    n_shelfini = LaunchConfiguration('n_shelfini', default='3')

    use_gui = LaunchConfiguration('use_gui', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    rviz_config_file = LaunchConfiguration('rviz_config_file', 
        default=os.path.join(shelfino_desc_pkg, 'rviz', 'shelfino.rviz'))
    gazebo_world_file = LaunchConfiguration('gazebo_world_file', 
        default=os.path.join(shelfino_gaze_pkg, 'worlds', 'empty.world'))
    robot_desc = LaunchConfiguration('robot_description', 
        default=os.path.join(shelfino_desc_pkg, 'models', 'shelfino_v1.xacro'))

    map_file = LaunchConfiguration('map_file', 
        default=os.path.join(shelfino_nav2_pkg, 'maps', 'dynamic_map.yaml'))
    nav2_params_file = LaunchConfiguration('nav2_params_file', 
        default=nav2_params_file_path)
    nav2_rviz_config_file = LaunchConfiguration('nav2_rviz_config_file', 
        default=os.path.join(shelfino_nav2_pkg, 'rviz', 'shelfino_nav.rviz'))
    map_env_params_file = LaunchConfiguration('map_env_params_file', 
        default=map_env_params_file_path)

    launch_args = [
        DeclareLaunchArgument('use_sim_time', default_value=use_sim_time, 
                            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('n_shelfini', default_value=n_shelfini, 
                            description='The number of Shelfini to spawn'),
        DeclareLaunchArgument('robot_description', default_value=robot_desc, 
                            description='Description of each robot'),
        DeclareLaunchArgument('use_gui', default_value=use_gui, 
                            choices=['true', 'false'], 
                            description='Flag to enable gazebo visualization'),
        DeclareLaunchArgument('use_rviz', default_value=use_rviz, 
                            choices=['true', 'false'], 
                            description='Flag to enable rviz visualization'),
        DeclareLaunchArgument('rviz_config_file', default_value=rviz_config_file, 
                            description='Rviz configuration file'),
        DeclareLaunchArgument('gazebo_world_file', default_value=gazebo_world_file, 
                            description='Gazebo world file'),
        DeclareLaunchArgument('map_file', default_value=map_file, 
                            description='Full path to map file to load'),
        DeclareLaunchArgument('nav2_params_file', default_value=nav2_params_file, 
                            description='Full path to the nav2 params file to use'),
        DeclareLaunchArgument('nav2_rviz_config_file', default_value=nav2_rviz_config_file, 
                            description='Full path to the nav2 rviz config file to use'),
        DeclareLaunchArgument('map_env_params_file', default_value=map_env_params_file, 
                            description='Full path to the map_pkg params file to use')
    ]

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(shelfino_gaze_pkg, 'launch'),
            '/shelfino.launch.py']
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'spawn_shlefino': 'false',
            'use_gui': use_gui,
            'use_rviz': use_rviz,
            'gazebo_world_file': gazebo_world_file,
        }.items()
    )

    map_env_conf_params = RewrittenYaml(
        source_file=map_env_params_file,
        param_rewrites={'n_victims': '0'},
        convert_types=True
    )

    map_pkg_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(map_env_pkg, 'launch', 'map_env.launch.py')
        ]),
        launch_arguments={
            'map_env_params_file': map_env_conf_params,
        }.items()
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config_file')],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    ld = LaunchDescription()

    ld.add_action(qt_qpa_platform)
    for launch_arg in launch_args:
        ld.add_action(launch_arg)

    ld.add_action(OpaqueFunction(function=get_map_name))
    ld.add_action(OpaqueFunction(function=print_env))
    ld.add_action(OpaqueFunction(function=evaluate_rviz))
    
    create_map_node = Node(
        package='map_pkg',
        executable='create_map_pgm.py',
        name='create_map_pgm',
        output='screen',
        parameters=[map_env_conf_params]
    )

    ld.add_action(gazebo_launch)
    ld.add_action(map_pkg_node)
    
    ld.add_action(TimerAction(period=2.0, actions=[create_map_node]))
    ld.add_action(TimerAction(period=5.0, actions=[OpaqueFunction(function=launch_rsp)]))
    ld.add_action(TimerAction(period=5.0, actions=[OpaqueFunction(function=launch_evacuation_nodes)]))
    ld.add_action(TimerAction(period=10.0, actions=[OpaqueFunction(function=spawn_shelfini)]))
    ld.add_action(TimerAction(period=15.0, actions=[rviz2_node]))
    ld.add_action(TimerAction(period=20.0, actions=[OpaqueFunction(function=launch_navigation)]))

    return ld