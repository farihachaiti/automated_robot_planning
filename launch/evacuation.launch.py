#!/usr/bin/env python3

import os
import yaml
from pathlib import Path
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression, Command
from launch.conditions import UnlessCondition
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
    cr_path = context.launch_configurations['rviz_config_file']
    
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
        
        # Create a single TF display that will show all frames
        tf_display_added = False
        
        # First add all non-TF and non-template displays
        for display in displays:
            if type(display) is not dict:
                logging.error(f"Display `{display}` is not a dictionary")
                raise Exception(f"Display `{display}` is not a dictionary")
                
            # Skip all TF displays - we'll add just one later
            if display.get('Class') == 'rviz_default_plugins/TF':
                # Save the first TF display config to use later
                if not tf_display_added:
                    tf_display = display.copy()
                    tf_display_added = True
                continue
                
            # Handle template displays with shelfinoX
            if "shelfinoX" in str(display):
                display_str = str(display)
                for shelfino_id in range(int(context.launch_configurations['n_shelfini'])):
                    output_config['Visualization Manager']['Displays'].append(
                        yaml.load(display_str.replace("shelfinoX", "shelfino" + str(shelfino_id)), Loader=yaml.FullLoader))
            else:
                # Add all other displays
                output_config['Visualization Manager']['Displays'].append(display)
        
        # Now add a single TF display at the end
        if tf_display_added:
            # Ensure the TF display is configured to show all frames
            tf_display['Name'] = 'TF'
            tf_display['Enabled'] = True
            # Add the single TF display
            output_config['Visualization Manager']['Displays'].append(tf_display)
        else:
            # If no TF display was found, create a basic one
            basic_tf = {
                'Class': 'rviz_default_plugins/TF',
                'Name': 'TF',
                'Enabled': True,
                'All Enabled': True,
                'Frame Timeout': 15,
                'Show Names': True,
                'Show Axes': True,
                'Show Arrows': True,
                'Marker Scale': 1.0
            }
            output_config['Visualization Manager']['Displays'].append(basic_tf)

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
                        yaml.load(tool_str.replace("shelfinoX", "shelfino" + str(shelfino_id)), Loader=yaml.FullLoader))
            else:
                output_config['Visualization Manager']['Tools'].append(tool)

        with open(rviz_path, 'w+') as f_out:
            yaml.dump(output_config, f_out, default_flow_style=False)

    context.launch_configurations['rviz_config_file'] = rviz_path
    logging.info(f"RVIZ configuration file updated: {rviz_path}")
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
        shelfino_name = "shelfino" + str(shelfino_id)
        

        # 3. Custom Controller
        robot_controller_node = Node(
            package='automated_robot_planning',
            executable='robot_controller.py',
            name='robot_controller',
            namespace=shelfino_name,  # This sets the namespace for the node
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
                # Add explicit remappings for the action
                ('/navigate_through_poses', f'/{shelfino_name}/navigate_through_poses'),
            ],
            output='screen',
            emulate_tty=True
        )

        nodes.extend([robot_controller_node])

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
    shelfino_gaze_pkg = get_package_share_directory('shelfino_gazebo')
    shelfino_desc_pkg = get_package_share_directory('shelfino_description')
    use_sim_time = context.launch_configurations['use_sim_time']
    robot_desc = context.launch_configurations['robot_description']
    gazebo_world_file = context.launch_configurations['gazebo_world_file']
    gazebo_bridge_plugins_params_file_path = os.path.join(
    get_package_share_directory('shelfino_description'),
    'config',
    'bridge.yaml'
    )
    robot_desc = context.launch_configurations['robot_description']
    
    for shelfino_id in range(int(context.launch_configurations['n_shelfini'])):
        shelfino_name = "shelfino" + str(shelfino_id)
        x_pos = -2.0 + (shelfino_id + 3)  # Same formula as in robot_controller.py
        y_pos = -2.0 + (shelfino_id + 3)
    
        # Create the SDF file path
        model_file = os.path.join(shelfino_desc_pkg, 'models', 'shelfino_v1.xacro')
        
        # Group all actions for this robot
        spawn_node = GroupAction([
            # Create a node to spawn each robot
            Node(
                package='ros_ign_gazebo',
                executable='create',
                arguments=[
                    '-topic', PythonExpression(["'/", shelfino_name, "/robot_description", "'"]),
                    '-entity', shelfino_name,
                    '-robot_namespace', shelfino_name,
                    '-name', shelfino_name,
                    '-x', str(x_pos),
                    '-y', str(y_pos),
                    '-z', '0.1',
                    '-Y', '0.0'
                ],
                output='screen'
            ),

            # Create a node to bridge each robot
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
                    '/gate_position@geometry_msgs/msg/PoseArray@gz.msgs.Pose_V',
                    '--ros-args',
                    '-p',
                    f'config_file:={gazebo_bridge_plugins_params_file_path}',
                ],
                output='screen'
            ),

            # Global map→odom transform (only once for first robot)
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments=['0', '0', '0', '0', '0', '0', '/map', '/odom'],
                condition=UnlessCondition(str(shelfino_id > 0)),
                output='screen'
            ),

            # Per-robot odom→robot/odom connection
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments=['0', '0', '0', '0', '0', '0', '/odom', f'/{shelfino_name}/odom'],
                output='screen'
            ),

            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                arguments=['0', '0', '0', '0', '0', '0', 
                        f'/{shelfino_name}/odom', 
                        f'/{shelfino_name}/base_link'],
                output='screen'
            ),

            # Each robot's tf publisher is now managed by robot_controller.py

            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace=shelfino_name,  # This handles namespacing
                parameters=[{
                    'robot_description': Command([
                        'xacro ', robot_desc,
                        ' robot_id:=', str(shelfino_id)
                    ]),
                    'use_sim_time': True,
                    'frame_prefix': '',  # EMPTY - let namespace handle it
                    'publish_frequency': 30.0,  # Increase publish frequency
                }]
            ),

            Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                namespace=shelfino_name,
                parameters=[{
                    'robot_description': Command([
                        'xacro ', robot_desc,
                        ' robot_id:=', str(shelfino_id)
                    ]),
                    'use_sim_time': True,
                    'source_list': ['joint_states'],  # Add source list
                    'rate': 30.0,  # Add publishing rate
                    'publish_default_positions': True,  # Add default positions
                    'publish_default_velocities': True,  # Add default velocities
                    'publish_default_efforts': True,  # Add default efforts
                }],
                remappings=[
                    ('/tf', 'tf'),
                    ('/tf_static', 'tf_static'),
                    ('/joint_states', 'joint_states')
                ],
                output='screen'
            ),

            # Remove all other static transform publishers for robot frames
            Node(
                package='shelfino_gazebo',
                executable='destroy_shelfino',
                name='destroy_shelfino',
                namespace=shelfino_name,
                output='screen',
                parameters=[{
                    'use_sim_time': PythonExpression(["'true' == '", use_sim_time, "'"])}]
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
                    'nav2_rviz_config_file': nav2_rviz_config_file,
                    # Use the same formula as in robot_controller.py and Gazebo spawn
                    # base_x, base_y = -2.0, -2.0
                    # offset = idx + 3
                    # x = base_x + offset
                    # y = base_y + offset
                    'initial_x': str(-2.0 + (shelfino_id + 3)),
                    'initial_y': str(-2.0 + (shelfino_id + 3)),
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
    map_env_params_file_path = os.path.join(map_env_pkg, 'config', 'map_config.yaml')
    nav2_params_file_path = os.path.join(shelfino_nav2_pkg, 'config', 'shelfino.yaml')

    qt_qpa_platform = SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    n_shelfini = LaunchConfiguration('n_shelfini', default='3')

    use_gui = LaunchConfiguration('use_gui', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    rviz_config_file = LaunchConfiguration('rviz_config_file', 
        default=os.path.join(shelfino_desc_pkg, 'rviz', 'shelfino.rviz'))
    gazebo_world_file = LaunchConfiguration('gazebo_world_file', default=os.path.join(shelfino_gaze_pkg, 'worlds', 'hexagon_world','model.sdf'))
    robot_desc = LaunchConfiguration('robot_description', 
        default=os.path.join(shelfino_desc_pkg, 'models', 'shelfino_v1.xacro'))

    map_file = LaunchConfiguration('map_file', 
        default=os.path.join(shelfino_nav2_pkg, 'maps', 'dynamic_map.yaml'))
    nav2_params_file = LaunchConfiguration('nav2_params_file', default=nav2_params_file_path)
    nav2_rviz_config_file = LaunchConfiguration('nav2_rviz_config_file', 
        default=os.path.join(shelfino_nav2_pkg, 'rviz', 'shelfino_nav.rviz'))
    map_env_params_file = LaunchConfiguration('map_env_params_file', default=map_env_params_file_path)

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
        DeclareLaunchArgument('nav2_params_file', default_value=nav2_params_file_path, 
                            description='Full path to the nav2 params file to use'),
        DeclareLaunchArgument('nav2_rviz_config_file', default_value=nav2_rviz_config_file, 
                            description='Full path to the nav2 rviz config file to use'),
        DeclareLaunchArgument('map_env_params_file', default_value=map_env_params_file_path, 
                            description='Full path to the map_pkg params file to use'),
        DeclareLaunchArgument(
            'publish_clock',
            default_value='true',
            description='Whether to publish the clock'
        ),
        DeclareLaunchArgument(
            'extra_gazebo_args',
            default_value='--ros-args --param use_sim_time:=true',
            description='Additional Gazebo arguments'
        ),    ]

    # Launch Gazebo with the hexagon world, but don't spawn robots yet
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(shelfino_gaze_pkg, 'launch'),
            '/shelfino.launch.py']
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'spawn_shelfino': 'false',  # Don't spawn robots here, we'll do it separately for each robot
            'use_gui': use_gui,
            'use_rviz': use_rviz,
            'gazebo_world_file': gazebo_world_file,
            'publish_clock': 'true',
            'extra_gazebo_args': '--ros-args --param use_sim_time:=true',
        }.items()
    )

    # Configure map environment parameters
    map_env_conf_params = RewrittenYaml(
        source_file=map_env_params_file,
        param_rewrites={
            'n_victims': '0',
            'map': 'hexagon',  # Force hexagon map
            'dx': '10.0',       # Set default size
            'dy': '10.0',
            'use_sim_time': 'false'        # Set default size
        },
        convert_types=True
    )

    # Launch map environment
    map_pkg_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(map_env_pkg, 'launch', 'spawn_map.launch.py')
        ]),
        launch_arguments={
            'map_env_params_file': map_env_conf_params,
            'use_sim_time': 'true' 
        }.items()
    )

    # Create map node with retry mechanism
    create_map_node = Node(
        package='map_pkg',
        executable='create_map_pgm.py',
        name='create_map_pgm',
        output='screen',
        parameters=[{
            'output_dir': os.path.join(
                get_package_share_directory('shelfino_navigation'),
                'maps'
            ),
        },
        map_env_params_file
        ]
    )

    # Generate config file for the map
    generate_config_node = Node(
        package='map_pkg',
        executable='generate_config_file.py',
        name='generate_config_file',
        output='screen',
        parameters=[{
            'map_env_params_file': map_env_params_file,
            'gen_map_params_file': os.path.join(
                get_package_share_directory('map_pkg'),
                'config',
                'full_config.yaml'
            ),
            'output_dir': os.path.join(
                get_package_share_directory('map_pkg'),
                'config'
            ),
            'generate_new_config': True
        }]
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            {'use_sim_time': use_sim_time},
            {'yaml_filename': map_file},
            {'frame_id': 'map'},
            {'topic_name': 'map'},
            {'map_subscribe_transient_local': True},
            {'save_map_timeout': 5.0},
            {'autostart': True}
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    # Global map server lifecycle manager
    map_server_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='map_server_lifecycle_manager',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server'],
            'bond_timeout': 60.0,
            'bond_heartbeat_period': 2.0,
            'attempt_respawn_reconnection': True
        }]
    )

    # Add lifecycle node activation commands with retry mechanism
    def configure_borders(context):
        try:
            # First check if node exists
            result = subprocess.run(['ros2', 'node', 'list'], capture_output=True, text=True)
            if '/send_borders' not in result.stdout:
                logging.warning("send_borders node not found, will retry")
                return None

            # Check current state
            result = subprocess.run(['ros2', 'lifecycle', 'get', '/send_borders'], capture_output=True, text=True)
            current_state = result.stdout.strip()
            
            if 'unconfigured' in current_state:
                subprocess.run(['ros2', 'lifecycle', 'set', '/send_borders', 'configure'], check=True)
                logging.info("Successfully configured send_borders node")
            elif 'inactive' in current_state:
                logging.info("send_borders node already configured")
            else:
                logging.warning(f"send_borders node in unexpected state: {current_state}")
                return None
        except subprocess.CalledProcessError as e:
            logging.warning(f"Failed to configure send_borders node, will retry: {e}")
            return None
        return []

    def activate_borders(context):
        try:
            # Check current state
            result = subprocess.run(['ros2', 'lifecycle', 'get', '/send_borders'], capture_output=True, text=True)
            current_state = result.stdout.strip()
            
            if 'inactive' in current_state:
                subprocess.run(['ros2', 'lifecycle', 'set', '/send_borders', 'activate'], check=True)
                logging.info("Successfully activated send_borders node")
            elif 'active' in current_state:
                logging.info("send_borders node already active")
            else:
                logging.warning(f"send_borders node in unexpected state: {current_state}")
                return None
        except subprocess.CalledProcessError as e:
            logging.warning(f"Failed to activate send_borders node, will retry: {e}")
            return None
        return []

    def configure_obstacles(context):
        try:
            # First check if node exists
            result = subprocess.run(['ros2', 'node', 'list'], capture_output=True, text=True)
            if '/send_obstacles' not in result.stdout:
                logging.warning("send_obstacles node not found, will retry")
                return None

            # Check current state
            result = subprocess.run(['ros2', 'lifecycle', 'get', '/send_obstacles'], capture_output=True, text=True)
            current_state = result.stdout.strip()
            
            if 'unconfigured' in current_state:
                subprocess.run(['ros2', 'lifecycle', 'set', '/send_obstacles', 'configure'], check=True)
                logging.info("Successfully configured send_obstacles node")
            elif 'inactive' in current_state:
                logging.info("send_obstacles node already configured")
            else:
                logging.warning(f"send_obstacles node in unexpected state: {current_state}")
                return None
        except subprocess.CalledProcessError as e:
            logging.warning(f"Failed to configure send_obstacles node, will retry: {e}")
            return None
        return []

    def activate_obstacles(context):
        try:
            # Check current state
            result = subprocess.run(['ros2', 'lifecycle', 'get', '/send_obstacles'], capture_output=True, text=True)
            current_state = result.stdout.strip()
            
            if 'inactive' in current_state:
                subprocess.run(['ros2', 'lifecycle', 'set', '/send_obstacles', 'activate'], check=True)
                logging.info("Successfully activated send_obstacles node")
            elif 'active' in current_state:
                logging.info("send_obstacles node already active")
            else:
                logging.warning(f"send_obstacles node in unexpected state: {current_state}")
                return None
        except subprocess.CalledProcessError as e:
            logging.warning(f"Failed to activate send_obstacles node, will retry: {e}")
            return None
        return []

    def configure_gates(context):
        try:
            # First check if node exists
            result = subprocess.run(['ros2', 'node', 'list'], capture_output=True, text=True)
            if '/send_gates' not in result.stdout:
                logging.warning("send_gates node not found, will retry")
                return None

            # Check current state
            result = subprocess.run(['ros2', 'lifecycle', 'get', '/send_gates'], capture_output=True, text=True)
            current_state = result.stdout.strip()
            
            if 'unconfigured' in current_state:
                subprocess.run(['ros2', 'lifecycle', 'set', '/send_gates', 'configure'], check=True)
                logging.info("Successfully configured send_gates node")
            elif 'inactive' in current_state:
                logging.info("send_gates node already configured")
            else:
                logging.warning(f"send_gates node in unexpected state: {current_state}")
                return None
        except subprocess.CalledProcessError as e:
            logging.warning(f"Failed to configure send_borders node, will retry: {e}")
            return None
        return []

    def activate_gates(context):
        try:
            # Check current state
            result = subprocess.run(['ros2', 'lifecycle', 'get', '/send_gates'], capture_output=True, text=True)
            current_state = result.stdout.strip()
            
            if 'inactive' in current_state:
                subprocess.run(['ros2', 'lifecycle', 'set', '/send_gates', 'activate'], check=True)
                logging.info("Successfully activated send_gates node")
            elif 'active' in current_state:
                logging.info("send_gates node already active")
            else:
                logging.warning(f"send_gates node in unexpected state: {current_state}")
                return None
        except subprocess.CalledProcessError as e:
            logging.warning(f"Failed to activate send_gates node, will retry: {e}")
            return None
        return []

    # Add a check to ensure map file exists before proceeding
    def check_map_exists(context):
        map_file = context.launch_configurations['map_file']
        if not os.path.exists(map_file):
            logging.warning(f"Map file {map_file} does not exist yet, will retry...")
            return None  # Return None to indicate retry needed
        else:
            logging.info(f"Map file {map_file} exists!")
        return []

    # Launch sequence with retry mechanism
    ld = LaunchDescription()
    ld.add_action(qt_qpa_platform)
    for launch_arg in launch_args:
        ld.add_action(launch_arg)

    ld.add_action(TimerAction(period=1.0, actions=[OpaqueFunction(function=get_map_name)]))
    ld.add_action(TimerAction(period=2.0, actions=[OpaqueFunction(function=print_env)]))
    ld.add_action(TimerAction(period=3.0, actions=[OpaqueFunction(function=evaluate_rviz)]))
    

    

    # 1. Launch Gazebo and map environment first
    ld.add_action(TimerAction(period=10.0, actions=[gazebo_launch]))
    ld.add_action(TimerAction(period=15.0, actions=[generate_config_node]))
    # 2. Wait for Gazebo to be fully loaded before starting map package
    
        # 3. Configure and activate lifecycle nodes with proper delays and retries
    # Add multiple attempts for each transition with increasing delays
    ld.add_action(TimerAction(period=25.0, actions=[OpaqueFunction(function=configure_borders)]))
    ld.add_action(TimerAction(period=30.0, actions=[OpaqueFunction(function=configure_borders)]))
    ld.add_action(TimerAction(period=35.0, actions=[OpaqueFunction(function=configure_borders)]))
    
    ld.add_action(TimerAction(period=40.0, actions=[OpaqueFunction(function=activate_borders)]))
    ld.add_action(TimerAction(period=45.0, actions=[OpaqueFunction(function=activate_borders)]))
    ld.add_action(TimerAction(period=50.0, actions=[OpaqueFunction(function=activate_borders)]))
    
    ld.add_action(TimerAction(period=55.0, actions=[OpaqueFunction(function=configure_obstacles)]))
    ld.add_action(TimerAction(period=60.0, actions=[OpaqueFunction(function=configure_obstacles)]))
    ld.add_action(TimerAction(period=65.0, actions=[OpaqueFunction(function=configure_obstacles)]))
    
    ld.add_action(TimerAction(period=70.0, actions=[OpaqueFunction(function=activate_obstacles)]))
    ld.add_action(TimerAction(period=75.0, actions=[OpaqueFunction(function=activate_obstacles)]))
    ld.add_action(TimerAction(period=80.0, actions=[OpaqueFunction(function=activate_obstacles)]))
    
    ld.add_action(TimerAction(period=85.0, actions=[OpaqueFunction(function=configure_gates)]))
    ld.add_action(TimerAction(period=90.0, actions=[OpaqueFunction(function=configure_gates)]))
    ld.add_action(TimerAction(period=95.0, actions=[OpaqueFunction(function=configure_gates)]))
    
    ld.add_action(TimerAction(period=100.0, actions=[OpaqueFunction(function=activate_gates)]))
    ld.add_action(TimerAction(period=105.0, actions=[OpaqueFunction(function=activate_gates)]))
    ld.add_action(TimerAction(period=110.0, actions=[OpaqueFunction(function=activate_gates)]))
    
    ld.add_action(TimerAction(period=112.0, actions=[map_pkg_node]))
    ld.add_action(TimerAction(period=115.0, actions=[create_map_node]))
    
    # Add multiple checks with increasing delays to ensure map is created
    ld.add_action(TimerAction(period=120.0, actions=[OpaqueFunction(function=check_map_exists)]))
    ld.add_action(TimerAction(period=125.0, actions=[OpaqueFunction(function=check_map_exists)]))
    ld.add_action(TimerAction(period=130.0, actions=[OpaqueFunction(function=check_map_exists)]))
    

    ld.add_action(TimerAction(period=132.0, actions=[map_server_node]))
    ld.add_action(TimerAction(period=135.0, actions=[map_server_lifecycle_manager]))

    # 5. Launch robot state publishers and spawn robots with increased delays
    # Reduce the delay between launching RSP and spawning robots to ensure they appear together
    ld.add_action(TimerAction(period=145.0, actions=[OpaqueFunction(function=launch_rsp)]))
    # Add more delay before spawning robots to ensure Gazebo is ready
    ld.add_action(TimerAction(period=145.0, actions=[OpaqueFunction(function=spawn_shelfini)]))

    # 6. Launch navigation with longer delay to ensure robots are fully spawned
    ld.add_action(TimerAction(period=155.0, actions=[OpaqueFunction(function=launch_navigation)]))
    ld.add_action(TimerAction(period=165.0, actions=[OpaqueFunction(function=launch_evacuation_nodes)]))

    # 7. Launch controller last with sufficient delay
    
    
    return ld
