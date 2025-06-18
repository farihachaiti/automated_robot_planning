#!/usr/bin/env python3

import os
import yaml
import random
import math
from pathlib import Path
import subprocess
import sys
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression, Command
from launch.conditions import UnlessCondition, IfCondition
from launch.actions import (OpaqueFunction, DeclareLaunchArgument, 
                          IncludeLaunchDescription, GroupAction, 
                          ExecuteProcess, SetEnvironmentVariable, TimerAction)
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
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

def launch_rsp(context):
    nodes = []
    shelfino_desc_pkg = get_package_share_directory('shelfino_description')
    robot_desc = os.path.join(shelfino_desc_pkg, 'models', 'shelfino_v1.xacro')
    use_sim_time = context.launch_configurations['use_sim_time']
    
    for shelfino_id in range(int(context.launch_configurations['n_shelfini'])):
        shelfino_name = "shelfino" + str(shelfino_id)
        # Robot State Publisher
        nodes.append(
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                namespace=shelfino_name,
                parameters=[{
                    'robot_description': Command(['xacro ', robot_desc, ' robot_id:=', str(shelfino_id)]),
                    'use_sim_time': True,
                }],
                output='screen'
            )
        )
    return nodes

def is_position_valid(x, y, robot_positions, obstacles, min_robot_distance=1.5, min_obstacle_distance=0.7):
    """Check if a position is valid considering robots and obstacles."""
    # Check distance from other robots
    for rx, ry, _ in robot_positions:
        if math.hypot(x - rx, y - ry) < min_robot_distance:
            return False
    
    # Check distance from obstacles
    for ox, oy, radius in obstacles:
        if math.hypot(x - ox, y - oy) < (radius + min_obstacle_distance):
            return False
    
    return True

def generate_robot_positions(n_robots, obstacles, map_bounds, max_attempts=10):
    """Generate positions for robots that avoid obstacles and other robots.
    
    Args:
        n_robots: Number of robot positions to generate
        obstacles: List of (x, y, radius) tuples representing obstacles
        map_bounds: Tuple of (x_min, x_max, y_min, y_max) for the map boundaries
        max_attempts: Maximum number of attempts to find a valid position for each robot
        
    Returns:
        List of (x, y, yaw) tuples for each robot
        
    Raises:
        RuntimeError: If a valid position cannot be found for a robot after max_attempts
    """
    robot_positions = []
    x_min, x_max, y_min, y_max = map_bounds
    
    for robot_num in range(1, n_robots + 1):
        position_found = False
        
        for attempt in range(max_attempts):
            x = random.uniform(x_min, x_max)
            y = random.uniform(y_min, y_max)
            yaw = random.uniform(-math.pi, math.pi)
            
            if is_position_valid(x, y, robot_positions, obstacles):
                robot_positions.append((x, y, yaw))
                position_found = True
                logging.info(f"Found position for robot {robot_num} after {attempt + 1} attempts")
                break
        
        if not position_found:
            raise RuntimeError(
                f"Failed to find valid position for robot {robot_num} after {max_attempts} attempts. "
                f"Current map bounds: {map_bounds}, existing robots: {len(robot_positions)}, "
                f"obstacles: {len(obstacles)}"
            )
    
    return robot_positions

def load_obstacles_from_yaml(yaml_path):
    """Load obstacle data and map dimensions from YAML configuration file.
    
    Returns:
        tuple: (obstacles, map_bounds) where map_bounds is (x_min, x_max, y_min, y_max)
    """
    obstacles = []
    map_bounds = (-6.0, 6.0, -6.0, 6.0)  # Default bounds if not specified in YAML
    
    try:
        with open(yaml_path, 'r') as file:
            config = yaml.safe_load(file)
            
        # Get map dimensions from root parameters
        root_params = config.get('/', {}).get('ros__parameters', {})
        dx = float(root_params.get('dx', 12.0))  # Default to 12.0 if not specified
        dy = float(root_params.get('dy', 12.0))  # Default to 12.0 if not specified
        
        # Calculate map bounds (centered at origin)
        map_bounds = (-dx/2, dx/2, -dy/2, dy/2)
        
        # Get obstacle parameters
        obs_params = config.get('/send_obstacles', {}).get('ros__parameters', {})
        
        # Process vector-based obstacles
        vect_x = obs_params.get('vect_x', [])
        vect_y = obs_params.get('vect_y', [])
        vect_dim_x = obs_params.get('vect_dim_x', [])
        vect_dim_y = obs_params.get('vect_dim_y', [])
        
        # Add vector-based obstacles
        for i in range(min(len(vect_x), len(vect_y))):
            x = vect_x[i]
            y = vect_y[i]
            # Use max dimension as radius for simplicity
            radius = max(
                vect_dim_x[i] if i < len(vect_dim_x) else 0.5,
                vect_dim_y[i] if i < len(vect_dim_y) else 0.5
            ) / 2.0
            obstacles.append((x, y, radius))
        
        logging.info(f"Loaded {len(obstacles)} obstacles from YAML config")
        
    except Exception as e:
        logging.error(f"Error loading obstacles from YAML: {e}")
        # Add some default obstacles if loading fails
        obstacles = [(0.0, 0.0, 1.0)]
    
    return obstacles, map_bounds

def spawn_shelfini(context):
    nodes = []
    
    shelfino_gaze_pkg = get_package_share_directory('shelfino_gazebo')
    shelfino_desc_pkg = get_package_share_directory('shelfino_description')
    shelfino_nav2_pkg = get_package_share_directory('shelfino_navigation')
    automated_robot_planning_pkg = get_package_share_directory('automated_robot_planning')
    use_sim_time = context.launch_configurations['use_sim_time']
    robot_desc = os.path.join(shelfino_desc_pkg, 'models', 'shelfino_v1.xacro')
    gazebo_world_file = context.launch_configurations['gazebo_world_file']
    map_file_path = os.path.join(shelfino_nav2_pkg, 'maps', 'dynamic_map.yaml')
    nav2_rviz_config_file_path = os.path.join(shelfino_nav2_pkg, 'rviz', 'shelfino_nav.rviz')
    nav2_params_file_path = os.path.join(shelfino_nav2_pkg, 'config', 'shelfino.yaml')
    tf_config_file_path = os.path.join(automated_robot_planning_pkg, 'tf_config.yaml')
    gazebo_bridge_plugins_params_file_path = os.path.join(
        get_package_share_directory('shelfino_description'),
        'config',
        'bridge.yaml'
    )
    
    # Load obstacles and map dimensions from YAML config
    yaml_path = os.path.join(
        get_package_share_directory('map_pkg'),
        'config',
        'full_config.yaml'
    )
    
    if os.path.exists(yaml_path):
        logging.info(f"Loading obstacles and map dimensions from: {yaml_path}")
        obstacles, map_bounds = load_obstacles_from_yaml(yaml_path)
        logging.info(f"Using map bounds: {map_bounds}")
    else:
        logging.warning(f"Could not find config file at {yaml_path}. Using default obstacles and map bounds.")
        obstacles = [(0.0, 0.0, 1.0)]  # Default obstacle
        map_bounds = (-6.0, 6.0, -6.0, 6.0)  # Default bounds
    
    # Generate robot positions avoiding obstacles
    n_robots = int(context.launch_configurations['n_shelfini'])
    robot_positions = generate_robot_positions(n_robots, obstacles, map_bounds)
    
    logging.info(f"Generated {len(robot_positions)} robot positions")
    for i, (x, y, yaw) in enumerate(robot_positions):
        logging.info(f"Robot {i}: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")
    
    # Verify we have enough valid positions
    if len(robot_positions) < n_robots:
        raise RuntimeError(
            f"Failed to generate valid positions for all robots. "
            f"Requested {n_robots} robots but only found valid positions for {len(robot_positions)}. "
            f"Consider increasing the map size or reducing the number of robots/obstacles."
        )
    
    # Create nodes with the generated positions
    for shelfino_id in range(int(context.launch_configurations['n_shelfini'])):
        shelfino_name = "shelfino" + str(shelfino_id)
        x_pos, y_pos, yaw_pos = robot_positions[shelfino_id]
        print(f"{shelfino_name} position: x={x_pos:.2f}, y={y_pos:.2f}, yaw={yaw_pos:.2f}")
    
        # Create the SDF file path
        model_file = os.path.join(shelfino_desc_pkg, 'models', 'shelfino_v1.xacro')
        
        # Navigation parameters
        nav2_params_file = context.launch_configurations['nav2_params_file']
        nav2_rviz_config_file = context.launch_configurations['nav2_rviz_config_file']
        
        # Group all actions for this robot
        spawn_node = GroupAction([
            # Static transform publisher: map -> odom for each robot
   
            # Create a node to spawn each robot
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-topic', PythonExpression(["'/", shelfino_name, "/robot_description", "'"]),
                    '-entity', shelfino_name,
                    '-robot_namespace', shelfino_name,
                    '-name', shelfino_name,
                    '-x', str(x_pos),
                    '-y', str(y_pos),
                    '-z', '0.1',
                    '-Y', str(yaw_pos),
                    '--wait-ms', '1000'  # Add a small delay to ensure Gazebo is ready
                ],
                output='screen',
                # Add a condition to prevent immediate execution
                condition=launch.conditions.IfCondition(use_sim_time)
            ),

            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                    f'/{shelfino_name}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                    f'/{shelfino_name}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                    f'/{shelfino_name}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                    f'/{shelfino_name}/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                    '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                    '/tf_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                    #'--ros-args',
                    #'-p',
                    #f'config_file:={os.path.join(get_package_share_directory("shelfino_description"), "config", "bridge.yaml")}',
                ],
                output='screen',
                condition=launch.conditions.IfCondition(LaunchConfiguration('use_sim_time'))
            ),
            
            #Node(
            #    package='tf2_ros',
            #    executable='static_transform_publisher',
            #    namespace=shelfino_name,
            #    arguments=['0', '0', '0', '0', '0', '0', 'map', f'{shelfino_name}/odom'],
            #    output='screen',
            #),
            # Bridge node is now created separately, not per-robot

            # Only publish map->odom transform if not in simulation (AMCL will handle it in simulation)
      
            # Remove the static transform from odom to base_link to allow dynamic movement
            # This transform should be published by your robot's odometry source
         
            #IncludeLaunchDescription(
            #    PythonLaunchDescriptionSource([
            #        os.path.join(get_package_share_directory('shelfino_node'), 'launch', 'bringup.launch.py')
            #    ]),
            #    launch_arguments={
            #        'use_sim_time': use_sim_time,
            #        'shelfino_id': str(shelfino_id),
            #        'map_file': map_file_path,
            #        'headless': 'true',
            #        'namespace': shelfino_name,
            #        'use_lidar': 'true'  # Disable LIDAR in simulation
            #    }.items(),
            #    condition=launch.conditions.IfCondition(use_sim_time)
            #),

            # Robot state publisher - runs immediately but with use_sim_time=True
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace=shelfino_name,
                parameters=[{
                    'robot_description': Command([
                        'xacro ', robot_desc,
                        ' robot_id:=', str(shelfino_id)
                    ]),
                    'use_sim_time': True,
                    'publish_frequency': 50.0,  # Increase publish frequency
                    'ignore_timestamp': False,  # Use timestamps from joint states
                }],
                output='screen',
            ),



            # Joint state publisher - runs immediately but with use_sim_time=True
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
                    'rate': 50.0,  # Match robot_state_publisher frequency
                }],
                output='screen',
            ),

            
          # QoS Bridge node
            Node(
                package='automated_robot_planning',
                namespace=shelfino_name,
                executable='QosBridge.py',
                parameters=[
                    {'use_sim_time': True},
                    tf_config_file_path
                ],
                output='screen'
            ),
            # Include navigation stack for this robot
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(get_package_share_directory('shelfino_navigation'), 'launch', 'shelfino_nav.launch.py')
                ]),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'robot_id': str(shelfino_id),
                    'map_file': map_file_path,
                    'nav2_params_file': nav2_params_file_path,
                    'nav2_rviz_config_file': nav2_rviz_config_file_path,
                    'headless': 'true',
                    'namespace': shelfino_name,
                    'robot_description': robot_desc,
                    'initial_x': str(x_pos),
                    'initial_y': str(y_pos),
                    'initial_yaw': str(yaw_pos),
                    'set_initial_pose': 'true',
                }.items(),
                condition=launch.conditions.IfCondition(use_sim_time)
            ),
     


            TimerAction(
                period=5.0,  # delay per robot
                actions=[
                    Node(
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
                            nav2_params_file
                        ],
                        remappings=[
                            ('/tf', '/tf'),
                            ('/tf_static', '/tf_static'),
                            ('/navigate_through_poses', 'navigate_through_poses'),
                            ('amcl_pose', 'amcl_pose'),
                            ('cmd_vel', 'cmd_vel'),
                        ],
                        output='screen',
                        emulate_tty=True,
                        condition=launch.conditions.IfCondition(use_sim_time)
                    )
                ]
            ),

            TimerAction(
                period=10.0,  # delay per robot
                actions=[
                    Node(
                        package='shelfino_gazebo',
                        executable='destroy_shelfino',
                        name='destroy_shelfino',
                        namespace=shelfino_name,
                        output='screen',
                        parameters=[{
                            'use_sim_time': True
                        }],
                        condition=launch.conditions.IfCondition(use_sim_time)
                    )
                ]
            ),

       
            ])
            
             
            
    
        
        nodes.append(spawn_node)
    
    return nodes

def generate_launch_description():
    shelfino_desc_pkg = get_package_share_directory('shelfino_description')
    shelfino_nav2_pkg = get_package_share_directory('shelfino_navigation')
    shelfino_gaze_pkg = get_package_share_directory('shelfino_gazebo')
    map_env_pkg = get_package_share_directory('map_pkg')
    
    map_env_params_file_path = os.path.join(map_env_pkg, 'config', 'map_config.yaml')
    nav2_params_file_path = os.path.join(shelfino_nav2_pkg, 'config', 'shelfino.yaml')
    gen_map_params_file_path = os.path.join(map_env_pkg, 'config', 'full_config.yaml')
   
    qt_qpa_platform = SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb')
    # Add FastDDS configuration to disable SHM transport and fix scan topic issues
    fastdds_config = SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', 
                                          os.path.join(os.getcwd(), 'fastdds_no_shm.xml'))
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    n_shelfini = LaunchConfiguration('n_shelfini', default='3')
    map_file_path = os.path.join(shelfino_nav2_pkg, 'maps', 'dynamic_map.yaml')
    use_gui = LaunchConfiguration('use_gui', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    rviz_config_file = LaunchConfiguration('rviz_config_file', 
        default=os.path.join(shelfino_desc_pkg, 'rviz', 'shelfino.rviz'))
    gazebo_world_file = LaunchConfiguration('gazebo_world_file', default=os.path.join(shelfino_gaze_pkg, 'worlds', 'hexagon_world','model.sdf'))
    robot_desc = os.path.join(shelfino_desc_pkg, 'models', 'shelfino_v1.xacro')
    gen_map_params_file = LaunchConfiguration('gen_map_params_file', default=gen_map_params_file_path)  
    map_file = LaunchConfiguration('map_file', 
        default=os.path.join(shelfino_nav2_pkg, 'maps', 'dynamic_map.yaml'))
    nav2_rviz_config_file_path = os.path.join(shelfino_nav2_pkg, 'rviz', 'shelfino_nav.rviz')
    nav2_params_file = LaunchConfiguration('nav2_params_file', default=nav2_params_file_path)
    nav2_rviz_config_file = LaunchConfiguration('nav2_rviz_config_file', 
        default=os.path.join(shelfino_nav2_pkg, 'rviz', 'shelfino_nav.rviz'))
    map_env_params_file = LaunchConfiguration('map_env_params_file', default=map_env_params_file_path)

    launch_args = [
        DeclareLaunchArgument(
            'gen_map_params_file',
            default_value=gen_map_params_file_path,
            description='Full path to the final params file file to use'
        ),
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
        DeclareLaunchArgument('map_file', default_value=map_file_path, 
                            description='Full path to map file to load'),
        DeclareLaunchArgument('nav2_params_file', default_value=nav2_params_file_path, 
                            description='Full path to the nav2 params file to use'),
        DeclareLaunchArgument('nav2_rviz_config_file', default_value=nav2_rviz_config_file_path, 
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

    gen_map_params_file_params = RewrittenYaml(
        source_file=map_env_params_file,
        param_rewrites={'send_victims' : {'victims_activated' : 'false'}},
        convert_types=True
    )

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
            'extra_gazebo_args': '--ros-args --param use_sim_time:=true --ros-args -r /clock:=/clock --ros-args',
        }.items()
    )

    # Add the bridge node (only once, not per-robot)
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '--ros-args',
            '-p',
            f'config_file:={os.path.join(get_package_share_directory("shelfino_description"), "config", "bridge.yaml")}',
        ],
        output='screen',
        condition=launch.conditions.IfCondition(LaunchConfiguration('use_sim_time'))
    )

    # Add a world reset action to clear existing entities before spawning new ones
    world_reset_action = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/world/hexagon/reset', 'gz.msgs.Empty'],
        output='screen',
        condition=launch.conditions.IfCondition(LaunchConfiguration('use_sim_time'))
    )

    # Add a more comprehensive world clearing action
    def clear_world_entities(context):
        try:
            # Clear all entities except the basic world elements
            entities_to_remove = [
                'borders', 'gate', 'box', 'cylinder', 'shelfino0', 'shelfino1', 'shelfino2',
                'ground_plane', 'sun'  # Remove these if they exist as duplicates
            ]
            
            for entity in entities_to_remove:
                try:
                    subprocess.run([
                        'ros2', 'service', 'call', '/world/hexagon/remove', 
                        'gz.msgs.Entity', '--ros-args', '-p', f'name:={entity}'
                    ], capture_output=True, timeout=5)
                    logging.info(f"Attempted to remove entity: {entity}")
                except subprocess.TimeoutExpired:
                    logging.warning(f"Timeout removing entity: {entity}")
                except Exception as e:
                    logging.debug(f"Entity {entity} may not exist or already removed: {e}")
            
            # Also try to remove any entities with timestamp-based names
            try:
                subprocess.run([
                    'ros2', 'service', 'call', '/world/hexagon/reset', 'gz.msgs.Empty'
                ], capture_output=True, timeout=5)
                logging.info("World reset completed")
            except Exception as e:
                logging.warning(f"World reset failed: {e}")
                
        except Exception as e:
            logging.warning(f"Failed to clear world entities: {e}")
        return []

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
            'use_sim_time': True
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
            'generate_new_config': True,
            'use_sim_time': True
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
            {'use_sim_time': True},
            {'yaml_filename': map_file},
            {'frame_id': 'map'},
            {'topic_name': 'map'},
            {'map_subscribe_transient_local': True},
            {'save_map_timeout': 5.0},
            {'autostart': True}
        ],
        remappings=[
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static')
        ]
    )

    # Global map server lifecycle manager
    map_server_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='map_server_lifecycle_manager',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server'],
            'bond_timeout': 60.0,
            'bond_heartbeat_period': 2.0,
            'attempt_respawn_reconnection': True
        }]
    )

    # Add lifecycle node activation commands with improved logic
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
            logging.warning(f"Failed to configure send_gates node, will retry: {e}")
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
    ld.add_action(fastdds_config)
    for launch_arg in launch_args:
        ld.add_action(launch_arg)

    ld.add_action(TimerAction(period=1.0, actions=[OpaqueFunction(function=get_map_name)]))
    ld.add_action(TimerAction(period=2.0, actions=[OpaqueFunction(function=print_env)]))
    ld.add_action(TimerAction(period=3.0, actions=[OpaqueFunction(function=evaluate_rviz)]))
    

    

    # 1. Launch Gazebo and map environment first
    ld.add_action(TimerAction(period=10.0, actions=[gazebo_launch]))
    # Add the bridge node (only once, not per-robot)
    ld.add_action(TimerAction(period=14.0, actions=[bridge_node]))
    ld.add_action(TimerAction(period=15.0, actions=[world_reset_action]))
    ld.add_action(TimerAction(period=16.0, actions=[OpaqueFunction(function=clear_world_entities)]))
    ld.add_action(TimerAction(period=17.0, actions=[generate_config_node]))
    # 2. Wait for Gazebo to be fully loaded before starting map package
    
    # 3. Configure and activate lifecycle nodes with single attempts and proper delays
    # Single attempt for each transition with proper delays
    ld.add_action(TimerAction(period=25.0, actions=[OpaqueFunction(function=configure_borders)]))
    ld.add_action(TimerAction(period=35.0, actions=[OpaqueFunction(function=activate_borders)]))
    
    ld.add_action(TimerAction(period=45.0, actions=[OpaqueFunction(function=configure_obstacles)]))
    ld.add_action(TimerAction(period=55.0, actions=[OpaqueFunction(function=activate_obstacles)]))
    
    ld.add_action(TimerAction(period=65.0, actions=[OpaqueFunction(function=configure_gates)]))
    ld.add_action(TimerAction(period=75.0, actions=[OpaqueFunction(function=activate_gates)]))
    
    ld.add_action(TimerAction(period=85.0, actions=[map_pkg_node]))
    ld.add_action(TimerAction(period=90.0, actions=[create_map_node]))
    
    # Add single check to ensure map is created
    ld.add_action(TimerAction(period=100.0, actions=[OpaqueFunction(function=check_map_exists)]))
    

    ld.add_action(TimerAction(period=105.0, actions=[map_server_node]))
    ld.add_action(TimerAction(period=110.0, actions=[map_server_lifecycle_manager]))



    # 5. Launch robot state publishers and spawn robots with navigation included
    # Launch RSP first
    #ld.add_action(TimerAction(period=145.0, actions=[OpaqueFunction(function=launch_rsp)]))
    
    # Spawn robots with navigation and controller included
    # Increased delay to ensure world reset and map spawning are complete
    ld.add_action(TimerAction(period=130.0, actions=[OpaqueFunction(function=spawn_shelfini)]))
    
    return ld
