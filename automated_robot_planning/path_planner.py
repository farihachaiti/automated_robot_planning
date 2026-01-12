#!/usr/bin/env python3

import threading
import rclpy
from rclpy.node import Node
#from automated_robot_planning.obstacle_detector import ObstacleDetector
import math
#import dubins
import numpy as np
import tf2_ros
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseArray, Polygon, PoseStamped, Point,Pose
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion, Twist
from obstacles_msgs.msg import ObstacleArrayMsg
from visualization_msgs.msg import MarkerArray, Marker
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import xacro
#from urdf_parser_py.urdf import URDF
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import JointState
from nav_msgs.msg import Path, Odometry
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from tf2_ros import TransformException



import tf2_ros
from nav2_msgs.action import FollowPath, ComputePathToPose
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import TransformStamped
import asyncio
#from dubins import DubinsPath
from dubins_curve import Dubins
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterType
from rclpy.parameter import Parameter
from rtree import index

collision_detected = False


class PathPlanner(Node): 
    def __init__(self):
        super().__init__('path_planner')
        self.declare_parameter('robot_name', 'shelfino0')
        robot_name = self.get_parameter('robot_name').get_parameter_value().string_value.strip().rstrip('/')
        self.namespace = robot_name
        #self.controller_node = controller_node   
        self.obs = None

        #self.gate_position = gate_position 
        self.const_linear_velocity = [0.2, 0.0, 0.5]
        self.walls = [] 
        self.solid_obs = []
        self.r = 0.1
        self.raytrace_min_range = 0.0
        self.raytrace_max_range = 3.0
        self.costmap_resolution = 0.05000000074505806
        self.costmap_size = [60, 60]
        self.costmap_origin = (-1.45, -1.45)
        # Use explicit dtype with full numpy.dtype specification for NumPy 2.0+ compatibility
        self.local_costmap = np.zeros(
            (60, 60),  # (height, width)
            dtype=np.dtype('int8')
        )
        #self.robot = ObstacleDetector.polygon_offsetting(robot, 0.1, isRobot=True)
        self.safety_distance = 0.1
        self.obstacle_threshold = 1
        self.robot_id = None
        PathPlanner.costmap = None
        PathPlanner.g_costmap = None
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)

        self.declare_parameter(
        'obstacles',
        "",
        ParameterDescriptor(type=ParameterType.PARAMETER_STRING)
    )

        obstacles = self.get_parameter('obstacles').get_parameter_value().string_value
        flat_obstacles = [float(x) for x in obstacles.split(',')]
        # Now reconstruct as needed
        #self.obstacles = [(flat_obstacles[i], flat_obstacles[i+1], flat_obstacles[i+2]) for i in range(0, len(flat_obstacles), 3)]
        
        self.initial_x = self.get_parameter('initial_x').value
        self.initial_y = self.get_parameter('initial_y').value
        self.initial_yaw = self.get_parameter('initial_yaw').value
        self.start_pose = PoseStamped()
        self.start_pose.header.frame_id = 'map'
        self.start_pose.header.stamp = self.get_clock().now().to_msg()
        self.start_pose.pose.position.x = self.initial_x
        self.start_pose.pose.position.y = self.initial_y
        self.start_pose.pose.position.z = 0.0
        q = quaternion_from_euler(0, 0, self.initial_yaw)
        self.start_pose.pose.orientation.x = q[0]
        self.start_pose.pose.orientation.y = q[1]
        self.start_pose.pose.orientation.z = q[2]
        self.start_pose.pose.orientation.w = q[3]

        a = 10.0  # hexagon side length

        # Compute workspace bounds
        x_max = a
        x_min = -a
        y_max = math.sqrt(3) * a / 2
        y_min = -math.sqrt(3) * a / 2

        # Print nicely
        print(f"x_min = {x_min:.2f}")
        print(f"x_max = {x_max:.2f}")
        print(f"y_min = {y_min:.2f}")
        print(f"y_max = {y_max:.2f}")

        # Optionally pack into a tuple
        #self.workspace_bounds = (x_min, y_min, x_max, y_max)
        self.workspace_bounds = [(x_min, x_max), (y_min, y_max), (-math.pi, math.pi)] 
        self.workspace_dimensions_lengths_np = np.array([(x_min, x_max), (y_min, y_max), (-math.pi, math.pi)]) 
        self.goal_pose = PoseStamped()
        self.obs_subscription = self.create_subscription(
            ObstacleArrayMsg,
            '/obstacles',
            self.obstacle_callback,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                depth=10
            )
        )
        self.gate_position = PoseArray()
        qos_profile = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.create_subscription(
            OccupancyGrid,
            f'/{self.namespace}/local_costmap/costmap',
            self.local_costmap_callback,
            qos_profile)
        
        self.create_subscription(
            OccupancyGrid,
            f'/{self.namespace}/global_costmap/costmap',
            self.global_costmap_callback,
            qos_profile)

        self.gate_sub = self.create_subscription(
            PoseArray,
            '/gate_position',
            self.gate_position_callback,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                depth=10
            )
        )
        self.plan_timer = self.create_timer(1.0, self.robot_task)
        # Initialize TF broadcaster
        '''self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.current_pose = None
        
        # Create a timer to publish TF at 50Hz (20ms)
        self.tf_timer = self.create_timer(0.02, self.publish_tf)  # 50Hz'''

        self.path_pub = self.create_publisher(Path, f'/{self.namespace}/plan', 10)
        self.path_pub1 = self.create_publisher(Path, f'/{self.namespace}/plan1', 10)
        self.follow_path_client = ActionClient(self, FollowPath, f'/{self.namespace}/follow_path')
        self.compute_path_client = ActionClient(self, ComputePathToPose, f'/{self.namespace}/compute_path_to_pose')
        self.plan_timer = self.create_timer(1.0, self.robot_task)


        # Define QoS profile for the publisher
   

        #self.get_logger().info(f'Published initialpose: x={self.start_pose[0]}, y={self.start_pose[1]}, yaw={self.start_pose[2]}')
   
        
     

        self.goal_pose_publisher = self.create_publisher(PoseStamped, f'/{self.namespace}/goal_pose', 10)
        self.cmd_vel_publisher = self.create_publisher(
            Twist, 
            f'/{self.namespace}/cmd_vel', 
            10)

        
        self.odom_sub = self.create_subscription(
            Odometry,
            f'/{self.namespace}/odom',
            self.odom_callback,
            10
        )

        self.gazebo_marker_pub = self.create_publisher(
            MarkerArray, 
            '/visualization_marker_array', 
            10
        )   

        self.goal_set_event = asyncio.Event()
        self.path_published_event = asyncio.Event()
        self.robot_task_event = asyncio.Event()
        self.navigation_complete = asyncio.Event()


    # ################################ These three functions are part of the logic that calls the action server for computing a path, and then saving that path object in the dedicated dictionary. ######################
    def compute_robots_paths(self):

        self.get_logger().info(f'{robot_name}')

        goal_msg = ComputePathToPose.Goal()

        goal_msg.goal.header.frame_id = 'map'
        goal_msg.goal.pose.position.x = self.goal_pose.pose.position.x
        goal_msg.goal.pose.position.y = self.goal_pose.pose.position.y

        self.compute_path_client.wait_for_server()
        robot_future = self.compute_path_client.send_goal_async(goal_msg)
        robot_future.add_done_callback(lambda msg: self.compute_robots_paths_response_callback(msg))

    def compute_robots_paths_response_callback(self, future):
        goal_handle = future.result()
        self._get_results_future = goal_handle.get_result_async()
        self._get_results_future.add_done_callback(lambda msg: self.compute_robots_paths_result_callback(msg))

    def compute_robots_paths_result_callback(self, future):
        self.robot_paths[self.namespace] = future.result().result.path
        self.robot_paths_todo[self.namespace] = future.result().result.path

    def obstacle_callback(self, msg):
        """
        Callback for processing incoming obstacle data.
        Converts obstacle messages into a format suitable for path planning.
        """
        self.get_logger().info(f'Received obstacle message')
        self.get_logger().info(f'Message type: {type(msg)}')
        self.get_logger().info(f'Message dir: {dir(msg)}')
        self.get_logger().info(f'Has obstacles attribute: {hasattr(msg, "obstacles")}')
        
        if hasattr(msg, 'obstacles'):
            self.get_logger().info(f'Number of obstacles: {len(msg.obstacles)}')
            if len(msg.obstacles) > 0:
                self.get_logger().info(f'First obstacle type: {type(msg.obstacles[0])}')
                self.get_logger().info(f'First obstacle attributes: {dir(msg.obstacles[0])}')
        else:
            self.get_logger().warning('Message does not have obstacles attribute')
            
        self.obstacles = []

        for i, obs in enumerate(msg.obstacles):
            try:
                radius = obs.radius
                polygon = obs.polygon

                self.get_logger().info(f"🧱 Obstacle {i}: radius={radius:.3f}, num_points={len(polygon.points)}")

                # --- Case 1: Cylinder (radius > 0, one point = center)
                if radius > 0.0:
                    if not polygon.points:
                        self.get_logger().warning(f"⚠️  Cylinder {i} has no center point — skipping.")
                        continue

                    cx = polygon.points[0].x
                    cy = polygon.points[0].y
                    x_min, x_max = cx - radius, cx + radius
                    y_min, y_max = cy - radius, cy + radius

                    self.get_logger().info(
                        f"✅ Cylinder {i} center=({cx:.2f},{cy:.2f}), "
                        f"radius={radius:.2f}, bbox=[{x_min:.2f},{x_max:.2f},{y_min:.2f},{y_max:.2f}]"
                    )

                # --- Case 2: Polygon (radius == 0, multiple points)
                else:
                    if not polygon.points:
                        self.get_logger().warning(f"⚠️  Polygon {i} has no vertices — skipping.")
                        continue

                    xs = [p.x for p in polygon.points]
                    ys = [p.y for p in polygon.points]

                    x_min, x_max = min(xs), max(xs)
                    y_min, y_max = min(ys), max(ys)

                    self.get_logger().info(
                        f"✅ Polygon {i} with {len(xs)} points, bbox=[{x_min:.2f},{x_max:.2f},{y_min:.2f},{y_max:.2f}]"
                    )

                # --- Finalize obstacle bounding box
                z_min, z_max = 0.0, 1.0
                obstacle_bounds = (x_min, y_min, z_min, x_max, y_max, z_max)
                self.obstacles.append(obstacle_bounds)

                self.get_logger().info(f"📦 Processed obstacle {i}: {obstacle_bounds}")

            except Exception as e:
                self.get_logger().error(f"❌ Exception processing obstacle {i}: {repr(e)}")

        self.dimensions = len(self.workspace_dimensions_lengths_np)
        p = index.Property()
        p.dimension = self.dimensions
        self.obstacles_np = np.array(self.obstacles) if self.obstacles else np.array([])
        
        if len(self.obstacles_np) > 0:
            for i, obs in enumerate(self.obstacles_np):
                self.get_logger().info(f"🗂️  Rtree input {i}: {obs}")

            self.obs = index.Index(self.obstacle_generator(self.obstacles_np), interleaved=True, properties=p)
            self.get_logger().info(f'Successfully processed {len(self.obstacles_np)} obstacles')
            self.get_logger().info(f'Obstacle bounds: {self.obstacles_np}')
        else:
            self.obs = None
            self.get_logger().info('No obstacles to process')


    def gate_position_callback(self, msg):
     
        self.get_logger().info(f'Received {len(msg.poses)} gate positions')
        self.gate_position = msg.poses[0]
        self.get_logger().info(f'Gate position: x={self.gate_position.position.x}, y={self.gate_position.position.y}')   
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = 'map' # Assuming gate position is in the map frame
        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
        self.goal_pose.pose.position.x = self.gate_position.position.x
        self.goal_pose.pose.position.y = self.gate_position.position.y
        self.goal_pose.pose.position.z = self.gate_position.position.z
        self.goal_pose.pose.orientation.x = self.gate_position.orientation.x
        self.goal_pose.pose.orientation.y = self.gate_position.orientation.y
        self.goal_pose.pose.orientation.z = self.gate_position.orientation.z
        self.goal_pose.pose.orientation.w = self.gate_position.orientation.w 
        #_, _, yaw = euler_from_quaternion([self.goal_pose.pose.orientation.x, self.goal_pose.pose.orientation.y, self.goal_pose.pose.orientation.z, self.goal_pose.pose.orientation.w])
        #self.goal_pose_publisher.publish(self.goal_pose)
        #self.compute_robots_paths()
        self.goal_set_event.set()
        self.get_logger().info('Published goal to goal_pose')
        if self.obs is not None:
            self.robot_task()
        # Create goal pose from the first gate position
        
    def publish_path_to_gazebo(self, path):
        """Convert Nav2 Path to Gazebo MarkerArray for visualization"""
        marker_array = MarkerArray()
        
        # 1. Create path line marker
        line_marker = Marker()
        line_marker.header = path.header
        line_marker.ns = "path"
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.1  # Line width
        line_marker.color.a = 1.0  # Alpha
        line_marker.color.r = 0.0  # Red
        line_marker.color.g = 1.0  # Green
        line_marker.color.b = 0.0  # Blue
        
        # 2. Create waypoint markers
        for i, pose in enumerate(path.poses):
            # Add point to line strip
            point = Point()
            point.x = pose.pose.position.x
            point.y = pose.pose.position.y
            point.z = pose.pose.position.z
            line_marker.points.append(point)
            
            # Add sphere at each waypoint (optional)
            wp_marker = Marker()
            wp_marker.header = path.header
            wp_marker.ns = "waypoints"
            wp_marker.id = i + 1000  # Unique ID
            wp_marker.type = Marker.SPHERE
            wp_marker.action = Marker.ADD
            wp_marker.pose = pose.pose
            wp_marker.scale.x = 0.2
            wp_marker.scale.y = 0.2
            wp_marker.scale.z = 0.2
            wp_marker.color.a = 1.0
            wp_marker.color.r = 1.0
            wp_marker.color.g = 0.0
            wp_marker.color.b = 0.0
            marker_array.markers.append(wp_marker)
        
        marker_array.markers.append(line_marker)
        self.gazebo_marker_pub.publish(marker_array)
        
    def odom_callback(self, msg: Odometry):
        _, _, yaw = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.get_current_pos_and_or = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw) # geometry_msgs/Pose

    def send_follow_path_goal(self, path_msg, max_retries=3):
        for attempt in range(max_retries):
            try:
                # Wait for the action server to become available
                index = len(path_msg.poses)
                if not self.follow_path_client.wait_for_server(timeout_sec=5.0):
                    self.get_logger().warn(f'Action server not available, attempt {attempt + 1}/{max_retries}')
                    time.sleep(1.0)
                    continue

                self.get_logger().info('Action server is available')
                print("hiiiiiiiiiiiiiiii", index)
                # Create goal message
                goal_msg = FollowPath.Goal()
                goal_msg.path.poses = path_msg.poses[:index]
                goal_msg.path.header.frame_id = 'map'
                goal_msg.path.header.stamp = self.get_clock().now().to_msg()

                goal_msg.goal_checker_id = "goal_checker"
                goal_msg.controller_id = "FollowPath"

                # send the final path to the robot
                self.follow_path_client.wait_for_server()
                self.follow_path_client.send_goal_async(goal_msg)

                self.get_logger().info('Sending path to FollowPath action server')

                '''# Send goal and wait synchronously for it to be sent
                send_goal_future = self.follow_path_client.send_goal_async(goal_msg)
                rclpy.spin_until_future_complete(self, send_goal_future)
                goal_handle = send_goal_future.result()

                if not goal_handle.accepted:
                    self.get_logger().error('FollowPath goal rejected!')
                    time.sleep(1.0)
                    continue

                self.get_logger().info('FollowPath goal accepted, waiting for result...')

                # Wait synchronously for the result
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, result_future)
                result = result_future.result()

                if result:
                    self.get_logger().info(f'FollowPath result: {result.status}')
                    if hasattr(result, 'result') and hasattr(result.result, 'error_code'):
                        self.get_logger().info(f"FollowPath result code: {result.result.error_code}")
                        self.get_logger().info(f"Error string: {result.result.error_string}")'''
                self.navigation_complete.set()
                return True

            except Exception as e:
                self.get_logger().error(f'Error in send_follow_path_goal: {str(e)}')
                time.sleep(1.0)

        self.get_logger().error('Failed to send follow path goal after maximum retries')
        return False


        
        
    '''def laser_scan_callback(self, msg):
        # Process laser scan data to detect obstacles
        try:
            print('hi', msg)
            ranges = np.array(msg.ranges)
            angle_min = msg.angle_min
            angle_increment = msg.angle_increment


            for i, range_value in enumerate(ranges):
                if self.raytrace_min_range <= range_value <= self.raytrace_max_range:
                    angle = angle_min + i * angle_increment
                    x = range_value * np.cos(angle)
                    y = range_value * np.sin(angle)
                    
                    grid_x = int((x / self.costmap_resolution) + (self.costmap_size[0] / 2))
                    grid_y = int((y / self.costmap_resolution) + (self.costmap_size[1] / 2))
                    
                    if 0 <= grid_x < self.costmap_size[0] and 0 <= grid_y < self.costmap_size[1]:
                        self.costmap[grid_x, grid_y] = 100

            # Convert numpy costmap to OccupancyGrid message
            costmap_msg = OccupancyGrid()
            costmap_msg.header.stamp = self.get_clock().now().to_msg()
            costmap_msg.header.frame_id = "odom"
            costmap_msg.info.resolution = self.costmap_resolution
            costmap_msg.info.width = self.costmap_size[0]
            costmap_msg.info.height = self.costmap_size[1]
            costmap_msg.info.origin.position.x = - (self.costmap_size[0] * self.costmap_resolution) / 2
            costmap_msg.info.origin.position.y = - (self.costmap_size[1] * self.costmap_resolution) / 2
            costmap_msg.info.origin.position.z = 0.0
            costmap_msg.info.origin.orientation.w = 1.0
            costmap_msg.data = self.costmap.flatten().tolist()
            
            # Publish the updated costmap
            self.local_costmap_pub.publish(costmap_msg)


            # Conditional publishing to global costmap
            #if self.should_update_global_costmap():
            #    self.global_costmap_pub.publish(costmap_msg)
            #    self.previous_global_costmap = self.costmap.copy()  # Update the previous costmap
        except Exception as e:
            self.get_logger().error(f"Laser Scan Callback failed: {e}")'''


    def should_update_global_costmap(self):
        # Check if previous global costmap is None (first time update)
        if self.previous_global_costmap is None:
            return True
        
        # Calculate the difference between the current and previous global costmap
        difference = np.abs(self.costmap - self.previous_global_costmap)
        
        # Calculate the percentage of cells that have changed significantly
        changed_cells = np.sum(difference > 0)
        total_cells = self.costmap_size
        change_percentage = changed_cells / total_cells
        
        # Update if the change exceeds the threshold
        return change_percentage > self.update_threshold
    def obs_callback(self, msg):
        # Access the list of obstacles from the ObstacleArrayMsg message
        try:
            print('hi', msg)
            self.get_logger().info('Received obstacle information')
            self.solid_obs = ObstacleDetector.polygon_offsetting(msg.obstacles, 1.0)
        except Exception as e:
            self.get_logger().error(f"Obstacles Callback failed: {e}")
        

    def gate_callback(self, msg):
        try:
            print('hi', msg)
            self.get_logger().info('Received gate info')
            #self.gate_position = [{'x' : msg.poses[0].position.x, 'y' : msg.poses[0].position.y}]
            
        except Exception as e:
            self.get_logger().error(f"Gate Callback failed: {e}")
        

    def wall_callback(self, msg):
        try:
            print('hi', msg)
            self.get_logger().info('Received wall info')
            self.walls[0] = msg.pol.dx
            self.walls[1] = msg.pol.dy
        except Exception as e:
            self.get_logger().error(f"Borders Callback failed: {e}")

        

    def local_costmap_callback(self, msg):
        try:

            self.get_logger().info('Received local costmap')
            #self.process_costmap(msg)
            PathPlanner.costmap = msg
            self.costmap_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
            self.costmap_resolution = msg.info.resolution
            self.costmap_width = msg.info.width
            self.costmap_height = msg.info.height
            self.costmap_size = (self.costmap_width, self.costmap_height)
            data = np.array(msg.data, dtype=np.int8).reshape((self.costmap_height, self.costmap_width))
            self.local_costmap = data 

            '''width = msg.info.width
            height = msg.info.height
            resolution = msg.info.resolution
            origin_x = msg.info.origin.position.x
            origin_y = msg.info.origin.position.y
            print('local costmap shape', self.local_costmap.shape[0], self.local_costmap.shape[1])
            self.get_logger().info(f'Costmap dimensions: {width} × {height}')
            self.get_logger().info(f'Resolution: {resolution} m/cell')
            self.get_logger().info(f'Origin: ({origin_x:.2f}, {origin_y:.2f})')

            # Convert the 1D data list to a 2D numpy array for easy inspection
            cost_array = np.array(msg.data, dtype=int).reshape((height, width))
            self.get_logger().info(f'Costmap array shape: {cost_array.shape}')
            
            
            # Example: Check value at cell (row=0, col=0), and the center cell
            self.get_logger().info(f'Cell (0, 0) cost: {cost_array[0, 0]}')
            self.get_logger().info(f'Center cell cost: {cost_array[height // 2, width // 2]}')

            # Optional: Simple analysis
            free_cells = (cost_array >= 0) & (cost_array < 50)  # e.g. treat <50 as free
            free_ratio = np.mean(free_cells)
            self.get_logger().info(f'Free space proportion: {free_ratio:.2%}') '''
        except Exception as e:
            self.get_logger().error(f"Local Costmap Callback failed: {e}")


    def global_costmap_callback(self, msg):
        try:
    
            self.get_logger().info('Received global costmap')
            #self.process_costmap(msg)
            PathPlanner.g_costmap = msg
        except Exception as e:
            self.get_logger().error(f"Global Costmap Callback failed: {e}")




    def process_costmap(self, costmap_msg):
        # Extracting costmap metadata
        global collision_detected
        width = costmap_msg.info.width
        height = costmap_msg.info.height
        resolution = costmap_msg.info.resolution
        origin_x = costmap_msg.info.origin.position.x
        origin_y = costmap_msg.info.origin.position.y

        # Converting costmap data to a numpy array
        costmap_data = np.array(costmap_msg.data, dtype=np.int8).reshape((height, width))

        # Detecting obstacles
        self.obstacles = np.argwhere(costmap_data > self.obstacle_threshold)
        
        if self.obstacles.size > 0:
            self.get_logger().info(f'Obstacles detected at: {self.obstacles}')
            self.avoid_obstacles(self.obstacles, self.robots, costmap_msg)
        else:
            self.get_logger().info('No obstacles detected.')




    def avoid_obstacles(self, obstacles, robots, costmap_msg):
        global collision_detected
        self.get_logger().info('Avoiding obstacles...')
        for robot in robots:
            robot_x, robot_y = robot.position
            for obstacle in obstacles:
                x, y = obstacle
                # Converting grid coordinates to world coordinates
                world_x = costmap_msg.info.origin.position.x + x * costmap_msg.info.resolution
                world_y = costmap_msg.info.origin.position.y + y * costmap_msg.info.resolution
                self.get_logger().info(f'Obstacle at world coordinates: ({world_x}, {world_y})')
                # Implement specific actions to avoid obstacles

                # Calculate the distance to the obstacle
                distance = math.sqrt((world_x - robot_x)**2 + (world_y - robot_y)**2)

                # Example action: if an obstacle is within a certain range, stop or change direction
                if distance < self.safety_distance or not self.is_clear(robot, self.solid_obs, self.walls):
                    self.get_logger().info('Obstacle too close! Changing direction...')
                    collision_detected = True
                    self.manage_collision(self.robot_task)

    

    def publish_velocities(self, pose, goal_pose):
        twist = Twist()
        K_linear = 0.5  # Proportional gain for linear velocity
        K_angular = 1.0  # Proportional gain for angular velocity

       

        dx = goal_pose.pose.position.x - pose.pose.position.x
        dy = goal_pose.pose.position.y - pose.pose.position.y
        distance = math.sqrt(dx**2 + dy**2)

        # Calculate the angle to the waypoint
        angle_to_waypoint = math.atan2(dy, dx)

        # Calculate the difference in orientation
        angle_diff = angle_to_waypoint - pose.pose.orientation.w

        # Normalize the angle difference
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

        # Calculate linear and angular velocities

        # Adjust speed based on proximity to the waypoint and segment type
        if distance < 0.5:
            # Slow down as the robot approaches the waypoint
            linear_velocity = K_linear * distance * (distance / 0.5)
        else:
            # Maintain base speed
            linear_velocity = self.const_linear_velocity[0]

        # Linear velocity
        twist.linear.x = linear_velocity  # Forward/backward speed
        twist.linear.y = 0.0  # Usually zero in 2D plane
        twist.linear.z = 0.0  # Usually zero in 2D plane

        # Angular velocity
        twist.angular.x = 0.0  # Usually zero in 2D plane
        twist.angular.y = 0.0  # Usually zero in 2D plane
        twist.angular.z = K_angular * angle_diff # Yaw rotation speed
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info(f'Published Velocities for waypoint{pose} {twist}')

    def spinner(self):
        while rclpy.ok():
            self.get_logger().info('Obstacle detection task is executing...')
            rclpy.spin_once(self, timeout_sec=1)


    '''def run_collision_detection(self, robots, obstacles, walls):
        # Detect collisions
        global collision_detected
        while True:
            for rob in robots:
                if not self.is_clear(rob, obstacles, walls):   
                    collision_detected = True             
                    manage_collision(self.robot_task, rob)
            time.sleep(1)'''
    

    
    def get_local_path(self, start, graph):
        path = [start]
        segments = []
     
        dubins_path = DubinsPath(start, end, 0.5)   
        # Get path segments
        #waypoint, seg = dubins_path.get_path()
        #waypoints.append(waypoint)
        #segments.append(seg)
        path.append(dubins_path.plan_path(start, end))
        #segment_type_mapping = {0: 'LSL', 1: 'LSR', 2: 'RSL', 3: 'RSR', 4: 'RLR', 5: 'LRL'}

        #segment_types = [segment_type_mapping[t] for t in segments]
        return path




    def create_waypoints(self, waypoints_list):
        waypoints = []
        
        # Example: creating a few PoseStamped waypoints
        for point in waypoints_list:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(point[0])
            pose.pose.position.y = float(point[1]) * 0.5
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = np.radians(point[2])  # No rotation
            waypoints.append(pose)

        return waypoints


    def update_pose(self, pose):
        """Update the current robot pose.
        
        Args:
            pose: A PoseStamped message containing the robot's current pose.
        """
        self.current_pose = pose
        
    '''def publish_tf(self):
        """Publish the current robot pose to the /tf topic."""
        if self.current_pose is None:
            return
            
        # Create and publish transform
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = f'{self.namespace}/base_link'
        
        # Set the transform translation and rotation from the current pose
        transform.transform.translation.x = self.current_pose.pose.position.x
        transform.transform.translation.y = self.current_pose.pose.position.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation = self.current_pose.pose.orientation
        
        # Publish the transform
        self.tf_broadcaster.sendTransform(transform)'''
        
    def get_transform(self, pose):
        """Get a transform from the robot's current pose.
        
        Args:
            pose: A PoseStamped message containing the target pose.
            
        Returns:
            TransformStamped: The transform from map to base_link.
        """
        # Update the current pose
        self.update_pose(pose)
        
        # Create TransformStamped message
        transform_stamped = TransformStamped()
        
        # Populate Header
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = "map"

        # Populate child_frame_id with namespace
        transform_stamped.child_frame_id = f"{self.namespace}/base_link"

        # Populate Transform
        transform_stamped.transform.translation.x = pose.pose.position.x
        transform_stamped.transform.translation.y = pose.pose.position.y
        transform_stamped.transform.translation.z = 0.0 
        transform_stamped.transform.rotation = pose.pose.orientation
        
        return transform_stamped
        # Append to transforms array




    def publish_joint_state(self, pose):
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()  # Use current time
        joint_state_msg.name = ['left_wheel', 'right_wheel', 'lidar2d_joint']  # List of joint names
        joint_state_msg.position = [pose.pose.position.x, pose.pose.position.y, pose.pose.orientation.w]  # List of joint positions
        #joint_state_msg.velocity = [0.2, 0.0, 0.5]  # List of joint velocities
        #joint_state_msg.effort = [10.0, 20.0]  # List of joint efforts
        self.joint_state_pub.publish(joint_state_msg)
        self.get_logger().info(f'Published Joint States {joint_state_msg}')

    def yaw_from_two_points(self,x1, y1, x2, y2):
        dx = x2 - x1
        dy = y2 - y1
        yaw = math.atan2(dy, dx)
        return yaw

    def robot_task(self):
        # Check if this task has already been executed for this namespace
        if self.obs is not None:              
            if hasattr(self, '_task_executed') and self._task_executed:
                self.get_logger().info(f"Task already executed for namespace: {self.namespace}")
                return
                
            # Check if goal_pose is set
            if self.goal_pose is None or not hasattr(self.goal_pose, 'pose'):
                self.get_logger().info("Waiting for goal pose to be set...")
                return
            if (self.goal_pose.pose.position.x == 0.0 or 
                self.goal_pose.pose.position.y == 0.0 or 
                self.goal_pose.pose.position.z == 0.0):
                self.get_logger().warn('Received invalid goal position (0.0, 0.0, 0.0), ignoring...')
                return
            if hasattr(self, 'plan_timer') and self.plan_timer is not None:
                self.plan_timer.cancel()
                self.plan_timer = None
            try:
                # Mark task as started
                self._task_executed = True
                
                # Get start pose from start_pose
                start = self.pose_to_tuple(self.start_pose.pose)
                goal = self.pose_to_tuple(self.goal_pose.pose)
                
                self.get_logger().info(f"Starting robot task for {self.namespace} with start: {start}, goal: {goal}")
                
                graph = [start]
                self.parents = {graph[0]: None} 
                
                self.get_logger().info(f"Planning trajectory to goal: {goal}")
                graph = self.get_trajectory(goal, graph)
                self.get_logger().info(f"Generated trajectory with {len(graph)} points")
                for i, point in enumerate(graph):
                    self.get_logger().info(f"Point {i}: {point}")

                path_msg = Path()
                path_msg.header.frame_id = 'map'
                path_msg.header.stamp = self.get_clock().now().to_msg()
                for i, q in enumerate(graph):
                    pose = PoseStamped()
                    pose.header.frame_id = 'map'
                    pose.header.stamp = self.get_clock().now().to_msg()
                    # Convert numpy arrays to float values for ROS2 Point message
                    x_val = float(q[0])
                    y_val = float(q[1])
                    yaw_val = float(q[2])
                    pose.pose.position = Point(x=x_val, y=y_val, z=0.0)
                    pose.pose.orientation = self.yaw_to_quaternion(yaw_val)
                    path_msg.poses.append(pose)
                for i, pose in enumerate(path_msg.poses[:5]):  # First 5 points
                    self.get_logger().info(f"First 5 Path point {i}: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})")
                for i, pose in enumerate(path_msg.poses[-5:]):  # Last 5 points
                    self.get_logger().info(f"Last 5 Path point {i}: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})")
                self.get_logger().info(f"Final path_msg has {len(path_msg.poses)} poses")
                self.path_pub.publish(path_msg)
                self.publish_path_to_gazebo(path_msg)
                self.get_logger().info(f"Published path with {len(path_msg.poses)} poses for {self.namespace}")
                self.path_published_event.set()
                
            except Exception as e:
                self.get_logger().error(f"Error in robot_task for {self.namespace}: {str(e)}")
                # Reset the flag to allow retry
                self._task_executed = False
                raise
            #waypoints_list, segments = self.get_waypoints(in_pose, graph)
            '''for i in range(len(graph) - 1):
                local_path_msg = Path()
                local_path_msg.header.frame_id = self.start_pose.header.frame_id
                start = graph[i]
                end = graph[i + 1]
                self.get_logger().info(f"Processing segment {i+1}/{len(graph)-1}: {start} -> {end}")
                
                dubins_path = DubinsPath(start, end, 0.5)   
                local_path = dubins_path.plan_path(start, end)'''

            generator = Dubins(3, 0.5)
            #dubins_path = DubinsPath(start, goal, 0.5)
            local_path = generator.run(graph)
            self.get_logger().info(f"Planned Dubins path with {len(local_path)} points")
            local_path_msg = Path()
            local_path_msg.header.frame_id = 'map'
            for q in local_path:
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                x_val = float(q[0])
                y_val = float(q[1])
                yaw_val = float(q[2])
                pose.pose.position = Point(x=x_val, y=y_val, z=0.0)
                pose.pose.orientation = self.yaw_to_quaternion(yaw_val)
                local_path_msg.poses.append(pose)
            
            
            self.get_logger().info("Sending path to follow action server")
            self.get_logger().info(f"Sending path with {len(local_path_msg.poses)} points to follow action server")    
            #success = self.send_follow_path_goal(local_path_msg)
            success = self.send_follow_path_goal(path_msg)
            if success:
                self.get_logger().info("Successfully executed path following")
            else:
                self.get_logger().error("Failed to execute path following")
            self.robot_task_event.set()
            '''self.publish_velocities(local_path[0], local_path[1])
            
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = local_path[0]
            pose.pose.position.y = local_path[1]
            pose.pose.position.z = 0.0
            
            yaw = self.yaw_from_two_points(start[0], start[1], end[0], end[1])
            quat = quaternion_from_euler(0, 0, yaw)
            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]
            
            #self.publish_joint_state(pose)
            #tf_msg = TFMessage()
            #tf_msg.transforms.append(self.get_transform(pose))
            #self.tf_pub.publish(tf_msg)
            path.poses.append(pose)
                
            self.get_logger().info(f"Publishing path with {len(path.poses)} poses")
            if hasattr(self, 'plan_timer') and self.plan_timer is not None:
                self.plan_timer.cancel()
                self.plan_timer = None 
            self.path_pub.publish(path)
    
            
            self.get_logger().info("Sending path to follow action server")
            success = self.send_follow_path_goal(path)
            self.get_logger().info(f'Published global plan with {len(path.poses)} poses')
            self.get_logger().info(f'Path details: {path}')
                #waypoints = self.create_waypoints(waypoints_list)
                #self.controller_node.follow_pth(shelfino.client, waypoints)
                #print(graph)
                
                #self.robot_task_event.set()
                                
            except Exception as e:
                self.get_logger().error(f"Task of Robot {self.get_namespace()} failed: {e}", exc_info=True)
                raise''' 
        else:
            return

    def yaw_to_quaternion(self, yaw):
        q = quaternion_from_euler(0, 0, yaw)
        return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        
    def pose_to_tuple(self, pose: Pose):
        x = pose.position.x
        y = pose.position.y
        yaw = self.quaternion_to_yaw(pose.orientation)
        return (x, y, yaw)  
    
    def get_mul_cost(self, config1, config2):
        for c in config2:
            if len(c) == 2:
                c = np.append(c, 0.0)
            cost = np.linalg.norm(np.array(config1) - np.array(c))
            if cost>self.robot_size:
                break
        return cost



    def get_cost(self, config1, config2):
        return np.linalg.norm(np.array(config1) - np.array(config2))



    def nearest(self, graph, new_node):
        nearest = None
        min_distance = float('inf')

        for idx, node in enumerate(graph):
            distance = self.get_cost(new_node, node)
            
            if (distance <= min_distance) and distance!=0:
                min_distance = distance
                nearest = node
        return nearest



    def extend(self, q_nearest, q_random, step_size=1.0):
        # Compute deltas
        dx = q_random[0] - q_nearest[0]
        dy = q_random[1] - q_nearest[1]
        dyaw = self.normalize_angle(q_random[2] - q_nearest[2])

        norm = np.linalg.norm([dx, dy, dyaw])
        if norm < 1e-6:
            return None  # Avoid zero movement

        # Normalize direction
        dx /= norm
        dy /= norm
        dyaw /= norm

        # Compute new state in direction of q_random
        if norm > step_size:
            new_x = q_nearest[0] + step_size * dx
            new_y = q_nearest[1] + step_size * dy
            new_yaw = self.normalize_angle(q_nearest[2] + step_size * dyaw)
        else:
            new_x, new_y, new_yaw = q_random

        new_node = (new_x, new_y, new_yaw)

        # Optional: collision check here
        #if not self.is_in_free_space(new_node):
        #    return None
        
        return new_node

    def normalize_angle(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi    

    def rewire(self, near_nodes, idx, q_new):
        node_to_rewire = near_nodes[idx]
        if node_to_rewire in self.parents:
            self.parents[node_to_rewire] = q_new
        

    def is_clear(self, robot, obstacles, walls):
        # Check if the path from 'start' to 'end' is clear
        # The obstacles parameter is a list of Node objects representing obstacle positions

        # Simple collision detection for illustration purposes
    
        #for obs in obstacles:
        if ObstacleDetector.intersects_circular_obstacles(robot, obstacles) or ObstacleDetector.intersects_point_obstacles(robot, obstacles) or ObstacleDetector.intersects_polygonal_obstacles(robot, obstacles) or ObstacleDetector.intersects_walls(robot, walls) or ObstacleDetector.intersects_other_robots(robot, self.robots):
            return False  # Collision detected
        else:
            return True  # Path is clear
    
    def world_to_map(self, x, y):
        map_x = int((x - self.costmap_origin[0]) / self.costmap_resolution)
        map_y = int((y - self.costmap_origin[1]) / self.costmap_resolution)
        return map_x, map_y


    def is_in_free_space(self, q):
        try:
            map_x, map_y = self.world_to_map(q[0], q[1])
            rows, cols = self.local_costmap.shape

            if not (0 <= map_y < rows and 0 <= map_x < cols):
                return False

            cost_value = self.local_costmap[map_y, map_x]
            return cost_value == 0  # only consider exactly 0 as free
        except Exception:
            return False


    def interpolate_to_target_point(self, last_point, target_point):
        """
        Interpolate one point along a Dubins path to the target point using cubic spline interpolation.
        """
        # Define the parameter space
        t = np.array([0, 1])  # Two control points
        
        # Ensure last_point and target_point are 1D arrays
        last_point = np.array(last_point)
        target_point = np.array(target_point)
        
        # Ensure dubins_points is a 2D array with two rows
        
        dubins_points = np.vstack((last_point, target_point))

        spline = CubicSpline(t, dubins_points)
        
        # Evaluate the spline at t=0.5 to get the interpolated point
        interpolated_point = spline(0.05)
        
        return tuple(interpolated_point)

    def sample_random_free_space(self, q_goal, max_attempts=10000000, goal_bias=0.1):
        """
        Pick a random point in the costmap that is free, with a bias towards the goal.
        
        Args:
            max_attempts: Maximum number of sampling attempts
            goal_bias: Probability (0-1) of sampling the goal position directly
            
        Returns:
            tuple: (x, y, theta) of a valid configuration, or None if none found
        """
        goal_pose = self.pose_to_tuple(self.goal_pose.pose) if hasattr(self, 'goal_pose') else None
        
        for _ in range(max_attempts):
            # With probability goal_bias, sample the goal position
            if goal_pose and np.random.random() < goal_bias:
                x, y, theta = goal_pose
                
            else:
                x, y, theta = [np.random.uniform(q_goal[0]-0.5, q_goal[0]+0.5), 
                        np.random.uniform(q_goal[1]-0.5, q_goal[1]+0.5),
                        np.random.uniform(q_goal[2]-0.5, q_goal[2]+0.5)] 
            
            #if self.is_in_free_space((x, y, theta)):
            return (x, y, theta)
 
        return None  # No free point found

    def steer(self, start, goal, d):
        """
        Return a point in the direction of the goal, that is distance away from start
        :param start: start location
        :param goal: goal location
        :param d: distance away from start
        :return: point in the direction of the goal, distance away from start
        """
        start, end = np.array(start), np.array(goal)
        v = end - start
        u = v / (np.sqrt(np.sum(v ** 2)))
        d = min(d, np.linalg.norm(goal - start))
        steered_point = start + u * d
        return tuple(steered_point)

    def es_points_along_line(self, start, end, r):
        """
        Equally-spaced points along a line defined by start, end, with resolution r
        :param start: starting point
        :param end: ending point
        :param r: maximum distance between points
        :return: yields points along line from start to end, separated by distance r
        """
        d = np.linalg.norm(np.array(end) - np.array(start))
        n_points = int(np.ceil(d / r))
        if n_points > 1:
            step = d / (n_points - 1)
            for i in range(n_points):
                next_point = self.steer(start, end, i * step)
                yield next_point

    
    def obstacle_generator(self, obstacles_np):
        for i, obs in enumerate(obstacles_np):
            # Each obs must have 6 values (xmin, ymin, zmin, xmax, ymax, zmax)
            if len(obs) != 6:
                self.get_logger().warning(f"⚠️ Obstacle {i} invalid shape: {obs}")
                continue
            yield (i, tuple(obs), None)


    
    def sample_point(self, q_init, q_goal, initial_range, max_attempts=100000, goal_bias=0.1):
        """
        Pick a random point in the costmap that is free, with a bias towards the goal.
        If a point is not in free space, gradually increases the sampling range.
        
        Args:
            q_goal: The goal configuration (x, y, theta)
            max_attempts: Maximum number of sampling attempts per range
            goal_bias: Probability (0-1) of sampling the goal position directly
            
        Returns:
            tuple: (x, y, theta) of a valid configuration, or None if none found
        """
       
        
        # Define the initial and maximum range for sampling around the goal
        #initial_range = 5.0
        min_range = 0.3  # Maximum range to avoid sampling too far
        range_decrement = 1.0  # How much to increase the range each time
        #workspace_bounds = [(-6.0, 6.0), (-6.0, 6.0), (-math.pi, math.pi)] 
        current_range = initial_range
        
        while current_range >= min_range:
            for _ in range(max_attempts):
                # With probability goal_bias, sample the goal position directly
                if q_goal and np.random.random() < goal_bias:
                    x, y, theta = q_goal
                    #if self.is_in_free_space((x, y, theta)):
                    return (x, y, theta)
                
                # Sample within current range around the goal
                '''q_random = tuple(
                    np.random.uniform(low, high) for low, high in workspace_bounds
                )'''
                
           
                x = np.random.uniform(q_goal[0] - min_range, q_goal[0] + min_range)
                y = np.random.uniform(q_goal[1] - min_range, q_goal[1] + min_range)
                theta = np.random.uniform(q_goal[2] - min_range, q_goal[2] + min_range)
                self.get_logger().info(f"Point found: ({x:.2f}, {y:.2f}, {theta:.2f})")
                
                # Check if the sampled point is in free space
                #if self.is_in_free_space((x, y, theta)):
                #    self.get_logger().info(f"Found valid point: ({x:.2f}, {y:.2f}, {theta:.2f})")
                #    return (x, y, theta)
                if self.collision_free(q_init, (x, y, theta), self.r):
                    print("collision_free:", (x, y, theta))
                    break
                else:
                    if max_attempts >= 100:
                         min_range += 1.0
                         print("min_range:", min_range)
                    continue
            print("min_point:", (x, y, theta))
            return (x, y, theta)
            #current_range -= range_decrement
            self.get_logger().debug(f"Decreased sampling range to ±{current_range:.1f}m around goal")
        return (x, y, theta)
        
        
        # If we've tried all ranges and still haven't found a valid point
        self.get_logger().warn("Failed to find a valid point in free space")
        return q_goal

    '''def get_trajectory(self, q_goal, graph):        
        self.get_logger().info("Starting trajectory planning...")
        self.get_logger().info(f"Start position: {graph[0]}")
        self.get_logger().info(f"Goal position: {q_goal}")
        q_init = graph[0]

        q_init = graph[0]
        iteration = 0
        max_iterations = 1000

        goal_sample_rate = 0.1  # 10% of the time, sample the goal directly
        step_size = 0.05        # Maximum extension length
        workspace_bounds = [(-6.0, 6.0), (-6.0, 6.0), (-math.pi, math.pi)]  # Example bounds

        stuck_counter = 0  # Track how long we've been stuck at same best distance
        best_distance = self.get_cost(q_goal, q_init)

        start_time = self.get_clock().now()
        last_log_time = start_time

        while self.get_cost(q_goal, q_init) > 0.05 and iteration < max_iterations:       
            q_random = [np.random.uniform(q_goal[0]-0.05, q_goal[0]+0.05), 
                        np.random.uniform(q_goal[1]-0.05, q_goal[1]+0.05),
                        np.random.uniform(q_goal[2]-0.05, q_goal[2]+0.05)] 
            iteration += 1
            current_time = self.get_clock().now()

            # Log progress periodically
            if iteration % 100 == 0 or (current_time - last_log_time).nanoseconds > 5e9:
                self.get_logger().info(
                    f"Iteration: {iteration}, Current best distance to goal: {self.get_cost(q_goal, q_init):.3f}"
                )
                last_log_time = current_time

            # Check if we're stuck (distance not improving)
            current_distance = self.get_cost(q_goal, q_init)
            if abs(current_distance - best_distance) < 1e-3:
                stuck_counter += 1
            else:
                stuck_counter = 0
                best_distance = current_distance

            # If stuck for too many iterations, jump to a random free-space point
            if stuck_counter > 500:
                random_free_point = self.sample_random_free_space()
                if random_free_point:
                    self.get_logger().info(f"random free point: {random_free_point}")
                    self.get_logger().warn(f"Stuck detected. Jumping to new free point: {random_free_point}")
                    q_init = random_free_point
                    graph.append(q_init)
                stuck_counter = 0
                continue

            if self.is_in_free_space(q_random):
                q_nearest = self.nearest(graph, q_random) 
                q_new = self.extend(q_nearest, q_random)
                min_cost = self.get_cost(q_goal, q_nearest)
                near_nodes = [node for node in graph if (self.get_cost(q_new, node) <= min_cost)] 
                for idx, near_node in enumerate(near_nodes):
                    if self.is_in_free_space(q_new):                   
                        new_cost = self.get_cost(q_goal, near_node) + self.get_cost(q_new, near_node)
                        if new_cost < min_cost:
                            min_cost = new_cost
                    else:
                        continue
                #if (self.is_joint_okay(q_new)) and (not self.is_singular(self.compute_forward_kinematics_from_configuration(q_new))):
                graph.append(q_new)
                for idx, near_node in enumerate(near_nodes):
                    if self.is_in_free_space(near_node): 
                        if (self.get_cost(q_goal, q_new) + self.get_cost(q_new, near_node))<self.get_cost(q_goal, near_node):
                            self.rewire(near_nodes, idx, q_new) 
                    else:
                        continue 
                self.get_logger().info(f"new point added: {q_new}")
                q_init = q_new  
        if iteration >= max_iterations:
            self.get_logger().warn(f"Reached max iterations ({max_iterations}) without goal")
            self.get_logger().warn(f"Closest distance achieved: {self.get_cost(q_goal, q_init):.3f}")

        planning_time = (self.get_clock().now() - start_time).nanoseconds / 1e9
        self.get_logger().info(f"Trajectory planning completed in {planning_time:.2f} seconds")
        self.get_logger().info(f"Final path has {len(graph)} nodes") 
        
        return graph'''

    def get_trajectory_actual(self, q_goal, graph):        
        q_init = graph[0]
        iteration = 0
        max_iterations = 100
        while self.get_cost(q_goal, q_init)>0.3 and iteration < max_iterations:        
            q_random = self.sample_point(q_goal)   
            if self.is_in_free_space(q_random):
                q_nearest = self.nearest(graph, q_random)
                q_new = self.extend(q_nearest, q_random)
                print("new node:", q_new)
                min_cost = self.get_cost(q_goal, q_nearest)
                near_nodes = [node for node in graph if (self.get_cost(q_new, node) <= min_cost)] 
                print("near nodes:", near_nodes)
                for idx, near_node in enumerate(near_nodes):
                    if self.is_in_free_space(q_new):                   
                        new_cost = self.get_cost(q_goal, near_node) + self.get_cost(q_new, near_node)
                        if new_cost < min_cost:
                            min_cost = new_cost
                    else:
                        continue
                #if (self.is_joint_okay(q_new)) and (not self.is_singular(self.compute_forward_kinematics_from_configuration(q_new))):
                graph.append(q_new)
                self.get_logger().info(f"New node added: {q_new}")
                for idx, near_node in enumerate(near_nodes):
                    if self.is_in_free_space(near_node):
                        if (self.get_cost(q_goal, q_new) + self.get_cost(q_new, near_node))<self.get_cost(q_goal, near_node):
                            self.rewire(near_nodes, idx, q_new) 
                    else:
                        continue 
                q_init = q_new 
            else:
                continue
            self.get_logger().info(
                f"Current best distance to goal: {self.get_cost(q_goal, q_init):.3f}"
            )
            iteration += 1
            
        return graph

    def get_trajectory_with_costmap(self, q_goal, graph):        
        q_init = graph[0]
        iteration = 0
        max_iterations = 100
        while self.get_cost(q_goal, q_init)>0.3 and iteration < max_iterations:        
            q_random = self.sample_point(q_goal)   
            if self.is_in_free_space(q_random):
                q_nearest = self.nearest(graph, q_random)
                q_new = self.extend(q_nearest, q_random)
                print("nearest node:", q_nearest)
                print("random node:", q_random)
                print("new node:", q_new)
                graph.append(q_new)
                q_min= q_nearest
                min_cost = self.get_cost(q_goal, q_nearest) + self.get_cost(q_goal, q_new)
                near_nodes = [node for node in graph if (self.get_cost(q_new, node) <= min_cost)] 
                print("near nodes:", near_nodes)
                for idx, near_node in enumerate(near_nodes):
                    if self.is_in_free_space(near_node):                   
                        new_cost = self.get_cost(q_goal, near_node) + self.get_cost(near_node, q_new)
                        if new_cost < min_cost:
                            min_cost = new_cost
                            q_min = near_node
                    else:
                        continue
                #if (self.is_joint_okay(q_new)) and (not self.is_singular(self.compute_forward_kinematics_from_configuration(q_new))):
                graph.append(q_min)
                self.get_logger().info(f"New node added: {q_new}")
                for idx, near_node in enumerate(near_nodes):
                    if self.is_in_free_space(near_node):
                        new_cost = self.get_cost(q_goal, q_new) + self.get_cost(q_new, near_node)
                        if new_cost<self.get_cost(q_goal, near_node):
                            self.rewire(near_nodes, idx, q_new) 
                    else:
                        continue 
                q_init = q_new 
                self.get_logger().info(
                    f"Current best distance to goal: {self.get_cost(q_goal, q_init):.3f}"
                )
            iteration += 1
            
        return graph


    def get_trajectory(self, q_goal, graph):        
        q_init = graph[0]
        iteration = 0
        max_iterations = 100
        while self.get_cost(q_goal, q_init)>0.3 and iteration < max_iterations:        
            q_random = self.sample_point(q_init, q_goal, self.get_cost(q_goal, q_init))   
            #if self.is_in_free_space(q_random):
            q_nearest = self.nearest(graph, q_random)
            q_new = self.extend(q_nearest, q_random)
            print("nearest node:", q_nearest)
            print("random node:", q_random)
            print("new node:", q_new)
            if self.collision_free(q_init, q_new, self.r):
                graph.append(q_new)
            else:
                continue
            graph.append(q_new)
            q_min= q_nearest
            min_cost = self.get_cost(q_goal, q_nearest) + self.get_cost(q_goal, q_new)
            near_nodes = [node for node in graph if (self.get_cost(q_new, node) <= min_cost)] 
            print("near nodes:", near_nodes)
            for idx, near_node in enumerate(near_nodes):
                #if self.is_in_free_space(q_new):                   
                new_cost = self.get_cost(q_goal, near_node) + self.get_cost(near_node, q_new)
                if new_cost < min_cost:
                    min_cost = new_cost
                    q_min = near_node
                #else:
                #    continue
                #if (self.is_joint_okay(q_new)) and (not self.is_singular(self.compute_forward_kinematics_from_configuration(q_new))):
            if self.collision_free(q_init, q_min, self.r):
                graph.append(q_min)
            else:
                continue
            graph.append(q_min)
            self.get_logger().info(f"New node added: {q_new}")
            for idx, near_node in enumerate(near_nodes):
                #if self.is_in_free_space(near_node):
                new_cost = self.get_cost(q_goal, q_new) + self.get_cost(q_new, near_node)
                if new_cost<self.get_cost(q_goal, near_node):
                    self.rewire(near_nodes, idx, q_new) 
                #else:
                    #    continue 
            q_init = q_new 
            self.get_logger().info(
                f"Current best distance to goal: {self.get_cost(q_goal, q_init):.3f}"
            )
            iteration += 1
            
        return graph

    def obstacle_free(self, x):
        """
        Check if a location resides inside of an obstacle
        :param x: location to check
        :return: True if not inside an obstacle, False otherwise
        """
        if self.obs is not None:
            return self.obs.count(x) == 0

    def collision_free(self, start, end, r):
        """
        Check if a line segment intersects an obstacle
        :param start: starting point of line
        :param end: ending point of line
        :param r: resolution of points to sample along edge when checking for collisions
        :return: True if line segment does not intersect an obstacle, False otherwise
        """
        if self.obs is not None:
            points = self.es_points_along_line(start, end, r)
            coll_free = all(map(self.obstacle_free, points))
            return coll_free
  
    

    def get_trajectory_new(self, q_goal, graph):
        self.get_logger().info("Starting trajectory planning...")
        self.get_logger().info(f"Start position: {graph[0]}")
        self.get_logger().info(f"Goal position: {q_goal}")
        
        q_init = graph[0]
        iteration = 0
        max_iterations = 1000

        goal_sample_rate = 0.3  # 10% of the time, sample the goal directly
        step_size = 0.1        # Maximum extension length
        workspace_bounds = [(-6.0, 6.0), (-6.0, 6.0), (-math.pi, math.pi)]  # Example bounds

        stuck_counter = 0  # Track how long we've been stuck at same best distance
        best_distance = self.get_cost(q_goal, q_init)

        start_time = self.get_clock().now()
        last_log_time = start_time

        while self.get_cost(q_goal, q_init) > 0.05 and iteration < max_iterations:
            iteration += 1
            current_time = self.get_clock().now()

            # Log progress periodically
            if iteration % 100 == 0 or (current_time - last_log_time).nanoseconds > 5e9:
                self.get_logger().info(
                    f"Iteration: {iteration}, Current best distance to goal: {self.get_cost(q_goal, q_init):.3f}"
                )
                last_log_time = current_time

            # Check if we're stuck (distance not improving)
            current_distance = self.get_cost(q_goal, q_init)
            if abs(current_distance - best_distance) < 1e-3:
                stuck_counter += 1
            else:
                stuck_counter = 0
                best_distance = current_distance

            # If stuck for too many iterations, jump to a random free-space point
            if stuck_counter > 200:
                #random_free_point = self.interpolate_to_target_point(q_init, q_goal)
                random_free_point = self.sample_random_free_space(q_goal)
                if random_free_point:
                    self.get_logger().warn(
                        f"Stuck detected at position {q_init}. "
                        f"Jumping to new free point: {random_free_point}"
                    )
                    q_init = random_free_point
                    graph.append(q_init)
                stuck_counter = 0
                continue

            # Goal biasing: sometimes sample the goal, otherwise sample from workspace
            if np.random.rand() < goal_sample_rate:
                q_random = q_goal
            else:
                '''q_random = tuple(
                    np.random.uniform(low, high) for low, high in workspace_bounds
                )'''
                q_random = self.sample_point(q_goal)
            
            # Skip if random sample is in collision
            if not self.is_in_free_space(q_random):
                if iteration % 200 == 0:
                    self.get_logger().debug(f"Skipping colliding sample at {q_random}")
                continue
      
            # Find nearest node in tree
            q_nearest = self.nearest(graph, q_random)

            # Move a fixed step toward q_random
            direction = np.array(q_random) - np.array(q_nearest)
            dist = np.linalg.norm(direction)
            if dist > step_size:
                direction = direction / dist * step_size
            q_new = tuple(np.array(q_nearest) + direction)
            #q_new = q_random
            # Skip if q_new itself is in collision
            if not self.is_in_free_space(q_new):
                continue

            # Find nearby nodes for rewiring
            min_cost = self.get_cost(q_goal, q_nearest)
            near_nodes = [node for node in graph if self.get_cost(q_new, node) <= min_cost]

            # Choose best parent
            best_parent = q_nearest
            best_cost = self.get_cost(q_goal, q_nearest) + self.get_cost(q_new, q_nearest)
            for near_node in near_nodes:
                if not self.is_in_free_space(near_node):
                    continue
                cost = self.get_cost(q_goal, near_node) + self.get_cost(q_new, near_node)
                if cost < best_cost:
                    best_parent = near_node
                    best_cost = cost

            # Add new node
            self.parents[q_new] = best_parent
            self.get_logger().info(f"New node added: {q_new}")
            graph.append(q_new)

            # Rewire
            for idx, near_node in enumerate(near_nodes):
                if not self.is_in_free_space(near_node):
                    continue
                current_cost = self.get_cost(q_goal, near_node)
                new_cost = self.get_cost(q_goal, q_new) + self.get_cost(q_new, near_node)
                if new_cost < current_cost:
                    self.rewire(near_nodes, idx, q_new)
            self.get_logger().info(f"New point: {q_new}")
            # Update current best
            q_init = q_new
            self.get_logger().info(f"Current best: {q_init}")

        if iteration >= max_iterations:
            self.get_logger().warn(f"Reached max iterations ({max_iterations}) without goal")
            self.get_logger().warn(f"Closest distance achieved: {self.get_cost(q_goal, q_init):.3f}")

        planning_time = (self.get_clock().now() - start_time).nanoseconds / 1e9
        self.get_logger().info(f"Trajectory planning completed in {planning_time:.2f} seconds")
        self.get_logger().info(f"Final path has {len(graph)} nodes")

        return graph


    def is_singular(self, J):
        okay = False
        num_dofs = 5  # Example: 3 degrees of freedom in task space
        if np.linalg.cond(J)>1e6:
            print('Singularity alert!')
            print('one')
        if np.linalg.matrix_rank(J)<num_dofs:
            print('Singularity alert!')
            print('two')
        elif np.linalg.eigvals(J) <=0.5:
            print('Singularity alert')
            print('three')
        else:
            print('no singularity! everything is fine :)!')
            okay = True

        # Perform singular value decomposition (SVD) on the Jacobian matrix
        if not okay:
            U, s, V = np.linalg.svd(J)

            # Find linearly dependent columns (where singular values are close to zero)
            linearly_dependent_indices = np.where(s < 1e-10)[0]


            print('Problematics columns:')
            for col_index in linearly_dependent_indices:                
                print(np.linalg.matrix_rank(J[:, col_index]))
            return True
        else:
            return False
        
    def manage_collision(self, collision, step_size=1.0):
        global collision_detected
        try:
            self.controller_node.get_current_pos_and_or += np.array([0.1, 0.1, 0.1])
            collision_detected = False
            collsion(self.controller_node.get_current_pos_and_or)
        except Exception as e:
            self.get_logger().error(f"Collision handling failed: {e}")


    def quaternion_to_yaw(self, q: Quaternion):
        _, _, gateyaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        return (gateyaw + math.pi) % (2 * math.pi) - math.pi

'''class DubinsPath():
    def __init__(self, start, end):
        self.turning_radius = 0.4
        self.step_size = 0.1
        self.start = startl
        self.end = end

    def get_path(self):
        path = dubins.shortest_path(self.start, self.end, self.turning_radius)
        waypoints = path.sample(self.step_size)
        # Determine the segment types based on the change in headings
        # Map from segment index to type

        # Get the path segments (each Dubins path has 3 segments)
        path_types = path.path_type() 
        
        return waypoints, path_types'''

'''def main(args=None):
    rclpy.init(args=args)
    
    try:
        # Create and initialize the node
        path_planner = PathPlanner()
        
        # Log startup message
        path_planner.get_logger().info('Path planner node started')
        
        # Keep the node alive until shutdown is requested
        rclpy.spin(path_planner)
            
    except KeyboardInterrupt:
        path_planner.get_logger().info('Path planner node stopped cleanly')
    except Exception as e:
        path_planner.get_logger().error(f'Error in path planner: {str(e)}')
        import traceback
        path_planner.get_logger().error(f'Traceback: {traceback.format_exc()}')
    finally:
        # Cleanup
        if 'path_planner' in locals():
            path_planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()'''

def quaternion_from_euler(ai, aj, ak):
    """Convert Euler angles to quaternion."""
    # Roll (x), Pitch (y), Yaw (z)
    cy = np.cos(ak * 0.5)
    sy = np.sin(ak * 0.5)
    cp = np.cos(aj * 0.5)
    sp = np.sin(aj * 0.5)
    cr = np.cos(ai * 0.5)
    sr = np.sin(ai * 0.5)
    
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    
    return [x, y, z, w]

def euler_from_quaternion(quat):
    """Convert quaternion to Euler angles (roll, pitch, yaw)."""
    if isinstance(quat, (list, tuple, np.ndarray)) and len(quat) == 4:
        x, y, z, w = quat
    else:
        x, y, z, w = quat.x, quat.y, quat.z, quat.w
    
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = np.copysign(np.pi / 2, sinp)  # Use 90 degrees if out of range
    else:
        pitch = np.arcsin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    
    return [roll, pitch, yaw]

async def ros_spin(node: Node):
    cancel = node.create_guard_condition(lambda: None)

    def _spin_task(fut, loop):
        while not fut.cancelled():
            rclpy.spin_once(node, timeout_sec=0.1)
        if not fut.done():
            loop.call_soon_threadsafe(fut.set_result, None)

    loop = asyncio.get_running_loop()
    fut = loop.create_future()
    thread = threading.Thread(target=_spin_task, args=(fut, loop), daemon=True)
    thread.start()

    try:
        # Keep spinning until cancelled
        await fut
    except asyncio.CancelledError:
        cancel.trigger()
    fut.cancel()  # Signal the spinning thread to stop
    thread.join()
    node.destroy_guard_condition(cancel)

async def main():
    rclpy.init()
    node = PathPlanner()
    node.loop = asyncio.get_running_loop()  # Store the main event loop in the node
    # Start spinning in asyncio
    spin_task = asyncio.create_task(ros_spin(node))

    # Wait for events after starting the spin task
    try:
        await node.goal_set_event.wait()
        await node.path_published_event.wait()
        await node.robot_task_event.wait()
        await node.navigation_complete.wait()
    except asyncio.CancelledError:
        pass

    spin_task.cancel()
    await spin_task

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    asyncio.run(main())


