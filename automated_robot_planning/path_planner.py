#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from automated_robot_planning.obstacle_detector import ObstacleDetector
import math
import dubins
import numpy as np
import tf2_ros
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseArray, Polygon, PoseStamped
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion, Twist
from obstacles_msgs.msg import ObstacleArrayMsg
from visualization_msgs.msg import MarkerArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import xacro
from urdf_parser_py.urdf import URDF
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import JointState
from nav_msgs.msg import Path


collision_detected = False


class PathPlanner(Node): 
    def __init__(self, controller_node, robot_name, start_pose, gate_position):
        super().__init__('path_planner', namespace=robot_name)
        self.namespace = robot_name
        self.controller_node = controller_node   
        self.raytrace_min_range = 0.0
        self.raytrace_max_range = 3.0
        self.costmap_resolution = 0.05
        self.costmap_size = [3, 3]
        self.gate_position = gate_position 
        self.const_linear_velocity = [0.2, 0.0, 0.5]
        self.walls = [] 
        self.solid_obs = []
        self.q_goal = PoseStamped()
        self.goal_point = (0.0, 0.0, 0.0)
        self.q_goal, self.goal_point = self.calculate_goal_pose()
        self.robot = ObstacleDetector.polygon_offsetting(robot, 0.1, isRobot=True)
        self.safety_distance = 0.1
        self.obstacle_threshold = 1
        self.robot_id = None
        PathPlanner.costmap = None
        PathPlanner.g_costmap = None
        self.start_pose = start_pose
        self.navigation_complete = asyncio.Event()
        self.path_published_event = asyncio.Event()
        #self.robot_task_event = asyncio.Event()

        self.path_pub = self.create_publisher(Path, f'/{self.robot_name}/plan1', 10)
        self.follow_path_client = ActionClient(self, FollowPath, f'/{self.robot_name}/follow_path')


        # Define QoS profile for the publisher
        qos_profile = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        qos_profile_initial_pose = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        self.initial_pose_pub = self.create_publisher(PoseStamped, f'/{self.robot_name}/initialpose', qos_profile_initial_pose)

        init_pose_msg = PoseWithCovarianceStamped()
        init_pose_msg.header.frame_id = "map"
        init_pose_msg.header.stamp = self.get_clock().now().to_msg()

        # Set your desired initial position
        init_pose_msg.pose.pose.position.x = self.start_pose[0]
        init_pose_msg.pose.pose.position.y = self.start_pose[1]
        init_pose_msg.pose.pose.position.z = 0.0
        quat = tf2_ros.transformations.quaternion_from_euler(0, 0, self.start_pose[2])
        init_pose_msg.pose.pose.orientation.x = quat[0]
        init_pose_msg.pose.pose.orientation.y = quat[1]
        init_pose_msg.pose.pose.orientation.z = quat[2]
        init_pose_msg.pose.pose.orientation.w = quat[3]


        self.initial_pose_pub.publish(init_pose_msg)
        self.get_logger().info(f'Published initialpose: x={self.start_pose[0]}, y={self.start_pose[1]}, yaw={self.start_pose[2]}')
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
        

        self.create_subscription(
            ObstacleArrayMsg,
            f'/{self.namespace}/obstacles',
            self.obs_callback,
            qos_profile)

        '''self.create_subscription(
            PoseArray,
            f'/{self.namespace}/gate_position',
            self.gate_callback,
            qos_profile)'''
        

        self.create_subscription(
            Polygon,
            f'/{self.namespace}/map_borders',
            self.wall_callback,
            qos_profile)

        self.tf_pub = self.create_publisher(
            TFMessage, 
            f'/{self.namespace}/tf_publisher', 
            QoSProfile(
                depth=20,
                durability=DurabilityPolicy.VOLATILE,
                reliability=ReliabilityPolicy.RELIABLE,
            ))
        
        self.plan_pub = self.create_publisher(
            Path,
            f'/{self.namespace}/received_global_plan',
            qos_profile) 

        self.waypoint_pub = self.create_publisher(
            MarkerArray,
            f'/{self.namespace}/waypoints',
            qos_profile)
        
        self.joint_state_pub = self.create_publisher(
            JointState, 
            f'/{self.namespace}/joint_states_publisher', 
            qos_profile)
        
        self.cmd_vel_publisher = self.create_publisher(
            Twist, 
            f'/{self.namespace}/cmd_vel', 
            10)

        # Create timers to run tasks every 1 second
        self.create_timer(0.1, self.spinner)

                # Parse the URDF
        self.local_costmap_pub = self.create_publisher(
            OccupancyGrid,
            f'/{self.namespace}/local_costmap/costmap',
            qos_profile)
        
        self.global_costmap_pub = self.create_publisher(
            OccupancyGrid,
            f'/{self.namespace}/global_costmap/costmap',
            qos_profile)

        self.create_subscription(
            LaserScan,
            f'/{self.namespace}/scan',
            self.laser_scan_callback,
            QoSProfile(
                depth=20,
                durability=DurabilityPolicy.VOLATILE,
                reliability=ReliabilityPolicy.RELIABLE,
            ))
        self.odom_sub = self.create_subscription(
            Odometry,
            f'/{self.namespace}/odom',
            self.odom_callback,
            10
        )

    def odom_callback(self, msg: Odometry):
        yaw = self.yaw_from_two_points(msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.get_current_pos_and_or = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw) # geometry_msgs/Pose

    def send_follow_path_goal(self, path_msg, max_retries=3):
        for attempt in range(max_retries):
            # Wait for the action server to be available
            if self.follow_path_client.wait_for_server(timeout_sec=5.0):
                
                #for attempt in range(max_retries):
                goal_msg = FollowPath.Goal()
                goal_msg.path = path_msg
                goal_msg.controller_id = "FollowPath"
                goal_msg.goal_checker_id = "simple_goal_checker"
                goal_msg.progress_checker_id = "simple_progress_checker"
                self.get_logger().info('Action server is available')
                send_goal_future = self.follow_path_client.send_goal_async(goal_msg)
                self.get_logger().info('Sending path to FollowPath action server')
                goal_handle = await send_goal_future

                if not goal_handle.accepted:
                    self.get_logger().error('FollowPath goal rejected!')
                    continue
                
                else:
                    self.get_logger().info('FollowPath goal accepted, waiting for result...')
                    result_future = goal_handle.get_result_async()
                    result = await result_future
                self.get_logger().info(f'FollowPath result: {result.status}')
                self.get_logger().info(f"FollowPath result code: {result.result.error_code}")
                self.get_logger().info(f"Error string: {result.result.error_string}")
                self.navigation_complete.set()
            break
        
        
    def laser_scan_callback(self, msg):
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
            costmap_msg.info.origin.position.z = 0
            costmap_msg.info.origin.orientation.w = 1.0
            costmap_msg.data = self.costmap.flatten().tolist()
            
            # Publish the updated costmap
            self.local_costmap_pub.publish(costmap_msg)


            # Conditional publishing to global costmap
            #if self.should_update_global_costmap():
            #    self.global_costmap_pub.publish(costmap_msg)
            #    self.previous_global_costmap = self.costmap.copy()  # Update the previous costmap
        except Exception as e:
            self.get_logger().error(f"Laser Scan Callback failed: {e}")


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
            print('hi', msg)
            self.get_logger().info('Received local costmap')
            #self.process_costmap(msg)
            PathPlanner.costmap = msg
            self.costmap_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
            self.costmap_resolution = msg.info.resolution
            self.costmap_width = msg.info.width
            self.costmap_height = msg.info.height
            self.costmap_size = (self.costmap_width, self.costmap_height)
            self.costmap_data = np.array(msg.data, dtype=np.int8).reshape((self.costmap_height, self.costmap_width))
            self.costmap_data = self.costmap_data.T
            self.costmap_data = self.costmap_data[::-1]
        except Exception as e:
            self.get_logger().error(f"Local Costmap Callback failed: {e}")


    def global_costmap_callback(self, msg):
        try:
            print('hi', msg)
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
                    self.manage_collision(self.robot_task, robot)

    def calculate_goal_pose(self):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"  # Assuming gate position is in the map frame
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = self.gate_position.position.x
        goal_pose.pose.position.y = self.gate_position.position.y
        goal_pose.pose.position.z = self.gate_position.position.z
        goal_pose.pose.orientation.x = self.gate_position.orientation.x
        goal_pose.pose.orientation.y = self.gate_position.orientation.y
        goal_pose.pose.orientation.z = self.gate_position.orientation.z
        goal_pose.pose.orientation.w = self.gate_position.orientation.w 
        _, _, yaw = tf2_ros.transformations.euler_from_quaternion([goal_pose.pose.orientation.x, goal_pose.pose.orientation.y, goal_pose.pose.orientation.z, goal_pose.pose.orientation.w])
        return goal_pose, (self.gate_position.position.x, self.gate_position.position.y, yaw)



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


    def get_transform(self, pose):
        
        # Create TransformStamped message
        transform_stamped = TransformStamped()
        
        # Populate Header
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = "map"

        # Populate child_frame_id
        transform_stamped.child_frame_id = "base_link"

        # Populate Transform
        transform_stamped.transform.translation = Vector3(z=pose.pose.position.x, y=pose.pose.position.y, z=0.0)
        transform_stamped.transform.rotation = Quaternion(x=0.0, y=0.0, z=0.0, w=pose.pose.orientation.w)
        
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

    def robot_task(self, in_pose):
        global collision_detected
        if collision_detected:        
            self.manage_collision(self.robot_task)
        else:
            try:
                graph = [in_pose]
                path = Path()
                path.header.frame_id = "map"
                path.header.stamp = self.get_clock().now().to_msg()
                graph = self.get_trajectory(self.goal_point, graph, shelfino)
                
                
                #waypoints_list, segments = self.get_waypoints(in_pose, graph)
                for i in range(len(graph) - 1):
                    start = graph[i]
                    end = graph[i + 1]
                    local_path = DubinsPath(start, end, 0.5)   
                    print(local_path)   
                    self.publish_velocities(local_path[0], local_path[1])
                    pose = PoseStamped()
                    pose.header.frame_id = "map"
                    pose.header.stamp = self.get_clock().now().to_msg()
                    pose.pose.position.x = local_path[0]
                    pose.pose.position.y = local_path[1]
                    pose.pose.position.z = 0.0
                    yaw = self.yaw_from_two_points(start[0], start[1], end[0], end[1])
                    quat = tf2_ros.transformations.quaternion_from_euler(0, 0, yaw)
                    pose.pose.orientation.x = quat[0]
                    pose.pose.orientation.y = quat[1]
                    pose.pose.orientation.z = quat[2]
                    pose.pose.orientation.w = quat[3]
                    self.publish_joint_state(pose)
                    tf_msg = TFMessage()
                    tf_msg.transforms.append(self.get_transform(pose))
                    self.tf_pub.publish(tf_msg)
                    path.poses.append(pose)
                self.path_pub.publish(path)
                self.send_follow_path_goal(path)
                self.get_logger().info(f'Published global plan {path}')
                #waypoints = self.create_waypoints(waypoints_list)
                #self.controller_node.follow_pth(shelfino.client, waypoints)
                print(graph)
                self.path_published_event.set()
                
                  
                
                               
            except Exception as e:
                self.get_logger().error(f"Task of Robot {shelfino.get_namespace()} failed: {e}")
        

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
        new_node = np.zeros(3)

        # Extend from the nearest node towards the sampled point
        delta_x1 = q_random[0] - q_nearest[0]
        delta_x2 = q_random[1] - q_nearest[1]
        delta_x3 = q_random[2] - q_nearest[2]
        #delta_x4 = q_random[3] - q_nearest[3]
        norm = np.linalg.norm(np.array([delta_x1, delta_x2, delta_x3]))
    
        # Check if norm is zero or very close to zero
        if norm > 1e-10:  # A small threshold to avoid division by zero
            delta_x1 /= norm
            delta_x2 /= norm
            delta_x3 /= norm
        else:
            # Handle the zero norm case appropriately
            print("Norm is zero or very close to zero, cannot normalize deltas")
            delta_x1 = 0.0
            delta_x2 = 0.0
            delta_x3 = 0.0
        
        if norm>step_size:            
            new_node[0] = (q_nearest[0] + (step_size * delta_x1))
            new_node[1] = (q_nearest[1] + (step_size * delta_x2))
            new_node[2] = (q_nearest[2] + (step_size * delta_x3))
            #new_node[3] = (q_nearest[3] + (step_size * delta_x4)).astype(int)    
        else:
            new_node = q_random
        
        
        return tuple(new_node)


    def rewire(self, near_nodes, idx, q_new):
        near_nodes[idx] = q_new
        

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
            mx, my = self.world_to_map(q[0], q[1])
            return (0 <= mx < self.local_costmap.shape[0] and
                    0 <= my < self.local_costmap.shape[1] and
                    self.local_costmap[mx, my] < 50)  # threshold
        except:
            return False



    def get_trajectory(self, q_goal, graph, robot):        
        q_init = graph[0]
        while self.get_cost(q_goal, q_init)>0.3:        
            q_random = (np.random.uniform(q_goal[0]-0.5, q_goal[0]+0.5), 
                        np.random.uniform(q_goal[1]-0.5, q_goal[1]+0.5),
                        np.random.uniform(q_goal[2]-0.5, q_goal[2]+0.5))    
            #if self.get_mul_cost(q_random, self.solid_obs)>self.robot_size and self.get_cost(q_random, self.robot_coords)>self.robot_size  and self.get_cost(q_random, self.walls)>self.robot_size:
            if not self.is_in_free_space(q_random):
                continue
            q_nearest = self.nearest(graph, q_random) 
            q_new = self.extend(q_nearest, q_random)
            if not self.is_in_free_space(q_new):
                continue
            min_cost = self.get_cost(q_goal, q_nearest)
            near_nodes = [node for node in graph if (self.get_cost(q_new, node) <= min_cost)] 
            for idx, near_node in enumerate(near_nodes):
                #if self.get_cost(q_new, self.robot_coords)>self.robot_size and self.get_mul_cost(q_new, self.solid_obs)>self.robot_size and self.get_cost(q_new, self.walls)>self.robot_size:                   
                if self.is_in_free_space(q_new):
                        new_cost = self.get_cost(q_goal, near_node) + self.get_cost(q_new, near_node)
                        if new_cost < min_cost:
                            min_cost = new_cost
                else:
                    collision_detected = True
                    self.manage_collision(self.robot_task)
                #if (self.is_joint_okay(q_new)) and (not self.is_singular(self.compute_forward_kinematics_from_configuration(q_new))):
                graph.append(q_new)
                for idx, near_node in enumerate(near_nodes):
                    #if self.get_cost(near_node, self.robot_coords)>self.robot_size and self.get_mul_cost(near_node, self.solid_obs)>self.robot_size and self.get_cost(near_node, self.walls)>self.robot_size: 
                    if self.is_in_free_space(near_node):
                        if (self.get_cost(q_goal, q_new) + self.get_cost(q_new, near_node))<self.get_cost(q_goal, near_node):
                            self.rewire(near_nodes, idx, q_new) 
                    else:
                        collision_detected = True 
                        self.manage_collision(self.robot_task)
                q_init = q_new   
            
        return graph



    def is_joint_okay(self, q_result):
        for i, joint in enumerate(self.robot_desc.joints):
            if q_result[i]>=joint.limit.lower and q_result[i]<=joint.limit.upper:
                print('Joint conf okay')
                if i==3:
                    break
            else:
                return False
        return True


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
    

