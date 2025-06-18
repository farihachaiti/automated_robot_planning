#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import (PoseStamped, PoseWithCovarianceStamped, 
                              Twist, TransformStamped, PoseArray, Point, Vector3)
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan, JointState
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from nav2_msgs.action import NavigateThroughPoses
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, LookupException, ConnectivityException, ExtrapolationException
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, LivelinessPolicy, HistoryPolicy
from rclpy.duration import Duration
import numpy as np
import time
import asyncio

class RobotController(Node):
    def __init__(self):
        # Initialize the node - namespace will be set by the launch file
        super().__init__('robot_controller')
        
        # Get robot name parameter
        self.declare_parameter('robot_name', 'shelfino0')
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        # Clean up the robot name
        self.robot_name = self.robot_name.strip().rstrip('/')
        
        # Scan monitoring
    
        self._amcl_pose = None
        self._last_amcl_update = None


        # QoS profiles
        self.qos_laser = QoSProfile(history=1, depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
        
        # QoS profile for AMCL pose subscription (matches AMCL publisher)
        self.qos_amcl = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # Matches AMCL publisher
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            liveliness=LivelinessPolicy.AUTOMATIC,
            liveliness_lease_duration=Duration()
        )
        

        
        # QoS profile for initial pose subscription (matches RViz2 publisher)
        self.qos_initial_pose = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            liveliness=LivelinessPolicy.AUTOMATIC,
            liveliness_lease_duration=Duration()
        )
        # Create a QoSProfile to match rmw_qos_profile_custom from utilities.hpp
        self.gate_qos = QoSProfile(
            history=1,  # KEEP_LAST
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,  # Changed from VOLATILE to TRANSIENT_LOCAL
            # Using default values for the following as in rmw_qos_profile_custom
            deadline=rclpy.duration.Duration(),  # RMW_QOS_DEADLINE_DEFAULT
            lifespan=rclpy.duration.Duration(),  # RMW_QOS_LIFESPAN_DEFAULT
            liveliness=LivelinessPolicy.SYSTEM_DEFAULT,  # RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT
            liveliness_lease_duration=rclpy.duration.Duration(),  # RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT
        )
        
      

        self.qos_odom_sub = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,  # KEEP_LAST
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            deadline=rclpy.duration.Duration(),
            lifespan=rclpy.duration.Duration(),
            liveliness=LivelinessPolicy.AUTOMATIC,
            liveliness_lease_duration=rclpy.duration.Duration()
        )

        
        # TF setup - separate buffer for robot
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        #self.create_timer(0.1, self.lookup_transform) 
        
        # TF broadcaster for odom to base_link transform
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribe to initial pose from RViz2
        self.initial_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            f'/{self.robot_name}/initialpose',
            self.initial_pose_callback,
            self.qos_initial_pose
        )
        self.latest_initial_pose = None  # Store the latest initial pose
        
        # Publishers - use relative topics (no leading slash)
        # The node's namespace will be automatically prepended
        # Odometry subscriber - using processed odometry from shelfino_node

            
        # QoS profile for path publisher (matching RViz2's subscription)
        self.qos_path = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            liveliness=LivelinessPolicy.AUTOMATIC,
            liveliness_lease_duration=Duration(seconds=10)
        )
        
        self.path_pub = self.create_publisher(
            Path, 'plan1', self.qos_path)
            
        # Publisher for waypoints visualization
        self.waypoints_pub = self.create_publisher(
            MarkerArray, 'waypoints', self.qos_path)
    

        self.nav_client = ActionClient(
            self, NavigateThroughPoses, 'navigate_through_poses'
        )

        self.goal_handle = None
        
        # Subscribers - use relative topics (no leading slash)
        # The node's namespace will be automatically prepended

        # AMCL pose subscriber - uses absolute topic name with namespace
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, 
            'amcl_pose',  # This will be namespaced as /shelfinoX/amcl_pose
            self.amcl_callback, 
            self.qos_amcl
        )
        

        
        self.cmd_vel_pub = self.create_publisher(
            Twist, 'cmd_vel', 10)
            
        # Subscribe to gate position (global topic, not namespaced)
        self.gate_sub = self.create_subscription(
            PoseArray, '/gate_position', self.gate_position_callback, self.gate_qos)
    
        self.current_pose = None
        
        self.amcl_pose = None
    

        self.gate_positions = None  # Store gate positions
        self.goal_result = None  # Store navigation result
        # Create a timer to publish TF transforms
       
        self.get_logger().info(f'Node name: {self.get_name()}, Namespace: {self.get_namespace()}')
        self.get_logger().info('Robot controller initialized')




    # Callback methods
    def initial_pose_callback(self, msg):
        """
        Callback for receiving initial pose updates from RViz2.
        
        Args:
            msg (PoseWithCovarianceStamped): The initial pose message
        """
        self.latest_initial_pose = msg
        self.get_logger().info(
            f'Received initial pose: x={msg.pose.pose.position.x:.2f}, '
            f'y={msg.pose.pose.position.y:.2f} in frame {msg.header.frame_id}'
        )
        




    def amcl_callback(self, msg):
        """
        Update robot's current pose from AMCL localization.
        
        Args:
            msg (PoseWithCovarianceStamped): The AMCL pose with covariance
        """
        try:
            # Store the complete message
            self._amcl_pose = msg
            self.current_pose = msg.pose.pose
            self._last_amcl_update = self.get_clock().now()
            
            # Log pose updates at a controlled rate
            current_time = time.time()
            if not hasattr(self, '_last_pose_log_time') or current_time - self._last_pose_log_time > 2.0:
                self.get_logger().info(
                    f'AMCL Pose - Frame: {msg.header.frame_id}, X: {self.current_pose.position.x:.2f}m, Y: {self.current_pose.position.y:.2f}m',
                    throttle_duration_sec=2.0
                )
                self._last_pose_log_time = current_time
                
                # Log covariance for debugging
                cov = msg.pose.covariance
                position_variance = (cov[0] + cov[7] + cov[14]) / 3.0  # Avg of x, y, z variances
                self.get_logger().debug(
                    f'Position variance: {position_variance:.6f}',
                    throttle_duration_sec=5.0
                )
                
        except Exception as e:
            self.get_logger().error(f'Error in AMCL callback: {str(e)}')

        

        
    def gate_position_callback(self, msg):
        """Process incoming gate position data"""
        self.gate_positions = msg
        self.get_logger().info(f'Received gate positions: {len(msg.poses)} gates')
        
        # Log the position of each gate
        for i, pose in enumerate(msg.poses):
            self.get_logger().info(f'Gate {i}: position=({pose.position.x:.2f}, {pose.position.y:.2f}), orientation=({pose.orientation.x:.2f}, {pose.orientation.y:.2f}, {pose.orientation.z:.2f}, {pose.orientation.w:.2f})')

    async def send_waypoints(self, waypoints):
        """Send a sequence of waypoints to the robot (returns Future)"""
        try:
            # Wait for server with timeout
            start_time = time.time()
            timeout = 30.0
            
            self.get_logger().info(f"Waiting for action server for {self.robot_name}...")
            
            while not self.nav_client.wait_for_server(timeout_sec=0.5):
                elapsed = time.time() - start_time
                if elapsed > timeout:
                    self.get_logger().error(f"Action server timeout after {timeout} seconds")
                    return None
           
                await asyncio.sleep(0.1)

            # Create and send waypoints
            goal_msg = NavigateThroughPoses.Goal()
            goal_msg.poses = waypoints
            
            # Visualize the path
            path_msg = Path()
            path_msg.header.frame_id = 'map'
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.poses = waypoints
            self.path_pub.publish(path_msg)
            
            self.get_logger().info(f"Sending {len(waypoints)} waypoints to {self.robot_name}")
            send_goal_future = self.nav_client.send_goal_async(goal_msg)
            
            # Wait for goal acceptance
            goal_handle = await send_goal_future
            if not goal_handle.accepted:
                self.get_logger().error(f"Waypoints rejected for {self.robot_name}")
                return None
                
            self.get_logger().info(f"Waypoints accepted by {self.robot_name}")
            
            # Get the result
            result_future = goal_handle.get_result_async()
            return await result_future
                
        except Exception as e:
            self.get_logger().error(f"Error: {str(e)}")
            return None

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"Goal rejected")
            return

        self.goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result().result
        self.goal_result = result
        self.get_logger().info(f"Navigation finished with result: {result}")
        
    async def navigate_to_gate(self, gate_index=0):
        """Navigate to a specific gate by index"""
        if self.gate_positions is None or len(self.gate_positions.poses) <= gate_index:
            self.get_logger().error(f'Gate index {gate_index} not available')
            return False
            
        # Get the gate position
        gate_pose = self.gate_positions.poses[gate_index]
        self.get_logger().info(f'Navigating to gate {gate_index} at position ({gate_pose.position.x:.2f}, {gate_pose.position.y:.2f})')
        
        # Create a goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose = gate_pose
        
        # Create a goal with just one pose (the gate)
        waypoints = [goal_pose]
        
        # Publish the plan for visualization
        self.publish_plan(waypoints)
        
        # Send the waypoints to the navigation stack
        return await self.send_waypoints(waypoints)

    def emergency_stop(self):
        """Send zero velocity command to the robot"""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info("Emergency stop triggered")

    def publish_plan(self, waypoints):
        """
        Publish the navigation plan to the 'plan1' topic and visualize waypoints.
        
        Args:
            waypoints: List of PoseStamped messages representing the plan
        """
        if not waypoints:
            self.get_logger().warn("Cannot publish empty waypoints list")
            return
            
        # Publish the path
        path_msg = Path()
        path_msg.header.frame_id = 'map'  # Assuming map frame
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.poses = waypoints
        self.path_pub.publish(path_msg)
        
        # Create and publish waypoint markers
        marker_array = MarkerArray()
        
        # Create a marker for each waypoint
        for i, pose_stamped in enumerate(waypoints):
            # Create a marker for the waypoint position
            marker = Marker()
            marker.header = pose_stamped.header
            marker.ns = "waypoints"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = pose_stamped.pose
            marker.scale = Vector3(x=0.1, y=0.1, z=0.1)  # Small sphere
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # Red
            marker.lifetime = Duration(seconds=0).to_msg()  # Never auto-delete
            marker_array.markers.append(marker)
            
            # Add text label with waypoint number
            text_marker = Marker()
            text_marker.header = pose_stamped.header
            text_marker.ns = "waypoint_labels"
            text_marker.id = i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose = pose_stamped.pose
            text_marker.pose.position.z += 0.2  # Position text above the marker
            text_marker.scale = Vector3(x=0.0, y=0.0, z=0.1)
            text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)  # White
            text_marker.text = str(i)
            text_marker.lifetime = Duration(seconds=0).to_msg()
            marker_array.markers.append(text_marker)
        
        # Publish the marker array
        self.waypoints_pub.publish(marker_array)
        self.get_logger().info(f"Published {len(waypoints)} waypoints for visualization")
        
    def get_robot_pose(self, max_retries=10, retry_delay=0.5, require_recent=False):
        """
        Get the current pose of the robot from AMCL with retry logic.
        
        Args:
            max_retries (int): Maximum number of retry attempts
            retry_delay (float): Delay between retries in seconds
            require_recent (bool): If True, only return pose if it was updated recently
            
        Returns:
            geometry_msgs.msg.Pose: Current pose of the robot from AMCL
            
        Raises:
            RuntimeError: If AMCL pose is not available after all retries or is too old
        """
        from copy import deepcopy
        
        # Check if we have a recent AMCL pose first
        if self._amcl_pose is not None:
            if not require_recent or \
               (self._last_amcl_update is not None and 
                (self.get_clock().now() - self._last_amcl_update).nanoseconds < 1e9):  # 1 second threshold
                return deepcopy(self._amcl_pose.pose.pose)
        
        # If we need to wait for a fresh pose
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(self)
        
        try:
            for attempt in range(max_retries):
                current_time = self.get_clock().now()
                
                # Check if we have a recent AMCL update
                if (self._amcl_pose is not None and 
                    (not require_recent or 
                     (self._last_amcl_update is not None and 
                      (current_time - self._last_amcl_update).nanoseconds < 1e9))):  # 1 second threshold
                    return deepcopy(self._amcl_pose.pose.pose)
                
                # Log status periodically
                if attempt % 2 == 0:  # Reduced logging frequency
                    status = "Waiting for AMCL pose"
                    if self._amcl_pose is not None and self._last_amcl_update is not None:
                        age = (current_time - self._last_amcl_update).nanoseconds / 1e9
                        status += f" (last update {age:.1f}s ago)"
                    self.get_logger().info(
                        f"{status} (attempt {attempt + 1}/{max_retries})..."
                    )
                
                # Process callbacks while waiting
                executor.spin_once(timeout_sec=retry_delay)
            
            # If we get here, we've exhausted our retries
            raise RuntimeError(
                f"Failed to get recent AMCL pose after {max_retries} attempts. "
                "Check if AMCL is running and publishing poses correctly."
            )
            
        finally:
            # Clean up the executor
            executor.shutdown()
            executor.remove_node(self)
            

        

    def get_robot_position(self):
        """
        Get the current 2D position (x, y) of the robot in the map frame.
        
        Returns:
            tuple: (x, y) coordinates of the robot
            
        Raises:
            RuntimeError: If the current pose is not available
        """
        pose = self.get_robot_pose()
        return (pose.position.x, pose.position.y)
        
    def get_robot_yaw(self):
        """
        Get the current yaw angle of the robot in radians.
        
        Returns:
            float: Yaw angle in radians
            
        Raises:
            RuntimeError: If the current pose is not available
        """
        from math import atan2
        pose = self.get_robot_pose()
        q = pose.orientation
        
        # Convert quaternion to yaw (simplified for 2D)
        yaw = atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return yaw

    def create_waypoint_path(self, start_x, start_y, end_x, end_y, num_waypoints=5):
        """Create a series of waypoints between start and end points"""
        waypoints = []
            
        for i in range(num_waypoints):
            # Linear interpolation between start and end
            fraction = i / (num_waypoints - 1)
            x = start_x + fraction * (end_x - start_x)
            y = start_y + fraction * (end_y - start_y)
                
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            # Keep the same orientation for all waypoints except the final one
            pose.pose.orientation.w = 1.0
                
            waypoints.append(pose)
            
        return waypoints

async def main(args=None):
    # Initialize ROS
    rclpy.init(args=args)
    
    # Create controller
    controller = RobotController()
    
    # Create executor
    executor = MultiThreadedExecutor()
    executor.add_node(controller)
    
    # Spin in a separate thread
    import threading
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    # Create waypoint paths for the robot
    try:
        # Initial pose is now handled by AMCL configuration or RViz2

        # Wait for gate positions to be received
        controller.get_logger().info("Waiting for gate positions...")
        wait_time = 0
        while controller.gate_positions is None and wait_time < 10:
            await asyncio.sleep(1.0)
            wait_time += 1
            controller.get_logger().info(f"Waiting for gate positions... {wait_time}/10")
        
        # Generate waypoints from current position towards the gate
        if controller.gate_positions is not None and len(controller.gate_positions.poses) > 0:
            # Get the gate position
            gate_pose = controller.gate_positions.poses[0]
            controller.get_logger().info(f"Gate detected at position: ({gate_pose.position.x:.2f}, {gate_pose.position.y:.2f})")
            
            # Get current robot position with fallback to initial pose or default position
            current_pose = controller.get_robot_pose()
            if current_pose is not None:
                current_x = current_pose.position.x
                current_y = current_pose.position.y
            elif hasattr(controller, 'latest_initial_pose') and controller.latest_initial_pose is not None:
                controller.get_logger().warn("Using initial pose as fallback for current position")
                current_x = controller.latest_initial_pose.pose.pose.position.x
                current_y = controller.latest_initial_pose.pose.pose.position.y
            else:
                # Fallback to a default position if neither AMCL nor initial pose is available
                controller.get_logger().error("Could not determine current position, using default (0, 0)")
                current_x, current_y = 0.0, 0.0
            
            # Create waypoints from current position to gate
            controller.get_logger().info(f"Creating waypoints from ({current_x:.2f}, {current_y:.2f}) to gate at ({gate_pose.position.x:.2f}, {gate_pose.position.y:.2f})")
            waypoints = controller.create_waypoint_path(
                current_x, current_y, 
                gate_pose.position.x, gate_pose.position.y, 
                num_waypoints=5
            )
            
            # Send waypoints
            controller.get_logger().info(f"Sending {len(waypoints)} waypoints to navigate to gate")
            result = await controller.send_waypoints(waypoints)
            
            if result:
                controller.get_logger().info(f"Navigation to gate completed with result: {result}")
            else:
                controller.get_logger().error("Navigation to gate failed")
        else:
            # Fallback to predefined waypoints if no gates are available
            controller.get_logger().warn("No gate positions received, using predefined waypoints")
            waypoints = controller.create_waypoint_path(-2.0, -2.0, 2.0, 1.0, num_waypoints=5)
            result = await controller.send_waypoints(waypoints)
            
            if result:
                controller.get_logger().info(f"Navigation result: {result}")
            else:
                controller.get_logger().error("Navigation failed")
            
    except KeyboardInterrupt:
        controller.emergency_stop()
    except Exception as e:
        controller.get_logger().error(f"Main error: {str(e)}")
    finally:
        controller.destroy_node()
        rclpy.shutdown()
        executor_thread.join()

if __name__ == '__main__':
    asyncio.run(main())
