#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import (PoseStamped, PoseWithCovarianceStamped, 
                              Twist, TransformStamped, PoseArray)
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan, JointState
from tf2_msgs.msg import TFMessage
from nav2_msgs.action import NavigateThroughPoses
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, LivelinessPolicy
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

        # QoS profiles
        self.qos_laser = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
        self.qos_odom = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
        self.qos_amcl = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
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
        # TF setup - separate buffer for robot
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        
        # TF broadcaster for odom to base_link transform
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Publishers - use relative topics (no leading slash)
        # The node's namespace will be automatically prepended
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', self.qos_amcl)
        self.cmd_vel_pub = self.create_publisher(
            Twist, 'cmd_vel', 10)
        self.path_pub = self.create_publisher(
            Path, 'planned_path', 10)
        
  
        self.nav_client = ActionClient(
            self, NavigateThroughPoses, 'navigate_through_poses'
        )

        self.goal_handle = None
        
        # Subscribers - use relative topics (no leading slash)
        # The node's namespace will be automatically prepended
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, self.qos_odom)
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self.amcl_callback, self.qos_amcl)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, self.qos_laser)
        
        # Subscribe to joint states
        self.joint_states_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_states_callback, 10)
            
        # Subscribe to gate position (global topic, not namespaced)
        self.gate_sub = self.create_subscription(
            PoseArray, '/gate_position', self.gate_position_callback, self.gate_qos)
        
        # Data storage
        self.current_pose = None
        self.odom = None
        self.amcl_pose = None
        self.scan = None
        self.joint_states = None  # Store joint states
        self.gate_positions = None  # Store gate positions
        self.goal_result = None  # Store navigation result
        
        # Create a timer to publish TF transforms
        self.tf_timer = self.create_timer(0.05, self.publish_tf)  # 20Hz

    def publish_tf(self):
        """Publish transform from odom to base_link for this robot"""
        if self.odom is None:
            return
            
        # Create transform from odom to base_link
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        # Use clean frame IDs without double slashes
        t.header.frame_id = f'{self.robot_name}/odom'
        t.child_frame_id = f'{self.robot_name}/base_link'
        
        # Set translation from odometry message
        t.transform.translation.x = self.odom.pose.pose.position.x
        t.transform.translation.y = self.odom.pose.pose.position.y
        t.transform.translation.z = self.odom.pose.pose.position.z
        
        # Set rotation from odometry message
        t.transform.rotation = self.odom.pose.pose.orientation
        
        # Publish the transform
        self.tf_broadcaster.sendTransform(t)

    def publish_initial_pose(self, x, y, yaw_deg=0.0):
        """Publish an initial pose with dynamically calculated covariance to AMCL."""
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"

        # Position
        pose_msg.pose.pose.position.x = x
        pose_msg.pose.pose.position.y = y
        pose_msg.pose.pose.position.z = 0.0

        # Orientation (yaw in degrees to quaternion)
        yaw_rad = np.deg2rad(yaw_deg)
        pose_msg.pose.pose.orientation.z = np.sin(yaw_rad * 0.5)
        pose_msg.pose.pose.orientation.w = np.cos(yaw_rad * 0.5)

        # Dynamic covariance calculation
        position_std_dev = 0.5  # meters
        yaw_std_dev = np.deg2rad(15)  # radians
        
        # Convert std deviations to variances
        var_x = position_std_dev ** 2
        var_y = position_std_dev ** 2
        var_yaw = yaw_std_dev ** 2
        
        # Create covariance matrix (6x6 for x, y, z, roll, pitch, yaw)
        covariance = np.zeros(36)       
        
        # Position covariance (x, y)
        covariance[0] = var_x   # X variance
        covariance[7] = var_y   # Y variance
        covariance[35] = var_yaw  # Yaw variance
        
        # Z and orientation (roll/pitch) are typically 0 for ground robots
        covariance[14] = 0.0  # Z variance
        covariance[21] = 0.0  # Roll variance
        covariance[28] = 0.0  # Pitch variance
        
        pose_msg.pose.covariance = covariance.tolist()

        self.initial_pose_pub.publish(pose_msg)
        self.get_logger().info(
            f"Published initial pose for {self.robot_name}: "
            f"x={x:.2f}±{position_std_dev:.2f}m, "
            f"y={y:.2f}±{position_std_dev:.2f}m, "
            f"yaw={yaw_deg:.1f}±{np.rad2deg(yaw_std_dev):.1f}°"
        )

    # Callback methods
    def odom_callback(self, msg):
        self.odom = msg

    def amcl_callback(self, msg):
        self.amcl_pose = msg
        self.current_pose = msg.pose.pose

    def scan_callback(self, msg):
        """Process incoming laser scan data"""
        self.scan = msg
        
    def joint_states_callback(self, msg):
        """Process incoming joint states data"""
        self.joint_states = msg
        # Log joint states at debug level to avoid flooding the console
        self.get_logger().debug(f'Received joint states: {len(msg.name)} joints')
        # You can process joint states here if needed
        
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
        return await self.send_waypoints(waypoints)

    def emergency_stop(self):
        """Send zero velocity command to the robot"""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info("Emergency stop triggered")

    def get_robot_pose(self):
        """Get the current pose of the robot in the map frame."""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                f'{self.robot_name}/base_link',
                rclpy.time.Time()
            )
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.orientation = transform.transform.rotation
            return pose
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {str(e)}")
            return None

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
        # Publish initial pose
        name = controller.robot_name           # e.g. "shelfino0"
        idx = int(name.replace('shelfino', ''))  # 0, 1 or 2

        # Compute offset = idx + 1
        offset = idx + 3

        # Base starting point, for example (0,0)
        base_x, base_y = -2.0, -2.0

        # Apply dynamic offset
        x = base_x + offset
        y = base_y + offset
        controller.publish_initial_pose(x, y, yaw_deg=0.0)

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
            
            # Get current robot position
            current_pose = controller.get_robot_pose()
            if current_pose is None:
                controller.get_logger().warn("Could not get current robot position, using initial pose")
                current_x, current_y = x, y  # Use the initial pose
            else:
                current_x = current_pose.pose.position.x
                current_y = current_pose.pose.position.y
            
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
