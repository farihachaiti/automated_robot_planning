#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import (PoseStamped, PoseWithCovarianceStamped, 
                              Twist, TransformStamped)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from nav2_msgs.action import NavigateToPose
from tf2_ros import Buffer, TransformListener
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.duration import Duration
import numpy as np

import asyncio

class RobotController(Node):
    def __init__(self, robot_names):
        super().__init__('robot_controller')
        self.robot_names = robot_names
        self.robot_data = {}
        
        # QoS profiles (unchanged)
        self.qos_laser = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
        self.qos_odom = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
        self.qos_amcl = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        tf_buffer = Buffer()
        # Initialize for each robot
        for name in robot_names:
            self.robot_data[name] = {
                # Action client
                'nav_client': ActionClient(self, NavigateToPose, f'/{name}/navigate_to_pose'),
                'goal_handle': None,
                
                # TF
                'tf_buffer': tf_buffer,
                'tf_listener': TransformListener(tf_buffer, self),
                
                # Subscribers (unchanged)
                'odom_sub': self.create_subscription(Odometry, f'/{name}/odom', lambda msg, n=name: self.odom_callback(msg, n), self.qos_odom),
                'amcl_sub': self.create_subscription(PoseWithCovarianceStamped, f'/{name}/amcl_pose', lambda msg, n=name: self.amcl_callback(msg, n), self.qos_amcl),
                'scan_sub': self.create_subscription(LaserScan, f'/{name}/scan', lambda msg, n=name: self.scan_callback(msg, n), self.qos_laser),
                
                # Publisher
                'cmd_vel_pub': self.create_publisher(Twist, f'/{name}/cmd_vel', 10),
                
                # Data storage
                'current_pose': None,
                'odom': None,
                'amcl_pose': None,
                'scan': None,
                'goal_result': None
            }

    # Callback methods
    def odom_callback(self, msg, robot_name):
        self.robot_data[robot_name]['odom'] = msg

    def amcl_callback(self, msg, robot_name):
        self.robot_data[robot_name]['amcl_pose'] = msg
        self.robot_data[robot_name]['current_pose'] = msg.pose.pose

    def scan_callback(self, msg, robot_name):
        self.robot_data[robot_name]['scan'] = msg

    async def send_goal_to_robot(self, robot_name, goal_pose):
        """Send a navigation goal to a specific robot."""
        if robot_name not in self.robot_names:
            self.get_logger().error(f"Unknown robot: {robot_name}")
            return False

        # Wait for the action server
        if not await self.robot_data[robot_name]['nav_client'].wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f"Action server not available for {robot_name}")
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        # Send goal and register callback
        send_goal_future = self.robot_data[robot_name]['nav_client'].send_goal_async(
            goal_msg,
            feedback_callback=lambda feedback, n=robot_name: self.feedback_callback(feedback, n)
        )
        send_goal_future.add_done_callback(
            lambda future, n=robot_name: self.goal_response_callback(future, n)
        )

        self.get_logger().info(f"Sent goal to {robot_name}: {goal_pose.pose.position.x}, {goal_pose.pose.position.y}")
        return True

    def goal_response_callback(self, future, robot_name):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"Goal rejected for {robot_name}")
            return

        self.robot_data[robot_name]['goal_handle'] = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future, n=robot_name: self.goal_result_callback(future, n)
        )

    def goal_result_callback(self, future, robot_name):
        result = future.result().result
        self.robot_data[robot_name]['goal_result'] = result
        self.get_logger().info(f"{robot_name} navigation finished with result: {result}")

    def feedback_callback(self, feedback, robot_name):
        # Optional: Handle feedback (e.g., display remaining distance)
        pass

    def emergency_stop(self):
        """Send zero velocity command to all robots"""
        cmd = Twist()
        for name in self.robot_names:
            self.robot_data[name]['cmd_vel_pub'].publish(cmd)
        self.get_logger().info("Emergency stop triggered")

    def get_robot_pose(self, robot_name):
        """Get the current pose of a robot in the map frame."""
        try:
            transform = self.robot_data[robot_name]['tf_buffer'].lookup_transform(
                'map',
                f'{robot_name}/base_link',
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
            self.get_logger().warn(f"TF lookup for {robot_name} failed: {str(e)}")
            return None

async def main(args=None):
    rclpy.init(args=args)


    robot_names = ["shelfino0", "shelfino1", "shelfino2"]
    controller = RobotController(robot_names)


    # Create goal pose
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = controller.get_clock().now().to_msg()
    goal_pose.pose.position.x = 2.0
    goal_pose.pose.position.y = 1.0
    goal_pose.pose.orientation.w = 1.0  # No rotation



    try:
        # Send goals concurrently
        await asyncio.gather(
            *[controller.send_goal_to_robot(name, goal_pose) for name in controller.robot_names]
        )
        
        # Keep node alive
        while rclpy.ok():
            await asyncio.sleep(1)
    except KeyboardInterrupt:
        controller.emergency_stop()
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    asyncio.run(main())
