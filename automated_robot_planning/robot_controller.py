#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped, PoseArray
from nav2_msgs.action import NavigateToPose  # Changed action type
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import asyncio
import threading

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # Get robot name parameter
        self.declare_parameter('robot_name', 'shelfino0')
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value.strip().rstrip('/')
        
        # Navigation action client - CHANGED TO NavigateToPose
        self.nav_client = ActionClient(
            self, NavigateToPose, f'/{self.robot_name}/navigate_to_pose'  # Updated action name
        )

        # Gate position subscriber
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
    
        self.gate_positions = None
        self.get_logger().info(f'Robot controller for {self.robot_name} initialized')

    def gate_position_callback(self, msg):
        """Process incoming gate position data"""
        self.gate_positions = msg
        self.get_logger().info(f'Received {len(msg.poses)} gate positions')

    async def send_goal_pose(self, pose):
        """Send a single goal pose to the robot"""
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

            # Create and send goal
            goal_msg = NavigateToPose.Goal()  # Different goal type
            goal_msg.pose = pose  # Single pose instead of list
            
            self.get_logger().info(f"Sending goal to: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f})")
            send_goal_future = self.nav_client.send_goal_async(goal_msg)
            
            # Wait for goal acceptance
            goal_handle = await send_goal_future
            if not goal_handle.accepted:
                self.get_logger().error(f"Goal rejected for {self.robot_name}")
                return None
                
            self.get_logger().info(f"Goal accepted by {self.robot_name}")
            
            # Get the result
            result_future = goal_handle.get_result_async()
            return await result_future
                
        except Exception as e:
            self.get_logger().error(f"Error: {str(e)}")
            return None
        
    async def navigate_to_gate(self, gate_index=0):
        """Navigate to a specific gate by index"""
        if self.gate_positions is None or len(self.gate_positions.poses) <= gate_index:
            self.get_logger().error(f'Gate index {gate_index} not available')
            return False
            
        # Create goal pose from gate position
        gate_pose = self.gate_positions.poses[gate_index]
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose = gate_pose
        
        self.get_logger().info(f'Navigating to gate {gate_index} at position ({gate_pose.position.x:.2f}, {gate_pose.position.y:.2f})')
        
        # Send the single goal pose
        return await self.send_goal_pose(goal_pose)

async def main(args=None):
    rclpy.init(args=args)
    controller = RobotController()
    executor = MultiThreadedExecutor()
    executor.add_node(controller)
    
    # Spin in a separate thread
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    try:
        # Wait for gate positions
        controller.get_logger().info("Waiting for gate positions...")
        await asyncio.sleep(2.0)  # Short initial wait
        
        # Wait up to 10 seconds for gate positions
        for i in range(10):
            if controller.gate_positions is not None:
                break
            controller.get_logger().info(f"Waiting for gate positions... {i+1}/10")
            await asyncio.sleep(1.0)
        
        # Navigate to gate if available
        if controller.gate_positions and len(controller.gate_positions.poses) > 0:
            controller.get_logger().info("Navigating to first gate")
            result = await controller.navigate_to_gate(0)
        else:
            # Fallback to predefined goal
            controller.get_logger().warn("No gate positions received, using predefined goal")
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = controller.get_clock().now().to_msg()
            goal_pose.pose.position.x = 2.0
            goal_pose.pose.position.y = 1.0
            goal_pose.pose.orientation.w = 1.0
            result = await controller.send_goal_pose(goal_pose)
        
        if result:
            controller.get_logger().info("Navigation completed successfully!")
        else:
            controller.get_logger().error("Navigation failed")
            
    except Exception as e:
        controller.get_logger().error(f"Main error: {str(e)}")
    finally:
        controller.destroy_node()
        rclpy.shutdown()
        executor_thread.join()

if __name__ == '__main__':
    asyncio.run(main())