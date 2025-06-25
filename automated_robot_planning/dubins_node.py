#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, PoseArray
from nav_msgs.msg import Path
import tf_transformations
import dubins
from nav2_msgs.action import FollowPath
from rclpy.action import ActionClient
import asyncio
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import threading
from rclpy.executors import MultiThreadedExecutor

class DubinsPathPlanner(Node):
    def __init__(self):
        super().__init__('dubins_path_planner')
        self.declare_parameter('robot_name', 'shelfino0')
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value.strip().rstrip('/')
        self.declare_parameter('turning_radius', 1.0)
        self.declare_parameter('step_size', 0.1)

        self.turning_radius = self.get_parameter('turning_radius').value
        self.step_size = self.get_parameter('step_size').value
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)

        initial_x = self.get_parameter('initial_x').value
        initial_y = self.get_parameter('initial_y').value
        initial_yaw = self.get_parameter('initial_yaw').value

        self.start_pose = PoseStamped()
        self.start_pose.header.frame_id = 'map'
        self.start_pose.header.stamp = self.get_clock().now().to_msg()
        self.start_pose.pose.position.x = initial_x
        self.start_pose.pose.position.y = initial_y
        self.start_pose.pose.position.z = 0.0
        q = tf_transformations.quaternion_from_euler(0, 0, initial_yaw)
        self.start_pose.pose.orientation.x = q[0]
        self.start_pose.pose.orientation.y = q[1]
        self.start_pose.pose.orientation.z = q[2]
        self.start_pose.pose.orientation.w = q[3]

        self.gate_sub = self.create_subscription(
            PoseArray,
            '/gate_position',
            self.gate_position_callback,
            QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                depth=1
            )
        )
        self.goal_pose = None
        self.gate_positions = None

        self.navigation_complete = asyncio.Event()
        self.goal_set_event = asyncio.Event()
        self.path_published_event = asyncio.Event()

        self.plan_timer = self.create_timer(1.0, self.try_plan)

        self.path_pub = self.create_publisher(Path, f'/{self.robot_name}/plan1', 10)
        self.follow_path_client = ActionClient(self, FollowPath, f'/{self.robot_name}/follow_path')
        self.get_logger().info('DubinsPathPlanner node started.')


    def gate_position_callback(self, msg):
        """Process incoming gate position data"""
        self.gate_positions = msg
        self.get_logger().info(f'Received {len(msg.poses)} gate positions')

        if not msg.poses:
            self.get_logger().warn("Received empty gate positions, goal not updated.")
            return

        # Create goal pose from the first gate position
        gate_pose = self.gate_positions.poses[0]
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = 'map'
        self.goal_pose.header.stamp = self.get_clock().now().to_msg()
        self.goal_pose.pose = gate_pose
        self.get_logger().info(f"Goal pose set to first gate: {gate_pose.position.x}, {gate_pose.position.y}")
        self.goal_set_event.set()
        self.try_plan()


    def try_plan(self):
        if self.start_pose and self.goal_pose:
            # Avoid re-planning if goal is the same
            # This is a simple check; could be more robust
            if hasattr(self, 'last_goal') and self.last_goal == self.goal_pose:
                return
            
            self.last_goal = self.goal_pose

            # Cancel the timer after first planning
            if hasattr(self, 'plan_timer') and self.plan_timer is not None:
                self.plan_timer.cancel()
                self.plan_timer = None

            start = self.pose_to_tuple(self.start_pose.pose)
            goal = self.pose_to_tuple(self.goal_pose.pose)

            path = dubins.shortest_path(start, goal, self.turning_radius)
            configurations, _ = path.sample_many(self.step_size)

            path_msg = Path()
            path_msg.header.frame_id = self.start_pose.header.frame_id
            for q in configurations:
                pose = PoseStamped()
                pose.header.frame_id = path_msg.header.frame_id
                pose.pose.position = Point(x=q[0], y=q[1], z=0.0)
                pose.pose.orientation = self.yaw_to_quaternion(q[2])
                path_msg.poses.append(pose)

            self.path_pub.publish(path_msg)
            self.get_logger().info(f"Published Dubins path with {len(path_msg.poses)} points.")
            self.path_published_event.set()

            asyncio.create_task(self.send_follow_path_goal(path_msg))

    def pose_to_tuple(self, pose: Pose):
        x = pose.position.x
        y = pose.position.y
        yaw = self.quaternion_to_yaw(pose.orientation)
        return (x, y, yaw)

    def quaternion_to_yaw(self, q: Quaternion):
        _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        return yaw

    def yaw_to_quaternion(self, yaw):
        q = tf_transformations.quaternion_from_euler(0, 0, yaw)
        return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

    async def send_follow_path_goal(self, path_msg):
        # Wait for the action server to be available
        if not self.follow_path_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('FollowPath action server not available!')
            self.navigation_complete.set()
            return

        goal_msg = FollowPath.Goal()
        goal_msg.path = path_msg

        self.get_logger().info('Sending path to FollowPath action server...')
        send_goal_future = self.follow_path_client.send_goal_async(goal_msg)
        goal_handle = await send_goal_future

        if not goal_handle.accepted:
            self.get_logger().error('FollowPath goal rejected!')
            self.navigation_complete.set()
            return

        self.get_logger().info('FollowPath goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        result = await result_future
        self.get_logger().info(f'FollowPath result: {result.status}')
        self.navigation_complete.set()

async def main(args=None):
    rclpy.init(args=args)
    node = DubinsPathPlanner()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    try:
        node.get_logger().info("Waiting for gate positions...")
        # Wait for goal to be set
        await node.goal_set_event.wait()
        node.get_logger().info("Goal set event received.")
        # Wait for path to be published
        await node.path_published_event.wait()
        node.get_logger().info("Path published event received.")
        # Wait for navigation to complete
        await node.navigation_complete.wait()
        node.get_logger().info("Dubins node has completed its task.")

    except Exception as e:
        node.get_logger().error(f"Main error: {str(e)}")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        executor_thread.join()

if __name__ == '__main__':
    asyncio.run(main())
