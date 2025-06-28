#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, PoseArray
from nav_msgs.msg import Path
import tf_transformations
import dubins
from dubins import DubinsPath
from nav2_msgs.action import FollowPath
from rclpy.action import ActionClient
import asyncio
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import threading
from rclpy.executors import MultiThreadedExecutor
import math
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterType
from rclpy.parameter import Parameter

class DubinsPathPlanner(Node):
    def __init__(self):
        super().__init__('dubins_path_planner')
        self.declare_parameter('robot_name', 'shelfino0')
        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value.strip().rstrip('/')
        self.declare_parameter('turning_radius', 1.0)
        self.declare_parameter('step_size', 0.1)
        self.declare_parameter(
        'robot_positions',
        [0.0],
        ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY)
    )

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

        positions = self.get_parameter('robot_positions').get_parameter_value().double_array_value
        self.robot_positions = [(positions[i], positions[i+1], positions[i+2]) for i in range(0, len(positions), 3)]

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
        self.goal_pose.pose.position.x = gate_pose.position.x
        self.goal_pose.pose.position.y = gate_pose.position.y
        self.goal_pose.pose.position.z = gate_pose.position.z
        self.goal_pose.pose.orientation.x = gate_pose.orientation.x
        self.goal_pose.pose.orientation.y = gate_pose.orientation.y
        self.goal_pose.pose.orientation.z = gate_pose.orientation.z
        self.goal_pose.pose.orientation.w = gate_pose.orientation.w
        self.get_logger().info(f"Goal pose set to first gate: {gate_pose.position.x}, {gate_pose.position.y}")
        self.goal_set_event.set()
        # Convert gate_pose to (x, y, theta) for Dubins path

        self.try_plan()


    def try_plan(self):
        if self.start_pose and self.goal_pose:
            # Avoid re-planning if goal is the same
            # This is a simple check; could be more robust
            

            # Cancel the timer after first planning
            if hasattr(self, 'plan_timer') and self.plan_timer is not None:
                self.plan_timer.cancel()
                self.plan_timer = None

        # --- Simple straight-line path test ---
        #self.get_logger().info('Testing with a simple straight-line path...')
        #self.send_simple_straight_path()
        # --- End test ---

        x = self.goal_pose.pose.position.x
        y = self.goal_pose.pose.position.y
        theta = self.quaternion_to_yaw(self.goal_pose.pose.orientation)
        goal = (x, y, theta)
        # Also convert start_pose to (x, y, theta)
        start = self.pose_to_tuple(self.start_pose.pose)
        # Plan Dubins path
        dubinspath = DubinsPath(start, goal, 1.0, self.robot_positions, self.get_logger())
        path = dubinspath.plan_path(start, goal)
        # Publish the path
        path_msg = Path()
        path_msg.header.frame_id = self.start_pose.header.frame_id

        for q in path:
            pose = PoseStamped()
            pose.header.frame_id = path_msg.header.frame_id
            # Convert numpy arrays to float values for ROS2 Point message
            x_val = float(q[0]) if hasattr(q[0], '__iter__') else q[0]
            y_val = float(q[1]) if hasattr(q[1], '__iter__') else q[1]
            yaw_val = float(q[2]) if hasattr(q[2], '__iter__') else q[2]
            pose.pose.position = Point(x=x_val, y=y_val, z=0.0)
            pose.pose.orientation = self.yaw_to_quaternion(yaw_val)
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)
        self.get_logger().info(f"Published Dubins path with {len(path_msg.poses)} points.")
        self.get_logger().info(f"Published Dubins path with Pose {idx}: pos=({p.x}, {p.y}, {p.z}), ori=({o.x}, {o.y}, {o.z}, {o.w})")
        self.path_published_event.set()
        if hasattr(self, 'loop'):
            asyncio.run_coroutine_threadsafe(self.send_follow_path_goal(path_msg), self.loop)
        else:
            self.get_logger().error('No asyncio event loop found in node.')
       
        self.get_logger().info(f"Path frame_id: {path_msg.header.frame_id}")
        '''if path_msg.poses:
            first = path_msg.poses[0].pose
            last = path_msg.poses[-1].pose
            self.get_logger().info(f"First pose: ({first.position.x}, {first.position.y}, {first.position.z}), orientation: ({first.orientation.x}, {first.orientation.y}, {first.orientation.z}, {first.orientation.w})")
            self.get_logger().info(f"Last pose: ({last.position.x}, {last.position.y}, {last.position.z}), orientation: ({last.orientation.x}, {last.orientation.y}, {last.orientation.z}, {last.orientation.w})")
            # Detailed logging for all poses and NaN/invalid checks
            for idx, pose_stamped in enumerate(path_msg.poses):
                p = pose_stamped.pose.position
                o = pose_stamped.pose.orientation
                pos_nan = any(math.isnan(v) for v in [p.x, p.y, p.z])
                ori_nan = any(math.isnan(v) for v in [o.x, o.y, o.z, o.w])
                ori_zero = (o.x == 0.0 and o.y == 0.0 and o.z == 0.0 and o.w == 0.0)
                self.get_logger().debug(f"Pose {idx}: pos=({p.x}, {p.y}, {p.z}), ori=({o.x}, {o.y}, {o.z}, {o.w})")
                self.get_logger().info(f"Published Dubins path with Pose {idx}: pos=({p.x}, {p.y}, {p.z}), ori=({o.x}, {o.y}, {o.z}, {o.w})")
                if pos_nan or ori_nan:
                    self.get_logger().warn(f"Pose {idx} contains NaN values: pos_nan={pos_nan}, ori_nan={ori_nan}")
                if ori_zero:
                    self.get_logger().warn(f"Pose {idx} orientation is all zeros (invalid quaternion)")
        else:
            self.get_logger().warn('Path is empty!')'''
        

    def pose_to_tuple(self, pose: Pose):
        x = pose.position.x
        y = pose.position.y
        yaw = self.quaternion_to_yaw(pose.orientation)
        return (x, y, yaw)

    def quaternion_to_yaw(self, q: Quaternion):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def yaw_to_quaternion(self, yaw):
        q = tf_transformations.quaternion_from_euler(0, 0, yaw)
        return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

    async def send_follow_path_goal(self, path_msg, max_retries=3):
        # Wait for the action server to be available
        for attempt in range(max_retries):
            if self.follow_path_client.wait_for_server(timeout_sec=5.0):
                break
            self.get_logger().warn(f'FollowPath action server not available, retrying ({attempt+1}/{max_retries})...')
            await asyncio.sleep(1.0)
        else:
            self.get_logger().error('FollowPath action server not available after retries!')
            self.navigation_complete.set()
            return

        for attempt in range(max_retries):
            goal_msg = FollowPath.Goal()
            goal_msg.path = path_msg
            goal_msg.controller_id = "FollowPath"
            goal_msg.goal_checker_id = "simple_goal_checker"
            goal_msg.progress_checker_id = "simple_progress_checker"
            self.get_logger().info(f'Sending path to FollowPath action server (attempt {attempt+1})...')
            send_goal_future = self.follow_path_client.send_goal_async(goal_msg)
            goal_handle = await send_goal_future

            if goal_handle.accepted:
                self.get_logger().info('FollowPath goal accepted, waiting for result...')
                result_future = goal_handle.get_result_async()
                result = await result_future
                self.get_logger().info(f'FollowPath result: {result.status}')
                self.get_logger().info(f"FollowPath result code: {result.result.error_code}")
                self.get_logger().info(f"Error string: {result.result.error_string}")
                self.navigation_complete.set()
                return
            else:
                self.get_logger().error('FollowPath goal rejected! Retrying...')
                await asyncio.sleep(1.0)

        self.get_logger().error('FollowPath goal rejected after all retries.')
        self.navigation_complete.set()

    def send_simple_straight_path(self):
        import copy
        path_msg = Path()
        path_msg.header.frame_id = self.start_pose.header.frame_id
        # Start pose
        start_pose = copy.deepcopy(self.start_pose)
        start_pose.header.frame_id = path_msg.header.frame_id
        # End pose: 1 meter ahead in the direction of current yaw
        yaw = self.quaternion_to_yaw(self.start_pose.pose.orientation)
        end_pose = copy.deepcopy(self.start_pose)
        end_pose.pose.position.x += math.cos(yaw)
        end_pose.pose.position.y += math.sin(yaw)
        end_pose.header.frame_id = path_msg.header.frame_id
        path_msg.poses = [start_pose, end_pose]
        self.get_logger().info(f"Simple path: start=({start_pose.pose.position.x}, {start_pose.pose.position.y}), end=({end_pose.pose.position.x}, {end_pose.pose.position.y})")
        # Schedule the coroutine on the main event loop from this thread
        if hasattr(self, 'loop'):
            asyncio.run_coroutine_threadsafe(self.send_follow_path_goal(path_msg), self.loop)
        else:
            self.get_logger().error('No asyncio event loop found in node.')

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
        await node.goal_set_event.wait()
        await node.path_published_event.wait()
        await node.navigation_complete.wait()  # Wait until navigation is complete
    except asyncio.CancelledError:
        cancel.trigger()
    fut.cancel()  # Signal the spinning thread to stop
    thread.join()
    node.destroy_guard_condition(cancel)


async def main():
    rclpy.init()
    node = DubinsPathPlanner()
    node.loop = asyncio.get_running_loop()  # Store the main event loop in the node
    # Start spinning in asyncio
    spin_task = asyncio.create_task(ros_spin(node))

    await node.goal_set_event.wait()
    await node.path_published_event.wait()
    await node.navigation_complete.wait()

    spin_task.cancel()
    await spin_task

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    asyncio.run(main())
