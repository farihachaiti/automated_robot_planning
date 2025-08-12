#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
import tf2_ros
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.clock import ClockType
import time
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf_transformations

class QoSBridge(Node):
    def __init__(self, namespace=''):
        super().__init__('qos_bridge')
        
        # Get clock for timing validation
        self.clock = self.get_clock()
        
        # Declare parameters with defaults
        self.declare_parameter('tf_tolerance', 0.1)  # 100ms tolerance
        self.declare_parameter('enable_timestamp_validation', True)
        self.declare_parameter('enable_timestamp_correction', True)
        # Declare initial pose parameters
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_yaw', 0.0)
        
        # Get parameters
        self.tf_tolerance = self.get_parameter('tf_tolerance').value
        self.enable_timestamp_validation = self.get_parameter('enable_timestamp_validation').value
        self.enable_timestamp_correction = self.get_parameter('enable_timestamp_correction').value
        # Get initial pose parameters
        self.initial_x = self.get_parameter('initial_x').value
        self.initial_y = self.get_parameter('initial_y').value
        self.initial_yaw = self.get_parameter('initial_yaw').value
        
        # QoS for subscriber (match source)
        sub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # QoS for publisher (match target)
        pub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # QoS for TF topics with TRANSIENT_LOCAL durability
        tf_pub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        # QoS for TF subscriptions with VOLATILE durability
        tf_sub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        # Scan topic bridge
        #self.scan_publisher = self.create_publisher(LaserScan, 'scan', pub_qos)
        '''self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            sub_qos
        )'''

        # Odometry topic bridge
        self.odom_publisher = self.create_publisher(Odometry, 'odom', pub_qos)
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            sub_qos
        )

        # TF topic bridge
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_publisher = self.create_publisher(TFMessage, '/tf', tf_pub_qos)
        self.tf_subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            tf_sub_qos
        )

        # TF static topic bridge
        self.tf_static_publisher = self.create_publisher(TFMessage, '/tf_static', tf_pub_qos)
        self.tf_static_subscription = self.create_subscription(
            TFMessage,
            '/tf_static',
            self.tf_static_callback,
            tf_sub_qos
        )

        # Publisher for initialpose - using BEST_EFFORT to match amcl's subscription
        qos_profile_initial_pose = QoSProfile(
            depth=20,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Changed to match amcl's subscription
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST
        )
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', qos_profile_initial_pose)
        # Add this after creating self.initial_pose_pub
        self.create_timer(1.0, self.publish_initial_pose)  # Publish every seconself.publish_initial_pose()

        self.get_logger().info(f'QoS Bridge initialized with TF timing validation (tolerance: {self.tf_tolerance}s)')
        self.get_logger().info(f'Timestamp validation: {self.enable_timestamp_validation}, Timestamp correction: {self.enable_timestamp_correction}')

    def validate_tf_timestamp(self, msg):
        """Validate TF message timestamp to prevent old data errors."""
        if not self.enable_timestamp_validation:
            return True
            
        try:
            current_time = self.get_clock().now()
            
            for transform in msg.transforms:
                transform_time = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
                current_time_sec = current_time.nanoseconds * 1e-9
                
                # Check if transform is too old
                age = current_time_sec - transform_time
                if age > 5.0:
                    #self.get_logger().warn(
                    #    f'Dropping very old TF transform: {transform.header.frame_id} -> {transform.child_frame_id} '
                    #    f'(age: {age:.3f}s > max 5.0s)'
                    #)
                    return False
                    
                # Check if transform is from the future (more than tolerance)
                if transform_time - current_time_sec > self.tf_tolerance:
                    #self.get_logger().warn(
                    #    f'Dropping future TF transform: {transform.header.frame_id} -> {transform.child_frame_id} '
                    #    f'(future: {transform_time - current_time_sec:.3f}s)'
                    #)
                    return False
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error validating TF timestamp: {str(e)}')
            return False

    def update_tf_timestamps(self, msg):
        """Update TF message timestamps to current time if needed."""
        if not self.enable_timestamp_correction:
            return msg
            
        try:
            current_time = self.get_clock().now()
            updated_msg = TFMessage()
            
            for transform in msg.transforms:
                updated_transform = TransformStamped()
                updated_transform.header = transform.header
                updated_transform.header.stamp = current_time.to_msg()
                updated_transform.child_frame_id = transform.child_frame_id
                updated_transform.transform = transform.transform
                updated_msg.transforms.append(updated_transform)
            
            return updated_msg
            
        except Exception as e:
            self.get_logger().error(f'Error updating TF timestamps: {str(e)}')
            return msg

    def scan_callback(self, msg):
        """Handle laser scan messages"""
        try:
            # Update timestamp to current time
            msg.header.stamp = self.clock.now().to_msg()
            self.scan_publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error in scan callback: {str(e)}')

    def odom_callback(self, msg):
        """Handle odometry messages"""
        try:
            # Update timestamp to current time
            msg.header.stamp = self.clock.now().to_msg()
            self.odom_publisher.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error in odom callback: {str(e)}')


    def tf_callback(self, msg):
        """Handle TF messages with timing validation."""
        try:
            if not self.validate_tf_timestamp(msg):
                updated_msg = self.update_tf_timestamps(msg)
                self.tf_publisher.publish(updated_msg)
            else:
                self.tf_publisher.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'Error in TF callback: {str(e)}')

    def tf_static_callback(self, msg):
        """Handle static TF messages with timing validation"""
        try:
            # For static transforms, we can be more lenient with timing
            # but still validate to prevent very old data
            if not self.validate_tf_timestamp(msg):
                # Update timestamps for static transforms too
                updated_msg = self.update_tf_timestamps(msg)
                self.tf_static_publisher.publish(updated_msg)
                self.get_logger().debug('Published static TF with updated timestamps')
            else:
                self.tf_static_publisher.publish(msg)
                
        except Exception as e:
            self.get_logger().error(f'Error in static TF callback: {str(e)}')


    def publish_initial_pose(self):
        #if not self.tf_buffer.can_transform('map', 'odom', rclpy.time.Time()):
        #    self.get_logger().warn("TF not ready, retrying initial pose...", throttle_duration_sec=1.0)
        #    return

        init_pose_msg = PoseWithCovarianceStamped()
        init_pose_msg.header.frame_id = "map"
        init_pose_msg.header.stamp = self.get_clock().now().to_msg() - rclpy.time.Duration(seconds=0.1)

        # Set your desired initial position
        init_pose_msg.pose.pose.position.x = self.initial_x
        init_pose_msg.pose.pose.position.y = self.initial_y
        init_pose_msg.pose.pose.position.z = 0.0
        quat = tf_transformations.quaternion_from_euler(0, 0, self.initial_yaw)
        init_pose_msg.pose.pose.orientation.x = quat[0]
        init_pose_msg.pose.pose.orientation.y = quat[1]
        init_pose_msg.pose.pose.orientation.z = quat[2]
        init_pose_msg.pose.pose.orientation.w = quat[3]


        self.initial_pose_pub.publish(init_pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = QoSBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('QoS Bridge shutting down...')
    except Exception as e:
        node.get_logger().error(f'QoS Bridge error: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
