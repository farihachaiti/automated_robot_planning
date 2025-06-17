#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class QoSBridge(Node):
    def __init__(self, namespace=''):
        super().__init__('qos_bridge')
        
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

        # Scan topic bridge
        self.scan_publisher = self.create_publisher(LaserScan, 'scan', pub_qos)
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            sub_qos
        )

        # Odometry topic bridge
        self.odom_publisher = self.create_publisher(Odometry, 'odom', pub_qos)
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            sub_qos
        )
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def scan_callback(self, msg):
        self.scan_publisher.publish(msg)        


    def odom_callback(self, msg):
        self.odom_publisher.publish(msg)
        t = TransformStamped()
        t.header = msg.header
        t.child_frame_id = msg.child_frame_id
        t.header.frame_id = msg.header.frame_id
        # Copy pose
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = QoSBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
