#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import time

class ScanMonitor(Node):
    def __init__(self):
        super().__init__('scan_monitor')
        self.scan_count = 0
        self.last_scan_time = None
        
        # Subscribe to all shelfino scan topics
        self.scan_subscribers = {}
        for i in range(3):  # shelfino0, shelfino1, shelfino2
            topic_name = f'/shelfino{i}/scan'
            self.scan_subscribers[topic_name] = self.create_subscription(
                LaserScan,
                topic_name,
                lambda msg, topic=topic_name: self.scan_callback(msg, topic),
                10
            )
            self.get_logger().info(f'Subscribed to {topic_name}')
    
    def scan_callback(self, msg, topic):
        current_time = time.time()
        self.scan_count += 1
        
        if self.last_scan_time is None:
            self.last_scan_time = current_time
            self.get_logger().info(f'First scan received on {topic} at {current_time}')
        else:
            time_diff = current_time - self.last_scan_time
            self.get_logger().info(f'Scan {self.scan_count} on {topic}: {len(msg.ranges)} ranges, time since last: {time_diff:.2f}s')
        
        self.last_scan_time = current_time

def main():
    rclpy.init()
    monitor = ScanMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 