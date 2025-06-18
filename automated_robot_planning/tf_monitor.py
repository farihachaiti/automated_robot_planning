#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from rclpy.clock import ClockType
import time
from collections import defaultdict

class TFMonitor(Node):
    def __init__(self):
        super().__init__('tf_monitor')
        
        # Get clock for timing
        self.clock = self.get_clock()
        
        # Declare parameters
        self.declare_parameter('monitor_interval', 5.0)  # Check every 5 seconds
        self.declare_parameter('max_age_threshold', 1.0)  # 1 second threshold
        self.declare_parameter('enable_detailed_logging', False)
        
        # Get parameters
        self.monitor_interval = self.get_parameter('monitor_interval').value
        self.max_age_threshold = self.get_parameter('max_age_threshold').value
        self.enable_detailed_logging = self.get_parameter('enable_detailed_logging').value
        
        # Statistics tracking
        self.tf_stats = defaultdict(lambda: {
            'count': 0,
            'old_count': 0,
            'future_count': 0,
            'last_seen': None,
            'max_age': 0.0,
            'min_age': float('inf')
        })
        
        # Subscribe to TF topics
        self.tf_subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10
        )
        
        self.tf_static_subscription = self.create_subscription(
            TFMessage,
            '/tf_static',
            self.tf_static_callback,
            10
        )
        
        # Create timer for periodic reporting
        self.timer = self.create_timer(self.monitor_interval, self.report_statistics)
        
        self.get_logger().info(f'TF Monitor initialized (checking every {self.monitor_interval}s)')
        self.get_logger().info(f'Max age threshold: {self.max_age_threshold}s')

    def analyze_tf_message(self, msg, is_static=False):
        """Analyze TF message for timing issues"""
        current_time = self.clock.now()
        current_time_sec = current_time.nanoseconds * 1e-9
        
        for transform in msg.transforms:
            # Convert transform timestamp to seconds
            transform_time = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
            age = current_time_sec - transform_time
            
            # Create a key for this transform
            tf_key = f"{transform.header.frame_id}->{transform.child_frame_id}"
            if is_static:
                tf_key += " (static)"
            
            # Update statistics
            stats = self.tf_stats[tf_key]
            stats['count'] += 1
            stats['last_seen'] = current_time_sec
            stats['max_age'] = max(stats['max_age'], age)
            stats['min_age'] = min(stats['min_age'], age)
            
            # Check for timing issues
            if age > self.max_age_threshold:
                stats['old_count'] += 1
                if self.enable_detailed_logging:
                    self.get_logger().warn(
                        f'Old TF detected: {tf_key} (age: {age:.3f}s)'
                    )
            
            if age < -0.1:  # Future transform (more than 100ms in future)
                stats['future_count'] += 1
                if self.enable_detailed_logging:
                    self.get_logger().warn(
                        f'Future TF detected: {tf_key} (future: {-age:.3f}s)'
                    )

    def tf_callback(self, msg):
        """Handle dynamic TF messages"""
        self.analyze_tf_message(msg, is_static=False)

    def tf_static_callback(self, msg):
        """Handle static TF messages"""
        self.analyze_tf_message(msg, is_static=True)

    def report_statistics(self):
        """Report TF timing statistics"""
        if not self.tf_stats:
            self.get_logger().info('No TF messages received yet')
            return
        
        current_time = self.clock.now()
        current_time_sec = current_time.nanoseconds * 1e-9
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('TF TIMING STATISTICS')
        self.get_logger().info('=' * 60)
        
        total_old = 0
        total_future = 0
        total_count = 0
        
        for tf_key, stats in self.tf_stats.items():
            if stats['count'] == 0:
                continue
                
            total_count += stats['count']
            total_old += stats['old_count']
            total_future += stats['future_count']
            
            # Calculate age since last seen
            age_since_last = current_time_sec - stats['last_seen'] if stats['last_seen'] else 0
            
            # Calculate percentages
            old_percent = (stats['old_count'] / stats['count']) * 100 if stats['count'] > 0 else 0
            future_percent = (stats['future_count'] / stats['count']) * 100 if stats['count'] > 0 else 0
            
            self.get_logger().info(f'{tf_key}:')
            self.get_logger().info(f'  Total messages: {stats["count"]}')
            self.get_logger().info(f'  Old messages: {stats["old_count"]} ({old_percent:.1f}%)')
            self.get_logger().info(f'  Future messages: {stats["future_count"]} ({future_percent:.1f}%)')
            self.get_logger().info(f'  Age range: {stats["min_age"]:.3f}s to {stats["max_age"]:.3f}s')
            self.get_logger().info(f'  Last seen: {age_since_last:.1f}s ago')
            
            # Highlight problematic transforms
            if old_percent > 10 or future_percent > 5:
                self.get_logger().warn(f'  ⚠️  WARNING: High timing issues detected!')
        
        # Overall statistics
        if total_count > 0:
            overall_old_percent = (total_old / total_count) * 100
            overall_future_percent = (total_future / total_count) * 100
            
            self.get_logger().info('-' * 60)
            self.get_logger().info(f'OVERALL: {total_count} messages, {total_old} old ({overall_old_percent:.1f}%), {total_future} future ({overall_future_percent:.1f}%)')
            
            if overall_old_percent > 5 or overall_future_percent > 2:
                self.get_logger().error('🚨 CRITICAL: High overall timing issues detected!')
            elif overall_old_percent > 2 or overall_future_percent > 1:
                self.get_logger().warn('⚠️  WARNING: Moderate timing issues detected!')
            else:
                self.get_logger().info('✅ TF timing looks good!')
        
        self.get_logger().info('=' * 60)

def main(args=None):
    rclpy.init(args=args)
    node = TFMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('TF Monitor shutting down...')
    except Exception as e:
        node.get_logger().error(f'TF Monitor error: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 