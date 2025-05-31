#!/usr/bin/env python3

import rclpy
import random
import math
import time
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Point
from visualization_msgs.msg import MarkerArray, Marker
from rclpy.qos import qos_profile_sensor_data

class PositionManager(Node):
    def __init__(self):
        super().__init__('position_manager')
        
        self.obstacles = []
        self.robot_positions = []
        
        # Parameters
        self.declare_parameter('map_bounds', [-7.0, 7.0, -7.0, 7.0])  # min_x, max_x, min_y, max_y
        self.declare_parameter('min_robot_distance', 1.5)
        self.declare_parameter('obstacle_buffer', 1.0)
        self.declare_parameter('max_attempts', 100)
        
        # Subscribers
        self.obstacle_sub = self.create_subscription(
            MarkerArray,
            '/obstacles',
            self.obstacle_callback,
            qos_profile=qos_profile_sensor_data
        )
        
        # Service to get valid position
        from automated_robot_planning.srv import GetValidPosition
        self.get_valid_pos_srv = self.create_service(
            GetValidPosition,
            'get_valid_position',
            self.get_valid_position_callback
        )
        
        self.get_logger().info('Position Manager ready')
    
    def obstacle_callback(self, msg):
        """Update internal obstacle list"""
        self.obstacles = []
        for marker in msg.markers:
            if marker.points:
                point = marker.points[0]
                radius = marker.scale.x / 2.0  # Assuming scale.x is diameter
                self.obstacles.append((point.x, point.y, radius))
    
    def is_valid_position(self, x, y, existing_positions):
        """Check if position is valid (not too close to obstacles or other robots)"""
        min_dist = self.get_parameter('min_robot_distance').value
        obs_buffer = self.get_parameter('obstacle_buffer').value
        
        # Check distance from other robots
        for pos in existing_positions:
            dx = x - pos.x
            dy = y - pos.y
            if math.sqrt(dx*dx + dy*dy) < min_dist:
                return False
                
        # Check distance from obstacles
        for ox, oy, radius in self.obstacles:
            dx = x - ox
            dy = y - oy
            distance = math.sqrt(dx*dx + dy*dy)
            if distance < (radius + obs_buffer):
                return False
                
        return True
    
    def get_valid_position_callback(self, request, response):
        """Service callback to get a valid position"""
        map_bounds = self.get_parameter('map_bounds').value
        max_attempts = self.get_parameter('max_attempts').value
        
        min_x, max_x, min_y, max_y = map_bounds
        
        for _ in range(max_attempts):
            x = random.uniform(min_x, max_x)
            y = random.uniform(min_y, max_y)
            
            if self.is_valid_position(x, y, self.robot_positions):
                position = Point(x=x, y=y, z=0.0)
                self.robot_positions.append(position)
                response.position = position
                response.success = True
                return response
        
        # If no valid position found
        response.success = False
        response.message = "Could not find valid position after {} attempts".format(max_attempts)
        return response

def main(args=None):
    rclpy.init(args=args)
    node = PositionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
