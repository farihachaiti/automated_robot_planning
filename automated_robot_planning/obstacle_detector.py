#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import pyclipper
import numpy as np
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import xacro
from urdf_parser_py.urdf import URDF
import math


class ObstacleDetector(Node):
    def __init__(self, costmap, namespace=''):
        super().__init__('obstacle_detector', namespace=namespace)
        # Initialize the previous costmap to None or an empty array
        self.namespace = namespace
        self.previous_global_costmap = None
        if costmap:
            self.costmap = costmap
        self.update_threshold = 0.05  # 5% change threshold
        self.raytrace_min_range = 0.0
        self.raytrace_max_range = 3.0
        self.costmap_resolution = 0.05
        self.costmap_size = [3, 3]
        self.robot_id = None
        qos_profile = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.declare_parameter('robot_name', '')
        self.declare_parameter('robot_description', '')
        if self.has_parameter('robot_description'):
            self.robot_description = self.get_parameter('robot_description').get_parameter_value().string_value
        else:
            self.get_logger().error("Parameter 'robot_description' is missing!")
        if self.has_parameter('robot_name'):
            self.robot_id = self.get_parameter('robot_name').get_parameter_value().string_value
        else:
            self.get_logger().error("Parameter 'robot_name' is missing!")


            # Parse the URDF
        try:
            doc = xacro.process_file(self.robot_description)
            urdf_string = doc.toxml()
            self.robot_desc = URDF.from_xml_string(urdf_string)

        except Exception as e:
            self.get_logger().error(f"Error parsing URDF: {e}")
            self.robot_desc = None





    def intersects_polygonal_obstacles(self, robot_polygon, obs_polygon):
        tree1 = HierarchicalTree(robot_polygon)
        tree2 = HierarchicalTree(obs_polygon)
        for bbox1 in tree1.tree:
            for bbox2 in tree2.tree:
                if (bbox1.x < bbox2.x + bbox2.width and
                    bbox1.x + bbox1.width > bbox2.x and
                    bbox1.y < bbox2.y + bbox2.height and
                    bbox1.y + bbox1.height > bbox2.y):
                    print("Collision detected!!! POLY!!!")
                    return True
        return False
    


    def intersects_circular_obstacles(self, robot, obs):
        # Get the robot's bounding box corners

        robot_left = self.centroid_x
        robot_right = self.centroid_x + self.width
        robot_bottom = self.centroid_y
        robot_top = self.centroid_y + self.height
        
        # Find the closest point on the robot's bounding box to the obstacle center
        closest_x = max(robot_left, min(2.00, robot_right))
        closest_y = max(robot_bottom, min(2.09, robot_top))
        
        # Calculate the distance from this closest point to the obstacle center
        distance_sq = (closest_x - 2.00)**2 + (closest_y - 2.09)**2
        
        # Check if this distance is less than or equal to the obstacle's radius squared
        if distance_sq <= 0.5**2:
            print("Collision detected!!! CIRC!!!")
            return True
        else:
            return False
        


    def intersects_point_obstacles(self, robot, obs):
        # Get the robot's bounding box corners
        robot_left = self.centroid_x
        robot_right = self.centroid_x + self.width
        robot_bottom = self.centroid_y
        robot_top = self.centroid_y + self.height

        # Find the closest point on the robot's bounding box to the obstacle point
        closest_x = max(robot_left, min(2.00, robot_right))
        closest_y = max(robot_bottom, min(2.09, robot_top))
        
        # Check if the closest point is exactly the obstacle point
        if (closest_x == 2.00) and (closest_y == 2.09):
            print("Collision detected!!! POINT!!!")
            return True
        else:
            return False


    def intersects_walls(self, robot, wall):
        # Get the position of the robot's centroid
        centroid = (self.centroid_x, self.centroid_y)  # Assuming robot.position is a tuple (x, y)

        # Get the start and end points of the wall
        start = wall[0]
        end = wall[1]

        # Calculate the squared length of the wall segment
        line_length_squared = self.get_cost(start, end)

        if line_length_squared == 0:
            # If the line segment has zero length, return the distance from the centroid to the start point
            return self.get_cost(centroid, start)

        # Calculate the projection of the centroid onto the line segment
        t = max(0, min(1, ((centroid[0] - start[0]) * (end[0] - start[0]) + (centroid[1] - start[1]) * (end[1] - start[1])) / line_length_squared))
        projection = (start[0] + t * (end[0] - start[0]), start[1] + t * (end[1] - start[1]))

        # Check if the distance from the centroid to the projection point is less than or equal to half of the robot's size
        if self.get_cost(centroid, projection) <= self.robot_size / 2:
            print("Collision detected!!! WALL!!!")
            return True
        else:
            return False
    
    def intersects_other_robots(self, robot, robots):
        # Perform bounding volume intersection check between two nodes
        # For simplicity, assume nodes have bounding boxes defined by (x, y, width, height)
        collision = False
        for rob in robots:
            tree1 = HierarchicalTree(robot)
            tree2 = HierarchicalTree(rob)
            for bbox1 in tree1.tree:
                for bbox2 in tree2.tree:                
                    # Check if bounding boxes intersect
                    if (bbox1.x < bbox2.x + bbox2.width and
                        bbox1.x + bbox1.width > bbox2.x and
                        bbox1.y < bbox2.y + bbox2.height and
                        bbox1.y + bbox1.height > bbox2.y):
                        print("Collision detected!!! ROBS!!!")
                        collision = True
                        continue
                    else:
                        return False
                if collision:
                    continue
                
            if collision:
                continue

        return True
    
    def create_circle_polygon(self, radius, num_sides=36):
        polygon = []
        for i in range(num_sides):
            angle = (2 * math.pi / num_sides) * i
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            polygon.append([x, y])
        return polygon


    def create_box_polygon(self, x, y, dx, dy):
        return [
            [x - dx, y - dy],
            [x + dx, y - dy],
            [x + dx, y + dy],
            [x - dx, y + dy]
        ]

    def polygon_offsetting(self, elem, buffer, isRobot=False):
        elements = []
        if isRobot:
            shelfino_footprint = [
            [-elem.body_length / 2, -elem.body_width / 2],
            [-elem.body_length / 2, elem.body_width / 2],
            [elem.body_length / 2, elem.body_width / 2],
            [elem.body_length / 2, -elem.body_width / 2]
            ]
            elements = [shelfino_footprint]
        else:
            elements = elem
        
            for e in elem:
                if e.radius and e.radius!=0.0:
                    elements.append(self.create_circle_polygon(e.radius))
                else:
                    elements.append(self.create_box_polygon(e.x, e.y, e.dx, e.dy))

        refined_elements = []   
        for elm in elements:     
            pco = pyclipper.PyclipperOffset()
            pco.AddPath(elm, pyclipper.JT_ROUND, pyclipper.ET_CLOSEDPOLYGON)
            solution = pco.Execute(buffer)
            refined_elements.extend(solution)
        return refined_elements[0]
    


class HierarchicalTree():
    def __init__(self, polygon, step_size=2):
        self.polygon = polygon
        self.bbox_overall = self.calculate_bounding_box()
        self.tree = self.construct_tree(self.bbox_overall, step_size)



    def calculate_bounding_box(self):
        min_x = min(point[0] for point in self.polygon)
        max_x = max(point[0] for point in self.polygon)
        min_y = min(point[1] for point in self.polygon)
        max_y = max(point[1] for point in self.polygon)

        return BBox(min_x, min_y, max_x - min_x, max_y - min_y)



    def construct_tree(self, bounding_box, step_size):
        tree = []
        # Subdivide the bounding box into smaller rectangles
        for x in range(bounding_box.x, (bounding_box.x + bounding_box.width)):
            for y in range(bounding_box.y, (bounding_box.y + bounding_box.height)):
                # Create a rectangle for the current grid cell
                bbox = BBox(x, y, x+step_size, y+step_size)
                # Check if the rectangle intersects with the polygon
                #if self.intersects(polygon, bbox):
                if self.intersects_polygon(bbox):
                    tree.append(bbox)
                else:
                    continue
        return tree
    


    def intersects_polygon(self, bbox):
        # Convert the bounding box to a polygon (list of points)
        bbox_polygon = [
            (bbox.x, bbox.y),
            (bbox.x + bbox.width, bbox.y),
            (bbox.x + bbox.width, bbox.y + bbox.height),
            (bbox.x, bbox.y + bbox.height)
        ]
        # Check if the bounding box intersects with the original polygon
        clipper = pyclipper.Pyclipper()
        clipper.AddPath(self.polygon, pyclipper.PT_SUBJECT, True)
        clipper.AddPath(bbox_polygon, pyclipper.PT_CLIP, True)
        solution = clipper.Execute(pyclipper.CT_INTERSECTION)
        return len(solution) > 0






class BBox():
    def __init__(self, x, y, width, height):
        self.x = x
        self.y = y
        self.width = width
        self.height = height

