#!/usr/bin/env python3

import numpy as np
import sympy as sp
import scipy


import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import TransformStamped, Quaternion, PoseStamped
from gazebo_msgs.srv import ApplyJointEffort

import tf2_ros
from urdf_parser_py.urdf import URDF
from builtin_interfaces.msg import Duration
from automated_robot_planning.shelfino_robot import ShelfinoRobot
from automated_robot_planning.path_planner import PathPlanner
from automated_robot_planning.obstacle_detector import ObstacleDetector
from nav2_msgs.action import FollowPath
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
import pyclipper
import math
import time
import dubins
import xacro
from launch.substitutions import PythonExpression
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy




class RobotController(Node):
    collision_detected = False
    def __init__(self, namespace='', enable_rosout=False):
        super().__init__('robot_controller', namespace=namespace, enable_rosout=enable_rosout)
        self.robot_id = None
        self.declare_parameter('robot_name', '')
        if self.has_parameter('robot_name'):
            self.robot_id = self.get_parameter('robot_name').get_parameter_value().string_value
        else:
            self.get_logger().error("Parameter 'robot_name' is missing!")
        if not self.robot_id:
            return
        self.namespace = self.robot_id
        qos_profile = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        buffer_size = 10.0
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer(cache_time=tf2_ros.Duration(seconds=10.0, nanoseconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        #self.joint_trajectory_publisher = self.create_publisher(JointTrajectory, '/joint_trajectory', qos_profile)
        #self.joint_effort_srv_client= self.create_client(ApplyJointEffort, '/apply_joint_effort')
        self.declare_parameter('robot_description', '')
        if self.has_parameter('robot_description'):
            self.robot_description = self.get_parameter('robot_description').get_parameter_value().string_value
        else:
            self.get_logger().error("Parameter 'robot_description' is missing!")


        # Parse the URDF
        try:
            #with open(self.robot_description, 'r') as xacro_file:
            doc = xacro.process_file(self.robot_description)
            urdf_string = doc.toxml()
            self.robot_desc = URDF.from_xml_string(urdf_string)
            '''print("Joints:")
            for joint in self.robot_desc.joints:
                print(joint.name)

            print("Links:")
            for link in self.robot_desc.links:
                print(link.name)'''
        except Exception as e:
            self.get_logger().error(f"Error parsing URDF: {e}")
            self.robot_desc = None
            # Wait for the service to be available
        '''while not self.joint_effort_srv_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Service not available, waiting...')'''


    def follow_pth(self, shelfino_robot_client, waypoints):
        goal_msg = FollowPath.Goal()
        goal_msg.poses = waypoints
        self.send_goal(shelfino_robot_client, goal_msg)


    def send_goal(self, client, goal_msg):
        self.get_logger().info('Sending goal request...')
        client.wait_for_server()
        self.future = client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info('Received feedback: {0}'.format(feedback_msg.feedback))

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.get_logger().info('Goal accepted')   
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback) 

    
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')

    def publish_tf(self, x, y, z, roll, pitch, yaw, frame, parent_frame):
        t = TransformStamped()
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = frame
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z

        q = self.quaternion_from_euler(roll, pitch, yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Publish the transforms
        try:
             self.tf_broadcaster.sendTransform(t)
             print("Successfully published transform.")
             print(f"Transform lookup time: {t.header.stamp}")
             print(f"{t.child_frame_id}")
             print(f"{t.header.frame_id}")
        except Exception as e:
             print(f"Failed to publish transform {e}.")
             print(f"Transform lookup time: {t.header.stamp}")
        

    def quaternion_from_euler(self, ai, aj, ak):
        ai /= 2.0
        aj /= 2.0
        ak /= 2.0
        ci = math.cos(ai)
        si = math.sin(ai)
        cj = math.cos(aj)
        sj = math.sin(aj)
        ck = math.cos(ak)
        sk = math.sin(ak)
        cc = ci*ck
        cs = ci*sk
        sc = si*ck
        ss = si*sk


        q = np.empty((4, ))
        q[0] = cj*sc - sj*cs
        q[1] = cj*ss + sj*cc
        q[2] = cj*cs - sj*sc
        q[3] = cj*cc + sj*ss

        return q
    

    def get_current_pos_and_or(self):
        current_pos = np.zeros(3)
        try:
            # Lookup the transform between "base_link" and "map"
            transform = self.tf_buffer.lookup_transform('map', f'/{self.robot_id}/base_link', rclpy.time.Time(), rclpy.time.Duration(seconds=1))
            position = transform.transform.translation
            orientation = transform.transform.rotation
            current_pos[0] = position.x
            current_pos[1] = position.y
            matrix = tf2_ros.transformations.quaternion_matrix([orientation.x, orientation.y, orientation.z, orientation.w])
            rpy = tf2_ros.transformations.euler_from_matrix(matrix)
            current_pos[2] = rpy[2]
            self.get_logger().info("Position: x={}, y={}, z={}".format(position.x, position.y, position.z))
            self.get_logger().info("Orientation: x={}, y={}, z={}, w={}".format(orientation.x, orientation.y, orientation.z, orientation.w))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error("Failed to lookup transform: {}".format(e))
        return current_pos




def main():
    rclpy.init()
    #robot_name = rclpy.parameter.Parameter('robot_name', rclpy.parameter.Parameter.Type.STRING, '').get_parameter_value().string_value 
    executor = MultiThreadedExecutor(num_threads=4)
    try:
        node = RobotController(namespace='', enable_rosout=False)
        '''#shelfino = ShelfinoRobot(node.robot_id)
        shelfino1 = ShelfinoRobot("shelfino_robot_0")
        shelfino2 = ShelfinoRobot("shelfino_robot_1")'''
        if node.robot_id:      
            print('hi0')
            print(node.namespace)
            print('hi0')

            #run_collision_detection_task = executor.create_task(node.planner_node.run_collision_detection, node.robots, node.obstacles, node.walls)
            #time.sleep(20)

            #shelfino.task = executor.create_task(planner_node.robot_task, shelfino1.in_pose, shelfino1.client)            
            print('hi1')
            shelfino = ShelfinoRobot(namespace=node.namespace)
            print('hi2')  
            #time.sleep(10)
            if shelfino:
                print('hi3')
                shelfino.client = ActionClient(shelfino, FollowPath, f'/{node.namespace}/follow_path')
                print('hi4')
                planner_node = PathPlanner(node, shelfino, namespace=node.namespace)    
                print('hi5')
                #time.sleep(10)
                if PathPlanner.costmap:
                    obstacle_detector_node = ObstacleDetector(PathPlanner.costmap, namespace=node.namespace)
                    print('hi6')
                    #time.sleep(10)
                shelfino.task = executor.create_task(planner_node.robot_task, shelfino.in_pose, shelfino) 
                #planner_node.robot_task(shelfino.in_pose, shelfino.client)
                print('hi7')       
                #time.sleep(10)    
                print('hi7hi')
                executor.add_node(obstacle_detector_node)
                executor.add_node(planner_node)  
                executor.add_node(shelfino)
                executor.add_node(node)
                while rclpy.ok():
                    print('hi8')
                    executor.spin()
                    print('hi9')          
                    # This starts the event loop and handles timers and callbacks
    except rclpy.exceptions.ROSInterruptException as e:
        print(f"Exception in ROS: {e}")
        node.get_logger().error(f"ROS error in main: {e}")
    except KeyboardInterrupt:
        print("Keyboard interrupt")
        node.get_logger().info("Keyboard interrupt")
    except Exception as e:
        print(f"Exception in main: {e}")
        node.get_logger().error(f"Exception in main: {e}")
    finally:
        print('bye bye bye!')
        shelfino.client._cancel_goal_async()
        shelfino.task.cancel()
        executor.shutdown()
        obstacle_detector_node.destroy_node()
        planner_node.destroy_node()
        shelfino.destroy_node()
        node.destroy_node()           
        rclpy.shutdown()



if __name__ == '__main__':
    main()
