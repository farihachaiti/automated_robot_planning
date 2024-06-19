#!/usr/bin/env python3


from tokenize import String
import rclpy
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry
import tf2_ros
from tf2_msgs.msg import TFMessage
import tf2_geometry_msgs  # This import is necessary for using transform_datatypes
import math
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState
from urdf_parser_py.urdf import URDF
import xacro
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy





class ShelfinoRobot(Node):
    def __init__(self, namespace=''):
        super().__init__('shelfino_robot', namespace=namespace) 
        print(f"ShelfinoRobot namespace: {namespace}")
        self.namespace = namespace
        self.client = None
        self.task = None
        self.in_pose = np.zeros(3)
        qos_profile = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.declare_parameter('robot_description', '') 
        self.declare_parameter('robot_name', '')
        self.robot_id = None
        self.body_width = 0.5
        self.body_length = 0.5

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
        print('hia')
        self.create_subscription(
            JointState,
            f'/{namespace}/joint_states_subscriber',
            self.joint_state_callback,
            qos_profile
        )        
        print('hib')
        self.create_subscription(
            TFMessage,
            f'/{namespace}/tf_subscriber',
            self.tf_msgs_callback,
            QoSProfile(
                depth=20,
                durability=DurabilityPolicy.VOLATILE,
                reliability=ReliabilityPolicy.RELIABLE,
            )
        )      
        print('hic')
        self.create_subscription(
            PoseWithCovarianceStamped,
            f'/{namespace}/initialpose',
            self.init_pose_callback,
            QoSProfile(
                depth=20,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE
            )
        ) 
        print('hid')
        self.create_subscription(
            PoseWithCovarianceStamped,
            f'/{namespace}/amcl_pose',
            self.amcl_pose_callback,
            qos_profile
        ) 
        print('hie')
        self.create_subscription(
            Odometry,
            f'/{namespace}/odom',
            self.odom_callback,
            QoSProfile(
                depth=20,
                durability=DurabilityPolicy.VOLATILE,
                reliability=ReliabilityPolicy.RELIABLE,
            )
        ) 



    def joint_state_callback(self, updated_msg): 
        try:
            print('hi', updated_msg)
            print("Received Joint State:")
            print("Header: ", updated_msg.header)
            print("Joint Names: ", updated_msg.name[0])
            print("Joint Positions: ", updated_msg.position[0])            
        except Exception as e:
            self.get_logger().error(f"Joint States Callback failed: {e}")
    



    def tf_msgs_callback(self, updated_msg):
        try:
            print('hi', updated_msg)
            print("Received TF Message:")
            print("Header: ", updated_msg.transforms[0].header)
            print("Child Frame ID: ", updated_msg.transforms[0].child_frame_id)
        except Exception as e:
            self.get_logger().error(f"TFmessages Callback failed: {e}")

        
    def init_pose_callback(self, msg):
        try:
            print('hi', msg)
            print("Received Initial Pose:")
        
            self.in_pose[0] = msg.pose.position.x
            self.in_pose[1] = msg.pose.position.y
            matrix = tf2_ros.transformations.quaternion_matrix([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
            rpy = tf2_ros.transformations.euler_from_matrix(matrix)
            self.in_pose[2] = rpy[2]
        except Exception as e:
            self.get_logger().error(f"Initial Pose Callback failed: {e}")
        


    def amcl_pose_callback(self, updated_msg):
        try:
            print('hi', updated_msg)
            print("Received AMCL Pose:")
            print("Header: ", updated_msg.header)
        except Exception as e:
            self.get_logger().error(f"AMCL Pose Callback failed: {e}")


    def odom_callback(self, updated_msg):
        try:
            print('hi', updated_msg)
            print("Received Odometry:")
            print("Header: ", updated_msg.header)
        except Exception as e:
            self.get_logger().error(f"Odom Callback failed: {e}")


   
