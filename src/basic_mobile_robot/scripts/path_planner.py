#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped, Vector3, PoseStamped
from tf2_ros import TransformBroadcaster
import math
from sensor_msgs.msg import NavSatFix ,LaserScan,PointCloud2
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class path_planner(Node):
    def __init__(self):
        super().__init__('path_planner')
        self.declare_parameter('machine_id',"Robot1")
        self.machine_id = self.get_parameter('machine_id').value   
        self.declare_parameter('initial_pose',[0,0,0,0])
        self.initial_pose = self.get_parameter('initial_pose').value  

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        ) 

        
        self.pub_odom_tf = self.create_publisher(
            Odometry,"/localization/kinematic_state", qos_profile=qos_profile)

        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.repub_odom_vel,
            10)

        self.multiply = 1.0


    def repub_odom_vel(self,message:Odometry):
        msg_tf = Odometry()
        msg_tf = message
        # msg_tf.header.stamp = self.get_clock().now().to_msg()
        msg_tf.header.frame_id = "map"
        print("cmd_vel_tf",msg_tf)
        self.pub_odom_tf.publish(msg_tf)


def main(args=None):
    rclpy.init(args=args)
    optitrack_tf_state= path_planner()
    rclpy.spin(optitrack_tf_state)
    #optitrack_tf_state.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()