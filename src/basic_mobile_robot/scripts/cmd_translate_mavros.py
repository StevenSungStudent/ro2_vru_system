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


class cmd_translate_mavros(Node):
    def __init__(self):
        super().__init__('cmd_translate_mavros')
        self.declare_parameter('machine_id',"Robot1")
        self.machine_id = self.get_parameter('machine_id').value   
        self.declare_parameter('initial_pose',[0,0,0,0])
        self.initial_pose = self.get_parameter('initial_pose').value  


        self.pub_cmd_tf = self.create_publisher(
            Odometry,"/localization/kinematic_state", 10)

        self.cmd_sub = self.create_subscription(
            Odometry,
            'odom',
            self.repub_cmd_vel,
            10)

        self.multiply = 1.0


    def repub_cmd_vel(self,message:Odometry):
        msg_tf = Odometry()
        msg_tf = message
        # msg_tf.header.stamp = self.get_clock().now().to_msg()
        msg_tf.header.frame_id = "map"
        print("cmd_vel_tf",msg_tf)
        self.pub_cmd_tf.publish(msg_tf)


def main(args=None):
    rclpy.init(args=args)
    optitrack_tf_state= cmd_translate_mavros()
    rclpy.spin(optitrack_tf_state)
    #optitrack_tf_state.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()