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

class Quaternion:
    w =1.0
    x =0.0
    y =0.0
    z =0.0

def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    # Output :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    qw = cy * cp * cr + sy * sp * sr
    qx = cy * cp * sr - sy * sp * cr
    qy = sy * cp * sr + cy * sp * cr
    qz = sy * cp * cr - cy * sp * sr
    return [qx, qy, qz, qw]

class joint_state_publisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.declare_parameter('machine_id',"Robot1")
        self.machine_id = self.get_parameter('machine_id').value   
        self.declare_parameter('initial_pose',[0,0,0,0])
        self.initial_pose = self.get_parameter('initial_pose').value   


        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.pub_odom_tf = self.create_publisher(
            Odometry,"odom", 10)

        self.status_sub = self.create_subscription(
            Odometry,
            '/mavros/local_position/odom',
            self.UpdateBaseLinkLocation,
            qos_profile)

        self.sbr_ = StaticTransformBroadcaster(self)
        self.br_ = TransformBroadcaster(self)
        self.first_gps_msg_ = True
        self.start_pos_ = TransformStamped()
        self.start_imu_ = Quaternion()
        self.start_pos_.header.stamp = self.get_clock().now().to_msg()
        # self.start_pos_.transform.translation.x = float(self.initial_pose[0])
        # self.start_pos_.transform.translation.y = float(self.initial_pose[1])
        # self.start_pos_.transform.translation.z = float(self.initial_pose[2])
        q = quaternion_from_euler(0,0,0)
        self.start_pos_.transform.rotation.x = q[0]
        self.start_pos_.transform.rotation.y = q[1]
        self.start_pos_.transform.rotation.z = q[2]
        self.start_pos_.transform.rotation.w = q[3]
        
        

        
        self.InitPublishBaseLink()


        ##joint_state_publisher 



        
    def InitPublishBaseLink(self):

        odom_msg_ = TransformStamped()
        odom_msg_ = self.start_pos_
        odom_msg_.header.stamp = self.get_clock().now().to_msg()
        odom_msg_.header.frame_id = 'map'
        odom_msg_.child_frame_id = "odom"
       

        self.sbr_.sendTransform(odom_msg_)

    def UpdateBaseLinkLocation(self,message:Odometry):
        base_link_message = TransformStamped()
        base_link_message.header.stamp = self.get_clock().now().to_msg()
        base_link_message.header.frame_id = "odom"
        base_link_message.child_frame_id = "base_link"
        base_link_message.transform.translation.x = message.pose.pose.position.x
        base_link_message.transform.translation.y = message.pose.pose.position.y
        base_link_message.transform.translation.z = message.pose.pose.position.z
        base_link_message.transform.rotation.x = message.pose.pose.orientation.x
        base_link_message.transform.rotation.y = message.pose.pose.orientation.y
        base_link_message.transform.rotation.z = message.pose.pose.orientation.z
        base_link_message.transform.rotation.w = message.pose.pose.orientation.w

        odom_msg_ = Odometry()
        odom_msg_ = message
        self.pub_odom_tf.publish(odom_msg_)
        try:
            self.br_.sendTransform([base_link_message])
            #self.br_.sendTransform(base_link_message)
        except BaseException:
            return

 


def main(args=None):
    rclpy.init(args=args)
    optitrack_tf_state= joint_state_publisher()
    rclpy.spin(optitrack_tf_state)
    #optitrack_tf_state.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()