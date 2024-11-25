#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped, Vector3, PoseStamped
from tf2_ros import TransformBroadcaster
import math
from sensor_msgs.msg import NavSatFix ,LaserScan,PointCloud2,Imu
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

def quaternion_to_euler(x, y, z, w):
    import math
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return roll_x, pitch_y, yaw_z # in radians

class imu_translate(Node):
    def __init__(self):
        super().__init__('imu_translate')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        ) 
        self.pub_odom_tf = self.create_publisher(
            Imu,"/imu/data",10)

        self.imu_sub = self.create_subscription(
            Imu,
            '/mavros/imu/data',
            self.set_rpy_vel,
            qos_profile)        
        
        self.imu_raw = self.create_subscription(
            Imu,
            '/mavros/imu/data_raw',
            self.repub_odom_vel,
            qos_profile)
        self.imu_data = Imu()
        self.multiply = 1.0
    def set_rpy_vel(self,message:Imu):
        self.imu_data  = message
    def repub_odom_vel(self,message:Imu):
        msg_tf = Imu()
        msg_tf = self.imu_data
        # msg_tf.header.stamp = self.get_clock().now().to_msg()
        msg_tf.header.frame_id = "imu_link"
        #transform q to rpz
        q1 = self.imu_data.orientation.x
        q2 = self.imu_data.orientation.y
        q3 = self.imu_data.orientation.z
        q4 = self.imu_data.orientation.w

        r,p,y = quaternion_to_euler(q1,q2,q3,q4)

        msg_tf.orientation.x = q1
        msg_tf.orientation.y = q2
        msg_tf.orientation.z = q3
        msg_tf.orientation.w = q4
        print("rpy",np.degrees(y))
        #print("imu",msg_tf)
        self.pub_odom_tf.publish(msg_tf)


def main(args=None):
    rclpy.init(args=args)
    optitrack_tf_state= imu_translate()
    rclpy.spin(optitrack_tf_state)
    #optitrack_tf_state.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()