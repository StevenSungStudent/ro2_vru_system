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


import math

import numpy as np

def calc_compass_bearing(pointA, pointB):

    lat1 = math.radians(pointA[0])
    lat2 = math.radians(pointB[0])

    diffLong = math.radians(pointB[1] - pointA[1])

    x = math.sin(diffLong) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1)
                                           * math.cos(lat2) * math.cos(diffLong))

    initial_bearing = math.atan2(x, y)

    initial_bearing = math.degrees(initial_bearing)
    compass_bearing = (initial_bearing + 360) % 360

    return compass_bearing

    

def calc_dist(lat1, lon1, lat2, lon2):

    R = 6378.137; # Radius of earth in KM
    dLat = math.radians(lat2) - math.radians(lat1)
    dLon = math.radians(lon2) - math.radians(lon1)
    a = math.sin(dLat/2) * math.sin(dLat/2) + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dLon/2) * math.sin(dLon/2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
    d = R * c
    return d * 1000 # meters

def angle_sub(angle1, angle2 ):
    result = angle1 - angle2 

    if result < 0:
        result +=360
    if result >= 360:
        result -=360    

    return result

## transform from ned to enu
def compass_bearing_to_regular(compass_bearing):
    regular_bearing =0
    if compass_bearing >=0 and compass_bearing <= 180:
        regular_bearing = 90 - compass_bearing
    else :
        regular_bearing = 90 - compass_bearing
        if regular_bearing < 180:
            regular_bearing+=360  
        if regular_bearing > 180:
            regular_bearing-=360  
    # convert from 0 to -180 to 0 to 360 
    if regular_bearing < 0:
        regular_bearing +=360     
    return regular_bearing


class navsat_transform(Node):
    def __init__(self):
        super().__init__('navsat_transform') 
        self.declare_parameter('origin',[51.98969350036926, 5.949716200007323, 0.0])
        self.origin = self.get_parameter('origin').value  


        self.pub_odom_tf = self.create_publisher(
            Odometry,"odom/gps", 10)

        self.create_subscription(
            NavSatFix,
            'gps/fix',
            self.repub_odom,
            10)

   


    def repub_odom(self,message:NavSatFix):
        gps = NavSatFix()
        gps = message
        heading = 0.0
        gps_info = [gps.latitude,gps.longitude, heading]
        bearing = calc_compass_bearing(self.origin , gps_info)
        dist = calc_dist(
                self.origin [0], self.origin [1], gps_info[0], gps_info[1])
        bearing_rad = math.radians(bearing)
        x = dist * math.sin(bearing_rad)
        y = dist * math.cos(bearing_rad)

        yaw = compass_bearing_to_regular(gps_info[2])
        yaw = angle_sub(yaw,self.origin[2])
        print(x,y, yaw)

        msg_odom = Odometry()
        msg_odom.header.stamp = gps.header.stamp
        msg_odom.header.frame_id = "odom"
        msg_odom.child_frame_id = "gps_link"
        msg_odom.pose.pose.orientation.x = 0.0
        msg_odom.pose.pose.orientation.y = 0.0
        msg_odom.pose.pose.orientation.z = 0.0
        msg_odom.pose.pose.orientation.w = 1.0

        msg_odom.pose.pose.position.x = x
        msg_odom.pose.pose.position.y = y
        msg_odom.pose.pose.position.z = 0.0

        msg_odom.pose.covariance = np.array([1.0, 0.e+00, 0.e+00, 0.e+00, 0.e+00, 0.e+00, 
                                             0.e+00, 1.0, 0.e+00, 0.e+00, 0.e+00, 0.e+00, 
                                             0.e+00, 0.e+00, 1.e+12, 0.e+00, 0.e+00, 0.e+00, 
                                             0.e+00, 0.e+00, 0.e+00, 1.e+12, 0.e+00, 0.e+00, 
                                             0.e+00, 0.e+00, 0.e+00, 0.e+00, 1.e+12, 0.e+00, 
                                             0.e+00, 0.e+00, 0.e+00, 0.e+00, 0.e+00, 1.e+12])

        self.pub_odom_tf.publish(msg_odom)
        # msg_tf.header.stamp = self.get_clock().now().to_msg()


def main(args=None):
    rclpy.init(args=args)
    optitrack_tf_state= navsat_transform()
    rclpy.spin(optitrack_tf_state)
    #optitrack_tf_state.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()