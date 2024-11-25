#!/usr/bin/env python3
############################################################################
#
#This is a simple example of a ROS2 node for px4 control. 
#TODO: ADD verification process for task and state, connect the system into the Orchestrator
#
############################################################################


import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from mavros_msgs.msg import OverrideRCIn
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from nav_msgs.msg import Odometry,Path

# Supported schemas:

# Serial: /path/to/serial/device[:baudrate]

# Serial: serial:///path/to/serial/device[:baudrate][/?ids=sysid,compid]

# Serial with hardware flow control: serial-hwfc:///path/to/serial/device[:baudrate][?ids=sysid,compid]

# UDP: udp://[bind_host][:port]@[remote_host][:port][/?ids=sysid,compid]

# UDP Broadcast: udp-b://[bind_host][:port]@[:port][/?ids=sysid,compid]

# TCP client: tcp://[server_host][:port][/?ids=sysid,compid]

# TCP server: tcp-l://[bind_host][:port][/?ids=sysid,compid]
import cv2
import os
import copy
from multiprocessing import Process

HEIGHT = 1
ROBOT_RADIUS = 0.5

img = cv2.imread("/home/yxwork/vru_ws/src/basic_mobile_robot/maps/parking_deck.png")
img = cv2.resize(img,(800,800))

points = []

path = []

def on_EVENT_LBUTTONDOWN(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        xy = "%d,%d" % (x, y)
        
        points.append([x,y])
        path.append([x/40 - 10 ,-(y/40 - 10)])
        print([x/40 - 10 ,-(y/40 - 10)])
        n = len(points)
        cv2.circle(img, (x, y), 2, (0, 0, 255), thickness=3)
        if n>=2:
            cv2.line(img,points[n-2],points[n-1],(0,0,255),thickness = 3)
        cv2.imshow("reference_frame", img)
        
        

cv2.namedWindow("reference_frame")
cv2.setMouseCallback("reference_frame", on_EVENT_LBUTTONDOWN)

cv2.imshow("reference_frame",img)
cv2.waitKey()
cv2.destroyAllWindows()



class OffboardControl(Node):

    def __init__(self):
        super().__init__('offcontrol_node_mavros')




        self.path = path
        self.get_logger().info("path: " + str(self.path))
        self.index = 0
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.status_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.vehicle_status_callback,
            qos_profile)

        self.odom_sub = self.create_subscription(
            Odometry,
            '/mavros/local_position/odom',
            self.update_chasing_point,
            qos_profile)
        self.local_pos_pub = self.create_publisher(PoseStamped,"mavros/setpoint_position/local",10)
        self.local_traj_pub = self.create_publisher(Path,"mavros/setpoint_trajectory/desired",10)
        self.path_pub = self.create_publisher(Path,"vru_current_path",10)
        self.reference_path_pub = self.create_publisher(Path,"vru_reference_path",10)
        
        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.publish_point)

        self.robot_state = State()


       
        self.indentation = 1
        self.current_chasing_point = PoseStamped()

        self.current_chasing_point.pose.position.x = float(self.path[self.index][0])
        self.current_chasing_point.pose.position.y = float(self.path[self.index][1])
        self.current_chasing_point.pose.position.z = float(HEIGHT)
    
        self.current_chasing_point.header.stamp = self.get_clock().now().to_msg()
        self.current_chasing_point.header.frame_id = "map"


        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"
        self.path_msg.header.stamp = self.get_clock().now().to_msg()

        for point in path:
            tmp = PoseStamped()
            tmp.pose.position.x = float(point[0])
            tmp.pose.position.y = float(point[1])
            tmp.pose.position.z = float(HEIGHT)
            self.path_msg.poses.append(tmp)

        self.reference_path_msg = copy.deepcopy(self.path_msg)
        
       
        
    def update_chasing_point(self,msg:Odometry):
        # print the current position
        print("current position: ", msg.pose.pose.position.x, msg.pose.pose.position.y)

        
        # if the current point is reached, update the next point
        if (abs(msg.pose.pose.position.x - self.current_chasing_point.pose.position.x) < (ROBOT_RADIUS * 2)) and (abs(msg.pose.pose.position.y - self.current_chasing_point.pose.position.y) < (ROBOT_RADIUS * 2)):
            self.index += 1
            print("updating next point")
            self.path_msg.poses.pop(0)
            self.path_pub.publish(self.path_msg)
            self.current_chasing_point.pose.position.x = float(self.path[self.index][0])
            self.current_chasing_point.pose.position.y = float(self.path[self.index][1])
            self.current_chasing_point.pose.position.z = float(HEIGHT)
            self.current_chasing_point.header.stamp = self.get_clock().now().to_msg()
            self.current_chasing_point.header.frame_id = "map"

        if self.index >= len(self.path)-1:
            self.index = 0
            self.path_msg = copy.deepcopy(self.reference_path_msg)

    def publish_point(self):  
        self.local_pos_pub.publish(self.current_chasing_point)

    def set_offboard_mode(self):
        os.system('ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: True}"')
        os.system('ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: "OFFBOARD"}"')
        
           
    def vehicle_status_callback(self, msg:State):
        
        print("state: ", msg)
        self.reference_path_pub.publish(self.reference_path_msg)
        self.local_traj_pub.publish(self.reference_path_msg)
        self.robot_state = msg
        # automatically set the offboard mode and arm the vehicle , this could be dangerous
        if not msg.armed:
            self.get_logger().info("arming")
            p = Process(target=lambda:os.system('ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: True}"'), args=())
            p.start()
            p.join()
        if msg.mode != "OFFBOARD":
            self.get_logger().info("setting offboard mode")
            p = Process(target=lambda:os.system('ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: "OFFBOARD"}"'), args=())
            p.start()
            p.join()
   

def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()