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
from vru_msgs.msg import Task,TaskList,Events
HEIGHT = 20

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
        
        





class plan_drawer(Node):

    def __init__(self):
        super().__init__('offcontrol_node_mavros')




        self.path = path
        self.get_logger().info("path: " + str(self.path))
        self.index = 0


  
        self.task_pub = self.create_publisher(TaskList,"Orchestrator/OUT/TaskList",10)
        
        cv2.namedWindow("reference_frame")
        cv2.setMouseCallback("reference_frame", on_EVENT_LBUTTONDOWN)

        cv2.imshow("reference_frame",img)
        cv2.waitKey()
        cv2.destroyAllWindows()
    

        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"
        self.path_msg.header.stamp = self.get_clock().now().to_msg()

        for point in path:
            tmp = PoseStamped()
            tmp.pose.position.x = float(point[0])
            tmp.pose.position.y = float(point[1])
            tmp.pose.position.z = float(HEIGHT)
            self.path_msg.poses.append(tmp)

        self.task = Task()
        self.task.task_type = Task.TYPE_PATH_FOLLOWING
        self.task.path = self.path_msg
        self.task.task_id = 1224
        startevent = Events()
        startevent.event_id = 1
        startevent.event_type = "START"
        
        self.task.start_event = startevent

        self.task_list = TaskList()
        self.task_list.list_id = 1
        self.task_list.tasks.append(self.task)
        self.task_list.tasks.append(self.task)
        self.task_list.tasks.append(self.task)
        self.task_list.tasks.append(self.task)
        
        self.task_list.list_lenght = 1
        self.task_pub.publish(self.task_list)
        
       
        
  

def main(args=None):
    rclpy.init(args=args)

    offboard_control = plan_drawer()

    rclpy.spin_once(offboard_control,timeout_sec=5.0)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()