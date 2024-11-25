#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2022 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
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
from nav_msgs.msg import Odometry

# Supported schemas:

# Serial: /path/to/serial/device[:baudrate]

# Serial: serial:///path/to/serial/device[:baudrate][/?ids=sysid,compid]

# Serial with hardware flow control: serial-hwfc:///path/to/serial/device[:baudrate][?ids=sysid,compid]

# UDP: udp://[bind_host][:port]@[remote_host][:port][/?ids=sysid,compid]

# UDP Broadcast: udp-b://[bind_host][:port]@[:port][/?ids=sysid,compid]

# TCP client: tcp://[server_host][:port][/?ids=sysid,compid]

# TCP server: tcp-l://[bind_host][:port][/?ids=sysid,compid]
import cv2


img = cv2.imread("AgriBakeLarge.jpg")
img = cv2.resize(img,(800,800))

points = []

path = []
 
def on_EVENT_LBUTTONDOWN(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        xy = "%d,%d" % (x, y)
        points.append([x,y])
        path.append([x/40 - 10 ,-(y/40 - 10)])
        n = len(points)
        cv2.circle(img, (x, y), 2, (0, 0, 255), thickness=3)
        if n>=2:
            cv2.line(img,points[n-2],points[n-1],(0,0,255),thickness = 3)
        cv2.imshow("Farm", img)
        
        

cv2.namedWindow("Farm")
cv2.setMouseCallback("Farm", on_EVENT_LBUTTONDOWN)

cv2.imshow("Farm",img)
cv2.waitKey()
cv2.destroyAllWindows()
print(path)


class OffboardControl(Node):

    def __init__(self):
        super().__init__('offcontrol_node_mavros')




        self.path = path
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
        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.publish_point)

        self.robot_state = State()
    
      
       
        self.indentation = 1
        self.current_chasing_point = PoseStamped()

        self.current_chasing_point.pose.position.x = float(self.path[self.index][0])
        self.current_chasing_point.pose.position.y = float(self.path[self.index][1])
        self.current_chasing_point.pose.position.z = float(20)
    
        self.current_chasing_point.header.stamp = self.get_clock().now().to_msg()
        self.current_chasing_point.header.frame_id = "map"
    def update_chasing_point(self,msg:Odometry):
        # print the current position
        print("current position: ", msg.pose.pose.position.x, msg.pose.pose.position.y)

        # if the current point is reached, update the next point
        if (abs(msg.pose.pose.position.x - self.current_chasing_point.pose.position.x) < 2) and (abs(msg.pose.pose.position.y - self.current_chasing_point.pose.position.y) < 2):
            self.index += 1
            print("updating next point")

            self.current_chasing_point.pose.position.x = float(self.path[self.index][0])
            self.current_chasing_point.pose.position.y = float(self.path[self.index][1])
            self.current_chasing_point.pose.position.z = float(20)
            self.current_chasing_point.header.stamp = self.get_clock().now().to_msg()
            self.current_chasing_point.header.frame_id = "map"
        if self.index >= len(self.path)-1:
            self.index = 0

    def publish_point(self):


        self.local_pos_pub.publish(self.current_chasing_point)
           
    def vehicle_status_callback(self, msg:State):
        
        print("state: ", msg)
        self.robot_state = msg
   

def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()