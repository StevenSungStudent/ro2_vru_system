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



class OffboardControl(Node):

    def __init__(self):
        super().__init__('offcontrol_node_mavros')
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


        self.local_pos_pub = self.create_publisher(PoseStamped,"mavros/setpoint_position/local",10)
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.publish_point)
        self.robot_state = State()
        
        self.theta = 0.0
        self.radius = 10.0
        self.omega = 0.3

        self.i = 0
        
    def publish_point(self):
       
        
            pose = PoseStamped()
            pose.pose.position.x = self.radius * np.cos(self.theta)
            pose.pose.position.y = self.radius * np.sin(self.theta)
            pose.pose.position.z = 20.5
            self.local_pos_pub.publish(pose)
           
            if self.i < 100:
                self.theta = self.theta + self.omega * self.dt
                self.i = self.i + 1
                print("publishing the pose in local frame", pose)

           
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