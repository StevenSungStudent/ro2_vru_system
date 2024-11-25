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
# Supported schemas:

# Serial: /path/to/serial/device[:baudrate]

# Serial: serial:///path/to/serial/device[:baudrate][/?ids=sysid,compid]

# Serial with hardware flow control: serial-hwfc:///path/to/serial/device[:baudrate][?ids=sysid,compid]

# UDP: udp://[bind_host][:port]@[remote_host][:port][/?ids=sysid,compid]

# UDP Broadcast: udp-b://[bind_host][:port]@[:port][/?ids=sysid,compid]

# TCP client: tcp://[server_host][:port][/?ids=sysid,compid]

# TCP server: tcp-l://[bind_host][:port][/?ids=sysid,compid]

from nav_msgs.msg import Odometry
class OffboardControl(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        qos_profile2 = QoSProfile(
            #reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )


        self.status_sub = self.create_subscription(
            Odometry,
            '/mavros/local_position/odom',
            self.vehicle_status_callback,
            qos_profile)
        # self.status_sub = self.create_subscription(
        #     Imu,
        #     '/mavros/imu/data',
        #     self.vehicle_status_callback,
        #     qos_profile)
        self.status_pub = self.create_publisher(OverrideRCIn,"/mavros/rc/override",qos_profile2)
        msg = OverrideRCIn()
        msg.channels[0] = 2000
        msg.channels[1] = 1500
        msg.channels[2] = 1500
        msg.channels[3] = 1500

    def vehicle_status_callback(self, msg):
        # TODO: handle NED->ENU transformation
        print("odom: ", msg)
 
   

def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()