#!/usr/bin/env python3

import os

os.system('ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: True}"')
os.system('ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: "OFFBOARD"}"')