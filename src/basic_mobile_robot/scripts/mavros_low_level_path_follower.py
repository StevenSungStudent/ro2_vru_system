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
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry,Path
from vru_msgs.msg import Task
import os
import copy
from multiprocessing import Process


ROBOT_RADIUS = 0.5

class MissionState:
    path_following = 0
    stopped = 512


class OffboardControl(Node):

    def __init__(self):
        super().__init__('offcontrol_node_mavros')


        self.state = MissionState.stopped

        
     
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
        
        self.path_sub = self.create_subscription(Path,'/lowlevelcontroller/mission_planning',self.load_path,10)
        self.command_sub = self.create_subscription(Task,'/lowlevelcontroller/set_mode',self.command_callback,10)

        self.local_pos_pub = self.create_publisher(PoseStamped,"mavros/setpoint_position/local",10)
        self.local_traj_pub = self.create_publisher(Path,"mavros/setpoint_trajectory/desired",10)
        self.path_pub = self.create_publisher(Path,"vru_current_path",10)
        self.reference_path_pub = self.create_publisher(Path,"vru_reference_path",10)
        
        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.publish_point)

        self.robot_state = State()
        self.path_msg = Path()
        self.reference_path_msg = Path()
        self.current_chasing_point = PoseStamped()
        
        
    def command_callback(self,msg:Task):
        if msg.task_type == Task.TYPE_PATH_FOLLOWING:
            self.state = MissionState.path_following
        if msg.task_type == Task.TYPE_IDLE:
            self.state = MissionState.stopped
    def load_path(self,msg:Path):

        #load the path
        self.path_msg = msg

        self.reference_path_msg = copy.deepcopy(self.path_msg)
        #load the current chasing point
        self.current_chasing_point.pose.position.x = copy.deepcopy(self.path_msg.poses[0].pose.position.x)

        self.current_chasing_point.pose.position.y = copy.deepcopy(self.path_msg.poses[0].pose.position.y)
        self.current_chasing_point.pose.position.z = copy.deepcopy(self.path_msg.poses[0].pose.position.z)
    
        self.current_chasing_point.header.stamp = self.get_clock().now().to_msg()
        self.current_chasing_point.header.frame_id = "map"

        self.state = MissionState.path_following


    def update_chasing_point(self,msg:Odometry):
        if self.state == MissionState.path_following:            
            # if the current point is reached, update the next point
            if (abs(msg.pose.pose.position.x - self.current_chasing_point.pose.position.x) < (ROBOT_RADIUS * 2)) and (abs(msg.pose.pose.position.y - self.current_chasing_point.pose.position.y) < (ROBOT_RADIUS * 2)):
    
                self.get_logger().info("Path_following, updating next point")
                self.path_msg.poses.pop(0)
                
                self.path_pub.publish(self.path_msg)
                try:
                    self.current_chasing_point.pose.position.x = copy.deepcopy(self.path_msg.poses[0].pose.position.x)
                    self.current_chasing_point.pose.position.y = copy.deepcopy(self.path_msg.poses[0].pose.position.y)
                    self.current_chasing_point.pose.position.z = copy.deepcopy(self.path_msg.poses[0].pose.position.z)
                    self.current_chasing_point.header.stamp = self.get_clock().now().to_msg()
                    self.current_chasing_point.header.frame_id = "map"
                except:
                    pass
            if len(self.path_msg.poses) == 0:
                # loop the path
                # self.path_msg = copy.deepcopy(self.reference_path_msg)
                self.get_logger().info("path finished")
                self.state = MissionState.stopped

    def publish_point(self):
        if self.state == MissionState.path_following:
            self.local_pos_pub.publish(self.current_chasing_point)

    def set_offboard_mode(self):
        if self.state == MissionState.path_following:
            os.system('ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: True}"')
            os.system('ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: "OFFBOARD"}"')
        if self.state == MissionState.stopped:
            os.system('ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: False}"')
            os.system('ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: "MANUAL"}"')
           
    def vehicle_status_callback(self, msg:State):
        self.get_logger().info("state: " + str(msg))
        if self.state == MissionState.path_following:
            
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
        if self.state == MissionState.stopped:
            if msg.armed:
                os.system('ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: False}"')


def main(args=None):
    rclpy.init(args=args)

    offboard_control = OffboardControl()

    rclpy.spin(offboard_control)

    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()