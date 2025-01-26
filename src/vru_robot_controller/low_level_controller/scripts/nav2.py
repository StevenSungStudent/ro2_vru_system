#!/usr/bin/env python3
############################################################################
#
#This is a simple example of a ROS2 node for px4 control. 
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
from vru_msgs.msg import Task,TaskStatus,Status
import os
import copy
from multiprocessing import Process
from sensor_msgs.msg import BatteryState

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
        #subscribers for mavros
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
        
        self.battery_sub = self.create_subscription(
            BatteryState,
            '/mavros/battery',
            self.battery_callback,
            qos_profile)
        #publishers and receiver for high level controller
        self.task_status_pub = self.create_publisher(TaskStatus,'VRU_robot_controller/TaskStatus',10)
        self.command_sub = self.create_subscription(Task,'VRU_robot_controller/Task',self.command_callback,10)
        self.robot_status_pub = self.create_publisher(Status,'VRU_robot_controller/Status',10)
        #publishers for mavros
        self.local_pos_pub = self.create_publisher(PoseStamped,"mavros/setpoint_position/local",10)
        self.local_traj_pub = self.create_publisher(Path,"mavros/setpoint_trajectory/desired",10)
        #publishers for rviz2
        self.path_pub = self.create_publisher(Path,"vru_current_path",10)
        self.reference_path_pub = self.create_publisher(Path,"vru_reference_path",10)
        #members for the node
        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.publish_point)

        self.robot_state = State()
        self.path_msg = Path()
        self.reference_path_msg = Path()
        self.current_chasing_point = PoseStamped()
        self.current_task = Task()
        self.task_status = TaskStatus()
        self.robot_status = Status()
    def battery_callback(self,msg:BatteryState):
        self.robot_status.battary_status = msg.percentage
   
                                      
    def command_callback(self,msg:Task):
        self.get_logger().info("Recived task for executing with ID: "  + str(msg.task_id))
        self.current_task = msg
        ## process the task
        self.process_task(msg)

    def process_task(self,task:Task):
        self.task_status.status_id += 1.0
        if task.task_type == Task.TYPE_PATH_FOLLOWING:
            self.get_logger().info("path_following " )
            self.load_path(task.path)
            self.state = MissionState.path_following           
            self.task_status.current_task = self.current_task
            self.task_status.compleated = False

            self.task_status_pub.publish(self.task_status) 
        if task.task_type == Task.TYPE_IDLE:
            self.state = MissionState.stopped
            self.task_status.current_task = self.current_task
            self.task_status.compleated = False
            
            self.task_status_pub.publish(self.task_status) 
        

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




    def update_chasing_point(self,msg:Odometry):
        
        self.robot_status.location_x = msg.pose.pose.position.x
        self.robot_status.location_y = msg.pose.pose.position.y
        self.robot_status.move_direction_x = self.current_chasing_point.pose.position.x
        self.robot_status.move_direction_y = self.current_chasing_point.pose.position.y
        self.robot_status.current_task_status = self.task_status
        self.robot_status_pub.publish(self.robot_status)
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
                self.task_status.compleated = True
                self.task_status_pub.publish(self.task_status) 
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
            self.get_logger().info("following path")
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