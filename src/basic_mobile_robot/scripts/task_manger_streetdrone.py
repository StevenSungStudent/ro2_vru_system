#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped, Vector3, PoseStamped
from tf2_ros import TransformBroadcaster
import math
from sensor_msgs.msg import NavSatFix, LaserScan, PointCloud2
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import copy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from vru_msgs.msg import TaskList, Status, Task
from autoware_auto_vehicle_msgs.msg import Engage
from autoware_adapi_v1_msgs.msg import OperationModeState
from autoware_auto_planning_msgs.msg import Trajectory

import time

##very buggy, need to be fixed with proper architecture


class sdStatus:
    autonomous = 0
    manual = 1
    stop = 2


class missionStatus:
    idle = 0
    running = 1
    finish = 2
    error = 3


class sdTaskManger(Node):
    def __init__(self):
        super().__init__('sdTaskManger')

        self.odom_frame_pos = [-51.4, -42.64, 0, 0] # the start point of the system in odom_streetdrone frame

        self.pub_dutStatus = self.create_publisher(
            Status, "/Dut/status", 10)

        self.create_subscription(
            TaskList,
            'Orchestrator/OUT/dutTaskList',
            self.get_tasklist_callback,
            10)

        self.create_subscription(
            Odometry,
            '/odom',
            self.get_odom_callback,
            10)

        self.goal_pub = self.create_publisher(
            PoseStamped,
            'planning/mission_planning/goal',
            10)
        self.autoawre_enage = self.create_publisher(
            Engage,
            'autoware/engage',
            10)
        self.create_subscription(
            OperationModeState, "/control/vehicle_cmd_gate/operation_mode", self.control_mode_call_back, 10)
        self.create_subscription(
            Trajectory, "/planning/scenario_planning/trajectory", self.trajectory_call_back, 10)
        self.create_subscription(Task,"Dut/Task",self.get_task_callback,10)

        self.create_timer(1.0, self.enage_timer)
        self.create_timer(0.02, self.timer_callback)
        self.sbr = StaticTransformBroadcaster(self)
        self.system_Status = sdStatus.stop
        self.current_task_list = TaskList()
        self.current_task = Task()
        self.current_task_status = missionStatus.idle
        self.control_mode_sd = OperationModeState()
        #publish the static transform
        sd_static_transform = TransformStamped()
        sd_static_transform.header.stamp = self.get_clock().now().to_msg()
        sd_static_transform.header.frame_id = "map"
        sd_static_transform.child_frame_id = "odom_dut"
        sd_static_transform.transform.translation.x = self.odom_frame_pos[0]
        sd_static_transform.transform.translation.y = self.odom_frame_pos[1]
        self.sbr.sendTransform(sd_static_transform)
        self.task_index = 0
        self.odom_msg = Odometry()
        self.goal = PoseStamped()
        self.distance = -99999.0
        self.is_current_path_end_with_trajectory = False
            
            
        
    def get_task_callback(self, message: Task):
        if message.task_type == Task.TYPE_IDLE:
            self.system_Status = sdStatus.stop
            self.current_task_list = TaskList()
            self.current_task = Task()
            self.current_task_status = missionStatus.idle
            self.control_mode_sd = OperationModeState()
            self.task_index = 0
            self.goal = PoseStamped()
            self.distance = -99999.0
            self.is_current_path_end_with_trajectory = False
            self.get_logger().warning("get_task_callback: stop everything")
            
    def get_odom_callback(self, message: Odometry):
        self.odom_msg = message
        # convert the message from map frame to odom frame
        dut_status = Status()
        dut_status.location_x = message.pose.pose.position.x - \
            self.odom_frame_pos[0]
        dut_status.location_y = message.pose.pose.position.y - \
            self.odom_frame_pos[1]
        dut_status.speed =  message.twist.twist.linear.x
        self.pub_dutStatus.publish(dut_status)
    def trajectory_call_back(self, msg: Trajectory):
        try:
           
            self.is_current_path_end_with_trajectory = False
            goal_x = self.current_task.path.poses[-1].pose.position.x + \
                            self.odom_frame_pos[0]
            goal_y = self.current_task.path.poses[-1].pose.position.y + \
                            self.odom_frame_pos[1]
            distance_set_msg = math.sqrt((self.goal.pose.position.x - goal_x)**2 + (self.goal.pose.position.y - goal_y)**2)
            self.is_current_path_end_with_trajectory =  distance_set_msg< 0.5
            print("trajectory_call_back", distance_set_msg)
        except BaseException as e:
            self.is_current_path_end_with_trajectory = False
            print("trajectory_call_back", e)
        self.distance = math.sqrt((msg.points[-1].pose.position.x - self.odom_msg.pose.pose.position.x)**2 + (
                msg.points[-1].pose.position.y - self.odom_msg.pose.pose.position.y)**2)


    def enage_timer(self):
        if self.system_Status == sdStatus.autonomous:

            if self.control_mode_sd.mode != 2 and self.current_task_status == missionStatus.running:

                self.get_logger().warning("send engage")
                engage = Engage()
                engage.engage = True
                self.autoawre_enage.publish(engage)
                    #self.system_Status = sdStatus.autonomous
        if self.current_task_status  == missionStatus.running and self.is_current_path_end_with_trajectory:


            self.get_logger().info("Distance_to_end: " + str(self.distance))
            # self.get_logger().info("Time_until_its_stopped: "+ str((distance/self.odom_msg.twist.twist.linear.x)))
            if self.distance < 1.0:
                self.current_task_status = missionStatus.idle
                if self.control_mode_sd.mode == 2:
                    self.get_logger().warning("send STOP")
                    engage = Engage()
                    engage.engage = False
                    self.autoawre_enage.publish(engage)
                    try:
                        self.current_task = self.current_task_list.tasks[self.task_index]
                        self.task_index +=1
                    except:
                        self.get_logger().warning("No more task")

                        self.system_Status = sdStatus.stop
        if self.current_task_status == missionStatus.idle and self.distance >=3.0 and self.is_current_path_end_with_trajectory:
            self.current_task_status = missionStatus.running 
    def control_mode_call_back(self, message: OperationModeState):
        self.control_mode_sd = message

        print("control_mode_call_back", message)
    def timer_callback(self):
        if self.system_Status == sdStatus.autonomous and self.current_task_status == missionStatus.idle:
            # plan a goal
            try:
                self.current_task.start_event.param[0] -= 0.02
                self.get_logger().info("timer_callback:  " + str(self.current_task.start_event.param[0])+ " s to start")
                if self.current_task.start_event.param[0] < 0.0:
                    
                    goal = PoseStamped()
                    goal.header.frame_id = "map"
                    goal.header.stamp = self.get_clock().now().to_msg()
                    goal.pose.position.x = self.current_task.path.poses[-1].pose.position.x + \
                        self.odom_frame_pos[0]
                    goal.pose.position.y = self.current_task.path.poses[-1].pose.position.y + \
                        self.odom_frame_pos[1]
                    goal.pose.position.z = 0.0
                    goal.pose.orientation.x = 0.0
                    goal.pose.orientation.y = 0.0
                    goal.pose.orientation.z = -0.8572797293371865
                    goal.pose.orientation.w = 0.5148509159626311
                    self.current_task_status = missionStatus.running
                    self.goal_pub.publish(goal)
                    self.goal = goal
            except:
                self.get_logger().warning("timer_callback: No task to plan")
                #self.system_Status = sdStatus.stop
                

    def get_tasklist_callback(self, message: TaskList):
        print("get_tasklist_callback", message)
        # trigger the system status
        if len(message.tasks) > 0:
            print("get_tasklist_callback", message.tasks)
            self.system_Status = sdStatus.autonomous
            self.current_task_list = message
            self.current_task = message.tasks[0]
            self.task_index +=1
        else:
            self.get_logger().warning("get_tasklist_callback: message.tasks is None")


def main(args=None):
    rclpy.init(args=args)
    optitrack_tf_state = sdTaskManger()
    rclpy.spin(optitrack_tf_state)
    # optitrack_tf_state.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
