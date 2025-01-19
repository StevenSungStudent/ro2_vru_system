#!/usr/bin/env python3

import os
from operator import truediv
import sys
from turtle import speed
from PyQt5.QtWidgets import (
    QApplication, QDialog, QMainWindow, QMessageBox, QGraphicsScene, QGraphicsRectItem, QGraphicsItem, QGraphicsPixmapItem,QFileDialog
)

from PyQt5 import QtCore, QtGui, QtWidgets
import xml.etree.ElementTree as ET
from PyQt5.QtCore import QPointF, QTimer, QPoint, QDateTime, Qt, QThread, QEvent
from click import command
from sympy import false, true
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from rosgraph_msgs.msg import Clock
import threading
import msgpack
import uuid
import time
import math
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from vru_msgs.msg import Task, TaskList, Events,Status
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from scipy import interpolate
try:
    from .coordinator_layout import Ui_MainWindow

except:
    from coordinator_layout import Ui_MainWindow
import cv2

import copy
import datetime
from OSMHandler import OSMHandler
import umsgpack
import xml.dom.minidom

points = []

path = []
bringup_dir = get_package_share_directory('coordinator')
file_path = bringup_dir + "/map/" + "reference_frame" + '.png'
img = cv2.imread(file_path)
img = cv2.resize(img, (800, 800))
img_map = img
# map_file_path = bringup_dir + "/map/" + "map" + '.png'
# img_map = cv2.imread(map_file_path)
# img_map = cv2.resize(img_map, (1200, 800))
class logger():
    # add this logger to write the log to a file as well
    def __init__(self, fd):
        self.fd = fd

    def info(self, msg):
        return "[INFO]: " + msg

    def warning(self, msg):
        return "[WARNING]: " + msg

    def error(self, msg):
        return "[ERROR]: " + msg

    def debug(self, msg):
        return "[DEBUG]: " + msg


def on_EVENT_LBUTTONDOWN_VRU(event, x, y, flags, param):

    if event == cv2.EVENT_LBUTTONDOWN:

        color = (0, 0, 255)
        update_imgae(x, y, color)


def on_EVENT_LBUTTONDOWN_DUT(event, x, y, flags, param):

    if event == cv2.EVENT_LBUTTONDOWN:

        color = (0, 255, 0)
        update_imgae(x, y, color)
        

def update_imgae(x, y, color):
    xy = "%d,%d" % (x, y)

    points.append([x, y])
    print([x, y])
    path.append([x/8 - 50, -(y/8 - 50)])
    print([x/8 - 50, -(y/8 - 50)])
    n = len(points)
    cv2.circle(img, (x, y), 2, color, thickness=3)
    if n >= 2:
        cv2.line(img, points[n-2], points[n-1], color, thickness=3)
    cv2.imshow("reference_frame_0_point_is the robot pos", img)


def quart_to_rpy(x, y, z, w):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(2 * (w * y - x * z))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (z * z + y * y))
    return roll, pitch, yaw


def calc_distance(point_a: list, point_b: list) -> float:
    dx = point_a[0] - point_b[0]
    dy = point_a[1] - point_b[1]
    return math.hypot(dx, dy)





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

def read_osm_file(osm_file):
       

    handler = OSMHandler()
    handler.ways = {}  # Initialize a dictionary to store ways
    handler.data = {"nodes": {}}

    handler.apply_file(osm_file)

    return handler.relations
# This is the class to handle the ros2 part of the UI


class Orchestrator_ui(Node):

    def __init__(self):
        super().__init__('Orchestrator_ui')
        
        self.get_logger().info("Ros2 module for Orchestrator UI is running")
             
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.task_pub = self.create_publisher(
            TaskList, "Orchestrator/OUT/vruTaskList", 10)
        self.task_dut_pub = self.create_publisher(
            TaskList, "Orchestrator/OUT/dutTaskList", 10)
        self.task_lowlevel_pub = self.create_publisher(
            Task, "VRU_robot_controller/Task", 10)
        self.task_lowlevel_pub_dut = self.create_publisher(
            Task, "Dut/Task", 10)
        self.status_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.vehicle_status_callback,
            qos_profile)
        
        self.arming_client = self.create_client(CommandBool, 'mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, 'mavros/set_mode')
        
        # system feedback
        self.create_subscription(Status,'VRU_robot_controller/Status', self.vru_status_callback, 10)
        self.create_subscription(Status,'Dut/status', self.dut_status_callback, 10)
        self.vru_reference_path_pub = self.create_publisher(Path,"vru_reference_path",10)
        self.dut_reference_path_pub = self.create_publisher(Path,"dut_reference_path",10)
        self.vru_status = Status()
        self.dut_status = Status()
        self.vru_state = State()

    
    def dut_status_callback(self,msg):
        self.dut_status = msg
    def vru_status_callback(self,msg):
        self.vru_status = msg
    def vehicle_status_callback(self, msg: State):
        self.vru_state = msg
# This is the class to handle the UI


class subWindow(Ui_MainWindow, QMainWindow):
    flag = 0
    R = 200
    B = 200
    G = 200
    timer_interval = 16  # in ms
    Max_waitingtime = 2  # in s
    logger_ = logger(sys.stdout)

    def __init__(self, Robotid=None):
        super().__init__(None)
        rclpy.init()
        self.setupUi(self)
        self.connectSignalsSlots()

        self.odom_vru_reference_point = [-95,-70,0,0] # the start point of the system in odom_vru frame
        self.odom_streetdrone_reference_point = [-51.4, -42.64, 0, 0] # the start point of the system in odom_streetdrone frame
        self.map_gps_orgin = [51.98940382497835 , 5.949848804210092, 0.0] # the start point of the system in map frame x,y heading in degree

        self.thread_ros2 = None
        self.ros_connect()
        self.pushButton_confirm_vru.setEnabled(False)
        self.pushButton_confirm_dut.setEnabled(False)
        self.length_counter = int(
            self.Max_waitingtime / (self.timer_interval / 1000))
        self.messagecounter = [0] * self.length_counter
        self.liveness = False

        self.tasklist_vru = TaskList()
        self.tasklist_dut = TaskList()

        self.current_task = Task()
        self.current_task.task_id = 0
        self.current_task.start_event.param.append(-1.0)
        QMainWindow.setWindowTitle(self, "Orchestrator UI")


        bringup_dir = get_package_share_directory('coordinator')
        map_full_path = bringup_dir + "/map/" + "/han_parking_deck_real_noa/lanelet2_map.osm"
        
        self.map_data = read_osm_file(map_full_path)
        self.main_deck_map =[self.map_data[7121][0],self.map_data[7121][1]]
        
        ## calculate the map in map frame
        self.map_data_local = []
        for gps_point in self.main_deck_map[0]:
            x, y = self.calc_map_base_link(self.map_gps_orgin,[gps_point[0],gps_point[1],0.0])
            self.map_data_local.append([x,y])
        for gps_point in self.main_deck_map[1]:
            x, y = self.calc_map_base_link(self.map_gps_orgin,[gps_point[0],gps_point[1],0.0])
            self.map_data_local.append([x,y])
        

    def calc_map_base_link(self,origin, map_data_point):
        bearing = calc_compass_bearing(origin , map_data_point)
        dist = calc_dist(
                origin [0], origin [1], map_data_point[0], map_data_point[1])
        bearing_rad = math.radians(bearing)
         ## transform gps_link to base_link
        x = dist * math.sin(bearing_rad) 
        y = dist * math.cos(bearing_rad) 

        # fix the offset in osm file
        theta = np.radians(2.5) 

        # Define the rotation matrix
        r_m = np.array([[np.cos(theta), -np.sin(theta)],
                    [np.sin(theta), np.cos(theta)]])
        rotated_point = np.dot(r_m,[x,y])
        
        return rotated_point[0],rotated_point[1]
    
    def connectSignalsSlots(self):
        self.set_route_vru.clicked.connect(lambda: self.plan_route("vru"))
        self.set_route_dut.clicked.connect(lambda: self.plan_route("dut"))
        self.pushButton_confirm_vru.clicked.connect(self.confirm_task_vru)
        self.pushButton_confirm_dut.clicked.connect(self.confirm_task_dut)
        self.pushButton_event_start.clicked.connect(self.event_start)
        self.pushButton_arm_vru.clicked.connect(self.armming_vru)
        self.pushButton_disarm_vru.clicked.connect(self.disarming_vru)
        self.pushButton_Manual_vru.clicked.connect(self.set_manual_vru_mode)
        self.pushButton_Manual_vru_2.clicked.connect(self.set_hold_vru_mode)
        self.pushButton__reset.clicked.connect(self.reset_system)
        self.pushButton_event_stop.clicked.connect(self.event_stop)
        self.pushButton_Return_vru.clicked.connect(self.set_return_vru_mode)
        self.textBrowser_tasklist_vru.setTextInteractionFlags(
            Qt.TextSelectableByMouse)
        self.textBrowser_tasklist_vru.viewport().installEventFilter(self)
        self.textBrowser_tasklist_dut.setTextInteractionFlags(
            Qt.TextSelectableByMouse)
        self.textBrowser_tasklist_dut.viewport().installEventFilter(self)
        self.pushButton.clicked.connect(self.load_config)
        self.pushButton_save_config.clicked.connect(self.save_config)
    
    def load_config(self):
        options = QFileDialog.Options()
        file_name, _ = QFileDialog.getOpenFileName(self, "Load Task List", "task_list.msgpack", "msgpack Files (*.msgpack);;All Files (*)", options=options)
        if file_name:
            with open(file_name, 'rb') as f:
                msgpack_data = f.read()
           
            structed_data = msgpack.unpackb(msgpack_data)

            if len(structed_data[0])>0:
                self.tasklist_vru = self.tasklist_to_ros2_msg(structed_data[0])
                self.refresh_tasklist_vru()
                self.system_output.append(self.logger_.info("Vru Tasklist loaded"))
            if len(structed_data[1])>0:
                self.tasklist_dut = self.tasklist_to_ros2_msg(structed_data[1])
                self.refresh_tasklist_dut()
                self.system_output.append(self.logger_.info("Dut Tasklist loaded"))
       
    def save_config(self):
        options = QFileDialog.Options()
        file_name, _ = QFileDialog.getSaveFileName(self, "Save Task List", "task_list.msgpack", "msgpack Files (*.msgpack);;All Files (*)", options=options)
        if file_name:
            vru_tasklist_structed = self.restruct_tasklist(self.tasklist_vru)
            dut_tasklist_structed = self.restruct_tasklist(self.tasklist_dut)
            msgpack_data = msgpack.packb([vru_tasklist_structed,dut_tasklist_structed])
            with open(file_name, 'wb') as f:
                f.write(msgpack_data)
            print(file_name)
    def restruct_tasklist(self, tasklist: TaskList):
        structed_data = []
        try:
            for task in tasklist.tasks:
       
                task_type = task.task_type
                task_id = task.task_id
                start_event_event_type = task.start_event.event_type
                start_event_event_id = task.start_event.event_id
                

                stop_event_event_type = task.stop_event.event_type
                stop_event_event_id = task.stop_event.event_id
                param = []
                path = []
                path_header_frame_id = task.path.header.frame_id

                start_event_param = []
                stop_event_param = []
                for parm in task.start_event.param:
                    start_event_param.append(parm)
                for parm in task.stop_event.param:
                    stop_event_param.append(parm)
                for point in task.path.poses:
                    path.append([point.pose.position.x, point.pose.position.y])
                for parm in task.param:
                    param.append(parm)
                temp = [task_type, task_id,param, start_event_event_type, start_event_event_id, start_event_param,
                        stop_event_event_type, stop_event_event_id, stop_event_param, path,path_header_frame_id]
                structed_data.append(temp)
            return structed_data
        except BaseException as error:
            self.system_output.append(self.logger_.error(
                "from function " + str(self.restruct_tasklist.__name__) + " " + str(error)))
            print(error)
            return None
    def tasklist_to_ros2_msg(self, structed_data):
        try:
            tasklist = TaskList()
            for task in structed_data:
                task_ros2 = Task()
                task_ros2.task_type = task[0]
                task_ros2.task_id = task[1]
                task_ros2.param = task[2]
                task_ros2.start_event.event_type = task[3]
                task_ros2.start_event.event_id = task[4]
                task_ros2.start_event.param = task[5]
                task_ros2.stop_event.event_type = task[6]
                task_ros2.stop_event.event_id = task[7]
                task_ros2.stop_event.param = task[8]
                path = Path()
                path.header.frame_id = task[10]
                path.header.stamp = self.ros2_handler.get_clock().now().to_msg()
                for point in task[9]:
                    tmp = PoseStamped()
                    tmp.pose.position.x = float(point[0])
                    tmp.pose.position.y = float(point[1])
                    tmp.pose.position.z = float(1.0)
                    path.poses.append(tmp)
                task_ros2.path = path
                tasklist.tasks.append(task_ros2)
            return tasklist
        except BaseException as error:
            self.system_output.append(self.logger_.error(
                "from function " + str(self.tasklist_to_ros2_msg.__name__) + " " + str(error)))
            print(error)
            return None
    def eventFilter(self, obj, event):
        if obj == self.textBrowser_tasklist_vru.viewport() and event.type() == QEvent.MouseButtonDblClick:
            cursor = self.textBrowser_tasklist_vru.cursorForPosition(
                event.pos())
            cursor.select(cursor.WordUnderCursor)
            selected_text = cursor.selectedText()
            if selected_text.strip():  # Check if there is non-whitespace text selected
                self.call_back_click_vru(selected_text, "vru")
                return True
        if obj == self.textBrowser_tasklist_dut.viewport() and event.type() == QEvent.MouseButtonDblClick:
            cursor = self.textBrowser_tasklist_dut.cursorForPosition(
                event.pos())
            cursor.select(cursor.WordUnderCursor)
            selected_text = cursor.selectedText()
            if selected_text.strip():  # Check if there is non-whitespace text selected
                self.call_back_click_vru(selected_text, "dut")

                return True
        return super().eventFilter(obj, event)

    def call_back_click_vru(self, selected_text, type="vru"):
        img_map = copy.deepcopy(img)
        try:
            task_id_ = eval(selected_text)
            is_task_in_list = False
            selected_task = Task()
            color = (0, 0, 0)
            # check if the task is in the list
            if type == "vru":
                
                color = (0, 0, 255)
                for task in self.tasklist_vru.tasks:
                    if task.task_id == task_id_ and task.task_type == Task.TYPE_PATH_FOLLOWING:
                        is_task_in_list = True
                        selected_task = task
                        break
                self.ros2_handler.vru_reference_path_pub.publish(selected_task.path)
            if type == "dut":
                color = (0, 255, 0)
                for task in self.tasklist_dut.tasks:
                    if task.task_id == task_id_ and task.task_type == Task.TYPE_PATH_FOLLOWING:
                        is_task_in_list = True
                        selected_task = task
                        break
                self.ros2_handler.dut_reference_path_pub.publish(selected_task.path)
                
            if is_task_in_list:
                points = []
                if type == "vru":
                    map_vru = []
                    for point in self.map_data_local:
                        map_vru.append([point[0]-self.odom_vru_reference_point[0],point[1]-self.odom_vru_reference_point[1]])
                    
                        ## draw the map on the image transform from the map frame to image format 
                    for i in range(len(map_vru)):
                        x = int((map_vru[i][0]+50)*8)
                        y = int((-map_vru[i][1]+50)*8)
                        
                        if i > 0 and i != (len(self.main_deck_map[0])):
                            x_pre = int((map_vru[i-1][0]+50)*8)
                            y_pre = int((-map_vru[i-1][1]+50)*8)
                            cv2.line(img_map, (x, y),(x_pre,y_pre), (0,0,0), thickness=2)

                if type == "dut":
                    map_dut = []
                    for i in range(len(self.map_data_local)):
                        map_dut.append([self.map_data_local[i][0]-self.odom_streetdrone_reference_point[0],self.map_data_local[i][1]-self.odom_streetdrone_reference_point[1]])
                        ## draw the map on the image transform from the map frame to image format
                    for i in range(len(map_dut)):
                        x = int((map_dut[i][0]+50)*8)
                        y = int((-map_dut[i][1]+50)*8)
                        
                        if i > 0 and i != (len(self.main_deck_map[0])):
                            x_pre = int((map_dut[i-1][0]+50)*8)
                            y_pre = int((-map_dut[i-1][1]+50)*8)
                            cv2.line(img_map, (x, y),(x_pre,y_pre), (0,0,0), thickness=2)

                for point in selected_task.path.poses:

                    x = int((point.pose.position.x+50)*8)
                    y = int((-point.pose.position.y+50)*8)
                    points.append([x, y])
                    n = len(points)
                    if n >= 2:
                        cv2.line(img_map, (points[n-2][0], points[n-2][1]),
                                 (points[n-1][0], points[n-1][1]), color, thickness=3)
                    cv2.circle(img_map, (x, y), 2, color, thickness=4)

                window_name = "reference_frame map: " + str(type)
                cv2.namedWindow(window_name)
                cv2.imshow(window_name, img_map)
                cv2.waitKey()
                cv2.destroyAllWindows()
                if type == "vru":
                    clear_path = Path()
                    clear_path.header.frame_id = "odom_vru"
                    clear_path.header.stamp = self.ros2_handler.get_clock().now().to_msg()
                    self.ros2_handler.vru_reference_path_pub.publish(clear_path)
                elif type == "dut":
                    clear_path = Path()
                    clear_path.header.frame_id = "odom_dut"
                    clear_path.header.stamp = self.ros2_handler.get_clock().now().to_msg()
                    self.ros2_handler.dut_reference_path_pub.publish(clear_path)
                # img_map = cv2.imread(map_file_path)
                # img_map = cv2.resize(img_map, (1200, 800))

            self.system_output.append(self.logger_.info(
                "Double-clicked on text:" + str(task_id_)))
            print()
        except BaseException as error:
            self.system_output.append(self.logger_.error(
                "from function " + str(self.call_back_click_vru.__name__) + " " + str(error)))
            return
    def event_stop(self):
        #stop vru
        stop_task = Task()
        stop_task.task_id = 0
        stop_task.task_type = Task.TYPE_IDLE
        self.ros2_handler.task_lowlevel_pub.publish(stop_task)
        #stop dut
        self.ros2_handler.task_lowlevel_pub_dut.publish(stop_task)

    def event_start(self):
        if (self.tasklist_vru.tasks) != 0:
            self.ros2_handler.task_pub.publish(self.tasklist_vru)
            # clear the list
            self.tasklist_vru.tasks.clear()
            self.refresh_tasklist_vru()
            self.system_output.append(self.logger_.info("Vru Tasklist sent"))

        if (self.tasklist_dut.tasks) != 0:
            self.ros2_handler.task_dut_pub.publish(self.tasklist_dut)
            # clear the list
            self.tasklist_dut.tasks.clear()
            self.refresh_tasklist_dut()
            self.system_output.append(self.logger_.info("Dut Tasklist sent"))

    def reset_system(self):
        self.tasklist_vru.tasks.clear()
        self.tasklist_dut.tasks.clear()
        self.refresh_tasklist_vru()
        self.refresh_tasklist_dut()
        self.system_output.append(self.logger_.info("System reset"))

    def plan_route(self, type="vru"):
        global path
        global points
        global img
        try:
            cv2.namedWindow("reference_frame_0_point_is the robot pos")

            if type == "vru":
                map_vru = []
                for point in self.map_data_local:
                    map_vru.append([point[0]-self.odom_vru_reference_point[0],point[1]-self.odom_vru_reference_point[1]])
                
                    ## draw the map on the image transform from the map frame to image format 
                for i in range(len(map_vru)):
                    x = int((map_vru[i][0]+50)*8)
                    y = int((-map_vru[i][1]+50)*8)
                    
                    if i > 0 and i != (len(self.main_deck_map[0])):
                        x_pre = int((map_vru[i-1][0]+50)*8)
                        y_pre = int((-map_vru[i-1][1]+50)*8)
                        cv2.line(img, (x, y),(x_pre,y_pre), (0,0,0), thickness=2)

                cv2.setMouseCallback("reference_frame_0_point_is the robot pos", on_EVENT_LBUTTONDOWN_VRU)
            if type == "dut":
                map_dut = []
                for i in range(len(self.map_data_local)):
                    map_dut.append([self.map_data_local[i][0]-self.odom_streetdrone_reference_point[0],self.map_data_local[i][1]-self.odom_streetdrone_reference_point[1]])
                    ## draw the map on the image transform from the map frame to image format
                for i in range(len(map_dut)):
                    x = int((map_dut[i][0]+50)*8)
                    y = int((-map_dut[i][1]+50)*8)
                    
                    if i > 0 and i != (len(self.main_deck_map[0])):
                        x_pre = int((map_dut[i-1][0]+50)*8)
                        y_pre = int((-map_dut[i-1][1]+50)*8)
                        cv2.line(img, (x, y),(x_pre,y_pre), (0,0,0), thickness=2)
                cv2.setMouseCallback("reference_frame_0_point_is the robot pos", on_EVENT_LBUTTONDOWN_DUT)
            
            if type == "vru":
                vru_x = int((self.ros2_handler.vru_status.location_x+50)*8)
                vru_y = int((-self.ros2_handler.vru_status.location_y+50)*8)
                update_imgae(vru_x, vru_y, (255, 0, 0))
            if type == "dut":
                dut_x = int((self.ros2_handler.dut_status.location_x+50)*8)
                dut_y = int((-self.ros2_handler.dut_status.location_y+50)*8)
                update_imgae(dut_x, dut_y, (255, 0, 0))

            cv2.imshow("reference_frame_0_point_is the robot pos", img)
            cv2.waitKey()
            cv2.destroyAllWindows()

            path_msg = Path()
            if type == "vru":
                path_msg.header.frame_id = "odom_vru"
            if type == "dut":
                path_msg.header.frame_id = "odom_dut"
            path_msg.header.stamp = self.ros2_handler.get_clock().now().to_msg()

            #spline interpolation
            #array of x from points 
            rx = []
            ry = []
            for point in path:
                rx.append(point[0])
                ry.append(point[1])
            tck, u = interpolate.splprep([rx, ry], s=0.8)
            unew = np.arange(-0.00, 1.00, 0.02)
            out = interpolate.splev(unew, tck)
            rx = out[0]
            ry = out[1]
            path.clear()
            for i in range(len(rx)):
                path.append([rx[i],ry[i]])
            print(path)

            
            for point in path:
                tmp = PoseStamped()
                tmp.pose.position.x = float(point[0])
                tmp.pose.position.y = float(point[1])
                tmp.pose.position.z = float(1.0)
                path_msg.poses.append(tmp)

            img = cv2.imread(file_path)
            img = cv2.resize(img, (800, 800))
            path.clear()
            points.clear()
            self.current_task.task_id += 1
            self.current_task.path = path_msg

            self.current_task.task_type = Task.TYPE_PATH_FOLLOWING
            self.current_task.start_event.param[0] = (-1.0)
            if type == "vru":
                self.pushButton_confirm_vru.setEnabled(True)
            if type == "dut":
                self.pushButton_confirm_dut.setEnabled(True)
            self.system_output.append(self.logger_.info("Route planned"))
        except BaseException as error:
            warning_msg = "path plan failed: " + str(error)
            QMessageBox.warning(self, 'Warning', warning_msg, QMessageBox.Ok)
            img = cv2.imread(file_path)
            img = cv2.resize(img, (800, 800))
            path.clear()
            points.clear()
            return

    def confirm_task_vru(self):
        global path
        global points
        global img

        try:
            if self.current_task.start_event.param[0] == (float(eval(self.lineEdit_vru_start_s.text()))):
                warning_msg = "Two task at the same time check it again"
                QMessageBox.warning(
                    self, 'Warning', warning_msg, QMessageBox.Ok)
                return
            else:
                
                self.current_task.start_event.param[0] = (
                    float(eval(self.lineEdit_vru_start_s.text())))
                
                self.current_task.start_event.event_type = "DELAY_START"
                
              
        except BaseException as e:
            warning_msg = "Enter a valid time is second: " + str(e)
            QMessageBox.warning(self, 'Warning', warning_msg, QMessageBox.Ok)
            img = cv2.imread(file_path)
            img = cv2.resize(img, (800, 800))
            path.clear()
            points.clear()
            return
        self.tasklist_vru.tasks.append(copy.deepcopy(self.current_task))
        self.system_output.append(self.logger_.info("Vru Task confirmed"))
        self.pushButton_confirm_vru.setEnabled(False)
        self.refresh_tasklist_vru()
        self.current_task.start_event.param[0] = -1.0
       

    def confirm_task_dut(self):
        global path
        global points
        global img
        try:
            if self.current_task.start_event.param[0] == (float(eval(self.lineEdit_dut_start_s.text()))):
                warning_msg = "Two task at the same time check it again"
                QMessageBox.warning(
                    self, 'Warning', warning_msg, QMessageBox.Ok)
                return
            else:
                self.current_task.start_event.event_type = "DELAY_START"

                self.current_task.start_event.param[0] = (
                    float(eval(self.lineEdit_dut_start_s.text())))
        except BaseException as e:
            warning_msg = "Enter a valid time is second: "+str(e)
            QMessageBox.warning(self, 'Warning', warning_msg, QMessageBox.Ok)
            img = cv2.imread(file_path)
            img = cv2.resize(img, (800, 800))
            path.clear()
            points.clear()
            return
        self.tasklist_dut.tasks.append(copy.deepcopy(self.current_task))
        self.pushButton_confirm_dut.setEnabled(False)
        self.refresh_tasklist_dut()
        self.current_task.start_event.param[0] = -1.0
        self.system_output.append(self.logger_.info("Dut Task confirmed"))

    def refresh_tasklist_vru(self):
        tasklist_title_vru = "DelaySec\t" + "TaskID\t" + "TaskType\t\t"
        self.textBrowser_tasklist_vru.setTextColor(QtGui.QColor(150, 15, 15))
        self.textBrowser_tasklist_vru.setText(tasklist_title_vru)
        self.textBrowser_tasklist_vru.setTextColor(QtGui.QColor(0, 0, 0))
        self.textBrowser_tasklist_vru.setFont(QtGui.QFont("SimHei"))

        for task in self.tasklist_vru.tasks:
            tasktime = self.timestamp_to_string(task.start_event.param[0])

            tasklist_str = str(tasktime) + "\t" + \
                str(task.task_id) + "\t"+str(task.task_type)
            self.textBrowser_tasklist_vru.append(tasklist_str)

        return

    def timestamp_to_string(self,timestamp):
        return str(timestamp)+str("s")
       

    def refresh_tasklist_dut(self):
        tasklist_title_dut = "DelaySec\t" + "TaskID\t" + "TaskType\t\t"
        self.textBrowser_tasklist_dut.setTextColor(QtGui.QColor(150, 15, 15))
        self.textBrowser_tasklist_dut.setText(tasklist_title_dut)
        self.textBrowser_tasklist_dut.setTextColor(QtGui.QColor(0, 0, 0))
        self.textBrowser_tasklist_dut.setFont(QtGui.QFont("SimHei"))
        
        for task in self.tasklist_dut.tasks:
            tasktime = self.timestamp_to_string(task.start_event.param[0])

            tasklist_str = str(tasktime) + "\t" + \
                str(task.task_id) + "\t"+str(task.task_type)
            self.textBrowser_tasklist_dut.append(tasklist_str)

    def running_breath(self):
        step = 4
        if (self.flag == 0):
            self.R -= step
            self.B -= step
            if (self.R < 10):
                self.flag = 1
        else:
            self.R += step
            self.B += step
            if (self.R > 200):
                self.flag = 0

    def ros_connect(self):
        thread_1 = threading.Thread(target=self.spin_node)
        try:
            if self.thread_ros2.is_alive():
                rclpy.shutdown()
                self.thread_ros2.join()
                rclpy.init()
        except:
            pass
        self.ros2_handler = Orchestrator_ui()

        thread_1.start()
        self.thread_ros2 = thread_1
        self.timer = QTimer()
        self.timer.timeout.connect(self.updateGUi)
        self.timer.start(self.timer_interval)
        self.system_output.append(self.logger_.info(
            "Ros2 module for Orchestrator UI is running"))

    def spin_node(self):
        rclpy.spin(self.ros2_handler)

    def closeEvent(self, event):
        rclpy.shutdown()
    def armming_vru(self):
        # os.system('ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: True}"') #TODO: make these service calls properly with clients and stuff, this lags the shit out of the GUI.
        
        arming_request = CommandBool.Request()
        arming_request.value = True
        self.ros2_handler.arming_client.call_async(arming_request)
        
    def disarming_vru(self):
        # os.system('ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: False}"')
        
        arming_request = CommandBool.Request()
        arming_request.value = False
        self.ros2_handler.arming_client.call_async(arming_request)

    def set_manual_vru_mode(self):
        # os.system('ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: "MANUAL"}"')
        
        mode_request = SetMode.Request()
        mode_request.custom_mode = "MANUAL"
        self.ros2_handler.mode_client.call_async(mode_request)
        
    def set_hold_vru_mode(self):
        # os.system('ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: "STABILIZED"}"')
        mode_request = SetMode.Request()
        mode_request.custom_mode = "STABILIZED"
        self.ros2_handler.mode_client.call_async(mode_request)
        
    def set_return_vru_mode(self):
        # os.system('ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: "OFFBOARD"}"')
        mode_request = SetMode.Request()
        mode_request.custom_mode = "OFFBOARD"
        self.ros2_handler.mode_client.call_async(mode_request)
        
    def updateGUi(self):
        self.running_breath()
        color = "background-color: rgb("+str(self.R) + \
            ","+str(self.G) + "," + str(self.B) + ");"
        if self.ros2_handler.vru_state.armed:
            self.pushButton_armed_vru.setStyleSheet(color)
        else:
            self.pushButton_armed_vru.setStyleSheet("background-color: rgb(255, 0, 0);")
        self.lineEdit_pos_vru_2.setText(self.ros2_handler.vru_state.mode)
        self.pushButton_alive_vru.setStyleSheet(color)
        self.pushButton_alive_dut.setStyleSheet(color)
        position_msg = "x: " + str(round(self.ros2_handler.vru_status.location_x,2)) + " y: " + str(round(self.ros2_handler.vru_status.location_y,2)) 
        self.lineEdit_pos_vru.setText(position_msg)
        position_msg_dut = "x: " + str(round(self.ros2_handler.dut_status.location_x,2)) + " y: " + str(round(self.ros2_handler.dut_status.location_y,2)) 
        self.lineEdit_pos_dut.setText(position_msg_dut)

def main():
    app = QApplication(sys.argv)
    try:
        print(sys.argv[1])
        win = subWindow(sys.argv[1])
    except:

        win = subWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()

    # ros2 bag record -o subset /robot1/robot_status /robot1/vehicle_status /robot2/robot_status /robot2/vehicle_status
