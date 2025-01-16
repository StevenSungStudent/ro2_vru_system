# Description:
# This launch file is for the nvidia Jetson, which is IN the vru robot itself. It launches all the nodes needed for offboard mode.
# This is meant to be used in conjunction with 'orchestrator.launch.py'

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

  # Launch configuration variables specific to simulation
  namespace = LaunchConfiguration('namespace')
  
  use_sim_time = LaunchConfiguration('use_sim_time')
  
  # Declare the launch arguments  
  declare_namespace_cmd = DeclareLaunchArgument(
    name='namespace',
    default_value='',
    description='Top-level namespace')

  declare_use_namespace_cmd = DeclareLaunchArgument(
    name='use_namespace',
    default_value='False',
    description='Whether to apply a namespace to the navigation stack')
    
  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='False',
    description='Use simulation (Gazebo) clock if true') 
  
  vru_low_level_controller_cmd = Node(
    package='low_level_controller',
    executable='mavros_offboard_controller',
    namespace=namespace,
    parameters=[{'use_sim_time': use_sim_time, }],
  )

  vru_high_level_controller_cmd = Node(
    package='high_level_controller',
    executable='task_manager',
    output='log',
    namespace=namespace,
    parameters=[{'use_sim_time': use_sim_time, }],
  )

  # Create the launch description and populate
  ld = LaunchDescription()

  ld.add_action(declare_namespace_cmd)
  ld.add_action(declare_use_namespace_cmd)
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(vru_low_level_controller_cmd)
  ld.add_action(vru_high_level_controller_cmd)
  return ld

##ros2 run mavros mavros_node --ros-args --param fcu_url:=serial:///dev/ttyACM0:57600  -r tf:=/px4/tf  -r tf_static:=/px4/tf_static 