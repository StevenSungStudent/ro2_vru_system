# Description: Launch file for the simulation of the VRU rover in Gazebo

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

  # Set the path to different files and folders.
  pkg_share = FindPackageShare(package='basic_mobile_robot').find('basic_mobile_robot')
  default_model_path = os.path.join(pkg_share, 'models/vru_rover_static.urdf')
  default_custom_model_path = os.path.join(pkg_share, 'models/r1_rover')
  default_rviz_config_path = os.path.join(pkg_share, 'rviz/sil_sim.rviz')
  px4_directory = os.path.expanduser('~/PX4-Autopilot')
  
  # Launch configuration variables specific to simulation
  rviz_config_file = LaunchConfiguration('rviz_config_file')
  model = LaunchConfiguration('model')
  namespace = LaunchConfiguration('namespace')
  use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
  use_sim_time = LaunchConfiguration('use_sim_time')
  use_rviz = LaunchConfiguration('use_rviz')
  custom_model = LaunchConfiguration('custom_model')
  
  # Map fully qualified names to relative ones so the node's namespace can be prepended.
  # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
  # https://github.com/ros/geometry2/issues/32
  # https://github.com/ros/robot_state_publisher/pull/30
  # TODO(orduno) Substitute with `PushNodeRemapping`
  #              https://github.com/ros2/launch_ros/issues/56
  remappings = [('robot_description','/vru/robot_description'),
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static')]
  
  # Declare the launch arguments  
  declare_namespace_cmd = DeclareLaunchArgument(
    name='namespace',
    default_value='',
    description='Top-level namespace')

  declare_use_namespace_cmd = DeclareLaunchArgument(
    name='use_namespace',
    default_value='False',
    description='Whether to apply a namespace to the navigation stack')
        
  declare_model_path_cmd = DeclareLaunchArgument(
    name='model', 
    default_value=default_model_path, 
    description='Absolute path to robot urdf file')
  
  declare_custom_model_path_cmd = DeclareLaunchArgument(
    name='custom_model',
    default_value=default_custom_model_path,
    description='path to the robot sdf file (TODO might merge with other model)'
  )

  declare_rviz_config_file_cmd = DeclareLaunchArgument(
    name='rviz_config_file',
    default_value=default_rviz_config_path,
    description='Full path to the RVIZ config file to use')

  declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
    name='use_robot_state_pub',
    default_value='True',
    description='Whether to start the robot state publisher')

  declare_use_rviz_cmd = DeclareLaunchArgument(
    name='use_rviz',
    default_value='True',
    description='Whether to start RVIZ')
    
  declare_use_sim_time_cmd = DeclareLaunchArgument(
    name='use_sim_time',
    default_value='True',
    description='Use simulation (Gazebo) clock if true')
  
  declare_px4_model_cmd = DeclareLaunchArgument(
      name='px4_model', default_value='gazebo-classic_r1_rover',
      description='PX4 SITL model to simulate')
  

  start_rviz_cmd = Node(
    condition=IfCondition(use_rviz),
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    parameters=[{'use_sim_time': use_sim_time,}],
    arguments=['-d', rviz_config_file])    
  
  # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
  start_robot_joint_state_publisher_cmd = Node(
    package='basic_mobile_robot',
    executable='joint_state_publisher.py',
    namespace=namespace,
    parameters=[{'use_sim_time': use_sim_time, }],
    remappings=remappings,)

  start_robot_state_publisher_cmd = Node(
    condition=IfCondition(use_robot_state_pub),
    package='robot_state_publisher',
    executable='robot_state_publisher',
    namespace=namespace,
    parameters=[{'use_sim_time': use_sim_time, 
    'robot_description': Command(['xacro ', model])}],
    remappings=remappings,
    arguments=[default_model_path])

  vru_low_level_controller_cmd = Node(
    package='low_level_controller',
    executable='mavros_low_level_path_follower.py',
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

  orchestrator_ui_cmd = Node(
    package='coordinator',
    executable='user_interface.py',
    namespace=namespace,
    parameters=[{'use_sim_time': use_sim_time, }],
  )

  dut_hight_level_controller_cmd = Node(
    package='basic_mobile_robot',
    executable='task_manger_streetdrone.py',
    parameters=[{'use_sim_time': use_sim_time, }],
  )
  
  px4_simulation_cmd = ExecuteProcess(
    cmd=['make', '-C', px4_directory, 'px4_sitl', LaunchConfiguration('px4_model')],
    additional_env={
        'GAZEBO_MODEL_PATH': 'custom_model'
    },
    output='screen'
  )
  print('HERE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
  print(px4_simulation_cmd)

  
  # Create the launch description and populate
  ld = LaunchDescription()

  # Declare the launch options
  ld.add_action(declare_namespace_cmd)
  ld.add_action(declare_use_namespace_cmd)
  ld.add_action(declare_use_robot_state_pub_cmd)  
  ld.add_action(declare_use_rviz_cmd) 
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_model_path_cmd)
  ld.add_action(declare_rviz_config_file_cmd)
  ld.add_action(declare_custom_model_path_cmd)
  ld.add_action(declare_px4_model_cmd)

  # Adding nodes and command
  ld.add_action(px4_simulation_cmd)
  ld.add_action(start_rviz_cmd)
  ld.add_action(start_robot_joint_state_publisher_cmd)
  ld.add_action(start_robot_state_publisher_cmd)
  ld.add_action(vru_low_level_controller_cmd)
  ld.add_action(vru_high_level_controller_cmd)
  ld.add_action(orchestrator_ui_cmd)
  ld.add_action(dut_hight_level_controller_cmd)
  

  return ld

##ros2 service call /datum robot_localization/srv/SetDatum '{geo_pose: {position: {latitude: 51.41588358903864, longitude: 5.876291768287732 , altitude: 13.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'

##ros2 run mavros mavros_node --ros-args --param fcu_url:=serial:///dev/ttyACM0:57600  -r tf:=/px4/tf  -r tf_static:=/px4/tf_static 
##ros2 run mavros mavros_node --ros-args --param fcu_url:=udp://:14540@ -r tf:=/px4/tf  -r tf_static:=/px4/tf_static 