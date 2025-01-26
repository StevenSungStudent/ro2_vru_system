# Description: Launch file for the simulation of the VRU rover in Gazebo

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription,ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression,FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import threading
USE_AMCL = False

def set_origin():
  os.system("ros2 service call /datum robot_localization/srv/SetDatum '{geo_pose: {position: {latitude: 51.98899371658756, longitude: 5.947916050886852, altitude: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'")

def generate_launch_description():
  # thread_set_origin = threading.Thread(target=set_origin)
  # thread_set_origin.start()
  # Create the launch description and populate
  ld = LaunchDescription()
  # Set the path to different files and folders.
  pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
  pkg_share = FindPackageShare(package='basic_mobile_robot').find('basic_mobile_robot')
  default_launch_dir = os.path.join(pkg_share, 'launch')
  default_model_path = os.path.join(pkg_share, 'models/vru_rover.urdf')
  robot_localization_file_path = os.path.join(pkg_share, 'config/ekf_gps_real.yaml') 
  robot_name_in_urdf = 'basic_mobile_bot'
  default_rviz_config_path = os.path.join(pkg_share, 'rviz/nav2_config.rviz')
  world_file_name = 'basic_mobile_bot_world/smalltown_vru.world'
  world_path = os.path.join(pkg_share, 'worlds', world_file_name)
  nav2_dir = FindPackageShare(package='nav2_bringup').find('nav2_bringup') 
  nav2_launch_dir = os.path.join(nav2_dir, 'launch') 
  static_map_path = os.path.join(pkg_share, 'maps', 'smalltown_world.yaml')
  nav2_params_path = os.path.join(pkg_share, 'params', 'nav2_params.yaml')
  nav2_bt_path = FindPackageShare(package='nav2_bt_navigator').find('nav2_bt_navigator')
  behavior_tree_xml_path = os.path.join(nav2_bt_path, 'behavior_trees', 'navigate_w_replanning_and_recovery.xml')
  
  # Launch configuration variables specific to simulation
  autostart = LaunchConfiguration('autostart')
  default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
  headless = LaunchConfiguration('headless')
  map_yaml_file = LaunchConfiguration('map')
  model = LaunchConfiguration('model')
  namespace = LaunchConfiguration('namespace')
  params_file = LaunchConfiguration('params_file')
  rviz_config_file = LaunchConfiguration('rviz_config_file')
  slam = LaunchConfiguration('slam')
  use_namespace = LaunchConfiguration('use_namespace')
  use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
  use_rviz = LaunchConfiguration('use_rviz')
  use_sim_time = LaunchConfiguration('use_sim_time')
  use_simulator = LaunchConfiguration('use_simulator')
  world = LaunchConfiguration('world')
  
  # Map fully qualified names to relative ones so the node's namespace can be prepended.
  # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
  # https://github.com/ros/geometry2/issues/32
  # https://github.com/ros/robot_state_publisher/pull/30
  # TODO(orduno) Substitute with `PushNodeRemapping`
  #              https://github.com/ros2/launch_ros/issues/56
  remappings = [('/tf', 'tf'),
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
        
  declare_autostart_cmd = DeclareLaunchArgument(
    name='autostart', 
    default_value='true',
    description='Automatically startup the nav2 stack')

  declare_bt_xml_cmd = DeclareLaunchArgument(
    name='default_bt_xml_filename',
    default_value=behavior_tree_xml_path,
    description='Full path to the behavior tree xml file to use')
        
  declare_map_yaml_cmd = DeclareLaunchArgument(
    name='map',
    default_value=static_map_path,
    description='Full path to map file to load')
        
  declare_model_path_cmd = DeclareLaunchArgument(
    name='model', 
    default_value=default_model_path, 
    description='Absolute path to robot urdf file')
    
  declare_params_file_cmd = DeclareLaunchArgument(
    name='params_file',
    default_value=nav2_params_path,
    description='Full path to the ROS2 parameters file to use for all launched nodes')
    
  declare_rviz_config_file_cmd = DeclareLaunchArgument(
    name='rviz_config_file',
    default_value=default_rviz_config_path,
    description='Full path to the RVIZ config file to use')

  declare_simulator_cmd = DeclareLaunchArgument(
    name='headless',
    default_value='False',
    description='Whether to execute gzclient')

  declare_slam_cmd = DeclareLaunchArgument(
    name='slam',
    default_value='False',
    description='Whether to run SLAM')
    
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
    default_value='False',
    description='Use simulation (Gazebo) clock if true')

  declare_use_simulator_cmd = DeclareLaunchArgument(
    name='use_simulator',
    default_value='True',
    description='Whether to start the simulator')

  declare_world_cmd = DeclareLaunchArgument(
    name='world',
    default_value=world_path,
    description='Full path to the world model file to load')
   
  # Specify the actions

  # Start Gazebo server
  start_gazebo_server_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
    condition=IfCondition(use_simulator),
    launch_arguments={'world': world}.items())

  # Start Gazebo client    
  start_gazebo_client_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
    condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))

  # Start robot localization using an Extended Kalman filter

  # Start the navsat transform node which converts GPS data into the world coordinate frame
  # start_navsat_transform_cmd = Node(
  #   package='robot_localization',
  #   executable='navsat_transform_node',
  #   name='navsat_transform',
  #   output='screen',
  #   parameters=[robot_localization_file_path, 
  #   {'use_sim_time': False}],
  #   remappings=[('imu', 'imu/data'),
  #               ('gps/fix', 'navsatfix'), 
  #               ('gps/filtered', 'gps/filtered'),
  #               ('odometry/gps', 'odometry/gps'),
  #               ('odometry/filtered', 'odometry/global')])

  # Start robot localization using an Extended Kalman filter...odom->base_footprint transform
  start_robot_localization_global_cmd = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node_map',
    output='screen',
    parameters=[robot_localization_file_path, 
    {'use_sim_time': False}],
    remappings=[('odometry/filtered', 'odometry/global'),
                ('/set_pose', '/initialpose_ekf')])

  # Start robot localization using an Extended Kalman filter...odom->base_footprint transform
  start_robot_localization_local_cmd = Node(
    package='robot_localization',
    executable='ekf_node',
    name='ekf_filter_node_odom',
    output='screen',
    parameters=[robot_localization_file_path, 
    {'use_sim_time': use_sim_time}],
    remappings=[('odometry/filtered', 'odometry/local'),
                ('/set_pose', '/initialpose_ekf')])


  # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
  start_robot_state_publisher_cmd = Node(
    condition=IfCondition(use_robot_state_pub),
    package='robot_state_publisher',
    executable='robot_state_publisher',
    namespace=namespace,
    parameters=[{'use_sim_time': use_sim_time, 
    'robot_description': Command(['xacro ', model])}],
    remappings=remappings,
    arguments=[default_model_path])

  # Launch RViz
  start_rviz_cmd = Node(
    condition=IfCondition(use_rviz),
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    output='screen',
    arguments=['-d', rviz_config_file])    
  if USE_AMCL:
    # Launch the ROS 2 Navigation Stack
    start_ros2_navigation_cmd = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
      launch_arguments = {'namespace': namespace,
                          'use_namespace': use_namespace,
                          'slam': slam,
                          'map': map_yaml_file,
                          'use_sim_time': use_sim_time,
                          'params_file': params_file,
                          'default_bt_xml_filename': default_bt_xml_filename,
                          'autostart': autostart}.items())
  else: 
    start_ros2_navigation_cmd = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'navigation_launch.py')),
      launch_arguments = {'namespace': namespace,
                          'use_namespace': use_namespace,
                          'slam': slam,
                          'map': map_yaml_file,
                          'use_sim_time': use_sim_time,
                          'params_file': params_file,
                          'default_bt_xml_filename': default_bt_xml_filename,
                          'autostart': autostart}.items())
    static_transformation = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['0.21', '0', '0', '0', '0', '0', 'odom', 'map']
        )
    ld.add_action(static_transformation)
    my_dir = FindPackageShare(package='basic_mobile_robot').find('basic_mobile_robot') 
    my_launch_dir = os.path.join(my_dir, 'launch')
    map_sever = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(os.path.join(my_launch_dir, 'map.launch.py')))
    #ld.add_action(map_sever)


  # Declare the launch options
  ld.add_action(declare_namespace_cmd)
  ld.add_action(declare_use_namespace_cmd)
  ld.add_action(declare_autostart_cmd)
  ld.add_action(declare_bt_xml_cmd)
  ld.add_action(declare_map_yaml_cmd)
  ld.add_action(declare_model_path_cmd)
  ld.add_action(declare_params_file_cmd)
  ld.add_action(declare_rviz_config_file_cmd)
  ld.add_action(declare_simulator_cmd)
  ld.add_action(declare_slam_cmd)
  ld.add_action(declare_use_robot_state_pub_cmd)  
  ld.add_action(declare_use_rviz_cmd) 
  ld.add_action(declare_use_sim_time_cmd)
  ld.add_action(declare_use_simulator_cmd)
  ld.add_action(declare_world_cmd)

  # Add any actions
  #ld.add_action(start_gazebo_server_cmd)
  #ld.add_action(start_gazebo_client_cmd)
  # ld.add_action(start_navsat_transform_cmd)
  ld.add_action(start_robot_localization_global_cmd)
  #ld.add_action(start_robot_localization_local_cmd)
  ld.add_action(start_robot_state_publisher_cmd)
  ld.add_action(start_rviz_cmd)
  #ld.add_action(start_ros2_navigation_cmd)


  return ld

##ros2 service call /datum robot_localization/srv/SetDatum '{geo_pose: {position: {latitude: 51.41588358903864, longitude: 5.876291768287732 , altitude: 13.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'