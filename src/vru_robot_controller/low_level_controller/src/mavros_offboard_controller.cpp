#include "mavros_offboard_controller.hpp"
#include <cstdlib>
#include <iostream>
using std::placeholders::_1;

//============================================================================================
//CONSTRUCTOR AND DESTRUCTOR

        //   self.task_status_pub = self.create_publisher(TaskStatus,'VRU_robot_controller/TaskStatus',10)
        // self.command_sub = self.create_subscription(Task,'VRU_robot_controller/Task',self.command_callback,10)
        // self.robot_status_pub = self.create_publisher(Status,'VRU_robot_controller/Status',10)
        // self.local_traj_pub = self.create_publisher(Path,"mavros/setpoint_trajectory/desired",10)

mavros_offboard_controller::mavros_offboard_controller() : Node("mavros_offboard_controller"), publishing_frequency(20), error_radius(2), current_mission_state(STOPPED){
  RCLCPP_INFO(rclcpp::get_logger("mavros_offboard_controller"), "Controller initialized");

  // Publishers, subscribers and clients.
  state_subscriber = this->create_subscription<mavros_msgs::msg::State>( "mavros/state", 10, std::bind(&mavros_offboard_controller::state_callback, this, _1));			
  odometry_subscriber = this->create_subscription<nav_msgs::msg::Odometry>( "mavros/local_position/odom", 10, std::bind(&mavros_offboard_controller::odometry_callback, this, _1));

  local_position_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>( "mavros/setpoint_position/local", 10);
  status_publisher = this->create_publisher<vru_msgs::msg::Status>("VRU_robot_controller/Status", 10);

  arming_client = this->create_client<mavros_msgs::srv::CommandBool>("mavros/cmd/arming");	
  mode_setting_client = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
  timer = this->create_wall_timer(std::chrono::milliseconds(publishing_frequency),std::bind(&mavros_offboard_controller::local_position_callback, this));

  current_task_status.status_id = 0;
  current_task_status.compleated = false;
}

mavros_offboard_controller::~mavros_offboard_controller() {
  RCLCPP_INFO(rclcpp::get_logger("mavros_offboard_controller"), "Controller destroyed");
}


//============================================================================================
//CALLBACKS
void mavros_offboard_controller::state_callback(const mavros_msgs::msg::State& msg)
{
  current_state = msg;

  if(!msg.armed)//TODO: This is probably not a safe way to do arm the robot, its probably better if arming is done via the GUI.
  {
    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = true;

    arming_client->async_send_request(request);
  }

  if(msg.mode != "OFFBOARD")
  {
    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->custom_mode = "OFFBOARD";

    mode_setting_client->async_send_request(request);
  }

}

void mavros_offboard_controller::odometry_callback(const nav_msgs::msg::Odometry& msg) //updates the desired waypoint.
{
  static unsigned long long path_index = 0;
  current_position = msg;


        //   self.robot_status.location_x = msg.pose.pose.position.x
        // self.robot_status.location_y = msg.pose.pose.position.y
        // self.robot_status.move_direction_x = self.current_chasing_point.pose.position.x
        // self.robot_status.move_direction_y = self.current_chasing_point.pose.position.y
        // self.robot_status.current_task_status = self.task_status
        // self.robot_status_pub.publish(self.robot_status)

  if(current_mission_state == PATH_FOLLOWING){
    if((abs(msg.pose.pose.position.x - desired_waypoint.pose.position.x) < error_radius) && (abs(msg.pose.pose.position.y - desired_waypoint.pose.position.y) < error_radius)){
      RCLCPP_INFO(rclcpp::get_logger("mavros_offboard_controller"), "Updating to next waypoint.");

      desired_waypoint.pose.position.x = current_task.path.poses.at(path_index).pose.position.x;
      desired_waypoint.pose.position.y = current_task.path.poses.at(path_index).pose.position.y;
      desired_waypoint.pose.position.z = current_task.path.poses.at(path_index).pose.position.z;
      desired_waypoint.header.frame_id = "map";

      path_index++;
    }
  }

  //TODO: self.path_pub.publish(self.path_msg)
  if(path_index >= current_task.path.poses.size()){
    path_index = 0;
    current_task_status.compleated = true;
    current_mission_state = STOPPED;
    RCLCPP_INFO(rclcpp::get_logger("mavros_offboard_controller"), "Path completed, destination reached.");
  }
}

//TODO: design thought, maybe make this publish the current locaction when the desire is reached.
//That way the robot stays in offboard mode, but idk if that wanted.
void mavros_offboard_controller::local_position_callback()
{
  if(current_mission_state == PATH_FOLLOWING){
    desired_waypoint.header.stamp = this->get_clock()->now();
    local_position_publisher->publish(desired_waypoint);
  }
}

void mavros_offboard_controller::command_callback(const vru_msgs::msg::Task & msg)//TODO: this feels kinda redundant.
{
  RCLCPP_INFO(rclcpp::get_logger("mavros_offboard_controller"), "Recived task for executing with ID: "  + msg.task_id);
  current_task = msg;
  process_task(msg);
}


//============================================================================================
//GENERAL FUNCTIONS
void mavros_offboard_controller::process_task(const vru_msgs::msg::Task & task)
{
  current_task_status.status_id += 1;

  current_task_status.current_task = current_task;
  if(task.task_type == task.TYPE_PATH_FOLLOWING){
    current_mission_state = PATH_FOLLOWING;
    current_task_status.compleated = false;
  } 
  else if(task.task_type == task.TYPE_IDLE){
    current_mission_state = STOPPED;
    current_task_status.compleated = true;
  }

  task_status_publisher->publish(current_task_status);
}

void mavros_offboard_controller::load_path(const nav_msgs::msg::Path & path)
{

}

void mavros_offboard_controller::publish_status(const nav_msgs::msg::Odometry& msg)
{
  vru_msgs::msg::Status robot_status;

  robot_status.location_x = msg.pose.pose.position.x;
  robot_status.location_y = msg.pose.pose.position.y;
  robot_status.move_direction_x = desired_waypoint.pose.position.x;
  robot_status.move_direction_y = desired_waypoint.pose.position.y;
  robot_status.current_task_status = current_task_status;

  status_publisher->publish(robot_status);
}