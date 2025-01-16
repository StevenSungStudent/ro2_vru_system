#ifndef MAVROS_OFFBOARD_CONTROLLER_HPP
#define MAVROS_OFFBOARD_CONTROLLER_HPP

// NOTE:
// I used the following example as a guide to write this code:
// https://docs.px4.io/main/en/ros/mavros_offboard_cpp.html
// BTW: this is the header file, the source file (where the implementation of the header code is) is in the src folder.

//TODO:
//There is a spelling mistake in the code: compleated.
//Figure out what an acceptable error radius is. 
//The rviz stuff needs to be added / published.
//Huge error on the position, gps calibration? might just be fixed when the GPS that is meant to be used is implemented.

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "vru_msgs/msg/task_status.hpp"
#include "vru_msgs/msg/task.hpp"
#include "vru_msgs/msg/status.hpp"
#include "vru_msgs/msg/task_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>


class mavros_offboard_controller : public rclcpp::Node
{
public:
  mavros_offboard_controller();
  ~mavros_offboard_controller();

private:
  // Enum
  enum mission_state{
    PATH_FOLLOWING,
    STOPPED
  };



  // Variables
  const unsigned short publishing_frequency;
  const double error_radius;                                                              // The allowed error around the waypoints, in meters. So how close the robot has to be to the waypoint before it decides it has reached it.

  rclcpp::QoS qos_profile;

  mavros_msgs::msg::State current_state;                                                  // TODO: maybe redundant.
  nav_msgs::msg::Odometry current_position;
  vru_msgs::msg::Task current_task;
  mission_state current_mission_state;
  vru_msgs::msg::TaskStatus current_task_status;
  geometry_msgs::msg::PoseStamped desired_waypoint;                                       //A waypoint TOWARD the desired position, so its NOT the end destination.


  // Subscribers, Publishers and Clients.
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_subscriber;  			      // current state of the robot. hold,// manual, offboard mode, etc.
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber;           // location of the robot (or at least where the robot thinks it is).
  rclcpp::Subscription<vru_msgs::msg::Task>::SharedPtr command_subscriber;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_position_publisher; // Publishes the waypoints for the robot to move to. (also without this the robot also cant arm in offboard mode)
  rclcpp::Publisher<vru_msgs::msg::TaskStatus>::SharedPtr task_status_publisher;
  rclcpp::Publisher<vru_msgs::msg::Status>::SharedPtr status_publisher;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr local_trajectory_publisher;

  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client;				          // arming the robot.
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_setting_client;     
  rclcpp::TimerBase::SharedPtr timer;                                                     // Timer for the local position publisher.


  // Callbacks
  void state_callback(const mavros_msgs::msg::State& msg);
  void odometry_callback(const nav_msgs::msg::Odometry& msg);                             // Updates the position of the next waypoint.
  void local_position_callback();
  void command_callback(const vru_msgs::msg::Task & msg);


  // General Functions
  void process_task(const vru_msgs::msg::Task & task);
  void load_path(const nav_msgs::msg::Path & path);                                       // TODO: I think this is redundant.
  void publish_status(const nav_msgs::msg::Odometry& msg);
};

#endif	// MAVROS_OFFBOARD_CONTROLLER_HPP
