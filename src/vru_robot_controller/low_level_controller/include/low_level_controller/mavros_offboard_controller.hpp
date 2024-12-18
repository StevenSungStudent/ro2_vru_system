#ifndef MAVROS_OFFBOARD_CONTROLLER_HPP
#define MAVROS_OFFBOARD_CONTROLLER_HPP

// NOTE:
// I used the following example as a guide to write this code:
// https://docs.px4.io/main/en/ros/mavros_offboard_cpp.html For those unaware:
// BTW: this is the header file, the source file (where the implementation of the header code is) is in the src folder.

//TODO:
// is the commandbool bugged? need to check.

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "nav_msgs/msg/odometry.hpp"
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
  // Variables
  const unsigned short publishing_frequency;
  mavros_msgs::msg::State current_state;//TODO: maybe redundant.
  nav_msgs::msg::Odometry current_position;


  // Subscribers, Publishers and Clients.
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_subscriber;  			      // current state of the robot. hold,// manual, offboard mode, etc.
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber;           // location of the robot (or at least where the robot thinks it is).

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_position_publisher; // for offboard mode happy.

  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client;				          // arming the robot.
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr mode_setting_client;     
  rclcpp::TimerBase::SharedPtr timer;                                                     // Timer for the local position publisher.


  // Callbacks
  void state_callback(const mavros_msgs::msg::State& msg);
  void odometry_callback(const nav_msgs::msg::Odometry& msg);
  void local_position_callback();

  // Functions
};

#endif	// MAVROS_OFFBOARD_CONTROLLER_HPP
