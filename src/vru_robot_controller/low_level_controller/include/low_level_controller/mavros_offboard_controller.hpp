#ifndef MAVROS_OFFBOARD_CONTROLLER_HPP
#define MAVROS_OFFBOARD_CONTROLLER_HPP


#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"

class mavros_offboard_controller : public rclcpp::Node {
public:
  mavros_offboard_controller();
  ~mavros_offboard_controller();

private:
    mavros_msgs::msg::State current_state;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_subscriber;//current state of the robot. hold, manual, offboard mode, etc.
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_position_publisher; //for offboard mode happy.
    rclcpp::Service<mavros_msgs::srv::CommandBool>::SharedPtr arming_client;//arming the robot.

    rclcpp::Rate publishing_frequency;

};

#endif // MAVROS_OFFBOARD_CONTROLLER_HPP
