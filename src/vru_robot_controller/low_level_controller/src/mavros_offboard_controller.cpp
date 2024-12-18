#include "mavros_offboard_controller.hpp"

mavros_offboard_controller::mavros_offboard_controller() : Node("mavros_offboard_controller")
{
  RCLCPP_INFO(rclcpp::get_logger("mavros_offboard_controller"),
              "Controller initialized");
}

mavros_offboard_controller::~mavros_offboard_controller() {
  RCLCPP_INFO(rclcpp::get_logger("mavros_offboard_controller"),
              "Controller destroyed");
}
