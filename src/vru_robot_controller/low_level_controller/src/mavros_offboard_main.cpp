#include "mavros_offboard_controller.hpp"

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<mavros_offboard_controller>());
    rclcpp::shutdown();

    return 0;
}
