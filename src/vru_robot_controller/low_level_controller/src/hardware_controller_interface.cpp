#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "vru_msgs/msg/hardware_controls.hpp"
#include "vru_msgs/msg/status.hpp"
#include "vru_msgs/msg/desired_motion.hpp"
#include "vru_msgs/msg/sensor_data.hpp"
using std::placeholders::_1;

class HardwareController : public rclcpp::Node
{
  public:
    HardwareController()
    : Node("HardwareController")
    {
      subscription_DesiredMotion = this->create_subscription<vru_msgs::msg::DesiredMotion>(
      "VRU_robot_controller/DesiredMotion", 10, std::bind(&HardwareController::topic_callback_DesiredMotion, this, _1));

      subscription_SensorData = this->create_subscription<vru_msgs::msg::SensorData>(
      "VRU_robot_controller/SensorData", 10, std::bind(&HardwareController::topic_callback_SensorData, this, _1));

      publisher_HardwareControls = this->create_publisher<vru_msgs::msg::HardwareControls>("VRU_robot_controller/HardwareControls", 10);

      publisher_Status = this->create_publisher<vru_msgs::msg::Status>("VRU_robot_controller/Status", 10);

    }

  private:

    //vru_msgs::msg::Task CurrentTask;

    void topic_callback_DesiredMotion(const vru_msgs::msg::DesiredMotion & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "Recived task for DesiredMotion with location_x: '%f'", msg.location_x);
      auto CurrentDesiredMotion = msg;
      ProccesTask(CurrentDesiredMotion);
    }

    void topic_callback_SensorData(const vru_msgs::msg::SensorData & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "Recived SensorData with: '%d'", msg.temp);
      //Procces and send Status
    }

    void send_HardwareControls(vru_msgs::msg::HardwareControls message) const
    {
      RCLCPP_INFO(this->get_logger(), "Publishing HardwareControls: '%d'", message.temp);
      publisher_HardwareControls->publish(message);
    }

    void ProccesTask(vru_msgs::msg::DesiredMotion & inputTask) const
    {
      //procces Task to send to controller (pixhawk or other device)
      vru_msgs::msg::HardwareControls TestControls = vru_msgs::msg::HardwareControls();
      TestControls.temp = inputTask.location_x;
      
      send_HardwareControls(TestControls);
    }
    rclcpp::Subscription<vru_msgs::msg::DesiredMotion>::SharedPtr subscription_DesiredMotion;
    rclcpp::Subscription<vru_msgs::msg::SensorData>::SharedPtr subscription_SensorData;
    rclcpp::Publisher<vru_msgs::msg::Status>::SharedPtr publisher_Status;
    rclcpp::Publisher<vru_msgs::msg::HardwareControls>::SharedPtr publisher_HardwareControls;
};


int main(int argc, char * argv[])
{
  (void) argc;
  (void) argv;

  printf("Start up HardwareController\n");
  
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HardwareController>());
  rclcpp::shutdown();
  return 0;
}

// #include <cstdio>

// int main(int argc, char ** argv)
// {
//   (void) argc;
//   (void) argv;

//   printf("hello world high_level_controller package\n");
//   return 0;
// }
