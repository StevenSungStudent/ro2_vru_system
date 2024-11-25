#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "vru_msgs/msg/events.hpp"
using std::placeholders::_1;

class EventHandler : public rclcpp::Node
{
  public:
    EventHandler()
    : Node("EventHandler")
    {
      subscription_ = this->create_subscription<vru_msgs::msg::Events>(
      "Orchestrator/Events", 10, std::bind(&EventHandler::topic_callback, this, _1));

      publisher_ = this->create_publisher<vru_msgs::msg::Events>("Orchestrator/OUT/Events", 10);
    }

  private:
    void topic_callback(const vru_msgs::msg::Events & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "Recived Event to handle with ID: '%d'", msg.event_id);
      auto sendingMsg = msg;
      DistributeEvents(sendingMsg);
    }

    void send_Events(vru_msgs::msg::Events message) const
    {
      RCLCPP_INFO(this->get_logger(), "Sending Event with id: '%d'", message.event_id);
      publisher_->publish(message);
    }

    void DistributeEvents(vru_msgs::msg::Events & inputEvents) const
    {
      //Selecting target for event and handling. 

      //#TODO

      send_Events(inputEvents);
      
    }
    rclcpp::Subscription<vru_msgs::msg::Events>::SharedPtr subscription_;
    rclcpp::Publisher<vru_msgs::msg::Events>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  (void) argc;
  (void) argv;

  printf("Start up EventHandler\n");
  
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EventHandler>());
  rclcpp::shutdown();
  return 0;
}


// #include <cstdio>

// int main(int argc, char ** argv)
// {
//   (void) argc;
//   (void) argv;

//   printf("hello world coordinator package\n");
//   return 0;
// }
