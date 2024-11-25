#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "vru_msgs/msg/task_list.hpp"
#include "vru_msgs/msg/status.hpp"
#include "vru_msgs/msg/events.hpp"
using std::placeholders::_1;

class Monitoring : public rclcpp::Node
{
  public:
    Monitoring()
    : Node("Monitoring")
    {
      subscription_Status = this->create_subscription<vru_msgs::msg::Status>(
      "VRU_robot_controller/OUT/Status", 10, std::bind(&Monitoring::topic_callback_Status, this, _1));

            subscription_TaskList = this->create_subscription<vru_msgs::msg::TaskList>(
      "Orchestrator/OUT/TaskList", 10, std::bind(&Monitoring::topic_callback_TaskList, this, _1));

      publisher_Events = this->create_publisher<vru_msgs::msg::Events>("Orchestrator/Events", 10);
    }

  private:
    void topic_callback_Status(const vru_msgs::msg::Status & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "Recived Status for monitoring with location_x: '%f'", msg.location_x);
      auto sendingMsg = msg;
      HandleStatus(sendingMsg);
    }

        void topic_callback_TaskList(const vru_msgs::msg::TaskList & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "Recived TaskList for monitoring with ID: '%d'", msg.list_id);
      auto sendingMsg = msg;
      HandleTaskList(sendingMsg);
    }


    void send_Events(vru_msgs::msg::Events message) const
    {
      RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.event_id);
      publisher_Events->publish(message);
    }

    void HandleTaskList(vru_msgs::msg::TaskList & inputTaskList) const
    {
      
      //Save tasklist and generate/activate event triggers
      inputTaskList = inputTaskList;

    }

        void HandleStatus(vru_msgs::msg::Status & inputStatus) const
    {
      
      //Compair status to expected status and event triggers
      if(inputStatus.speed <= 0)
      {
        auto TESTEvent = vru_msgs::msg::Events();
        TESTEvent.event_type = "VRU_negetive speed";
        TESTEvent.event_id = inputStatus.speed;
        TESTEvent.param = {11, 12, 13};
        send_Events(TESTEvent);
      }
      
    }
    rclcpp::Subscription<vru_msgs::msg::Status>::SharedPtr subscription_Status;
    rclcpp::Subscription<vru_msgs::msg::TaskList>::SharedPtr subscription_TaskList;
    rclcpp::Publisher<vru_msgs::msg::Events>::SharedPtr publisher_Events;
};

int main(int argc, char * argv[])
{
  (void) argc;
  (void) argv;

  printf("Start up Monitoring\n");
  
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Monitoring>());
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
