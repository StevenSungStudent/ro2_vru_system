#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "vru_msgs/msg/task_list.hpp"
using std::placeholders::_1;

class Distributer : public rclcpp::Node
{
  public:
    Distributer()
    : Node("Distributer")
    {
      subscription_ = this->create_subscription<vru_msgs::msg::TaskList>(
      "Orchestrator/Verified_TaskList", 10, std::bind(&Distributer::topic_callback, this, _1));

      publisher_ = this->create_publisher<vru_msgs::msg::TaskList>("Orchestrator/OUT/TaskList", 10);
    }

  private:
    void topic_callback(const vru_msgs::msg::TaskList & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "Recived task list for Distribution with ID: '%d'", msg.list_id);
      auto sendingMsg = msg;
      DistributeTaskList(sendingMsg);
    }

    void send_tasklist(vru_msgs::msg::TaskList message) const
    {
      RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.list_id);
      publisher_->publish(message); 
    }

    //Distribute the task list to the correct VRU, select the namespace, and possibly consider the availability.
    void DistributeTaskList(vru_msgs::msg::TaskList & inputTaskList) const
    {
      // Adding 1 to the ID to validate this functions has been passed.
      inputTaskList.list_id = inputTaskList.list_id + 1;
      
      //#TODO

      //room for distribution, I sugest looking at Namespaces here But other options might be advantages.

      send_tasklist(inputTaskList);
      
    }
    rclcpp::Subscription<vru_msgs::msg::TaskList>::SharedPtr subscription_;
    rclcpp::Publisher<vru_msgs::msg::TaskList>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  (void) argc;
  (void) argv;

  printf("Start up Distributer\n");
  
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Distributer>());
  rclcpp::shutdown();
  return 0;
}
