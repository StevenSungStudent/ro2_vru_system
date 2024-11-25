#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "vru_msgs/msg/task_list.hpp"
using std::placeholders::_1;

class TaskVerificator : public rclcpp::Node
{
  public:
    TaskVerificator()
    : Node("TaskVerificator")
    {
      subscription_ = this->create_subscription<vru_msgs::msg::TaskList>(
      "Orchestrator/Formated_TaskList", 10, std::bind(&TaskVerificator::topic_callback, this, _1));

      publisher_ = this->create_publisher<vru_msgs::msg::TaskList>("Orchestrator/Verified_TaskList", 10);
    }

  private:
    void topic_callback(const vru_msgs::msg::TaskList & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "Recived task list for verification with ID: '%d'", msg.list_id);
      auto sendingMsg = msg;
      bool succes = verifyTaskList(sendingMsg);

      if(succes == true)
      {
        send_tasklist(sendingMsg);
      }
    }

    void send_tasklist(vru_msgs::msg::TaskList message) const
    {
      RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", message.list_id);
      publisher_->publish(message);
    }

    //Check for missing information or unexpected values, This prevents Unexecutable tasks from reaching the VRU robots.
    bool verifyTaskList(vru_msgs::msg::TaskList & inputTaskList) const
    {
      // Adding 100 to the ID to validate this functions has been passed.
      inputTaskList.list_id = inputTaskList.list_id + 100;

      //#TODO

      //Place for Verfifivcation.
      return true;
    }
    rclcpp::Subscription<vru_msgs::msg::TaskList>::SharedPtr subscription_;
    rclcpp::Publisher<vru_msgs::msg::TaskList>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  (void) argc;
  (void) argv;

  printf("Start up TaskVerification\n");
  
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TaskVerificator>());
  rclcpp::shutdown();
  return 0;
}
