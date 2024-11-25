#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "vru_msgs/msg/task_list.hpp"
using std::placeholders::_1;

class Reformatter : public rclcpp::Node
{
  public:
    Reformatter()
    : Node("Reformatter")
    {
      subscription_ = this->create_subscription<vru_msgs::msg::TaskList>(
      "UserInterface/OUT/TaskList", 10, std::bind(&Reformatter::topic_callback, this, _1));

      publisher_ = this->create_publisher<vru_msgs::msg::TaskList>("Orchestrator/Formated_TaskList", 10);
    }

  private:
    void topic_callback(const vru_msgs::msg::TaskList & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "Recived task list for reformating with ID: '%d'", msg.list_id);
      auto sendingMsg = msg;
      bool succes = reformatTaskList(sendingMsg);
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

    //Based on the input the Tasklist is proparly reformatted this is to allow for Variant input form the User Interface.
    bool reformatTaskList(vru_msgs::msg::TaskList & inputTaskList) const
    {
      // Adding 10000 to the ID to validate this functions has been passed.
      inputTaskList.list_id = inputTaskList.list_id + 10000;

      //#TODO

      //check for optional reformating
      return true;
    }
    rclcpp::Subscription<vru_msgs::msg::TaskList>::SharedPtr subscription_;
    rclcpp::Publisher<vru_msgs::msg::TaskList>::SharedPtr publisher_;
};


int main(int argc, char * argv[])
{
  (void) argc;
  (void) argv;

  printf("Start up Reformatter\n");
  
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Reformatter>());
  rclcpp::shutdown();
  return 0;
}
