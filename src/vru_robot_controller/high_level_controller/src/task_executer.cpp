// #include <memory>

// #include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"
// #include "vru_msgs/msg/task.hpp"
// #include "vru_msgs/msg/task_status.hpp"
// #include "vru_msgs/msg/desired_motion.hpp"
// using std::placeholders::_1;

// class Task_executer : public rclcpp::Node
// {
//   public:
//     Task_executer()
//     : Node("Task_executer")
//     {
//       subscription_Task = this->create_subscription<vru_msgs::msg::Task>(
//       "VRU_robot_controller/Task", 10, std::bind(&Task_executer::topic_callback, this, _1));

//       publisher_DesiredMotion = this->create_publisher<vru_msgs::msg::DesiredMotion>("VRU_robot_controller/DesiredMotion", 10);

//       publisher_TaskStatus = this->create_publisher<vru_msgs::msg::TaskStatus>("VRU_robot_controller/TaskStatus", 10);

//     }

//   private:

//     //vru_msgs::msg::Task CurrentTask;

//     void topic_callback(const vru_msgs::msg::Task & msg) 
//     {
//       RCLCPP_INFO(this->get_logger(), "Recived task for executing with ID: '%d'", msg.task_id);
//       auto CurrentTask = msg;
//       ProccesTask(CurrentTask);
//     }

//     void send_DesiredMotion(vru_msgs::msg::DesiredMotion message) const
//     {
//       RCLCPP_INFO(this->get_logger(), "Publishing task for DesiredMotion with location_x: '%f'", message.location_x);
//       publisher_DesiredMotion->publish(message);
//     }

//     void send_TaskStatus(vru_msgs::msg::TaskStatus message) const
//     {
//       RCLCPP_INFO(this->get_logger(), "Publishing TaskStatus for for managing with taskID: '%d'", message.current_task.task_id);
//       publisher_TaskStatus->publish(message);
//     }

//     void ProccesTask(vru_msgs::msg::Task & inputTask) 
//     {
//       if(inputTask.task_type == "StopTask")
//       {
//         //stop current task
//       }
//       else if(inputTask.task_type == "FollowDefinedPath")
//       {
//         //procces Task to compleation
//         vru_msgs::msg::DesiredMotion TestMotion = vru_msgs::msg::DesiredMotion();
//         TestMotion.location_x = inputTask.param[0];
//         TestMotion.location_y = inputTask.param[1];
//         TestMotion.move_direction_x = inputTask.param[2];
//         TestMotion.move_direction_y = inputTask.param[3];
//         send_DesiredMotion(TestMotion);
//         //controlles based on Pixhawk capabilaty.
//       }
//       else if(inputTask.task_type == "ManualControl")
//       {
//         subscription_DesiredMotion_manualControl = this->create_subscription<vru_msgs::msg::DesiredMotion>(
//         "manualControl", 10, std::bind(&Task_executer::manualControl_callback, this, _1));



//       }
//             //procces Task_status to compleation
//       vru_msgs::msg::TaskStatus TestTaskStatus = vru_msgs::msg::TaskStatus();
//       TestTaskStatus.current_task = inputTask;
//       TestTaskStatus.compleated = false;
//       TestTaskStatus.status_id = 10;
//       send_TaskStatus(TestTaskStatus);
//     }

//     void manualControl_callback(const vru_msgs::msg::DesiredMotion & msg) const
//     {
//       RCLCPP_INFO(this->get_logger(), "Recived Manualcontrols with location_x: '%f'", msg.location_x);
//       auto DesiredMotion = msg;
//       send_DesiredMotion(DesiredMotion);
//     }

//     rclcpp::Subscription<vru_msgs::msg::Task>::SharedPtr subscription_Task;
//     rclcpp::Subscription<vru_msgs::msg::DesiredMotion>::SharedPtr subscription_DesiredMotion_manualControl;
//     rclcpp::Publisher<vru_msgs::msg::DesiredMotion>::SharedPtr publisher_DesiredMotion;
//     rclcpp::Publisher<vru_msgs::msg::TaskStatus>::SharedPtr publisher_TaskStatus;
// };


// int main(int argc, char * argv[])
// {
//   (void) argc;
//   (void) argv;

//   printf("Start up Task_executer\n");
  
//   rclcpp::init(argc, argv);
//   rclcpp::spin(std::make_shared<Task_executer>());
//   rclcpp::shutdown();
//   return 0;
// }

#include <cstdio>

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("not used\n");
  return 0;
}
