
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "vru_msgs/msg/task.hpp"
#include "vru_msgs/msg/task_status.hpp"
#include "vru_msgs/msg/task_list.hpp"
#include "vru_msgs/msg/events.hpp"
#include "vru_msgs/msg/status.hpp"
using std::placeholders::_1;

int TESTVALUE = 0;

using namespace std::chrono_literals;

class Task_manager : public rclcpp::Node
{
  public:
    Task_manager()
    : Node("Task_manager")
    {
      subscription_TaskList = this->create_subscription<vru_msgs::msg::TaskList>(
      "Orchestrator/OUT/vruTaskList", 10, std::bind(&Task_manager::topic_callback_TaskList, this, _1));

      subscription_TaskStatus = this->create_subscription<vru_msgs::msg::TaskStatus>(
      "VRU_robot_controller/TaskStatus", 10, std::bind(&Task_manager::topic_callback_TaskStatus, this, _1));
      // start / stop the first task in the list 
      subscription_Events = this->create_subscription<vru_msgs::msg::Events>(
      "Orchestrator/OUT/Events", 10, std::bind(&Task_manager::topic_callback_Events, this, _1));

      subscription_Status = this->create_subscription<vru_msgs::msg::Status>(
      "VRU_robot_controller/Status", 10, std::bind(&Task_manager::topic_callback_Status, this, _1));

      publisher_Task = this->create_publisher<vru_msgs::msg::Task>("VRU_robot_controller/Task", 10);

      publisher_Status = this->create_publisher<vru_msgs::msg::Status>("VRU_robot_controller/OUT/Status", 10);
      timer_ = this->create_wall_timer(10ms, std::bind(&Task_manager::mainloop_callback, this));
      CurrentTaskStatus.compleated = true;
    }

  private:
    rclcpp::TimerBase::SharedPtr timer_;
    vru_msgs::msg::TaskList CurrentTaskList;
    vru_msgs::msg::TaskStatus CurrentTaskStatus;
    vru_msgs::msg::Task NextTask;
    long unsigned int task_index = 0;

    void mainloop_callback()
    {

      // RCLCPP_INFO(this->get_logger(), "The taskID for the next task: '%d'", NextTask.task_id);
      // RCLCPP_INFO(this->get_logger(), "The start_event for the next task:'%s'", NextTask.start_event.event_type.c_str());
      
      if (CurrentTaskStatus.compleated == true){
        if(NextTask.start_event.event_type == "DELAY_START" )
          {           
                
              if(NextTask.start_event.param[0] <= 0.0)
              {
                sendTask(NextTask);
                return;
              }
      
              NextTask.start_event.param[0]-= 0.01;
              RCLCPP_INFO(this->get_logger(), "Second:'%2.f s to start'", NextTask.start_event.param[0]);
              
          }
      }
     
    }
    void topic_callback_TaskList(const vru_msgs::msg::TaskList & msg)
    {
      // RCLCPP_INFO(this->get_logger(), "Recived task list for for managing with ID: '%d'", msg.list_id);
  
      auto temp = msg;
      reciveNewTaskList(temp);
    }

    void topic_callback_TaskStatus(const vru_msgs::msg::TaskStatus & msg) 
    {
      RCLCPP_INFO(this->get_logger(), "Recived TaskStatus for for managing with taskID: '%d'", msg.current_task.task_id);

      //Handel status info
      CurrentTaskStatus = msg;
      //Sending TESTstatus for testing, needs to be combined with topic_callback_Status
      // if(TESTVALUE == 0)
      // {
      // auto TESTstatus = vru_msgs::msg::Status();
      // TESTstatus.current_task_status = msg;
      // TESTstatus.location_x = 10;
      // TESTstatus.location_y = 10;
      // TESTstatus.speed = 10;
      // sendStatus(TESTstatus);
      // TESTVALUE = 1;
      // }
      // else
      // {
      // auto TESTstatus = vru_msgs::msg::Status();
      // TESTstatus.current_task_status = msg;
      // TESTstatus.location_x = 10;
      // TESTstatus.location_y = 10;
      // TESTstatus.speed = -1;
      // sendStatus(TESTstatus);
      // TESTVALUE = 0;
      // }
    }
    // for E-STOP
    void topic_callback_Events(const vru_msgs::msg::Events & msg)
    {
      // RCLCPP_INFO(this->get_logger(), "Recived Events for for managing with task: '%d'", msg.event_id);
      //Handel Events
      auto temp = msg;
      handleEvent(temp);
    }

     void topic_callback_Status(const vru_msgs::msg::Status & msg)
    {
      // RCLCPP_INFO(this->get_logger(), "Recived Status for for managing with task: '%d'", msg.current_task_status.current_task.task_id);

              
      //Handel Status Info
      auto temp = msg;
      handleExecuterStatus(temp);
    }

    void sendTask(vru_msgs::msg::Task message) 
    {
      RCLCPP_INFO(this->get_logger(), "Publishing to Task Executer task With ID: '%d'", message.task_id);

      if (task_index > (CurrentTaskList.tasks.size()-1))
      {
        RCLCPP_WARN_STREAM(this->get_logger(), "No more tasks in the list");
        return;
      }
      publisher_Task->publish(message);
      
      task_index++;
      if (task_index > (CurrentTaskList.tasks.size()-1))
      {
        RCLCPP_WARN_STREAM(this->get_logger(), "No more tasks in the list");
        return;
      }
      NextTask = CurrentTaskList.tasks[task_index];

    }

    void sendStatus(vru_msgs::msg::Status message) const
    {
      // RCLCPP_INFO(this->get_logger(), "Publishing to Status With ID: '%d'", message.current_task_status.current_task.task_id);
      publisher_Status->publish(message);
    }

    void reciveNewTaskList(vru_msgs::msg::TaskList & inputTaskList)
    {
      if (inputTaskList.tasks.size() == 0)
      {
        RCLCPP_WARN_STREAM(this->get_logger(), "No tasks in the list");
        return;
      }
      if(inputTaskList.tasks[0].task_type != "")
      {
        //NextTask = inputTaskList.tasks[0];
    
        CurrentTaskList = inputTaskList;
        task_index = 0;

        NextTask = CurrentTaskList.tasks[task_index];

      }

      //check for Errors and till the Executer is redy for the next task.
      //Link Evetns to TaskList activation
    }

    void handleEvent(vru_msgs::msg::Events & inputEvent)
    {
      bool isSpecialEvent = checkSpecialEvents(inputEvent);
      if(!isSpecialEvent)
      {
        if(inputEvent.event_type == NextTask.stop_event.event_type)
        {
          //stop Task Executer
          vru_msgs::msg::Task stopTask;
          stopTask.task_type = stopTask.TYPE_IDLE;
          sendTask(stopTask);
        }
        else if(inputEvent.event_type == NextTask.start_event.event_type)
        {
          sendTask(NextTask);
        }
      }
      else
      {
        //handle Special Casses
        return;
      }
    }

    bool checkSpecialEvents(vru_msgs::msg::Events & inputEvent)
    {
      if(inputEvent.event_type == "EmergencyBrake")
      {
        //stop Task Executer
        vru_msgs::msg::Task stopTask;
        stopTask.task_type = stopTask.TYPE_IDLE;
        sendTask(stopTask);
        return true;
      }
      return false;
    }

    void handleExecuterStatus(vru_msgs::msg::Status & inputStatus)
    {
      //check for status information
      sendStatus(inputStatus);

      printf("The current task index: %ld , %ld task left\n",task_index, CurrentTaskList.tasks.size()-task_index);
      //start Next task if applicable
      if(inputStatus.current_task_status.compleated == true)
      {
        if (CurrentTaskList.tasks.size() != 0)
        {
          if(NextTask.start_event.event_type == "" || NextTask.start_event.event_type == "afterPreviousFinish")
          {           
            sendTask(NextTask);

          }
        }
      }
    }

    rclcpp::Subscription<vru_msgs::msg::TaskList>::SharedPtr subscription_TaskList;
    rclcpp::Subscription<vru_msgs::msg::TaskStatus>::SharedPtr subscription_TaskStatus;
    rclcpp::Subscription<vru_msgs::msg::Events>::SharedPtr subscription_Events;
    rclcpp::Subscription<vru_msgs::msg::Status>::SharedPtr subscription_Status;
    rclcpp::Publisher<vru_msgs::msg::Task>::SharedPtr publisher_Task;
    rclcpp::Publisher<vru_msgs::msg::Status>::SharedPtr publisher_Status;
};


int main(int argc, char * argv[])
{
  (void) argc;
  (void) argv;

  printf("Start up Task_manager\n");
  
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Task_manager>());
  rclcpp::shutdown();
  return 0;
}

