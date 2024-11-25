cmake_minimum_required(VERSION 3.8)
project(vru_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
#-------v
find_package(rosidl_default_generators REQUIRED)
find_package(nav_msgs REQUIRED)

set(dependencies
  builtin_interfaces 
  std_msgs
  nav_msgs
  rosidl_default_generators
)

rosidl_generate_interfaces(${PROJECT_NAME}

    "msg/DesiredMotion.msg"
    "msg/EmergencyBrake.msg"
    "msg/Events.msg"
    "msg/Status.msg"
    "msg/Task.msg"
    "msg/TaskList.msg"
    "msg/TaskStatus.msg"
    "msg/TopicData.msg"
    "msg/VRURobotData.msg"
    "msg/HardwareControls.msg"
    "msg/SensorData.msg"

    DEPENDENCIES ${dependencies}
) 


#-------^

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies})

ament_package()