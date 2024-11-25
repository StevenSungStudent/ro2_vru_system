cmake_minimum_required(VERSION 3.8)
project(low_level_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(vru_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
#-----------------------------------------------------
add_executable(hardware_controller src/hardware_controller.cpp)
target_include_directories(hardware_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(hardware_controller PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(hardware_controller rclcpp std_msgs vru_msgs nav_msgs mavros_msgs)

install(TARGETS hardware_controller
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS

	scripts/mavros_low_level_path_follower.py

  
  DESTINATION lib/${PROJECT_NAME}
)
ament_python_install_package(${PROJECT_NAME})
#-----------------------------------------------------
add_executable(hardware_controller_interface src/hardware_controller_interface.cpp)
target_include_directories(hardware_controller_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(hardware_controller_interface PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(hardware_controller_interface rclcpp std_msgs vru_msgs)

install(TARGETS hardware_controller_interface
  DESTINATION lib/${PROJECT_NAME})
  
#-----------------------------------------------------

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

ament_package()
