cmake_minimum_required(VERSION 3.8)
project(coordinator)

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


#---------------------------------------------
add_executable(event_handler src/event_handler.cpp)
target_include_directories(event_handler PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(event_handler PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(event_handler nav_msgs rclcpp std_msgs vru_msgs)

install(TARGETS event_handler
  DESTINATION lib/${PROJECT_NAME})
  
#---------------------------------------------
add_executable(monitoring src/monitoring.cpp)
target_include_directories(monitoring PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(monitoring PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(monitoring rclcpp nav_msgs std_msgs vru_msgs)


install(TARGETS monitoring
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS

  scripts/user_interface.py
  scripts/coordinator_layout.py
  scripts/OSMHandler.py
  
  
  DESTINATION lib/${PROJECT_NAME}
)

# install(PROGRAMS

#   scripts/user_interface.py
#   scripts/coordinator_layout.py
  
  
#   DESTINATION share/${PROJECT_NAME}
# )

install(DIRECTORY
  map
  #launch
  DESTINATION share/${PROJECT_NAME}
)
ament_python_install_package(${PROJECT_NAME})
#---------------------------------------------   

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
