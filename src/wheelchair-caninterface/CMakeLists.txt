cmake_minimum_required(VERSION 3.5)
project(wheelchair-caninterface)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(can_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
add_executable(caninterface src/caninterface.cpp src/can_wheelchair.c )
add_executable(cmd_speed_x_y src/cmd_speed_x_y.cpp)
add_executable(wheel_speed_to_odometry src/wheel_speed_to_odom.cpp)
ament_target_dependencies(wheel_speed_to_odometry rclcpp std_msgs nav_msgs tf2_geometry_msgs tf2 can_msgs geometry_msgs)
ament_target_dependencies(caninterface rclcpp std_msgs can_msgs geometry_msgs)
ament_target_dependencies(cmd_speed_x_y rclcpp std_msgs geometry_msgs sensor_msgs)


target_include_directories(caninterface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS caninterface
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS
  cmd_speed_x_y
  wheel_speed_to_odometry
  DESTINATION lib/${PROJECT_NAME})
  
  install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
  
  
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
