cmake_minimum_required(VERSION 3.5)
project(qr_follower_opencv)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Set policies to suppress warnings
if(POLICY CMP0012)
  cmake_policy(SET CMP0012 NEW)
endif()
if(POLICY CMP0057)
  cmake_policy(SET CMP0057 NEW)
endif()

# Add executable
add_executable(qr_follow_opencv src/qr_follow_opencv_node.cpp)

# Link dependencies
ament_target_dependencies(qr_follow_opencv
  rclcpp
  image_transport
  sensor_msgs
  cv_bridge
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(qr_follow_opencv
  ${OpenCV_LIBRARIES}
)

# Install targets
install(TARGETS qr_follow_opencv
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
