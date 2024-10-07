cmake_minimum_required(VERSION 3.5)
project(camera_publisher)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

add_executable(camera_publisher src/camera_publisher.cpp)

include_directories(include ${PCL_INCLUDE_DIRS} ${pcl_conversions_INCLUDE_DIRS})


ament_target_dependencies(camera_publisher
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  yaml-cpp
  message_filters
  PCL 
  pcl_conversions
)

target_link_libraries(camera_publisher ${OpenCV_LIBS} yaml-cpp)

install(TARGETS
  camera_publisher
  DESTINATION lib/${PROJECT_NAME})

ament_package()
