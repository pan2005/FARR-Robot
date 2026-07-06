cmake_minimum_required(VERSION 3.8)
project(farr_pointcloud_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(slice_cloud_node src/slice_cloud_node.cpp)
ament_target_dependencies(slice_cloud_node rclcpp sensor_msgs)

install(TARGETS slice_cloud_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
