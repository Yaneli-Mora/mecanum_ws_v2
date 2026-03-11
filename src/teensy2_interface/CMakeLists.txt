cmake_minimum_required(VERSION 3.8)
project(teensy2_interface)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(teensy2_bridge_node src/teensy2_bridge_node.cpp)
target_include_directories(teensy2_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(teensy2_bridge_node rclcpp std_msgs)

install(TARGETS teensy2_bridge_node DESTINATION lib/${PROJECT_NAME})
ament_package()
