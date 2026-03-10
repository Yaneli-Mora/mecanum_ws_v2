cmake_minimum_required(VERSION 3.8)
project(mecanum_hardware)

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(mecanum_hardware SHARED
  src/mec_system.cpp
  src/encoder_reader.cpp
  src/motor_driver.cpp
  src/serial_manager.cpp)

target_include_directories(mecanum_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

ament_target_dependencies(mecanum_hardware
  hardware_interface pluginlib rclcpp rclcpp_lifecycle)

pluginlib_export_plugin_description_file(
  hardware_interface mecanum_hardware_plugin.xml)

install(TARGETS mecanum_hardware DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(FILES mecanum_hardware_plugin.xml DESTINATION share/${PROJECT_NAME})
ament_package()
