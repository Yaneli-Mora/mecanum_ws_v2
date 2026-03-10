cmake_minimum_required(VERSION 3.8)
project(mecanum_localization)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(DEPS rclcpp nav_msgs sensor_msgs geometry_msgs std_msgs tf2 tf2_ros tf2_geometry_msgs)

foreach(node odometry_publisher initial_pose_publisher white_line_corrector flow_odometry_node)
  add_executable(${node} src/${node}.cpp)
  target_include_directories(${node} PUBLIC include)
  ament_target_dependencies(${node} ${DEPS})
  install(TARGETS ${node} DESTINATION lib/${PROJECT_NAME})
endforeach()

install(DIRECTORY include/ DESTINATION include)
ament_package()