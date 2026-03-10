cmake_minimum_required(VERSION 3.8)
project(duck_operations)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

set(DEPS rclcpp geometry_msgs std_msgs sensor_msgs nav_msgs
         tf2 tf2_geometry_msgs cv_bridge image_transport)

foreach(node duck_detector_node plow_controller_node)
  add_executable(${node} src/${node}.cpp)
  target_include_directories(${node} PUBLIC include ${OpenCV_INCLUDE_DIRS})
  ament_target_dependencies(${node} ${DEPS})
  target_link_libraries(${node} ${OpenCV_LIBS})
  install(TARGETS ${node} DESTINATION lib/${PROJECT_NAME})
endforeach()

install(DIRECTORY include/ DESTINATION include)
ament_package()
