
cmake_minimum_required(VERSION 3.8)
project(mecanum_sensors)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

set(DEPS rclcpp std_msgs sensor_msgs geometry_msgs cv_bridge image_transport)

add_executable(teensy_bridge_node    src/teensy_bridge_node.cpp)
add_executable(teensy2_bridge_node   src/teensy2_bridge_node.cpp)
add_executable(fixture_detector_node src/fixture_detector_node.cpp)
add_executable(imu_node              src/imu_node.cpp)
add_executable(camera_node           src/camera_node.cpp)

foreach(node teensy_bridge_node teensy2_bridge_node fixture_detector_node imu_node camera_node)
  target_include_directories(${node} PUBLIC include ${OpenCV_INCLUDE_DIRS})
  ament_target_dependencies(${node} ${DEPS})
  target_link_libraries(${node} ${OpenCV_LIBS})
  install(TARGETS ${node} DESTINATION lib/${PROJECT_NAME})
endforeach()

install(DIRECTORY include/ DESTINATION include)
ament_package()
