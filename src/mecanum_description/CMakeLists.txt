cmake_minimum_required(VERSION 3.8)
project(mecanum_description)

find_package(ament_cmake REQUIRED)

# Install URDF files
install(
  DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# Install launch directory if it exists
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
