cmake_minimum_required(VERSION 3.15)
project(cybergear_description)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

# Install all needed directories
install(
  DIRECTORY launch meshes robot config
  DESTINATION share/${PROJECT_NAME}
)

# Declare this as an ament package
ament_package()
