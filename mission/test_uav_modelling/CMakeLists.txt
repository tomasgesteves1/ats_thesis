cmake_minimum_required(VERSION 3.8)
project(test_uav_modelling)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install python scripts
install(
  PROGRAMS scripts/excitation_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
