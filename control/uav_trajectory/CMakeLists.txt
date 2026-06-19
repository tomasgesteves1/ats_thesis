cmake_minimum_required(VERSION 3.8)
project(uav_trajectory)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(include)

# 1. Shared library for logic pipeline (Rule 1 of CODE_STANDARDS.md)
add_library(uav_trajectory_pipeline
  src/uav_trajectory_pipeline.cpp
)

# 2. Executables for nodes
add_executable(uav_trajectory_circle_node
  src/uav_trajectory_circle_node.cpp
  src/uav_trajectory_circle_main.cpp
)
target_link_libraries(uav_trajectory_circle_node
  uav_trajectory_pipeline
)
ament_target_dependencies(uav_trajectory_circle_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(uav_trajectory_boat_follower_node
  src/uav_trajectory_boat_follower_node.cpp
  src/uav_trajectory_boat_follower_main.cpp
)
target_link_libraries(uav_trajectory_boat_follower_node
  uav_trajectory_pipeline
)
ament_target_dependencies(uav_trajectory_boat_follower_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# 3. Install targets
install(
  TARGETS
  uav_trajectory_pipeline
  uav_trajectory_circle_node
  uav_trajectory_boat_follower_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
