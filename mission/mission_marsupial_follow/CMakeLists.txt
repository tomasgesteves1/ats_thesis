cmake_minimum_required(VERSION 3.8)
project(mission_marsupial_follow)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Include directories
include_directories(include)

# 1. Pipeline library (pure logic)
add_library(mission_marsupial_follow_pipeline
  src/mission_marsupial_follow_pipeline.cpp
)

# 2. Node wrapper executable
add_executable(mission_marsupial_follow_node
  src/mission_marsupial_follow_node.cpp
  src/mission_marsupial_follow_main.cpp
)
target_link_libraries(mission_marsupial_follow_node
  mission_marsupial_follow_pipeline
)
ament_target_dependencies(mission_marsupial_follow_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
)

# Install targets
install(
  TARGETS
  mission_marsupial_follow_pipeline
  mission_marsupial_follow_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
