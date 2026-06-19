cmake_minimum_required(VERSION 3.8)
project(test_usv_circle)

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
add_library(test_usv_circle_pipeline
  src/test_usv_circle_pipeline.cpp
)

# 2. Node wrapper executable
add_executable(test_usv_circle_node
  src/test_usv_circle_node.cpp
  src/test_usv_circle_main.cpp
)
target_link_libraries(test_usv_circle_node
  test_usv_circle_pipeline
)
ament_target_dependencies(test_usv_circle_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
)

# Install targets
install(
  TARGETS
  test_usv_circle_pipeline
  test_usv_circle_node
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
