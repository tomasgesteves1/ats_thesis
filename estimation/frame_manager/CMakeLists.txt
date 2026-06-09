cmake_minimum_required(VERSION 3.8)
project(frame_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Include directories
include_directories(include)

# Executables
add_executable(frame_manager_node
  src/frame_manager_main.cpp
  src/frame_manager_node.cpp
  src/frame_manager_pipeline.cpp
  src/frame_manager_geometry.cpp
)

ament_target_dependencies(frame_manager_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

install(TARGETS
  frame_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_lint_auto_find_test_dependencies REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
