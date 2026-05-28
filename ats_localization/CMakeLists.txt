cmake_minimum_required(VERSION 3.8)
project(ats_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Executables
add_executable(frame_aligner src/frame_aligner.cpp)
ament_target_dependencies(frame_aligner
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  px4_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

install(TARGETS
  frame_aligner
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
