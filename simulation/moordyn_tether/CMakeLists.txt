cmake_minimum_required(VERSION 3.8)
project(moordyn_tether)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(moordyn_vendor REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# --- Pipeline library (pure C++ + MoorDyn, no ROS) ---
add_library(moordyn_tether_pipeline SHARED
  src/moordyn_tether_pipeline.cpp
  src/moordyn_tether_kinematics.cpp
  src/moordyn_tether_winch.cpp
  src/moordyn_tether_filter.cpp
)
target_include_directories(moordyn_tether_pipeline PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${moordyn_vendor_INCLUDE_DIRS}
)
target_link_libraries(moordyn_tether_pipeline
  ${moordyn_vendor_LIBRARIES}
)

# --- Node executable (entry + ROS wrapper) ---
add_executable(moordyn_tether_node
  src/moordyn_tether_main.cpp
  src/moordyn_tether_node.cpp
)
target_include_directories(moordyn_tether_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(moordyn_tether_node moordyn_tether_pipeline)
ament_target_dependencies(moordyn_tether_node
  rclcpp
  ament_index_cpp
  ros_gz_interfaces
  nav_msgs
  tf2_ros
  geometry_msgs
  visualization_msgs
)

# --- Install ---
install(TARGETS moordyn_tether_pipeline moordyn_tether_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
