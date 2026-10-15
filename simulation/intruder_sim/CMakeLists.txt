cmake_minimum_required(VERSION 3.8)
project(intruder_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include)

# 1. Pipeline library (pure C++ behaviour of the intruder)
add_library(intruder_sim_pipeline src/intruder_sim_pipeline.cpp)

# 2. Node wrapper executable
add_executable(intruder_sim_node src/intruder_sim_node.cpp src/intruder_sim_main.cpp)
target_link_libraries(intruder_sim_node intruder_sim_pipeline)
ament_target_dependencies(intruder_sim_node rclcpp nav_msgs geometry_msgs ros_gz_interfaces tf2_ros)

install(TARGETS intruder_sim_pipeline intruder_sim_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
