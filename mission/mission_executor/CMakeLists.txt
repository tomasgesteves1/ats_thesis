cmake_minimum_required(VERSION 3.8)
project(mission_executor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(mission_maneuvers REQUIRED)

include_directories(include)

# 1. Pipeline library (pure C++: mission loading and sequencing of the maneuvers)
add_library(mission_executor_pipeline
  src/mission_executor_loader.cpp
  src/mission_executor_pipeline.cpp
)
target_link_libraries(mission_executor_pipeline mission_maneuvers::mission_maneuvers yaml-cpp)

# 2. Node wrapper executable
add_executable(mission_executor_node
  src/mission_executor_node.cpp
  src/mission_executor_main.cpp
)
target_link_libraries(mission_executor_node mission_executor_pipeline)
ament_target_dependencies(mission_executor_node
  rclcpp nav_msgs geometry_msgs std_msgs px4_msgs tf2_ros ament_index_cpp)

install(TARGETS mission_executor_pipeline mission_executor_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch missions DESTINATION share/${PROJECT_NAME})

ament_package()
