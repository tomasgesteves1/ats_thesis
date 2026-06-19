cmake_minimum_required(VERSION 3.8)
project(mission_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mission_manager_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Include directories
include_directories(include)

# 1. Registry + Executor logic pipeline library
add_library(mission_manager_pipeline
  src/mission_manager_registry.cpp
  src/mission_manager_executor.cpp
  src/mission_manager_pipeline.cpp
)
target_link_libraries(mission_manager_pipeline
  yaml-cpp
)
ament_target_dependencies(mission_manager_pipeline
  rclcpp
  mission_manager_interfaces
)

# 2. Main executable
add_executable(mission_manager_node
  src/mission_manager_node.cpp
  src/mission_manager_main.cpp
)
target_link_libraries(mission_manager_node
  mission_manager_pipeline
)
ament_target_dependencies(mission_manager_node
  rclcpp
  mission_manager_interfaces
  ament_index_cpp
)

# 3. Install targets
install(
  TARGETS
  mission_manager_pipeline
  mission_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY missions/
  DESTINATION share/${PROJECT_NAME}/missions
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
