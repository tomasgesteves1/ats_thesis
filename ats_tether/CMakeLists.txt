cmake_minimum_required(VERSION 3.8)
project(ats_tether)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(moordyn_vendor REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)

add_library(TetherMoorDynSystem SHARED
  src/TetherMoorDynSystem.cpp
)

target_include_directories(TetherMoorDynSystem PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${moordyn_vendor_INCLUDE_DIRS}
)

target_link_libraries(TetherMoorDynSystem
  ${moordyn_vendor_LIBRARIES}
)

ament_target_dependencies(TetherMoorDynSystem
  rclcpp
  ament_index_cpp
)

add_executable(ats_tether_node src/ats_tether_node.cpp)

target_link_libraries(ats_tether_node TetherMoorDynSystem)

ament_target_dependencies(ats_tether_node
  rclcpp
  ros_gz_interfaces
  nav_msgs
)

install(TARGETS TetherMoorDynSystem ats_tether_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
