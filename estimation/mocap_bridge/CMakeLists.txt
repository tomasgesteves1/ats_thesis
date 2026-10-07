cmake_minimum_required(VERSION 3.8)
project(mocap_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_lib
  src/mocap_bridge_kinematics.cpp
  src/mocap_bridge_pipeline.cpp
)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_lib Eigen3::Eigen)

add_executable(mocap_bridge_node
  src/mocap_bridge_main.cpp
  src/mocap_bridge_node.cpp
)
target_link_libraries(mocap_bridge_node
  ${PROJECT_NAME}_lib
)
ament_target_dependencies(mocap_bridge_node
  rclcpp
  nav_msgs
  geometry_msgs
  px4_msgs
)

install(TARGETS
  ${PROJECT_NAME}_lib
  mocap_bridge_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
