cmake_minimum_required(VERSION 3.5)
project(ats_control)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)

# add executable boat_controller
add_executable(boat_controller src/boat_controller.cpp)
target_include_directories(boat_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(boat_controller rclcpp geometry_msgs sensor_msgs std_msgs)

# add executable drone_tracker
add_executable(drone_tracker src/drone_tracker.cpp)
target_include_directories(drone_tracker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(drone_tracker rclcpp nav_msgs px4_msgs std_msgs)

# install
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# O DESTINATION correto é lib/${PROJECT_NAME}
install(TARGETS
  boat_controller
  drone_tracker
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
